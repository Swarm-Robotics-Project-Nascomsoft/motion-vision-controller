// ============================================================================
//  CENTRALIZED WAREHOUSE PLANNER v2.0 — "The Brain"
// ============================================================================
//
//  Architecture: Sense -> Plan -> Act
//
//  [Sense]   Receives multi-marker vision data via UDP from Python vision system
//  [Plan]    Converts to metric coordinates, manages task state machines,
//            runs A* pathfinding on an inflated occupancy grid
//  [Act]     Computes differential-drive wheel velocities via a proportional
//            pursuit controller and transmits commands to robot firmware
//
//  Compile (MSVC):   cl /EHsc /std:c++14 planner.cpp
//  Compile (MinGW):  g++ -std=c++14 planner.cpp -o planner.exe -lws2_32
//
//  Vision Payload IN  (UDP 5005): "ID,X,Y,Theta;ID,X,Y,Theta"  (pixels, degrees)
//  Command Payload OUT(UDP 5000): "ID,V_LEFT,V_RIGHT,ACTUATOR"  (m/s, state int)
//
// ============================================================================

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <array>
#include <limits>
#include <winsock2.h>

// Link the Windows socket library automatically (MSVC)
#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ============================================================================
//  SECTION 1: CONFIGURATION
// ============================================================================
//  All tuneable constants live here.  Hardware-specific values (track width,
//  wheel radius, bounding box) are placeholders — update them once the
//  hardware team provides the real measurements.
// ============================================================================

namespace Config {

    // ---- Network Ports ----
    static const int    VISION_RX_PORT   = 5005;          // Receive vision data
    static const int    COMMAND_TX_PORT  = 5000;          // Transmit wheel commands
    static const int    VIZ_TX_PORT      = 5006;          // Transmit visualizer AR paths
    static const char*  LOCALHOST        = "127.0.0.1";

    // ---- Arena & Metric Calibration ----
    //  The calibration factor converts raw pixel coordinates from the overhead
    //  camera into physical meters.  Measure a known reference distance in the
    //  arena (e.g. two markers 1 m apart) and divide the pixel distance by 1 m.
    //
    //  Example: two markers are 400 px apart and you know they are 1.0 m apart
    //           -> PIXELS_PER_METER = 400.0
    static const float  PIXELS_PER_METER = 400.0f;

    //  Physical arena dimensions (metres).  Used for the occupancy grid bounds
    //  and for flipping the Y-axis so the maths uses a standard right-hand
    //  coordinate frame (Y increases upward).
    static const float  ARENA_WIDTH_M    = 2.0f;
    static const float  ARENA_HEIGHT_M   = 2.0f;

    // ---- Occupancy Grid ----
    //  Cell size controls path resolution.  Smaller cells = smoother paths but
    //  more memory and slower A*.  5 cm is a good balance for a 2 m arena.
    static const float  CELL_SIZE_M      = 0.05f;
    static const int    GRID_COLS        = (int)(ARENA_WIDTH_M  / CELL_SIZE_M); // 40
    static const int    GRID_ROWS        = (int)(ARENA_HEIGHT_M / CELL_SIZE_M); // 40

    // ---- Robot Physical Parameters (PLACEHOLDERS — get from hardware team) ----
    static const float  TRACK_WIDTH_M    = 0.15f;   // Distance between left and right wheels
    static const float  WHEEL_RADIUS_M   = 0.033f;  // Wheel radius
    static const float  ROBOT_RADIUS_M   = 0.07f;   // Effective physical radius (~7 cm)
    static const float  SAFETY_MARGIN_M  = 0.03f;   // Extra clearance (~3 cm)
    static const int    MAX_INFLATION_CELLS = 2;    // Cap inflation cells for tabletop/camera testing

    // ---- Motion Controller Gains ----
    static const float  MAX_LINEAR_VEL   = 0.30f;   // m/s   — top forward speed
    static const float  MAX_ANGULAR_VEL  = 2.00f;   // rad/s — top turning rate
    static const float  HEADING_KP       = 2.50f;   // Proportional gain on heading error
    static const float  WAYPOINT_TOL_M   = 0.08f;   // Waypoint "reached" radius
    static const float  GOAL_TOL_M       = 0.08f;   // Final goal "reached" radius
    static const float  HEADING_SLOW_RAD = 0.40f;   // Slow linear speed when heading error > this

    // ---- State Machine Timing ----
    static const float  ACTUATOR_DWELL_S = 1.5f;    // Seconds to hold PICKING / DROPPING state
    static const float  VISION_TIMEOUT_S = 2.0f;    // Mark entity stale after this long

    // ---- Entity Counts ----
    static const int    MAX_ROBOTS       = 3;
    static const int    MAX_PALLETS      = 3;
    static const int    MAX_DESTINATIONS = 3;

    // ---- Control Loop ----
    static const int    LOOP_RATE_HZ     = 20;
    static const int    LOOP_PERIOD_MS   = 1000 / LOOP_RATE_HZ;
}


// ============================================================================
//  SECTION 2: MATH UTILITIES
// ============================================================================

static float degToRad(float deg) { return deg * (float)M_PI / 180.0f; }
static float radToDeg(float rad) { return rad * 180.0f / (float)M_PI; }

/// Wrap any angle into [-pi, +pi].
static float normalizeAngle(float rad) {
    while (rad >  (float)M_PI) rad -= 2.0f * (float)M_PI;
    while (rad < -(float)M_PI) rad += 2.0f * (float)M_PI;
    return rad;
}

/// Euclidean distance between two 2-D points.
static float dist2D(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}


// ============================================================================
//  SECTION 3: DATA STRUCTURES
// ============================================================================

// ---- 2-D Pose (metric, right-hand frame) ----
struct Pose {
    float x     = 0.0f;   // metres
    float y     = 0.0f;   // metres
    float theta = 0.0f;   // radians, CCW from +X
};

// ---- Entity classification matching the Python vision system ----
enum class EntityType { ROBOT, PALLET, DESTINATION, UNKNOWN };

inline const char* entityTypeName(EntityType t) {
    switch (t) {
        case EntityType::ROBOT:       return "Robot";
        case EntityType::PALLET:      return "Pallet";
        case EntityType::DESTINATION: return "Dest";
        default:                      return "Unknown";
    }
}

struct Entity {
    int         rawId    = -1;           // ArUco marker ID (0-8)
    EntityType  type     = EntityType::UNKNOWN;
    Pose        pose;
    bool        active   = false;        // Currently visible this frame
    double      lastSeen = 0.0;          // Timestamp of last observation (seconds)
};

// ---- Grid Cell for the occupancy map / A* ----
struct GridCell {
    int row = 0;
    int col = 0;

    bool operator==(const GridCell& o) const { return row == o.row && col == o.col; }
    bool operator!=(const GridCell& o) const { return !(*this == o); }
};

// ---- Wheel velocity pair ----
struct WheelVel {
    float vLeft  = 0.0f;
    float vRight = 0.0f;
};


// ============================================================================
//  SECTION 4: METRIC CALIBRATOR
// ============================================================================
//  Converts raw pixel coordinates from the overhead camera into a metric
//  right-hand coordinate system (origin at bottom-left of arena, Y-up).
//
//  The conversion uses a single scalar PIXELS_PER_METER ratio.  For
//  production use, replace this with a full perspective homography derived
//  from four known calibration points at the arena corners.
// ============================================================================

class MetricCalibrator {
public:
    float ppm;              // pixels-per-metre
    float arenaH;           // arena height in metres (for Y-flip)

    MetricCalibrator()
        : ppm(Config::PIXELS_PER_METER), arenaH(Config::ARENA_HEIGHT_M) {}

    /// Convert a vision pixel pose into metric coordinates.
    ///  - X_m  = X_px / ppm
    ///  - Y_m  = arenaH - Y_px / ppm       (flips image-Y to math-Y)
    ///  - theta remains unchanged (vision already outputs standard CCW angle)
    Pose pixelToMetric(float px_x, float px_y, float theta_deg) const {
        Pose p;
        p.x     = px_x / ppm;
        p.y     = arenaH - (px_y / ppm);
        p.theta = degToRad(theta_deg);
        return p;
    }

    /// Map a metric (x, y) position to its occupancy-grid cell.
    GridCell metricToGrid(float xm, float ym) const {
        GridCell c;
        c.col = std::max(0, std::min((int)(xm / Config::CELL_SIZE_M), Config::GRID_COLS - 1));
        c.row = std::max(0, std::min((int)(ym / Config::CELL_SIZE_M), Config::GRID_ROWS - 1));
        return c;
    }

    /// Convert a grid cell back to the metric centre of that cell.
    Pose gridToMetric(const GridCell& c) const {
        Pose p;
        p.x     = (c.col + 0.5f) * Config::CELL_SIZE_M;
        p.y     = (c.row + 0.5f) * Config::CELL_SIZE_M;
        p.theta = 0.0f;
        return p;
    }

    /// Convert metric coordinates back to image pixel coordinates.
    void metricToPixel(float xm, float ym, int& px, int& py) const {
        px = (int)std::round(xm * ppm);
        py = (int)std::round((arenaH - ym) * ppm);
    }
};


// ============================================================================
//  SECTION 5: OCCUPANCY GRID
// ============================================================================
//  A binary 2-D grid where each cell is either FREE (0) or BLOCKED (1).
//  Before each A* query the grid is rebuilt:
//    1. Clear all cells.
//    2. Mark arena boundary cells as blocked.
//    3. Mark cells occupied by obstacles (other robots, non-target pallets).
//    4. Inflate every blocked cell by the robot's effective radius so the
//       pathfinder treats the robot as a point.
// ============================================================================

class OccupancyGrid {
public:
    std::vector<uint8_t> cells;   // 0 = free, 1 = blocked
    int rows, cols;

    OccupancyGrid() : rows(Config::GRID_ROWS), cols(Config::GRID_COLS) {
        cells.resize(rows * cols, 0);
    }

    int  idx(int r, int c)          const { return r * cols + c; }
    bool inBounds(int r, int c)     const { return r >= 0 && r < rows && c >= 0 && c < cols; }
    bool isBlocked(int r, int c)    const { return !inBounds(r, c) || cells[idx(r, c)] != 0; }

    void clear() { std::fill(cells.begin(), cells.end(), (uint8_t)0); }

    /// Mark a single cell as blocked.
    void block(int r, int c) {
        if (inBounds(r, c)) cells[idx(r, c)] = 1;
    }

    /// Mark a single cell as free (unblocked).
    void unblock(int r, int c) {
        if (inBounds(r, c)) cells[idx(r, c)] = 0;
    }

    /// Mark arena boundary (1-cell-wide walls).
    void addBoundaryWalls() {
        for (int c = 0; c < cols; ++c) { block(0, c); block(rows - 1, c); }
        for (int r = 0; r < rows; ++r) { block(r, 0); block(r, cols - 1); }
    }

    /// Mark a circular obstacle at grid cell (cr, cc) with given cell-radius.
    void blockCircle(int cr, int cc, int cellRadius) {
        for (int dr = -cellRadius; dr <= cellRadius; ++dr) {
            for (int dc = -cellRadius; dc <= cellRadius; ++dc) {
                if (dr * dr + dc * dc <= cellRadius * cellRadius) {
                    block(cr + dr, cc + dc);
                }
            }
        }
    }

    /// Unblock a circular area (clears clearance around start and goal).
    void unblockCircle(int cr, int cc, int cellRadius) {
        for (int dr = -cellRadius; dr <= cellRadius; ++dr) {
            for (int dc = -cellRadius; dc <= cellRadius; ++dc) {
                unblock(cr + dr, cc + dc);
            }
        }
    }

    /// Convenience: compute the inflation radius (in cells) from robot dimensions.
    static int robotInflationCells() {
        float totalRadius = Config::ROBOT_RADIUS_M + Config::SAFETY_MARGIN_M;
        int cells = (int)std::ceil(totalRadius / Config::CELL_SIZE_M);
        return std::max(1, std::min(cells, Config::MAX_INFLATION_CELLS));
    }
};


// ============================================================================
//  SECTION 6: A* PATHFINDER
// ============================================================================
//  Standard grid-based A* with 8-connected neighbours (4 cardinal +
//  4 diagonal).  The heuristic is Euclidean distance which is admissible
//  and consistent for 8-connectivity.
//
//  Returns a vector of GridCells from start to goal (inclusive) or an
//  empty vector if no path exists.
// ============================================================================

class AStarPathfinder {
public:
    /// A* node stored in the priority queue.
    struct Node {
        int   index;       // row * cols + col
        float g;           // cost-so-far
        float f;           // g + heuristic
        bool operator>(const Node& o) const { return f > o.f; }
    };

    /// Find a path on the given grid from `start` to `goal`.
    static std::vector<GridCell> findPath(OccupancyGrid& grid,
                                          const GridCell& start,
                                          const GridCell& goal)
    {
        const int R = grid.rows;
        const int C = grid.cols;
        const int N = R * C;

        if (!grid.inBounds(start.row, start.col) || !grid.inBounds(goal.row, goal.col)) {
            return {};
        }

        // Ensure start and goal are always traversable
        grid.unblock(start.row, start.col);
        grid.unblock(goal.row, goal.col);

        if (start.row == goal.row && start.col == goal.col) {
            return { start };
        }

        // Cost and parent arrays
        std::vector<float> gCost(N, std::numeric_limits<float>::max());
        std::vector<int>   parent(N, -1);
        std::vector<bool>  closed(N, false);

        auto cellIdx = [C](int r, int c) { return r * C + c; };

        // Heuristic: Euclidean distance (admissible for 8-connected)
        auto heuristic = [&](int r, int c) -> float {
            float dr = (float)(goal.row - r);
            float dc = (float)(goal.col - c);
            return std::sqrt(dr * dr + dc * dc);
        };

        // Min-heap priority queue
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

        int si = cellIdx(start.row, start.col);
        gCost[si] = 0.0f;
        open.push({si, 0.0f, heuristic(start.row, start.col)});

        // 8-connected neighbour offsets: {dRow, dCol, moveCost}
        //   Cardinal moves cost 1.0,  diagonal moves cost sqrt(2)
        static const float SQRT2 = 1.41421356f;
        static const int DR[] = {-1, -1, -1,  0,  0,  1,  1,  1};
        static const int DC[] = {-1,  0,  1, -1,  1, -1,  0,  1};
        static const float MC[] = {SQRT2, 1, SQRT2, 1, 1, SQRT2, 1, SQRT2};

        while (!open.empty()) {
            Node cur = open.top();
            open.pop();

            if (closed[cur.index]) continue;
            closed[cur.index] = true;

            int cr = cur.index / C;
            int cc = cur.index % C;

            // Goal reached — reconstruct path
            if (cr == goal.row && cc == goal.col) {
                std::vector<GridCell> path;
                int idx = cur.index;
                while (idx != -1) {
                    path.push_back({idx / C, idx % C});
                    idx = parent[idx];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            // Expand neighbours
            for (int d = 0; d < 8; ++d) {
                int nr = cr + DR[d];
                int nc = cc + DC[d];
                if (!grid.inBounds(nr, nc))        continue;
                if (grid.isBlocked(nr, nc))        continue;
                int ni = cellIdx(nr, nc);
                if (closed[ni])                    continue;

                float ng = cur.g + MC[d];
                if (ng < gCost[ni]) {
                    gCost[ni]  = ng;
                    parent[ni] = cur.index;
                    open.push({ni, ng, ng + heuristic(nr, nc)});
                }
            }
        }

        return {};   // No path found
    }

    /// Thin a raw A* cell path down to just the turning-point waypoints.
    ///  Consecutive cells in the same direction are collapsed into a single
    ///  segment, keeping only the endpoints and every change-of-direction.
    static std::vector<GridCell> simplifyPath(const std::vector<GridCell>& raw) {
        if (raw.size() <= 2) return raw;

        std::vector<GridCell> simplified;
        simplified.push_back(raw.front());

        for (size_t i = 1; i + 1 < raw.size(); ++i) {
            int dr1 = raw[i].row - raw[i - 1].row;
            int dc1 = raw[i].col - raw[i - 1].col;
            int dr2 = raw[i + 1].row - raw[i].row;
            int dc2 = raw[i + 1].col - raw[i].col;
            if (dr1 != dr2 || dc1 != dc2) {
                simplified.push_back(raw[i]);    // direction changed — keep this point
            }
        }

        simplified.push_back(raw.back());
        return simplified;
    }
};


// ============================================================================
//  SECTION 7: PROPORTIONAL PURSUIT CONTROLLER
// ============================================================================
//  Given the robot's current pose and a target waypoint the controller:
//    1. Computes the desired heading  angle to the waypoint.
//    2. Computes the heading error    (desired − current), normalised to ±π.
//    3. Sets angular velocity         ω = Kp · error   (clamped).
//    4. Scales linear velocity        V — full speed when well-aligned,
//                                     reduced when the heading error is large
//                                     so the robot turns in place first.
//    5. Converts (V, ω) to differential-drive wheel velocities:
//                  v_R = V + (ω · L) / 2
//                  v_L = V − (ω · L) / 2
// ============================================================================

class PursuitController {
public:
    /// Compute wheel velocities to drive `pose` toward `targetX, targetY`.
    static WheelVel compute(const Pose& pose, float targetX, float targetY) {
        float dx = targetX - pose.x;
        float dy = targetY - pose.y;
        float desiredTheta = std::atan2(dy, dx);
        float headingErr   = normalizeAngle(desiredTheta - pose.theta);

        // Angular velocity: proportional control, clamped
        float omega = Config::HEADING_KP * headingErr;
        omega = std::max(-Config::MAX_ANGULAR_VEL, std::min(omega, Config::MAX_ANGULAR_VEL));

        // Linear velocity: reduce when heading is off so robot pivots first
        float absErr = std::fabs(headingErr);
        float V = Config::MAX_LINEAR_VEL;
        if (absErr > Config::HEADING_SLOW_RAD) {
            // Scale V down linearly; at 90° error V ≈ 0 (pure rotation)
            V *= std::max(0.0f, 1.0f - (absErr / (float)M_PI));
        }

        // Differential-drive inverse kinematics
        //   V = (v_R + v_L) / 2        =>  v_R = V + ωL/2
        //   ω = (v_R - v_L) / L        =>  v_L = V − ωL/2
        float halfLW = Config::TRACK_WIDTH_M / 2.0f;
        WheelVel wv;
        wv.vRight = V + omega * halfLW;
        wv.vLeft  = V - omega * halfLW;
        return wv;
    }
};


// ============================================================================
//  SECTION 8: ROBOT STATE MACHINE  &  TASK MANAGER
// ============================================================================
//  Each robot runs an independent finite state machine:
//
//    IDLE ──► DRIVING_TO_PALLET ──► PICKING_UP ──► CARRYING_TO_DEST
//                                                       │
//                                     IDLE ◄── DROPPING_OFF ◄──┘
//
//  The TaskManager owns the world model (all entities), builds the
//  occupancy grid, assigns idle robots to undelivered pallets, and
//  steps every robot's FSM each control cycle.
// ============================================================================

enum class RobotState {
    IDLE,
    DRIVING_TO_PALLET,
    PICKING_UP,
    CARRYING_TO_DEST,
    DROPPING_OFF
};

static const char* robotStateName(RobotState s) {
    switch (s) {
        case RobotState::IDLE:              return "IDLE";
        case RobotState::DRIVING_TO_PALLET: return "DRIVE->PALLET";
        case RobotState::PICKING_UP:        return "PICKING_UP";
        case RobotState::CARRYING_TO_DEST:  return "DRIVE->DEST";
        case RobotState::DROPPING_OFF:      return "DROPPING_OFF";
        default:                            return "???";
    }
}

/// Per-robot controller state.
struct RobotCtrl {
    RobotState             state       = RobotState::IDLE;
    int                    targetPallet= -1;    // raw marker ID of assigned pallet
    int                    targetDest  = -1;    // raw marker ID of assigned destination
    std::vector<GridCell>  path;                // current A* path (grid cells)
    int                    wpIdx       = 0;     // index of next waypoint in `path`
    double                 stateTimer  = 0.0;   // for timed dwell states
    double                 lastWarnTime= 0.0;   // for rate-limiting no-path warning logs
    WheelVel               cmdVel;              // latest computed command
    int                    actuator    = 0;     // 0=travel, 1=pick, 2=drop
};


// ---- The Task Manager ties everything together ----

class TaskManager {
public:
    // World model — indexed by raw ArUco marker ID (0-8)
    std::array<Entity, 9>     entities;

    // Per-robot controllers — indexed by robot slot (0, 1, 2)
    std::array<RobotCtrl, 3>  robots;

    // Track which pallets have been delivered or are assigned
    std::array<bool, 3>       palletAssigned;    // slot index 0-2 for pallet IDs 3-5
    std::array<bool, 3>       palletDelivered;

    MetricCalibrator calibrator;
    OccupancyGrid    grid;

    TaskManager() {
        palletAssigned.fill(false);
        palletDelivered.fill(false);
    }

    // ------------------------------------------------------------------
    //  Parse a vision UDP packet and update the world model
    // ------------------------------------------------------------------
    void ingestVisionPacket(const std::string& packet, double now) {
        // Mark all entities inactive before this frame
        for (auto& e : entities) e.active = false;

        // Packet format: "ID,X,Y,Theta;ID,X,Y,Theta;..."
        std::istringstream stream(packet);
        std::string token;

        while (std::getline(stream, token, ';')) {
            if (token.empty()) continue;

            int rawId, px, py, thetaDeg;
            char c1, c2, c3;
            std::istringstream ts(token);
            if (!(ts >> rawId >> c1 >> px >> c2 >> py >> c3 >> thetaDeg)) continue;

            if (rawId < 0 || rawId > 8) continue;

            // Classify
            EntityType type;
            if      (rawId <= 2) type = EntityType::ROBOT;
            else if (rawId <= 5) type = EntityType::PALLET;
            else if (rawId <= 8) type = EntityType::DESTINATION;
            else                 type = EntityType::UNKNOWN;

            // Convert pixels -> metric
            entities[rawId].rawId    = rawId;
            entities[rawId].type     = type;
            entities[rawId].pose     = calibrator.pixelToMetric((float)px, (float)py,
                                                                (float)thetaDeg);
            entities[rawId].active   = true;
            entities[rawId].lastSeen = now;
        }
    }

    // ------------------------------------------------------------------
    //  Build the occupancy grid for a specific robot's planning query
    //    - blocks other robots
    //    - blocks non-target pallets
    //    - guarantees start and goal regions are traversable
    // ------------------------------------------------------------------
    void buildGridForRobot(int robotId, int targetPalletId,
                           const GridCell& startCell, const GridCell& goalCell)
    {
        grid.clear();
        grid.addBoundaryWalls();

        int inflationR = OccupancyGrid::robotInflationCells();

        for (int i = 0; i < 9; ++i) {
            if (!entities[i].active && (entities[i].lastSeen == 0.0)) continue;  // never seen
            if (i == robotId) continue;                          // don't block yourself
            if (i == targetPalletId) continue;                   // don't block your target

            bool isObstacle = false;

            if (entities[i].type == EntityType::ROBOT) {
                isObstacle = true;                               // other robots are obstacles
            }
            else if (entities[i].type == EntityType::PALLET) {
                // Pallets that aren't this robot's target are obstacles
                isObstacle = true;
            }
            // Destinations are traversable — the robot needs to drive onto them

            if (isObstacle) {
                GridCell gc = calibrator.metricToGrid(entities[i].pose.x, entities[i].pose.y);
                grid.blockCircle(gc.row, gc.col, inflationR);
            }
        }

        // NOTE: Obstacles are already inflated with blockCircle(..., inflationR).
        // Do NOT call grid.inflate() here, which would double-inflate obstacles.

        // Clear a 1-cell clearance around the robot's current position and target goal
        // so the robot can always navigate, even when markers are clustered closely on a test image.
        grid.unblockCircle(startCell.row, startCell.col, 1);
        grid.unblockCircle(goalCell.row, goalCell.col, 1);
    }

    // ------------------------------------------------------------------
    //  Assign idle robots to undelivered, unassigned pallets
    //    - greedy nearest-first matching
    //    - pallet N (ID 3+N) delivers to destination N (ID 6+N)
    // ------------------------------------------------------------------
    void assignTasks() {
        for (int r = 0; r < Config::MAX_ROBOTS; ++r) {
            if (robots[r].state != RobotState::IDLE) continue;
            int robotId = r;  // ArUco IDs 0-2

            if (!entities[robotId].active) continue;

            // Find nearest unassigned, undelivered pallet
            float bestDist = std::numeric_limits<float>::max();
            int   bestSlot = -1;

            for (int p = 0; p < Config::MAX_PALLETS; ++p) {
                if (palletAssigned[p] || palletDelivered[p]) continue;

                int palletId = 3 + p;
                if (!entities[palletId].active) continue;

                float d = dist2D(entities[robotId].pose.x, entities[robotId].pose.y,
                                 entities[palletId].pose.x, entities[palletId].pose.y);
                if (d < bestDist) {
                    bestDist = d;
                    bestSlot = p;
                }
            }

            if (bestSlot >= 0) {
                palletAssigned[bestSlot] = true;
                robots[r].targetPallet  = 3 + bestSlot;   // ArUco ID
                robots[r].targetDest    = 6 + bestSlot;   // ArUco ID (fixed pairing)
                robots[r].state         = RobotState::DRIVING_TO_PALLET;
                robots[r].wpIdx         = 0;
                robots[r].path.clear();

                std::cout << "[TASK] Robot " << (r + 1)
                          << " assigned -> Pallet " << (bestSlot + 1)
                          << " -> Dest " << (bestSlot + 1) << std::endl;
            }
        }
    }

    // ------------------------------------------------------------------
    //  Step every robot's state machine for one control cycle
    // ------------------------------------------------------------------
    void update(double now) {
        // 1. Try to assign any idle robots
        assignTasks();

        // 2. Step each robot FSM
        for (int r = 0; r < Config::MAX_ROBOTS; ++r) {
            int robotId = r;
            if (!entities[robotId].active) {
                // Robot not visible — send zero velocity as safety stop
                robots[r].cmdVel = {0.0f, 0.0f};
                robots[r].actuator = 0;
                continue;
            }

            const Pose& rpose = entities[robotId].pose;

            switch (robots[r].state) {

            // ........................................................
            case RobotState::IDLE:
                robots[r].cmdVel   = {0.0f, 0.0f};
                robots[r].actuator = 0;
                break;

            // ........................................................
            case RobotState::DRIVING_TO_PALLET:
            {
                int palletId = robots[r].targetPallet;
                if (palletId < 0 || !entities[palletId].active) {
                    robots[r].cmdVel = {0.0f, 0.0f};
                    break;
                }

                // (Re)plan if we have no path yet
                if (robots[r].path.empty()) {
                    GridCell start = calibrator.metricToGrid(rpose.x, rpose.y);
                    GridCell goal  = calibrator.metricToGrid(entities[palletId].pose.x,
                                                            entities[palletId].pose.y);
                    buildGridForRobot(robotId, palletId, start, goal);
                    auto raw = AStarPathfinder::findPath(grid, start, goal);
                    robots[r].path  = AStarPathfinder::simplifyPath(raw);
                    robots[r].wpIdx = 0;

                    if (robots[r].path.empty()) {
                        if (now - robots[r].lastWarnTime >= 2.0) {
                            std::cout << "[WARN] Robot " << (r + 1)
                                      << ": no path to Pallet " << (palletId - 2) << std::endl;
                            robots[r].lastWarnTime = now;
                        }
                        robots[r].cmdVel = {0.0f, 0.0f};
                        break;
                    }
                    std::cout << "[PATH] Robot " << (r + 1) << " path to Pallet "
                              << (palletId - 2) << " (" << robots[r].path.size()
                              << " waypoints)" << std::endl;
                }

                // Follow path
                followPath(r, rpose);

                // Check if close enough to pallet to transition
                if (dist2D(rpose.x, rpose.y,
                           entities[palletId].pose.x,
                           entities[palletId].pose.y) < Config::GOAL_TOL_M)
                {
                    robots[r].state     = RobotState::PICKING_UP;
                    robots[r].stateTimer = now;
                    robots[r].cmdVel    = {0.0f, 0.0f};
                    robots[r].actuator  = 1;   // engage pick-up mechanism
                    robots[r].path.clear();
                    std::cout << "[STATE] Robot " << (r + 1) << " -> PICKING_UP" << std::endl;
                }
                break;
            }

            // ........................................................
            case RobotState::PICKING_UP:
                robots[r].cmdVel   = {0.0f, 0.0f};
                robots[r].actuator = 1;
                if ((now - robots[r].stateTimer) >= Config::ACTUATOR_DWELL_S) {
                    robots[r].state = RobotState::CARRYING_TO_DEST;
                    robots[r].path.clear();
                    std::cout << "[STATE] Robot " << (r + 1) << " -> CARRYING_TO_DEST" << std::endl;
                }
                break;

            // ........................................................
            case RobotState::CARRYING_TO_DEST:
            {
                int destId = robots[r].targetDest;
                if (destId < 0 || !entities[destId].active) {
                    robots[r].cmdVel = {0.0f, 0.0f};
                    break;
                }

                // (Re)plan if needed
                if (robots[r].path.empty()) {
                    GridCell start = calibrator.metricToGrid(rpose.x, rpose.y);
                    GridCell goal  = calibrator.metricToGrid(entities[destId].pose.x,
                                                            entities[destId].pose.y);
                    buildGridForRobot(robotId, -1, start, goal);   // no special target pallet to ignore
                    auto raw = AStarPathfinder::findPath(grid, start, goal);
                    robots[r].path  = AStarPathfinder::simplifyPath(raw);
                    robots[r].wpIdx = 0;

                    if (robots[r].path.empty()) {
                        if (now - robots[r].lastWarnTime >= 2.0) {
                            std::cout << "[WARN] Robot " << (r + 1)
                                      << ": no path to Dest " << (destId - 5) << std::endl;
                            robots[r].lastWarnTime = now;
                        }
                        robots[r].cmdVel = {0.0f, 0.0f};
                        break;
                    }
                    std::cout << "[PATH] Robot " << (r + 1) << " path to Dest "
                              << (destId - 5) << " (" << robots[r].path.size()
                              << " waypoints)" << std::endl;
                }

                robots[r].actuator = 0;   // travel mode while carrying
                followPath(r, rpose);

                // Check if close enough to destination
                if (dist2D(rpose.x, rpose.y,
                           entities[destId].pose.x,
                           entities[destId].pose.y) < Config::GOAL_TOL_M)
                {
                    robots[r].state      = RobotState::DROPPING_OFF;
                    robots[r].stateTimer = now;
                    robots[r].cmdVel     = {0.0f, 0.0f};
                    robots[r].actuator   = 2;   // disengage / drop
                    robots[r].path.clear();
                    std::cout << "[STATE] Robot " << (r + 1) << " -> DROPPING_OFF" << std::endl;
                }
                break;
            }

            // ........................................................
            case RobotState::DROPPING_OFF:
                robots[r].cmdVel   = {0.0f, 0.0f};
                robots[r].actuator = 2;
                if ((now - robots[r].stateTimer) >= Config::ACTUATOR_DWELL_S) {
                    int palletSlot = robots[r].targetPallet - 3;
                    if (palletSlot >= 0 && palletSlot < 3) {
                        palletDelivered[palletSlot] = true;
                    }

                    robots[r].state        = RobotState::IDLE;
                    robots[r].targetPallet = -1;
                    robots[r].targetDest   = -1;
                    robots[r].actuator     = 0;
                    std::cout << "[STATE] Robot " << (r + 1)
                              << " -> IDLE  (delivery complete!)" << std::endl;
                }
                break;
            }
        }
    }

private:
    /// Advance a robot along its pre-computed path using the pursuit controller.
    void followPath(int r, const Pose& rpose) {
        auto& ctrl = robots[r];
        if (ctrl.path.empty() || ctrl.wpIdx >= (int)ctrl.path.size()) {
            ctrl.cmdVel = {0.0f, 0.0f};
            return;
        }

        // Current target waypoint (metric)
        Pose wp = calibrator.gridToMetric(ctrl.path[ctrl.wpIdx]);

        // Advance waypoint if we're close enough
        float dToWp = dist2D(rpose.x, rpose.y, wp.x, wp.y);
        while (dToWp < Config::WAYPOINT_TOL_M &&
               ctrl.wpIdx < (int)ctrl.path.size() - 1)
        {
            ctrl.wpIdx++;
            wp = calibrator.gridToMetric(ctrl.path[ctrl.wpIdx]);
            dToWp = dist2D(rpose.x, rpose.y, wp.x, wp.y);
        }

        // Compute wheel velocities via pursuit controller
        ctrl.cmdVel = PursuitController::compute(rpose, wp.x, wp.y);
    }
};


// ============================================================================
//  SECTION 9: NETWORK MANAGER
// ============================================================================
//  Wraps the two UDP sockets:
//    - rxSock: non-blocking, bound to VISION_RX_PORT, receives vision data
//    - txSock: sends velocity commands to COMMAND_TX_PORT
//
//  Command packet format (text, UTF-8):
//    "ROBOT_ID,V_LEFT,V_RIGHT,ACTUATOR_STATE\n"
//  e.g. "0,0.25,-0.10,0\n"
// ============================================================================

class NetworkManager {
public:
    SOCKET rxSock = INVALID_SOCKET;
    SOCKET txSock = INVALID_SOCKET;
    sockaddr_in txAddr;
    sockaddr_in vizAddr;

    bool init() {
        // ---- Initialise Winsock ----
        WSADATA wd;
        if (WSAStartup(MAKEWORD(2, 2), &wd) != 0) {
            std::cerr << "[NET] WSAStartup failed!" << std::endl;
            return false;
        }

        // ---- RX socket (vision data in) ----
        rxSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (rxSock == INVALID_SOCKET) {
            std::cerr << "[NET] RX socket creation failed!" << std::endl;
            return false;
        }

        sockaddr_in rxAddr;
        rxAddr.sin_family      = AF_INET;
        rxAddr.sin_port        = htons(Config::VISION_RX_PORT);
        rxAddr.sin_addr.s_addr = INADDR_ANY;

        if (bind(rxSock, (SOCKADDR*)&rxAddr, sizeof(rxAddr)) == SOCKET_ERROR) {
            std::cerr << "[NET] RX bind to port " << Config::VISION_RX_PORT << " failed!" << std::endl;
            closesocket(rxSock);
            return false;
        }

        // Set RX socket to non-blocking so the control loop doesn't stall
        u_long mode = 1;
        ioctlsocket(rxSock, FIONBIO, &mode);

        // ---- TX socket (commands out) ----
        txSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (txSock == INVALID_SOCKET) {
            std::cerr << "[NET] TX socket creation failed!" << std::endl;
            return false;
        }

        memset(&txAddr, 0, sizeof(txAddr));
        txAddr.sin_family      = AF_INET;
        txAddr.sin_port        = htons(Config::COMMAND_TX_PORT);
        txAddr.sin_addr.s_addr = inet_addr(Config::LOCALHOST);

        memset(&vizAddr, 0, sizeof(vizAddr));
        vizAddr.sin_family      = AF_INET;
        vizAddr.sin_port        = htons(Config::VIZ_TX_PORT);
        vizAddr.sin_addr.s_addr = inet_addr(Config::LOCALHOST);

        std::cout << "[NET] RX listening on UDP :" << Config::VISION_RX_PORT << std::endl;
        std::cout << "[NET] TX targeting  UDP " << Config::LOCALHOST
                  << ":" << Config::COMMAND_TX_PORT << std::endl;
        std::cout << "[NET] VIZ targeting UDP " << Config::LOCALHOST
                  << ":" << Config::VIZ_TX_PORT << std::endl;
        return true;
    }

    /// Non-blocking receive.  Returns the latest complete packet or "".
    std::string receiveVision() {
        char buf[1024];
        std::string latest;

        // Drain the buffer — keep only the most recent packet
        while (true) {
            int n = recvfrom(rxSock, buf, sizeof(buf) - 1, 0, nullptr, nullptr);
            if (n <= 0) break;     // WSAEWOULDBLOCK or error — buffer empty
            buf[n] = '\0';
            latest = buf;
        }
        return latest;
    }

    /// Send a velocity + actuator command for one robot.
    void sendCommand(int robotId, float vL, float vR, int actuator) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << robotId << "," << vL << "," << vR << "," << actuator;
        std::string pkt = oss.str();
        sendto(txSock, pkt.c_str(), (int)pkt.size(), 0,
               (SOCKADDR*)&txAddr, sizeof(txAddr));
    }

    /// Broadcast AR trajectory waypoints (in pixel coordinates) to the visualizer.
    void sendVisualizerPath(int robotId, const std::string& stateStr,
                            const std::vector<std::pair<int, int>>& pixelPoints)
    {
        std::ostringstream oss;
        oss << "PATH;" << robotId << ";" << stateStr;
        for (const auto& pt : pixelPoints) {
            oss << ";" << pt.first << "," << pt.second;
        }
        std::string pkt = oss.str();
        sendto(txSock, pkt.c_str(), (int)pkt.size(), 0,
               (SOCKADDR*)&vizAddr, sizeof(vizAddr));
    }

    /// Tell the visualizer that a robot has no active path.
    void clearVisualizerPath(int robotId) {
        std::ostringstream oss;
        oss << "CLEAR;" << robotId;
        std::string pkt = oss.str();
        sendto(txSock, pkt.c_str(), (int)pkt.size(), 0,
               (SOCKADDR*)&vizAddr, sizeof(vizAddr));
    }

    void shutdown() {
        if (rxSock != INVALID_SOCKET) closesocket(rxSock);
        if (txSock != INVALID_SOCKET) closesocket(txSock);
        WSACleanup();
    }
};


// ============================================================================
//  SECTION 10: MAIN — THE SENSE-PLAN-ACT LOOP
// ============================================================================

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << "  CENTRALIZED WAREHOUSE PLANNER v2.0" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << std::endl;
    std::cout << "  Arena      : " << Config::ARENA_WIDTH_M << " x "
              << Config::ARENA_HEIGHT_M << " m" << std::endl;
    std::cout << "  Grid       : " << Config::GRID_COLS << " x "
              << Config::GRID_ROWS << " cells  ("
              << Config::CELL_SIZE_M * 100 << " cm / cell)" << std::endl;
    std::cout << "  Calibration: " << Config::PIXELS_PER_METER << " px/m" << std::endl;
    std::cout << "  Loop rate  : " << Config::LOOP_RATE_HZ << " Hz" << std::endl;
    std::cout << std::endl;

    // ---- Init network ----
    NetworkManager net;
    if (!net.init()) return 1;

    // ---- Init task manager ----
    TaskManager mgr;

    std::cout << std::endl << "--- Brain is ONLINE.  Waiting for vision data... ---"
              << std::endl << std::endl;

    // ---- Monotonic clock for timing ----
    auto epoch = std::chrono::steady_clock::now();
    auto timeNow = [&]() -> double {
        auto d = std::chrono::steady_clock::now() - epoch;
        return std::chrono::duration<double>(d).count();
    };

    int  cycleCount     = 0;
    bool firstPacket    = true;

    // =========== MAIN LOOP ============
    while (true) {
        double now = timeNow();

        // -------- SENSE --------
        std::string packet = net.receiveVision();
        if (!packet.empty()) {
            if (firstPacket) {
                std::cout << "[SENSE] First vision packet received!" << std::endl;
                firstPacket = false;
            }
            mgr.ingestVisionPacket(packet, now);
        }

        // -------- PLAN & ACT --------
        mgr.update(now);

        // -------- TRANSMIT --------
        for (int r = 0; r < Config::MAX_ROBOTS; ++r) {
            const auto& ctrl = mgr.robots[r];
            net.sendCommand(r, ctrl.cmdVel.vLeft, ctrl.cmdVel.vRight, ctrl.actuator);

            // Broadcast AR trajectory waypoints for the visualizer
            if (ctrl.path.empty() || !mgr.entities[r].active) {
                net.clearVisualizerPath(r);
            } else {
                std::vector<std::pair<int, int>> pxPoints;
                // Point 0: Robot's current pixel position on the live camera frame
                int rpx, rpy;
                mgr.calibrator.metricToPixel(mgr.entities[r].pose.x, mgr.entities[r].pose.y, rpx, rpy);
                pxPoints.push_back({rpx, rpy});

                // Points 1..N: Path waypoints (from current active waypoint to goal)
                for (size_t i = ctrl.wpIdx; i < ctrl.path.size(); ++i) {
                    Pose wp = mgr.calibrator.gridToMetric(ctrl.path[i]);
                    int wpx, wpy;
                    mgr.calibrator.metricToPixel(wp.x, wp.y, wpx, wpy);
                    pxPoints.push_back({wpx, wpy});
                }
                net.sendVisualizerPath(r, robotStateName(ctrl.state), pxPoints);
            }
        }

        // -------- STATUS PRINT (every ~2 seconds) --------
        if (cycleCount % (Config::LOOP_RATE_HZ * 2) == 0 && !firstPacket) {
            std::cout << "-------- cycle " << cycleCount << " --------" << std::endl;
            for (int r = 0; r < Config::MAX_ROBOTS; ++r) {
                int rid = r;
                const auto& e = mgr.entities[rid];
                const auto& c = mgr.robots[r];
                if (!e.active && e.lastSeen == 0.0) continue;

                std::cout << "  Robot " << (r + 1)
                          << "  state=" << std::setw(15) << std::left
                          << robotStateName(c.state)
                          << "  pos=(" << std::fixed << std::setprecision(2)
                          << e.pose.x << ", " << e.pose.y << ")"
                          << "  hdg=" << std::setw(4) << (int)radToDeg(e.pose.theta) << "deg"
                          << "  cmd=(" << std::setprecision(3)
                          << c.cmdVel.vLeft << ", " << c.cmdVel.vRight << ")"
                          << "  act=" << c.actuator
                          << std::endl;
            }

            // Check if all pallets delivered
            bool allDone = true;
            for (int p = 0; p < Config::MAX_PALLETS; ++p) {
                if (!mgr.palletDelivered[p]) { allDone = false; break; }
            }
            if (allDone) {
                std::cout << std::endl
                          << "*** ALL PALLETS DELIVERED — mission complete! ***"
                          << std::endl;
            }
        }

        cycleCount++;

        // -------- RATE LIMIT --------
        Sleep(Config::LOOP_PERIOD_MS);
    }

    net.shutdown();
    return 0;
}
