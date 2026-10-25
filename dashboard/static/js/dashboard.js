/* ============================================================================
   AELINE OPERATIONS CONSOLE — Client-Side Controller
   ============================================================================ */

// ---- Standardized Vector SVG Icons (No Emojis) ----
const SVG_ICONS = {
    robot: `<svg class="icon icon-robot" viewBox="0 0 24 24"><rect x="3" y="11" width="18" height="10" rx="2"/><circle cx="12" cy="5" r="2"/><path d="M12 7v4"/><line x1="8" y1="16" x2="8" y2="16"/><line x1="16" y1="16" x2="16" y2="16"/></svg>`,
    pallet: `<svg class="icon icon-pallet" viewBox="0 0 24 24"><path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/><polyline points="3.27 6.96 12 12.01 20.73 6.96"/><line x1="12" y1="22.08" x2="12" y2="12"/></svg>`,
    dest: `<svg class="icon icon-dest" viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/></svg>`,
    deactivate: `<svg class="icon icon-xs" viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/><line x1="4.93" y1="4.93" x2="19.07" y2="19.07"/></svg>`,
    activate: `<svg class="icon icon-xs" viewBox="0 0 24 24"><polyline points="20 6 9 17 4 12"/></svg>`,
    lifted: `<svg class="icon icon-xs" viewBox="0 0 24 24"><line x1="12" y1="19" x2="12" y2="5"/><polyline points="5 12 12 5 19 12"/></svg>`
};

// System status cache
let currentStatus = {
    planner: {},
    markers: {},
    camera_connected: false,
    planner_connected: false,
    ppm: 1200.0,
    ppm_calibrated: false
};

// Selected inspector target: { type: 'robot'|'pallet'|'dest', id: number }
let inspectedEntity = { type: 'robot', id: 0 };


// ============================================================================
//  REAL-TIME TELEMETRY STREAMING & POLLING
// ============================================================================

function setupEventStream() {
    if (window.EventSource) {
        const source = new EventSource('/api/stream');

        source.onmessage = function (event) {
            try {
                const data = JSON.parse(event.data);
                currentStatus = data;
                updateUI(data);
            } catch (e) {
                console.error('[SSE] Error parsing stream:', e);
            }
        };

        source.onerror = function () {
            fetchStatus();
        };
    }

    // Polling safety net
    setInterval(fetchStatus, 800);
}

async function fetchStatus() {
    try {
        const res = await fetch('/api/status');
        if (res.ok) {
            const data = await res.json();
            currentStatus = data;
            updateUI(data);
        }
    } catch (e) {}
}

async function fetchInitialLogs() {
    try {
        const res = await fetch('/api/logs');
        if (res.ok) {
            const logs = await res.json();
            const logContainer = document.getElementById('activity-log');
            logContainer.innerHTML = '';
            logs.reverse().forEach(entry => appendLogEntry(entry));
        }
    } catch (e) {}
}


// ============================================================================
//  UI REFRESH CONTROLLER
// ============================================================================

function updateUI(data) {
    // 1. Connection status pills
    updateStatusPill('camera-status', data.camera_connected);
    updateStatusPill('planner-status', data.planner_connected);

    // 2. Telemetry HUD indicators
    const ppmEl = document.getElementById('ppm-value');
    if (ppmEl) {
        ppmEl.textContent = data.ppm_calibrated ? Math.round(data.ppm) : 'Locked';
    }

    const cycleEl = document.getElementById('cycle-value');
    if (cycleEl && data.planner && data.planner.cycle !== undefined) {
        cycleEl.textContent = data.planner.cycle;
    }

    // 3. Robot fleet cards
    if (data.planner && data.planner.robots) {
        renderRobotCards(data.planner.robots, data.markers);
    }

    // 4. Warehouse pallet cards
    if (data.planner && data.planner.pallets) {
        renderPalletCards(data.planner.pallets, data.markers);
    }

    // 5. Destination zone cards
    if (data.planner && data.planner.destinations) {
        renderDestCards(data.planner.destinations, data.markers);
    }

    // 6. Component inspector
    updateInspector(data);
}

function updateStatusPill(elementId, connected) {
    const el = document.getElementById(elementId);
    if (!el) return;
    if (connected) {
        el.classList.add('connected');
    } else {
        el.classList.remove('connected');
    }
}


// ============================================================================
//  BENTO CARD GENERATION
// ============================================================================

function renderRobotCards(robots, markers) {
    const container = document.getElementById('robot-cards');
    if (!container) return;

    let html = '';
    robots.forEach(robot => {
        const stateStr = robot.state || 'IDLE';
        const stateClass = getRobotStateClass(stateStr);
        const isActive = robot.active !== false;
        const deactivatedClass = isActive ? '' : 'deactivated';
        const inspectedClass = (inspectedEntity && inspectedEntity.type === 'robot' && inspectedEntity.id === robot.id) ? 'inspected' : '';

        // Live coordinate readout
        const marker = markers ? markers[String(robot.id)] : null;
        let coordStr = '';
        if (marker) {
            coordStr = `Live (${marker.px}, ${marker.py}) | ${marker.heading}°`;
        } else if (robot.x !== undefined) {
            coordStr = `Pose (${robot.x.toFixed(2)}m, ${robot.y.toFixed(2)}m)`;
        } else {
            coordStr = 'Awaiting optical lock...';
        }

        const taskStr = (robot.target_pallet >= 0 && stateStr !== 'IDLE')
            ? `Target: Pallet ${robot.target_pallet - 2} → Dest ${robot.target_dest - 5}`
            : (robot.manual ? 'Manual Task Set' : 'Standby / Available');

        html += `
        <div class="entity-bento-card ${deactivatedClass} ${inspectedClass}"
             onclick="inspectEntity('robot', ${robot.id})"
             onmouseenter="inspectEntity('robot', ${robot.id})">
            <div class="card-top-row">
                <div class="card-identity">
                    <span class="card-title">${SVG_ICONS.robot} Robot ${robot.id + 1}</span>
                </div>
                <span class="card-status-pill ${stateClass}">${stateStr}</span>
            </div>
            <div class="card-coordinates">${coordStr}</div>
            <div class="card-task-detail">${taskStr}</div>
            <div class="card-action-row">
                ${isActive
                    ? `<button class="card-pill-btn btn-deact" onclick="event.stopPropagation(); sendDeactivate(${robot.id})">${SVG_ICONS.deactivate} Deactivate</button>`
                    : `<button class="card-pill-btn btn-act" onclick="event.stopPropagation(); sendActivate(${robot.id})">${SVG_ICONS.activate} Activate</button>`
                }
                ${(stateStr.includes('PICK') || stateStr.includes('PALLET'))
                    ? `<button class="card-pill-btn btn-lift" onclick="event.stopPropagation(); sendLifted(${robot.id})">${SVG_ICONS.lifted} Lifted</button>`
                    : ''
                }
            </div>
        </div>`;
    });

    container.innerHTML = html;
}

function renderPalletCards(pallets, markers) {
    const container = document.getElementById('pallet-cards');
    if (!container) return;

    let html = '';
    pallets.forEach(pallet => {
        const isActive = pallet.active !== false;
        const deactivatedClass = isActive ? '' : 'deactivated';
        const inspectedClass = (inspectedEntity && inspectedEntity.type === 'pallet' && inspectedEntity.id === pallet.id) ? 'inspected' : '';

        const marker = markers ? markers[String(pallet.id)] : null;
        let coordStr = marker
            ? `Live (${marker.px}, ${marker.py}) px`
            : (pallet.x !== undefined ? `Pose (${pallet.x.toFixed(2)}m, ${pallet.y.toFixed(2)}m)` : 'Awaiting vision...');

        const statusLabel = pallet.delivered ? 'DELIVERED' : (pallet.assigned ? 'ASSIGNED' : 'AVAILABLE');
        const statusClass = pallet.delivered ? 'status-delivered' : (pallet.assigned ? 'status-driving' : 'status-active');

        html += `
        <div class="entity-bento-card ${deactivatedClass} ${inspectedClass}"
             onclick="inspectEntity('pallet', ${pallet.id})"
             onmouseenter="inspectEntity('pallet', ${pallet.id})">
            <div class="card-top-row">
                <div class="card-identity">
                    <span class="card-title">${SVG_ICONS.pallet} Pallet ${pallet.id - 2}</span>
                </div>
                <span class="card-status-pill ${statusClass}">${statusLabel}</span>
            </div>
            <div class="card-coordinates">${coordStr}</div>
            <div class="card-task-detail">Slot: ID ${pallet.id}</div>
            <div class="card-action-row">
                ${isActive
                    ? `<button class="card-pill-btn btn-deact" onclick="event.stopPropagation(); sendDeactivate(${pallet.id})">${SVG_ICONS.deactivate} Deactivate</button>`
                    : `<button class="card-pill-btn btn-act" onclick="event.stopPropagation(); sendActivate(${pallet.id})">${SVG_ICONS.activate} Activate</button>`
                }
            </div>
        </div>`;
    });

    container.innerHTML = html;
}

function renderDestCards(destinations, markers) {
    const container = document.getElementById('dest-cards');
    if (!container) return;

    let html = '';
    destinations.forEach(dest => {
        const isActive = dest.active !== false;
        const deactivatedClass = isActive ? '' : 'deactivated';
        const inspectedClass = (inspectedEntity && inspectedEntity.type === 'dest' && inspectedEntity.id === dest.id) ? 'inspected' : '';

        const marker = markers ? markers[String(dest.id)] : null;
        let coordStr = marker
            ? `Live (${marker.px}, ${marker.py}) px`
            : (dest.x !== undefined ? `Pose (${dest.x.toFixed(2)}m, ${dest.y.toFixed(2)}m)` : 'Awaiting vision...');

        html += `
        <div class="entity-bento-card ${deactivatedClass} ${inspectedClass}"
             onclick="inspectEntity('dest', ${dest.id})"
             onmouseenter="inspectEntity('dest', ${dest.id})">
            <div class="card-top-row">
                <div class="card-identity">
                    <span class="card-title">${SVG_ICONS.dest} Dest ${dest.id - 5}</span>
                </div>
                <span class="card-status-pill ${isActive ? 'status-active' : 'status-deactivated'}">${isActive ? 'ACTIVE' : 'OFFLINE'}</span>
            </div>
            <div class="card-coordinates">${coordStr}</div>
            <div class="card-task-detail">Drop-Off Zone: ID ${dest.id}</div>
            <div class="card-action-row">
                ${isActive
                    ? `<button class="card-pill-btn btn-deact" onclick="event.stopPropagation(); sendDeactivate(${dest.id})">${SVG_ICONS.deactivate} Deactivate</button>`
                    : `<button class="card-pill-btn btn-act" onclick="event.stopPropagation(); sendActivate(${dest.id})">${SVG_ICONS.activate} Activate</button>`
                }
            </div>
        </div>`;
    });

    container.innerHTML = html;
}

function getRobotStateClass(state) {
    if (!state) return 'status-idle';
    const s = state.toUpperCase();
    if (s.includes('IDLE')) return 'status-idle';
    if (s.includes('PALLET') || s.includes('DRIVE')) return 'status-driving';
    if (s.includes('PICK')) return 'status-picking';
    if (s.includes('CARRY') || s.includes('DEST')) return 'status-carrying';
    return 'status-idle';
}


// ============================================================================
//  ENTITY INSPECTOR (AELINE BENTO TELEMETRY TILES)
// ============================================================================

function inspectEntity(type, id) {
    inspectedEntity = { type, id };
    updateInspector(currentStatus);
}

function updateInspector(data) {
    const inspector = document.getElementById('entity-inspector');
    if (!inspector || !inspectedEntity) return;

    let titleHtml = '';
    let isActive = true;
    let tiles = [];

    if (inspectedEntity.type === 'robot' && data.planner && data.planner.robots) {
        const r = data.planner.robots.find(item => item.id === inspectedEntity.id);
        const m = data.markers ? data.markers[String(inspectedEntity.id)] : null;
        if (r) {
            isActive = r.active !== false;
            titleHtml = `
                <div class="inspector-header">
                    <div class="inspector-title">
                        ${SVG_ICONS.robot} Robot ${r.id + 1} <span style="font-size:12px; font-weight:600; color:var(--text-muted);">(Marker ID ${r.id})</span>
                    </div>
                    <span class="inspector-status-badge ${isActive ? '' : 'inactive'}">
                        ${isActive ? 'Active in Fleet' : 'Deactivated'}
                    </span>
                </div>
            `;
            tiles = [
                { label: 'Metric Coordinates', value: `${(r.x || 0).toFixed(3)}m, ${(r.y || 0).toFixed(3)}m`, mono: true },
                { label: 'Heading Orientation', value: `${r.heading !== undefined ? r.heading : (m ? m.heading : '--')}°`, mono: false },
                { label: 'Camera Sensor Pixels', value: `(${m ? m.px : (r.px || '--')}, ${m ? m.py : (r.py || '--')}) px`, mono: true },
                { label: 'Drive Speeds (vL / vR)', value: `${(r.vL || 0).toFixed(3)} / ${(r.vR || 0).toFixed(3)} m/s`, mono: true },
                { label: 'Mission State', value: r.state || 'IDLE', mono: false },
                { label: 'Assigned Target', value: r.target_pallet >= 0 ? `Pallet ${r.target_pallet - 2} → Dest ${r.target_dest - 5}` : 'None', mono: false }
            ];
        }
    } else if (inspectedEntity.type === 'pallet' && data.planner && data.planner.pallets) {
        const p = data.planner.pallets.find(item => item.id === inspectedEntity.id);
        const m = data.markers ? data.markers[String(inspectedEntity.id)] : null;
        if (p) {
            isActive = p.active !== false;
            titleHtml = `
                <div class="inspector-header">
                    <div class="inspector-title">
                        ${SVG_ICONS.pallet} Pallet ${p.id - 2} <span style="font-size:12px; font-weight:600; color:var(--text-muted);">(Marker ID ${p.id})</span>
                    </div>
                    <span class="inspector-status-badge ${isActive ? '' : 'inactive'}">
                        ${isActive ? 'Active Station' : 'Deactivated'}
                    </span>
                </div>
            `;
            tiles = [
                { label: 'Metric Coordinates', value: `${(p.x || 0).toFixed(3)}m, ${(p.y || 0).toFixed(3)}m`, mono: true },
                { label: 'Camera Sensor Pixels', value: `(${m ? m.px : (p.px || '--')}, ${m ? m.py : (p.py || '--')}) px`, mono: true },
                { label: 'Assignment Status', value: p.assigned ? 'Assigned to Robot' : 'Available', mono: false },
                { label: 'Delivery State', value: p.delivered ? 'Completed' : 'Pending', mono: false }
            ];
        }
    } else if (inspectedEntity.type === 'dest' && data.planner && data.planner.destinations) {
        const d = data.planner.destinations.find(item => item.id === inspectedEntity.id);
        const m = data.markers ? data.markers[String(inspectedEntity.id)] : null;
        if (d) {
            isActive = d.active !== false;
            titleHtml = `
                <div class="inspector-header">
                    <div class="inspector-title">
                        ${SVG_ICONS.dest} Destination ${d.id - 5} <span style="font-size:12px; font-weight:600; color:var(--text-muted);">(Marker ID ${d.id})</span>
                    </div>
                    <span class="inspector-status-badge ${isActive ? '' : 'inactive'}">
                        ${isActive ? 'Active Target Zone' : 'Deactivated'}
                    </span>
                </div>
            `;
            tiles = [
                { label: 'Metric Coordinates', value: `${(d.x || 0).toFixed(3)}m, ${(d.y || 0).toFixed(3)}m`, mono: true },
                { label: 'Camera Sensor Pixels', value: `(${m ? m.px : (d.px || '--')}, ${m ? m.py : (d.py || '--')}) px`, mono: true },
                { label: 'Station Status', value: isActive ? 'Ready for Drop-off' : 'Offline', mono: false },
                { label: 'Zone Index', value: `Zone ${d.id - 5}`, mono: false }
            ];
        }
    }

    if (tiles.length === 0) {
        inspector.innerHTML = '<p class="inspector-hint">Hover or click any component to inspect live telemetry</p>';
        return;
    }

    const tilesHtml = tiles.map(t => `
        <div class="inspector-tile">
            <span class="tile-label">${t.label}</span>
            <span class="tile-value ${t.mono ? 'mono' : ''}">${t.value}</span>
        </div>
    `).join('');

    inspector.innerHTML = `
        ${titleHtml}
        <div class="inspector-bento-grid">
            ${tilesHtml}
        </div>
    `;
}


// ============================================================================
//  ACTIONS & COMMANDS
// ============================================================================

async function sendAssign() {
    const robotId = parseInt(document.getElementById('assign-robot').value);
    const palletId = parseInt(document.getElementById('assign-pallet').value);
    const destId = parseInt(document.getElementById('assign-dest').value);

    try {
        const res = await fetch('/api/assign', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ robot_id: robotId, pallet_id: palletId, dest_id: destId })
        });
        const data = await res.json();
        if (data.log) prependLogEntry(data.log);
        fetchStatus();
    } catch (e) {
        console.error('Assignment error:', e);
    }
}

async function sendEstop() {
    if (!confirm('CONFIRM EMERGENCY STOP?\nThis will immediately halt ALL robots and zero wheel velocities.')) {
        return;
    }
    try {
        const res = await fetch('/api/estop', { method: 'POST' });
        const data = await res.json();
        if (data.log) prependLogEntry(data.log);
        fetchStatus();
    } catch (e) {}
}

async function sendPause() {
    try {
        const res = await fetch('/api/pause', { method: 'POST' });
        const data = await res.json();
        if (data.log) prependLogEntry(data.log);
        fetchStatus();
    } catch (e) {}
}

async function sendResume() {
    try {
        const res = await fetch('/api/resume', { method: 'POST' });
        const data = await res.json();
        if (data.log) prependLogEntry(data.log);
        fetchStatus();
    } catch (e) {}
}

async function sendReset() {
    if (!confirm('CONFIRM SYSTEM RESET?\nThis will clear all active tasks and reset robots to IDLE.')) {
        return;
    }
    try {
        const res = await fetch('/api/reset', { method: 'POST' });
        const data = await res.json();
        if (data.log) prependLogEntry(data.log);
        fetchStatus();
    } catch (e) {}
}

async function sendLifted(target) {
    try {
        const res = await fetch('/api/lifted', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ target: String(target) })
        });
        const data = await res.json();
        if (data.log) prependLogEntry(data.log);
        fetchStatus();
    } catch (e) {}
}

async function sendDeactivate(entityId) {
    try {
        const res = await fetch('/api/deactivate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ entity_id: entityId })
        });
        const data = await res.json();
        if (data.log) prependLogEntry(data.log);
        fetchStatus();
    } catch (e) {}
}

async function sendActivate(entityId) {
    try {
        const res = await fetch('/api/activate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ entity_id: entityId })
        });
        const data = await res.json();
        if (data.log) prependLogEntry(data.log);
        fetchStatus();
    } catch (e) {}
}


// ============================================================================
//  ACTIVITY LOGGING
// ============================================================================

function prependLogEntry(entry) {
    const container = document.getElementById('activity-log');
    if (!container) return;
    const el = createLogElement(entry);
    container.prepend(el);

    while (container.children.length > 200) {
        container.removeChild(container.lastChild);
    }
}

function appendLogEntry(entry) {
    const container = document.getElementById('activity-log');
    if (!container) return;
    container.appendChild(createLogElement(entry));
}

function createLogElement(entry) {
    const div = document.createElement('div');
    div.className = 'log-item';
    div.innerHTML = `
        <span class="log-time">${entry.time || ''}</span>
        <span class="log-chip chip-${entry.level || 'INFO'}">${entry.level || 'INFO'}</span>
        <span class="log-text">${entry.message || ''}</span>
    `;
    return div;
}

function clearLog() {
    const container = document.getElementById('activity-log');
    if (container) container.innerHTML = '';
}


// ============================================================================
//  THEME CONTROLLER (LIGHT / DARK AELINE SYSTEM)
// ============================================================================

function initTheme() {
    try {
        const savedTheme = localStorage.getItem('aeline_theme');
        if (savedTheme) {
            setTheme(savedTheme, false);
        } else {
            const prefersDark = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;
            setTheme(prefersDark ? 'dark' : 'light', false);
        }
    } catch (e) {
        setTheme('light', false);
    }

    if (window.matchMedia) {
        window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', e => {
            if (!localStorage.getItem('aeline_theme')) {
                setTheme(e.matches ? 'dark' : 'light', false);
            }
        });
    }
}

function toggleTheme() {
    const current = document.documentElement.getAttribute('data-theme') || 'light';
    const nextTheme = current === 'dark' ? 'light' : 'dark';
    setTheme(nextTheme, true);
}

function setTheme(theme, save = true) {
    if (theme === 'dark') {
        document.documentElement.setAttribute('data-theme', 'dark');
    } else {
        document.documentElement.setAttribute('data-theme', 'light');
    }

    if (save) {
        try {
            localStorage.setItem('aeline_theme', theme);
        } catch (e) { }
    }
}


// ============================================================================
//  BOOTSTRAP
// ============================================================================

function generatePlaceholders() {
    renderRobotCards([
        { id: 0, state: 'STANDBY', active: true },
        { id: 1, state: 'STANDBY', active: true },
        { id: 2, state: 'STANDBY', active: true }
    ], {});

    renderPalletCards([
        { id: 3, active: true, assigned: false, delivered: false },
        { id: 4, active: true, assigned: false, delivered: false },
        { id: 5, active: true, assigned: false, delivered: false }
    ], {});

    renderDestCards([
        { id: 6, active: true },
        { id: 7, active: true },
        { id: 8, active: true }
    ], {});
}

initTheme();
generatePlaceholders();
fetchInitialLogs();
setupEventStream();
