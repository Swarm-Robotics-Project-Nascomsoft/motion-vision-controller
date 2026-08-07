#include <iostream>
#include <string>
#include <winsock2.h> // Windows network library

// Tell the compiler to link the Windows socket library
#pragma comment(lib, "ws2_32.lib") 

#define PORT 5005
#define BUFFER_SIZE 1024

int main() {
    // 1. Initialize Windows Sockets
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Winsock initialization failed!" << std::endl;
        return 1;
    }

    // 2. Create the UDP Socket
    SOCKET recvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (recvSocket == INVALID_SOCKET) {
        std::cerr << "Socket creation failed!" << std::endl;
        WSACleanup();
        return 1;
    }

    // 3. Bind the socket to port 5005
    sockaddr_in recvAddr;
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = htons(PORT);
    recvAddr.sin_addr.s_addr = INADDR_ANY; // Listen on all local interfaces

    if (bind(recvSocket, (SOCKADDR*)&recvAddr, sizeof(recvAddr)) == SOCKET_ERROR) {
        std::cerr << "Bind failed!" << std::endl;
        closesocket(recvSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "--- C++ Brain is listening on UDP Port " << PORT << " ---" << std::endl;

    // 4. Listen continuously
    char buffer[BUFFER_SIZE];
    sockaddr_in senderAddr;
    int senderAddrSize = sizeof(senderAddr);

    while (true) {
        // Clear the buffer
        memset(buffer, 0, BUFFER_SIZE); 

        // Wait to receive a packet from Python
        int bytesReceived = recvfrom(recvSocket, buffer, BUFFER_SIZE - 1, 0, 
                                     (SOCKADDR*)&senderAddr, &senderAddrSize);

        if (bytesReceived > 0) {
            // Print exactly what Python sent us!
            std::cout << "Vision Data Received: " << buffer << std::endl;
        }
    }

    // Clean up (This won't be reached due to the infinite loop, but is good practice)
    closesocket(recvSocket);
    WSACleanup();
    return 0;
}