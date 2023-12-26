#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fstream>
using namespace std;
int main() {
    int temp=0;;
    int clientSocket;
    struct sockaddr_in serverAddr;
    char sendBuffer[1024];
    char recvBuffer[1024];

    // Create socket
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12344); // Server port

    // Manually specify the server's IP address as a 32-bit integer
    serverAddr.sin_addr.s_addr = 0x0100007F; // 127.0.0.1 in hexadecimal

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error connecting to the server" << std::endl;
        close(clientSocket);
        return 1;
    }

    while (true) {
        // Send data to the server
        
        // std::cout << " \n*To Exit Client press         0 *";
        // std::cout << " \n*To Upload Mission file press MISSION *";
        std::cout << "\nEnter a Command :  ";
        // std::cin>>temp;
       
        
        std::cin.getline(sendBuffer, sizeof(sendBuffer));
        send(clientSocket, sendBuffer, strlen(sendBuffer), 0);
       
        //std::cout << "\to send from server: " << sendBuffer << std::endl;
        if(strcmp(sendBuffer, "Mission") == 0){
            std::ifstream file("a.txt", std::ios::binary);
            if (!file) {
                perror("File not found");
                return 1;
            }

            char buffer[1024];
            while (!file.eof()) {
                file.read(buffer, sizeof(buffer));
                send(clientSocket, buffer, file.gcount(), 0);
            }
            // send(clientSocket, "NULL", file.gcount(), 0);
            file.close();
            //close(clientSocket);
        }

        

        if (strcmp(sendBuffer, "exit") == 0) {
            break; // Exit the loop if the user enters 'exit'
        }

        // Receive acknowledgment from the server
        int bytesRead = recv(clientSocket, recvBuffer, sizeof(recvBuffer), 0);
        if (bytesRead <= 0) {
            std::cerr << "Connection closed by the server." << std::endl;
            break;
        }
        recvBuffer[bytesRead] = '\0'; // Null-terminate the received data
        std::cout << "Received from server: " << recvBuffer << std::endl;
        if (strcmp(recvBuffer, "Enter altitude") == 0){
            std::cout << "\nEnter ALTITUDE in Meters: ";
            std::cin.getline(sendBuffer, sizeof(sendBuffer));
            send(clientSocket, sendBuffer, strlen(sendBuffer), 0);

        }
    }

    // Close the socket
  
    close(clientSocket);

    return 0;
}
