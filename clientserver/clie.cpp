#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include<stdio.h>
using namespace std;
int main() {
    int clientSocket;
    struct sockaddr_in serverAddr;
    char buffer[1024];

    // Create socket
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12347); // Server port

    // Convert IP address from string to binary form
    if (inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr) <= 0) {
        std::cerr << "Error converting IP address" << std::endl;
        close(clientSocket);
        return 1;
    }

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error connecting to the server" << std::endl;
        close(clientSocket);
        return 1;
    }

    // Send a message to the server
    //char temp[200];
    //const char* message ;

    // string x;
    // cout<<"enter message\n";
    // cin>>x;
    // const char* message;
    // cin.getline(message);
    char input[100]; // Create a character array to store user input

    std::cout << "Enter a command: ";
    std::cin.getline(input, sizeof(input)); // Read user input
    const char* message= input;
   //const char* message= x.c_str();
   //const char* message 
    
    //scanf("%s", &temp);
    //const char* message = temp;
    send(clientSocket, message, strlen(message), 0);

    // Receive a response from the server
    recv(clientSocket, buffer, sizeof(buffer), 0);
    std::cout << "Received from server: " << buffer << std::endl;

    // Close the socket
    close(clientSocket);

    return 0;
}
