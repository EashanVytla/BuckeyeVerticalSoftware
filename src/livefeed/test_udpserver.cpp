// Server side implementation of UDP client-server model 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
// #include <netinet/in.h> 
#include <string>
#include <safequeue.h>
#include <messages.h>
#include <thread>
#include <mutex> 
#include <hex.h>

#define PORT     8080 
#define MAXLINE 1024 
   
// Driver code 
int main() { 
    int sockfd; 
    char buffer[MAXLINE]; 
    const char *hello = "Hello from server"; 
    struct sockaddr_in servaddr, cliaddr; 
       
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
       
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
       
    // Bind the socket with the server address 
    if ( ::bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    socklen_t len;
  int n; 
   
    len = sizeof(cliaddr);  //len is value/result 
   
    while (true) {

        vector<char> buff(MAXLINE);

        n = recvfrom(sockfd, buff.data(), MAXLINE,  
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr, 
                    &len); 

        cout << n << endl;

        cout << Hex::toHex(buff, n) << endl;

        buffer[n] = '\0'; 
        printf("Client : %s\n", buffer); 
        sendto(sockfd, (const void*)buff.data(), n,  
            0, (const struct sockaddr *) &cliaddr, 
                len); 
        std::cout<<"Hello message sent."<<std::endl;  
        
    }
    return 0; 
}