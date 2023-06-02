#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdlib.h>

#define IP_ADDR "10.42.0.49"

int main(int argc, char const *argv[])
{
    int sockfd;
    int len;
    int result;
    struct sockaddr_in addr;
    char string[20];

    if(argc != 2) {
        printf("Error, Falta el comando a enviar\n");
        return(0);
    }

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(IP_ADDR);
    addr.sin_port = htons(3333);
    len = sizeof(addr);
    result = connect(sockfd, (struct sockaddr *)&addr, len);
    if(result == -1) {
        perror("Connection failed");
        exit(1);
    }

    write(sockfd, argv[1], strlen(argv[1]));
    len = read(sockfd, string, 19);
    if(len < 0) {
        perror("Oops: Client1");
        exit(1);
    }
    string[len] = '\0';
    printf("%s\n", string);
    close(sockfd);
    exit(0);
    return 0;
}
