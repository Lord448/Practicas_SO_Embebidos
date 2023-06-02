#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdlib.h>

int main(int argc, char const *argv[])
{
    int sockfd;
    int len;
    struct sockaddr_in addr;
    char string[10];

    if(argc != 2) {
        printf("Please put the command to send \n");
        return 1;
    }

    return 0;
}
