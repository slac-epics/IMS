#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<string.h>
#include<strings.h>
#include<netdb.h>
#include<sys/socket.h>
#include<sys/select.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<ctype.h>

#define MAX 256
#define SA struct sockaddr

void talk(int sockfd)
{
    char buf[MAX];
    int n;
    fd_set rd;
    for (;;) {
        FD_ZERO(&rd);
        FD_SET(0, &rd);
        FD_SET(sockfd, &rd);
        n = select(sockfd+1, &rd, NULL, NULL, NULL);
        if (n < 0) {
            printf("select returned %d?\n", n);
            return;
        }
        if (FD_ISSET(0, &rd)) {
	    int s;
            if (!fgets(buf, sizeof(buf)-1, stdin))
                return;
            n = strlen(buf);
            buf[n-1] = '\r'; /* \n --> \r\n */
	    buf[n]   = '\n';
	    buf[n+1] = 0;
	    printf("> %s", buf);
            write(sockfd, buf, n+1);
        }
        if (FD_ISSET(sockfd, &rd)) {
            n = read(sockfd, buf, sizeof(buf));
            if (n < 0) {
                printf("Read returned %d?\n", n);
                return;
            }
	    buf[n] = 0;
	    printf("< %s", buf);
        }
    }
}

int main(int argc, char **argv)
{
    int sockfd, connfd;
    struct sockaddr_in servaddr, cli;
     
    // socket create and varification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
	printf("socket creation failed...\n");
	exit(0);
    }

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    printf("addr = %s:%d\n", argv[1], atoi(argv[2]));
    servaddr.sin_addr.s_addr = inet_addr(argv[1]);
    servaddr.sin_port = htons(atoi(argv[2]));
     
    if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
	printf("connection with the server failed...\n");
	exit(0);
    }
     
    talk(sockfd);
    close(sockfd);
}
