#include <stdio.h>
#include <sys/types.h>	       /* See NOTES */
#include <sys/socket.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

#define N 64

int main(int argc, const char * argv[]) {
	int sockfd;
	char buf[N] = {0};
	struct sockaddr_in peeraddr;

	if (argc != 3 ) {
		printf("usage:%s broadcastip port\n", argv[0]);
		return 0;
	}

	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		perror("socket");
		return -1;
	}

	int on = 1;
	if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) == -1) {
		perror("setsockopt");
		return -1;
	}

	memset(&peeraddr, 0, sizeof(peeraddr));
	peeraddr.sin_family = AF_INET;
	peeraddr.sin_port = htons(atoi(argv[2]));
	peeraddr.sin_addr.s_addr = inet_addr(argv[1]);

std::string mycmd;
	while(1){
        std::cout<<"-------------------------------------------------"<<std::endl;
        std::cout<<"Choose the number : "<<std::endl;
        std::cout<<"[0] update marker"<<std::endl;
        std::cout<<"[1] Z"<<std::endl;
        std::cout<<"[2] -Z"<<std::endl;
        std::cout<<"[3] X"<<std::endl;
        std::cout<<"[4] -X"<<std::endl;
        std::cout<<"[5] Y"<<std::endl;
        std::cout<<"[6] -Y"<<std::endl;
	std::cout<<"[7] Roll"<<std::endl;
	std::cout<<"[8] -Roll"<<std::endl;
	std::cout<<"[9] Pitch"<<std::endl;
	std::cout<<"[10] -Pitch"<<std::endl;
	std::cout<<"[11] Yaw"<<std::endl;
	std::cout<<"[12] -Yaw"<<std::endl;
	std::cout<<"[13] Step2"<<std::endl;
	std::cout<<"[14] Step5"<<std::endl;
	std::cout<<"[15] Step20"<<std::endl;
        std::cout<<"[16] gripper"<<std::endl;       
        std::cout<<"-------------------------------------------------"<<std::endl;
        std::getline(std::cin,mycmd);
        if(!mycmd.compare("0")){
	    strcpy(buf,"update marker\n");
	}
        else if(!mycmd.compare("1")){
	    strcpy(buf,"Z\n");
	}
        else if(!mycmd.compare("2")){
	    strcpy(buf,"-Z\n");
	}
        else if(!mycmd.compare("3")){
	    strcpy(buf, "X\n");
	}
        else if(!mycmd.compare("4")){
	    strcpy(buf,"-X\n");
	}
        else if(!mycmd.compare("5")){
	    strcpy(buf, "Y\n");
	}
        else if(!mycmd.compare("6")){
	    strcpy(buf, "-Y\n");
	}
        else if(!mycmd.compare("7")){
	    strcpy(buf, "Roll\n");
	}
        else if(!mycmd.compare("8")){
	    strcpy(buf, "-Roll\n");
	}
        else if(!mycmd.compare("9")){
	    strcpy(buf,"Pitch\n");
	}
        else if(!mycmd.compare("10")){
	    strcpy(buf,"-Pitch\n");
	}
        else if(!mycmd.compare("11")){
	    strcpy(buf,"Yaw\n");
	}
        else if(!mycmd.compare("12")){
	    strcpy(buf,"-Yaw\n");
	}
	else if(!mycmd.compare("13")){
	    strcpy(buf,"Step2\n");
	}
	else if(!mycmd.compare("14")){
	    strcpy(buf,"Step5\n");
	}
        else if(!mycmd.compare("15")){
	    strcpy(buf,"Step20\n");
	}
        else if(!mycmd.compare("16")){
	    strcpy(buf,"gripper\n");
	}
        else {
	    strcpy(buf,mycmd.c_str());
	}

        sendto(sockfd, buf, strlen(buf)+1, 0, (struct sockaddr *)&peeraddr, sizeof(peeraddr));
}
	close(sockfd);

	return 0;
}

