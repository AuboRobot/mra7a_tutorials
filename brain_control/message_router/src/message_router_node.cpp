#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
/*UDP broadcast*/
#include <sys/socket.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#define N 64

#include <sensor_msgs/Joy.h>
using namespace std;

sensor_msgs::Joy joy_msg;

typedef std::uint64_t hash_t;
constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;
hash_t hash_(char const* str)
{
    hash_t ret{basis};
    while(*str){
        ret ^= *str;
        ret *= prime;
        str++;
    }
    return ret;
}

constexpr hash_t hash_compile_time(char const* str, hash_t last_value = basis)
{
    return *str ? hash_compile_time(str+1, (*str ^ last_value) * prime) : last_value;
}

void switch_msg_to_joy(const char *str)
{
    switch(hash_(str)){
    //if use fgets(buf, N, stdin) to send, the buf will have \n, however, std::getline(std::cin,mycmd); don't have.
//    case hash_compile_time("Z\n"):
//        joy_msg.buttons[8] = 1;
//        break;
    case hash_compile_time("Z"):
        joy_msg.buttons[8] = 1;
        break;
    case hash_compile_time("-Z"):
        joy_msg.buttons[9] = 1;
        break;
    case hash_compile_time("X"):
        joy_msg.axes[1] = 2.5;
        break;
    case hash_compile_time("-X"):
        joy_msg.axes[1] = -2.5;
        break;
    case hash_compile_time("Y"):
        joy_msg.axes[0] = 2.5;
        break;
    case hash_compile_time("-Y"):
        joy_msg.axes[0] = -2.5;
        break;
    case hash_compile_time("Roll"):
        joy_msg.buttons[7] = 1;
        break;
    case hash_compile_time("-Roll"):
        joy_msg.buttons[5] = 1;
        break;
    case hash_compile_time("Pitch"):
        joy_msg.buttons[4] = 1;
        break;
    case hash_compile_time("-Pitch"):
        joy_msg.buttons[6] = 1;
        break;
    case hash_compile_time("Yaw"):
        joy_msg.buttons[10] = 1;
        break;
    case hash_compile_time("-Yaw"):
        joy_msg.buttons[11] = 1;
        break;
    case hash_compile_time("Step5"):
        joy_msg.buttons[12] = 1;
        break;
    case hash_compile_time("Step20"):
        joy_msg.buttons[14] = 1;
        break;
    case hash_compile_time("update marker"):
        joy_msg.buttons[3] = 1;
        break;
    case hash_compile_time("gripper"):
        joy_msg.buttons[16] = 1;
        break;
    default:
        cout << "switch_msg_to_joy: no msg match : " << str << endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_router_node");
    ros::NodeHandle n_;

    /**
     * Firstly, lookup the bradcast_ip and broadcast_port in the rosparameter server.
     */
    std::string broadcast_ip;
    std::string broadcast_port;
    sleep(1);
    if(!ros::param::get("/message_router/broadcast_ip",broadcast_ip) || !ros::param::get("/message_router/broadcast_port",broadcast_port)){
        if (argc != 3 ) {
            printf("usage:%s broadcast_ip port\n", argv[0]);
            return 0;
        }
        broadcast_ip = argv[1];
        broadcast_port = argv[2];
    }

    joy_msg.axes.resize(27);
    joy_msg.buttons.resize(19);

    ros::Publisher pub_joy = n_.advertise<sensor_msgs::Joy>("/joy",1);


    /*UDP*/
    int sockfd;//socket file descriptor
    struct sockaddr_in addr, peeraddr; //socket net address
    socklen_t len;
    std::string msg;
    char buf[N] = {0};
    //open UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        return -1;
    }
    //bind
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(atoi(broadcast_port.c_str()));
    addr.sin_addr.s_addr = inet_addr(broadcast_ip.c_str());
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        perror("bind");
        return -1;
    }
    //receive msg
    len = sizeof(peeraddr);
    memset(&peeraddr, 0, sizeof(peeraddr));
    while (ros::ok()) {
        recvfrom(sockfd, buf, N, 0, (struct sockaddr *)&peeraddr, &len);
        printf("from %s: %s\n", inet_ntoa(peeraddr.sin_addr), buf);

        string msg = buf;
        switch_msg_to_joy(msg.c_str());
        pub_joy.publish(joy_msg);//publish the /joy topic msg
        joy_msg.axes.clear();
        joy_msg.buttons.clear();
        joy_msg.axes.resize(27);
        joy_msg.buttons.resize(19);

    }

    close(sockfd);

    return 0;
}
