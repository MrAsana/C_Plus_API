#include <iostream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"

#include "udp.h"
#include "signal.h"

struct robot_joint_position_req{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
    float position[8];
    float time;
};

union robot_joint_position_u{
    robot_joint_position_req rsr;
    uint8_t buffer[44];
};
//

struct robot_mode_data{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
    uint8_t respond;
};

union robot_mode_data_u{
    robot_mode_data rsr;
    uint8_t buffer[9];
};

bool is_killed=false;

void signal_callback_handler(int signum) {
    printf("Caught kill signal\n");
    // Terminate program
    is_killed = true;
    exit(signum);
}

int main() {
    signal(SIGINT, signal_callback_handler);
    struct sockaddr_in destAddrUdp;

    memset(&destAddrUdp, 0, sizeof(destAddrUdp));  //set zero
    destAddrUdp.sin_family = AF_INET; //internet use
    destAddrUdp.sin_addr.s_addr = inet_addr("192.168.51.65"); //set the ip connect
    destAddrUdp.sin_port = htons(25001); //set the port to use


    struct sockaddr_storage src_addr = {0};
    socklen_t addrlen = sizeof src_addr;

    char *iface_addr_str = "0.0.0.0";
    char *iface_port_str = "12321";

    /*const int recvlen = 120;
    unsigned char *recvbuf = malloc(recvlen);

    trans_data_t rec_data;*/

    robot_joint_position_u tmp_1;

    tmp_1.rsr.cmd_no = 4;
    tmp_1.rsr.length = 44;

    /*for (int i = 0; i < 8; ++i) {
        tmp_1.rsr.position[i] = 1.0;
    }*/
    tmp_1.rsr.position[0] = 1.1410963;
    tmp_1.rsr.position[1] = 0.30072023;
    tmp_1.rsr.position[2] = 1.0821;
    tmp_1.rsr.position[3] = -0.681027474;
    tmp_1.rsr.position[4] = 0.1741839;
    tmp_1.rsr.position[5] = 0.38885936;
    tmp_1.rsr.position[6] = -2.0943951;
    tmp_1.rsr.position[7] = 0;
    tmp_1.rsr.time = 2.0;


    /*robot_joint_position_u tmp_2;

    tmp_2.rsr.cmd_no = 4;
    tmp_2.rsr.length = 44;

    for (int i = 0; i < 8; ++i) {
        tmp_2.rsr.position[i] = -1.0;
    }
    tmp_2.rsr.time = 2.0;*/

    robot_mode_data_u rs_u;

    int sock = udp_init_host(iface_addr_str, iface_port_str);

    //while(!is_killed) {
        send_packet(sock, tmp_1.buffer, 44, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

        ssize_t nbytes;
        nbytes = wait_for_packet(sock, rs_u.buffer, 9,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 9){
            std::cout<<(uint32_t)rs_u.rsr.respond<<std::endl;
        }
        sleep(3);

        /*send_packet(sock, tmp_2.buffer, 44, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

        nbytes = wait_for_packet(sock, rs_u.buffer, 9,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 9){
            std::cout<<(uint32_t)rs_u.rsr.respond<<std::endl;
        }
        sleep(3);*/
    //}
    return 0;
}
