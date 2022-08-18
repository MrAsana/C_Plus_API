#include <iostream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"

#include "udp.h"
#include "signal.h"


struct robot_state{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
    float joint_position[8];
    float joint_velocity[8];
    float task_space_position[3];
    float task_space_rpy[3];
    float task_space_linear_velocity[3];
    float task_space_anguluar_velocity[3];
    float arm_angle;
};

union robot_state_u{
    robot_state rs;
    uint8_t buffer[124];
};

struct robot_state_req{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
};

union robot_state_req_u{
    robot_state_req rsr;
    uint8_t buffer[8];
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
    robot_state_req_u tmp_req;
    tmp_req.rsr.cmd_no = 1;
    tmp_req.rsr.length = 8;

    robot_state_u rs_u;

    int sock = udp_init_host(iface_addr_str, iface_port_str);

    //while(!is_killed) {
        send_packet(sock, tmp_req.buffer, 8, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

        ssize_t nbytes;
        nbytes = wait_for_packet(sock, rs_u.buffer, 124,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 124){
            //for (int i = 0; i < 8; ++i) {
                std::cout<<rs_u.rs.task_space_position[0]<<" "<<rs_u.rs.task_space_position[1]<<" "<<rs_u.rs.task_space_position[2]<<std::endl;
                std::cout<<rs_u.rs.task_space_rpy[0]<<" "<<rs_u.rs.task_space_rpy[1]<<" "<<rs_u.rs.task_space_rpy[2]<<std::endl;
                std::cout<<rs_u.rs.arm_angle<<std::endl;
                //std::cout<<rs_u.rs.task_space_position[2]<<" "<<rs_u.rs.task_space_position[1]<<" "<<rs_u.rs.task_space_position[0]<<std::endl;
            //}
        }
        sleep(10);
    //}
    return 0;
}
