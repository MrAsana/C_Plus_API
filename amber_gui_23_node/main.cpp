#include <iostream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"

#include "udp.h"
#include "signal.h"

struct robot_mode_req{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
};

union robot_mode_req_u{
    robot_mode_req rsr;
    uint8_t buffer[8];
};

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
    robot_mode_req_u tmp_req;
    tmp_req.rsr.cmd_no = 2;
    tmp_req.rsr.length = 8;

    robot_mode_req_u tmp_2_req;
    tmp_2_req.rsr.cmd_no = 3;
    tmp_2_req.rsr.length = 8;

    robot_mode_data_u rs_u;

    int sock = udp_init_host(iface_addr_str, iface_port_str);

    //while(!is_killed) {
        send_packet(sock, tmp_req.buffer, 8, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

        ssize_t nbytes;
        nbytes = wait_for_packet(sock, rs_u.buffer, 9,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 9){
            std::cout<<(uint32_t)rs_u.rsr.respond<<std::endl;
        }
        sleep(5);

        send_packet(sock, tmp_2_req.buffer, 8, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

        nbytes = wait_for_packet(sock, rs_u.buffer, 9,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 9){
            std::cout<<(uint32_t)rs_u.rsr.respond<<std::endl;
        }
        sleep(5);
    //}
    return 0;
}
