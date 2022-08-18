#include <iostream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"

#include "udp.h"
#include "signal.h"

struct state_point {
    uint32_t sub_cmd_no;
    float time;
    float xyzrpya[7];
};

struct arc_point {
    uint32_t sub_cmd_no;
    float time;
    float xyzrpya_a[7];
    float xyzrpya_b[7];
};

struct pause_point {
    uint32_t sub_cmd_no;
    float time;
};

struct two_state {
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
    uint32_t cmd_num;
    state_point a_point;
    pause_point b_point;
    state_point c_point;
};

struct three_state {
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
    uint32_t cmd_num;
    arc_point a_point;
    pause_point b_point;
    state_point c_point;
};


union two_state_u {
    two_state rs;
    uint8_t buffer[92];
};

union three_state_u {
    three_state rs;
    uint8_t buffer[120];
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
    /*two_state_u tmp;
    tmp.rs.cmd_no = 20;
    tmp.rs.length = 92;
    tmp.rs.cmd_num = 3;
    tmp.rs.a_point.sub_cmd_no = 2;
    tmp.rs.a_point.xyzrpya[0] = 0.1;
    tmp.rs.a_point.xyzrpya[1] = 0.0;
    tmp.rs.a_point.xyzrpya[2] = 0.5;
    tmp.rs.a_point.xyzrpya[3] = 0.0;
    tmp.rs.a_point.xyzrpya[4] = 0.0;
    tmp.rs.a_point.xyzrpya[5] = 0.0;
    tmp.rs.a_point.xyzrpya[6] = 0.0;
    tmp.rs.a_point.time = 4;
    tmp.rs.b_point.sub_cmd_no = 4;
    tmp.rs.b_point.time = 4;
    tmp.rs.c_point.sub_cmd_no = 1;
    tmp.rs.c_point.xyzrpya[0] = 0.0;
    tmp.rs.c_point.xyzrpya[1] = 0.0;
    tmp.rs.c_point.xyzrpya[2] = 0.65;
    tmp.rs.c_point.xyzrpya[3] = 0.0;
    tmp.rs.c_point.xyzrpya[4] = 0.0;
    tmp.rs.c_point.xyzrpya[5] = 0.0;
    tmp.rs.c_point.xyzrpya[6] = 0.0;
    tmp.rs.c_point.time = 4;*/

    three_state_u tmp;
    tmp.rs.cmd_no = 20;
    tmp.rs.length = 120;
    tmp.rs.cmd_num = 3;
    tmp.rs.a_point.sub_cmd_no = 3;
    tmp.rs.a_point.xyzrpya_a[0] = 0.0;
    tmp.rs.a_point.xyzrpya_a[1] = 0.0;
    tmp.rs.a_point.xyzrpya_a[2] = 0.72;
    tmp.rs.a_point.xyzrpya_a[3] = 0;
    tmp.rs.a_point.xyzrpya_a[4] = 0.0;
    tmp.rs.a_point.xyzrpya_a[5] = 0.0;
    tmp.rs.a_point.xyzrpya_a[6] = 0.0;
    tmp.rs.a_point.time = 8;
    tmp.rs.a_point.xyzrpya_b[0] = 0.0;
    tmp.rs.a_point.xyzrpya_b[1] = 0.2;
    tmp.rs.a_point.xyzrpya_b[2] = 0.45;
    tmp.rs.a_point.xyzrpya_b[3] = 0.0;
    tmp.rs.a_point.xyzrpya_b[4] = 0.0;
    tmp.rs.a_point.xyzrpya_b[5] = 0.0;
    tmp.rs.a_point.xyzrpya_b[6] = 0.0;
    tmp.rs.b_point.sub_cmd_no = 4;
    tmp.rs.b_point.time = 4;
    tmp.rs.c_point.sub_cmd_no = 2;
    tmp.rs.c_point.xyzrpya[0] = 0.0;
    tmp.rs.c_point.xyzrpya[1] = 0.0;
    tmp.rs.c_point.xyzrpya[2] = 0.65;
    tmp.rs.c_point.xyzrpya[3] = 0.0;
    tmp.rs.c_point.xyzrpya[4] = 0.0;
    tmp.rs.c_point.xyzrpya[5] = 0.0;
    tmp.rs.c_point.xyzrpya[6] = 0.0;
    tmp.rs.c_point.time = 4;

    robot_mode_data_u rs_u;

    int sock = udp_init_host(iface_addr_str, iface_port_str);

    //while(!is_killed) {
        send_packet(sock, tmp.buffer, 120, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

        ssize_t nbytes;
        nbytes = wait_for_packet(sock, rs_u.buffer, 9,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 9){
            std::cout<<(uint32_t)rs_u.rsr.respond<<std::endl;
        }
        sleep(30);
    //}
    return 0;
}
