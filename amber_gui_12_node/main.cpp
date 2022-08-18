#include <iostream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"

#include "udp.h"
#include "signal.h"

struct can_error_req{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
};

union can_error_req_u{
    can_error_req cer;
    uint8_t buffer[8];
};

struct can_error_data{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
    uint32_t status;
    int error_counter[7];
};

union can_error_data_u{
    can_error_data ced;
    uint8_t buffer[40];
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

    int sock = udp_init_host(iface_addr_str, iface_port_str);

    can_error_req_u tmp_1;
    tmp_1.cer.cmd_no = 12;
    tmp_1.cer.length = 8;

    can_error_data_u cedd;

    //while(!is_killed) {
        send_packet(sock, tmp_1.buffer, 8, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

        ssize_t nbytes;
        nbytes = wait_for_packet(sock, cedd.buffer, 44,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 44){
            std::cout<<cedd.ced.status<<std::endl;
            for (int i = 0; i < 8; ++i) {
                std::cout<<cedd.ced.error_counter[i]<<" ";
            }
            std::cout<<std::endl;
        }
        sleep(10);
    //}
    return 0;
    return 0;
}
