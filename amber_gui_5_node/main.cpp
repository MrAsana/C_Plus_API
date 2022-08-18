#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"
#include<vector>
#include "udp.h"
#include "signal.h"
#include "rrt.h"
using namespace std;

struct state_point {
    float position[8];//弧度=57.29578 度
    float time;
};

struct two_state {
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
    uint32_t number_point;
    //vector<state_point> point;
    state_point a_point;
    //state_point b_point;
};

union two_state_u {
    two_state rs;
    uint8_t buffer[48];
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

    char *iface_addr_str =(char*) "0.0.0.0";
    char *iface_port_str = (char*)"12321";

    /*const int recvlen = 120;
    unsigned char *recvbuf = malloc(recvlen);

    trans_data_t rec_data;*/
    two_state_u tmp;
    tmp.rs.cmd_no = 5;
    tmp.rs.length = 48;
    tmp.rs.number_point = 1;
    //tmp.rs.a_point.time = 4.0;
vector<double> asd;
asd.push_back(-0.0351);asd.push_back(0.0351);asd.push_back(0);asd.push_back(0.001);asd.push_back(0.001);asd.push_back(0.61);
//asd.push_back(100);asd.push_back(100);asd.push_back(100);asd.push_back(1);asd.push_back(1);asd.push_back(1);

    vector<vector<double>> obstacleList={asd};
    // ��ʼ���Ŀ���
    node* startNode= new node(0.0651,0,0.6499);
    node* goalNode  = new node(0,-0.0651,0.6499);
    double Searchsize=2;
    RRT rrt(startNode, goalNode, obstacleList, 0.01, 5,Searchsize);
    vector<vector<double>> joint=rrt.check();
    //cout<<"@"<<endl;
    int sock = udp_init_host(iface_addr_str, iface_port_str);
    for (size_t i = 0; i < joint.size(); i++)
    {
        for (int j = 0; j < 8; ++j) {
        tmp.rs.a_point.position[j] =(float)joint[i][j];
        cout<<tmp.rs.a_point.position[j]<<"   "<<j;
        }
        cout<<"   "<<endl;
        // tmp.rs.a_point.position[7] = 1;
        // cout<<tmp.rs.a_point.position[7]<<"   "<<7<<"   "<<endl;
        tmp.rs.a_point.time = 1.0;
    //     for (int i = 0; i < 8; ++i) {
    //     tmp.rs.b_point.position[i] = -1;
    //     }
    // tmp.rs.b_point.time = 4.0;
        robot_mode_data_u rs_u;
    //std::cout<<1<<endl;
    //while(!is_killed) {
        send_packet(sock, tmp.buffer, 48, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));
        ssize_t nbytes;
        nbytes = wait_for_packet(sock, rs_u.buffer, 9,
                                 (struct sockaddr *) &src_addr, &addrlen);
        if(nbytes == 9){
            std::cout<<(uint32_t)rs_u.rsr.respond<<std::endl;
        }
        sleep(2);
    
    
    }
    
    
    

    // for (int i = 0; i < 8; ++i) {
    //     tmp.rs.point_2.position[i] = -1;
    // }
    // tmp.rs.point_2.time = 8.0;


    
    //}
    return 0;
}

