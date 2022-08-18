#include <iostream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"
#include <cmath>
#include "udp.h"
#include "signal.h"
#include "huaxian.h"
struct robot_joint_position_req{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
    float xyz[3];
    float rpy[3];
    float arm_angle;
    float time;
};

union robot_joint_position_u{
    robot_joint_position_req rsr;
    uint8_t buffer[40];
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
double* Circelcenter_or_Rad(double p1[] ,double p2[] ,double p3[] )//形参输入采用字符串分割方法" ";
        {
            double v1[3],v2[3],v1n[3],v2n[3],nv[3],U[3],cW[3],W[3],V[3];
            double bx,cx,cy,h,m_rad;
            double* m_center=new double [3];
            double sd[3];
             for(int i = 0; i < 3; i++)
            {
                v1[i] = p2[i] - p1[i];
                v2[i] = p3[i] - p1[i];
            }
             for(int i = 0; i < 3; i++)
            {
                v1n[i] = v1[i] / sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);//v1的单位向量
                v2n[i] = v2[i]/ sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);//v2的单位向量
            }
            nv[0] = v1n[1] * v2n[2] - v1n[2] * v2n[1];
            nv[1] = v1n[2] * v2n[0] - v1n[0] * v2n[2];
            nv[2] = v1n[0] * v2n[1] - v1n[1] * v2n[0];
            if (nv[0] == 0 & nv[1] == 0 & nv[2] == 0)
            {
                return m_center;
            }
            //else if (Math.Abs(nv[0]+nv[1]+nv[2])< 1.0000e-05)
            //{
            //    MessageBox.Show("Three points too close to straight line!");return;
            //}
            else
            {
                
                memcpy(U,v1n,4 * sizeof(double));
                //W = cross(v2,v1)/norm((cross(v2,v1)));
                cW[0] = v2[1] * v1[2] - v2[2] * v1[1];
                cW[1] = v2[2] * v1[0] - v2[0] * v1[2];
                cW[2] = v2[0] * v1[1] - v2[1] * v1[0];
                for(int i = 0; i < 3; i++)
                {
                    W[i] = cW[i] / sqrt(cW[0] * cW[0] + cW[1] * cW[1] + cW[2] * cW[2]);
                }
                //v = cross(w,u);
                V[0] = W[1] * U[2] - W[2] * U[1];
                V[1] = W[2] * U[0] - W[0] * U[2];
                V[2] = W[0] * U[1] - W[1] * U[0];
 
                //bx = dot(v1,u)
                bx = v1[0] * U[0] + v1[1] * U[1] + v1[2] * U[2];
                //cx = dot(v2,u)
                cx = v2[0] * U[0] + v2[1] * U[1] + v2[2] * U[2];
                //cy = dot(v2,v)
                cy = v2[0] * V[0] + v2[1] * V[1] + v2[2] * V[2];
 
                //h = ((cx-bx/2)*(cx-bx/2)+cy*cy-(bx/2)*(bx/2))/(2*cy);
                h = ((cx - bx / 2) * (cx - bx / 2) + cy * cy - (bx / 2) * (bx / 2)) / (2 * cy);
                for (int i = 0; i < 3; i++)
                {
                    m_center[i] = p1[i] + ((bx / 2) * U[i]) + (h * V[i]);
                    sd[i]=m_center[i];
                }
                m_rad = sqrt((m_center[0] - p1[0]) * (m_center[0] - p1[0]) + (m_center[1] - p1[1]) * (m_center[1] - p1[1]) + (m_center[2] - p1[2]) * (m_center[2] - p1[2]));
            }
            return m_center;
        }
int main() {
    // double a1[3]={1,7,3};
    // double a2[3]={2,3,4};
    // double a3[3]={4,5,6};
    // double* a=Circelcenter_or_Rad(a1,a2,a3);
    // for (int i = 0; i < 3; i++)
    // {
    //     cout<<a[i]<<std::endl;
    // }

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

    trans_data_t rec_data;*/ 	//|X-Y-Z|R-P-Y|Arm angle|Time to target|

    robot_joint_position_u tmp_1;

    tmp_1.rsr.cmd_no = 6;
    tmp_1.rsr.length = 40;

    tmp_1.rsr.xyz[1] = -0.0651;//m
    tmp_1.rsr.xyz[0] = 0.0;
    tmp_1.rsr.xyz[2] = 0.6499;

    tmp_1.rsr.rpy[0] = 0.035595;//弧度=57.29578 度
    tmp_1.rsr.rpy[1] = -0.12545;
    tmp_1.rsr.rpy[2] = -0.2592;

    tmp_1.rsr.arm_angle = 0.7;

    tmp_1.rsr.time = 4.0;


    robot_joint_position_u tmp_2;

    tmp_2.rsr.cmd_no = 6;
    tmp_2.rsr.length = 40;

    tmp_2.rsr.xyz[0] = 0.2;
    tmp_2.rsr.xyz[1] = 0;
    tmp_2.rsr.xyz[2] = 0.60;

    //tmp_2.rsr.rpy[0] = 0.0;
    //tmp_2.rsr.rpy[1] = 0.0;
    tmp_2.rsr.rpy[2] = 0.0;

    tmp_2.rsr.time = 4.0;

    robot_mode_data_u rs_u;

    int sock = udp_init_host(iface_addr_str, iface_port_str);

    //while(!is_killed) {
        std::cout<<tmp_1.buffer<<std::endl;
        send_packet(sock, tmp_1.buffer, 40, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

         ssize_t nbytes;
        nbytes = wait_for_packet(sock, rs_u.buffer, 9,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 9){
            std::cout<<(uint32_t)rs_u.rsr.respond<<std::endl;
        }
         sleep(6);
std::cout<<tmp_2.buffer<<std::endl;
        send_packet(sock, tmp_2.buffer, 40, (struct sockaddr *) &destAddrUdp, sizeof(destAddrUdp));

        nbytes = wait_for_packet(sock, rs_u.buffer, 9,
                                 (struct sockaddr *) &src_addr, &addrlen);

        if(nbytes == 9){
            std::cout<<(uint32_t)rs_u.rsr.respond<<std::endl;
        }
        sleep(6);
    //}
    return 0;

}
// void Circelcenter_or_Rad(double[] p1,double[] p2,double[] p3)//形参输入采用字符串分割方法" ";
//         {
//              for(int i = 0; i < 3; i++)
//             {
//                 v1[i] = p2[i] - p1[i];
//                 v2[i] = p3[i] - p1[i];
//             }
//              for(int i = 0; i < 3; i++)
//             {
//                 v1n[i] = v1[i] / sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);//v1的单位向量
//                 v2n[i] = v2[i]/ sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);//v2的单位向量
//             }
//             nv[0] = v1n[1] * v2n[2] - v1n[2] * v2n[1];
//             nv[1] = v1n[2] * v2n[0] - v1n[0] * v2n[2];
//             nv[2] = v1n[0] * v2n[1] - v1n[1] * v2n[0];
//             if (nv[0] == 0 & nv[1] == 0 & nv[2] == 0)
//             {
//                 MessageBox.Show("Three points collinear!");return;
//             }
//             //else if (Math.Abs(nv[0]+nv[1]+nv[2])< 1.0000e-05)
//             //{
//             //    MessageBox.Show("Three points too close to straight line!");return;
//             //}
//             else
//             {
//                 U = v1n;
//                 //W = cross(v2,v1)/norm((cross(v2,v1)));
//                 cW[0] = v2[1] * v1[2] - v2[2] * v1[1];
//                 cW[1] = v2[2] * v1[0] - v2[0] * v1[2];
//                 cW[2] = v2[0] * v1[1] - v2[1] * v1[0];
//                 for(int i = 0; i < 3; i++)
//                 {
//                     W[i] = cW[i] / sqrt(cW[0] * cW[0] + cW[1] * cW[1] + cW[2] * cW[2]);
//                 }
//                 //v = cross(w,u);
//                 V[0] = W[1] * U[2] - W[2] * U[1];
//                 V[1] = W[2] * U[0] - W[0] * U[2];
//                 V[2] = W[0] * U[1] - W[1] * U[0];
 
//                 //bx = dot(v1,u)
//                 bx = v1[0] * U[0] + v1[1] * U[1] + v1[2] * U[2];
//                 //cx = dot(v2,u)
//                 cx = v2[0] * U[0] + v2[1] * U[1] + v2[2] * U[2];
//                 //cy = dot(v2,v)
//                 cy = v2[0] * V[0] + v2[1] * V[1] + v2[2] * V[2];
 
//                 //h = ((cx-bx/2)*(cx-bx/2)+cy*cy-(bx/2)*(bx/2))/(2*cy);
//                 h = ((cx - bx / 2) * (cx - bx / 2) + cy * cy - (bx / 2) * (bx / 2)) / (2 * cy);
//                 for (int i = 0; i < 3; i++)
//                 {
//                     m_center[i] = p1[i] + ((bx / 2) * U[i]) + (h * V[i]);
//                 }
//                 m_rad = sqrt((m_center[0] - p1[0]) * (m_center[0] - p1[0]) + (m_center[1] - p1[1]) * (m_center[1] - p1[1]) + (m_center[2] - p1[2]) * (m_center[2] - p1[2]));
//             }
//         }
// class LinearPath
//     {
//         int number = 0;
//         double LineLength = 0;
//         double dx, dy, dz = 0;
//         public List<double> x_start = new List<double>();
//         public List<double> y_start = new List<double>();
//         public List<double> z_start = new List<double>();
//         public void clear()
//         {
//             x_start.Clear();
//             y_start.Clear();
//             z_start.Clear();
//         }
//         public void Plan_path(double xs,double ys,double zs,double xe,double ye,double ze,double speed)
//         {
//             clear();            
//             x_start.Add(xs);
//             y_start.Add(ys);
//             z_start.Add(zs);
//             LineLength = sqrt((xe - xs) * (xe - xs) + (ye - ys) * (ye - ys) + (ze - zs) * (ze - zs));
//             number = Convert.ToInt32(LineLength / speed);
//             dx = (xe - xs) / number;
//             dy = (ye - ys) / number;
//             dz = (ze - zs) / number;
//             for(int i = 0; i < number; i++)
//             {
//                 x_start.Add(x_start[i] + dx);
//                 y_start.Add(y_start[i] + dy);
//                 z_start.Add(z_start[i] + dz);
//             }
//         }
 
//     };