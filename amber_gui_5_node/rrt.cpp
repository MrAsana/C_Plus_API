#include "rrt.h"
#include <iostream>
#include"InverseKinematics.h"
#include "/usr/include/eigen3/Eigen/Core"
#include "ForwardKinematics.h"
node::node(double _x, double _y, double _z) : x(_x), y(_y), z(_z), parent(nullptr), cost(0) {}

double node::getX() { return x; }
double node::getY() { return y; }
double node::getZ() { return z; }
void node::setX(double _x) { x=_x; }
void node::setY(double _y) { y=_y; }
void node::setZ(double _z) { z=_z; }

void node::setParent(node* _parent) { parent = _parent; }
node* node::getParent() { return parent; }
    // 0.0825
    // 0.1678   0.0853
    // 0.2967   0.1289
    // 0.382    0.0853
    // 0.5071   0.1251
    // 0.5861   0.0790
    // 0.7097   0.1236 {0.0825,0.0853,0.1289,0.0853,0.1251,0.0790,0.1236},
RRT::RRT(node* _startNode, node* _goalNode, const vector<vector<double>>& _obstacleList,
         double _stepSize, 
         int _goal_sample_rate ,double _searchsize)
    : startNode(_startNode), goalNode(_goalNode),
      obstacleList(_obstacleList),
      stepSize(_stepSize), goal_sample_rate(_goal_sample_rate),
      goal_gen(goal_rd()), goal_dis(uniform_int_distribution<int>(0, 100)),
      area_gen(area_rd()), area_dis(uniform_real_distribution<double>(0, 1)),
      searchsize(_searchsize)
      {}

node* RRT::getNearestNode(const vector<double>& randomPosition) {
    int minID = -1;
    double minDistance = numeric_limits<double>::max(); // 

    // 找到和随机位置距离最小的节点
    for (int i = 0; i < nodeList.size(); i++) {
        // 在这里距离不需要开根号
        double distance = pow(nodeList[i]->getX() - randomPosition[0], 2) + pow(nodeList[i]->getY() - randomPosition[1], 2)+ pow(nodeList[i]->getZ() - randomPosition[2], 2);
        if (distance < minDistance) {
            minDistance = distance;
            minID = i;
        }
    }

    return nodeList[minID];
}

vector<node*> RRT::planning() {
   // 建立一个窗口,WINDOW_NORMAL表示可以调整窗口大小
    // namedWindow("RRT", WINDOW_NORMAL);
// vector<node*> path33;
// return path33;
    int count = 0;

   // 建立背景
    // const int imageSize = 15;
    // const int imageResolution = 50;
    // Mat background(imageSize * imageResolution, imageSize * imageResolution,
    //     CV_8UC3, cv::Scalar(255, 255, 255)); // CV_[位数][是否带符号]C[通道数]

    // 画出起始位置和目标位置
    // circle(background, Point(startNode->getX() * imageResolution, startNode->getY() * imageResolution,startNode->getZ() * imageResolution), 20, Scalar(0, 0, 255), -1);
    // circle(background, Point(goalNode->getX() * imageResolution, goalNode->getY() * imageResolution, goalNode->getZ() * imageResolution), 20, Scalar(255, 0, 0), -1);
     // 画出障碍物
    for (auto item : obstacleList)
        // circle(background, Point(item[0] * imageResolution, item[1] * imageResolution, item[2] * imageResolution), Scalar(0, 0, 0), -1);

    // RRT
    nodeList.push_back(startNode);
    while (1) {
        // 生成一个随机位置
        vector<double> randomPosition;
        if (goal_dis(goal_gen) > goal_sample_rate) { // 这里可以优化成直接用节点来表示
            double randX = area_dis(goal_gen);
            double randY = area_dis(goal_gen);
            double randZ = area_dis(goal_gen);
            randomPosition.push_back(randX);
            randomPosition.push_back(randY);
            randomPosition.push_back(randZ);
        }
        else { // 找到了目标,将目标位置保存
            randomPosition.push_back(goalNode->getX());
            randomPosition.push_back(goalNode->getY());
            randomPosition.push_back(goalNode->getZ());
        }

        // 找到和新生成随机节点距离最近的节点
        node* nearestNode = getNearestNode(randomPosition);
        
        double mx=randomPosition[0]-nearestNode->getX();
        double my=randomPosition[1]-nearestNode->getY();
        double mz=randomPosition[2]-nearestNode->getZ();
        double sum=sqrt(pow(mx,2)+pow(my,2)+pow(mz,2));
        vector<double> movingVec;
        movingVec.push_back(mx/sum);
        movingVec.push_back(my/sum);
        movingVec.push_back(mz/sum);


        // 利用反正切计算角度,然后利用角度和步长计算新坐标
        // float theta = atan2(randomPosition[1] - nearestNode->getY(), randomPosition[0] - nearestNode->getX());
        // double dz = -nearestNode->getZ() + randomPosition[2];
        // double dy = -nearestNode->getY() + randomPosition[1];
        // double dx = -nearestNode->getX() + randomPosition[0];
        // float xoy = atan2( abs(dz), sqrt(dx * dx + dy * dy) );
        // if(dz>0){
        //     xoy=M_PI/2-xoy;
        // }else{
        //     xoy-=M_PI/2;
        // }
        //node* newNode = new node(nearestNode->getX() + stepSize * cos(theta), nearestNode->getY() + stepSize * sin(theta),nearestNode->getZ() + stepSize * cos(xoy));
        node* newNode = new node(nearestNode->getX() + stepSize * movingVec[0], nearestNode->getY() + stepSize * movingVec[1],nearestNode->getZ() + stepSize * movingVec[2]);
        
        newNode->setParent(nearestNode);
        vector<node*> NODE1;
        NODE1.push_back(nearestNode);
        NODE1.push_back(newNode);
        if (!collisionCheck1(stepSize,NODE1)) continue;
        nodeList.push_back(newNode);

        // 
        // line(background,
        //     Point(static_cast<int>(newNode->getX() * imageResolution), static_cast<int>(newNode->getY() * imageResolution), static_cast<int>(newNode->getZ() * imageResolution)),
        //     Point(static_cast<int>(nearestNode->getX() * imageResolution), static_cast<int>(nearestNode->getY() * imageResolution), static_cast<int>(nearestNode->getZ() * imageResolution)),
        //     Scalar(0, 255, 0),10);

        // count++;
        // imshow("RRT", background);
        //waitKey(5);

        if (sqrt(pow(newNode->getX() - goalNode->getX(), 2) + pow(newNode->getY() - goalNode->getY(), 2)+ pow(newNode->getZ() - goalNode->getZ(), 2)) <= stepSize) {
            cout << "The path has been found!" << endl;
            break;
        };
    }

   // 最终得到的路径
    vector<node*> path;
    path.push_back(goalNode);
    node* tmpNode = nodeList.back();
    while (tmpNode->getParent() != nullptr) {
        // line(background,
        //     Point(static_cast<int>(tmpNode->getX() * imageResolution), static_cast<int>(tmpNode->getY() * imageResolution), static_cast<int>(tmpNode->getZ() * imageResolution)),
        //     Point(static_cast<int>(tmpNode->getParent()->getX() * imageResolution), static_cast<int>(tmpNode->getParent()->getY() * imageResolution), static_cast<int>(tmpNode->getParent()->getY() * imageResolution)),
        //     Scalar(255, 0, 255), 10);
        path.push_back(tmpNode);
        tmpNode = tmpNode->getParent();
    }

    //展示背景
    //imshow("RRT", background);
    //waitKey(0);
    //cout<<"1234567890";
    path.push_back(startNode);
    return path;
}
vector<vector<double>> RRT::check(){
    vector<node*> robot={new node(0, 0, 0.0825),new node(0,0.0275, 0.1678),new node(0, 0, 0.2967),new node(0, -0.0275, 0.382),new node(0, 0, 0.5071),new node(0,0.025101,0.5861),new node(0, 0, 0.7097)};
    vector<double>  asd;
    
    double r=0.05;
    float a=0,b=0,c=0;
    // for (int i = 0; i < robot.size(); i++)
    //     {
    //         r[i]=sqrt(pow((robot[i]->getX())-a,2)+pow((robot[i]->getY())-b,2)+pow((robot[i]->getZ())-c,2));//杆长取均值
    //         a=robot[i]->getX();b=robot[i]->getY();c=robot[i]->getZ();
    //     }
    //cout<<obstacleList[0][0]<<endl;
    //由于rrt算法搜寻出的路径不一定符合机械臂的运动关节约束，因此最多尝试10次，假如10次还不能搜出一条合适的路径，就认为不存在这样的路径
    for (int i = 0; i < 10; i++)
    {
        
        //vector<vector<double>> ik_joint_;
        double ik_joint_1[7]={0,0,0,0,0,0,0};
        vector<vector<double>> joint;
        vector<double> ik_joint_2={0,0,0,0,0,0,0};
        int key=0;
        int index[8]={1,2,3,4,5,6,7,8};
        
        double pos_matrix[16]={1,0,0,robot[robot.size()-1]->getX(),
                                0,1,0,robot[robot.size()-1]->getY(),
                                0,0,1,robot[robot.size()-1]->getZ(),
                                0,0,0,1};
          vector<node*> path=planning();
        // for (int ii = 0; ii < path.size(); ii++)
        // {
        //     cout<<path[ii]->getX()<<"   "<<path[ii]->getY()<<"   "<<path[ii]->getZ()<<"   "<<endl;
        // }
        
    //    cout<<"qwrewqedsdfgrewf"<<endl;
            
            

        
        //pos_matrix[3]=a->getX();pos_matrix[7]=a->getY();pos_matrix[11]=a->getZ();
        // a->setX(0.1);
        // a->setY(0.1);
        // a->setZ(0.6);
        //ouble xyzrpya[7]={a->getX(),a->getY(),a->getZ(),0,0,0,M_PI} ;  
        //cout<<"!"<<endl;
        // vector<node*> path;
        // node* a;node* b;node* c;node* d;
        // if(obstacleList[0][0]!=100){
        //         a=new node(-0.0651,0,0.6499);
        // c=new node(0,0.0651,0.6499);
        // b=new node(-0.0351,0.0351,0.7);
        // d=new node(0,1.387e-06,0.7097);
        // path.push_back(d);
        // path.push_back(a);path.push_back(b);
        // path.push_back(c);
        //     }else{

        //         c=new node(-0.0651,0,0.6499);
        // a=new node(0,0.0651,0.6499);
        // path.push_back(a);
        // path.push_back(c);
        //      }         
        int pathlength=path.size();
        // if(obstacleList.size()==0){//暂时没有检测
        //     for (int j = 0; j < pathlength; j++)
        // {
        //     for (int k = 0; k < 8; k++)
        //     {
        
        //     InverseKinematics(pos_matrix, 0, k, ik_joint_1);

        //     for (int hk = 0; hk < 7; hk++)
        //     {
        //         ik_joint_2[hk]=ik_joint_1[hk];//需要优化
        //     }
        //     }
        //     ik_joint_.push_back(ik_joint_2);
        //     for (int hk = 0; hk < 7; hk++)
        //     {
        //         sum[hk]=ik_joint_[hk][0]+ik_joint_[hk][1]+ik_joint_[hk][2]+ik_joint_[hk][3]+ik_joint_[hk][4]+ik_joint_[hk][5]+ik_joint_[hk][6]+ik_joint_[hk][7];
        //     }
        //     double sort1;
        //     for (int hk = 0; hk < 6; hk++)
        //     {
        //         for(int k=hk+1;k<7;k++){
        //             if(sum[hk]>sum[k]){
        //                 sort1=sum[hk];
        //                 sum[hk]=sum[k];
        //                 sum[k]=sort1;
        //                 index[hk]=k;
        //             }
        //         }
        //     }

        // }
        // joint.push_back(ik_joint_[index[0]]);
        // return joint;
        // }
        //碰撞检测
        //{0.0825,0.0853,0.1289,0.0853,0.1251,0.0790,0.1236};
        int abc=0;
        for (int j = 0; j < pathlength; j++)
        {
            cout<<j<<endl;
            vector<vector<double>> ik_joint_;
            double c1=path[j]->getX();double c2=path[j]->getY();double c3=path[j]->getZ();
            double xyzrpya[7]={c1,c2,c3,0,0,0,M_PI} ;  
            Eigen::Matrix3d rot_x;
                rot_x << 1, 0, 0,
                        0, cos(xyzrpya[3]), -sin(xyzrpya[3]),
                        0, sin(xyzrpya[3]), cos(xyzrpya[3]);

                Eigen::Matrix3d rot_y;
                rot_y << cos(xyzrpya[4]), 0, sin(xyzrpya[4]),
                        0, 1, 0,
                        -sin(xyzrpya[4]), 0, cos(xyzrpya[4]);

                Eigen::Matrix3d rot_z;

                rot_z << cos(xyzrpya[5]), -sin(xyzrpya[5]), 0,
                        sin(xyzrpya[5]), cos(xyzrpya[5]), 0,
                        0, 0, 1;

                Eigen::Matrix3d rot = rot_z*rot_y*rot_x;

                Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
                se3.block(0, 0, 3, 3) = rot;
                se3(0, 3) = xyzrpya[0];
                se3(1, 3) = xyzrpya[1];
                se3(2, 3) = xyzrpya[2];
                for (int i=0;i<4;i++){
                    for (int j=0;j<4;j++){
                        pos_matrix[i*4 + j] = se3(j,i);
                        //cout<<pos_matrix[i*4 + j]<<"   ";
                    }
                    //cout<<endl;
                }
             double ik_joint_11[7]   ;
             double fk_matrix[16];
             double tmp_arm_angle;
            //先代入逆解方程，位姿矩阵,臂角,机器人配置8位数字,各关节值大小
            for (int k = 0; k < 8; k++)
            {
                for (int hi = 0; hi < 7; hi++)
                {
                    ik_joint_11[hi]=ik_joint_1[hi];

                }

                InverseKinematics(pos_matrix, xyzrpya[6], k, ik_joint_11);//需要优化
                //ForwardKinematics(ik_joint_11, fk_matrix, &tmp_arm_angle);

                //if(abs(fk_matrix[12]-xyzrpya[0])<0.015 && abs(fk_matrix[13]-xyzrpya[1])<0.015 && abs(fk_matrix[14]-xyzrpya[2])<0.015){
                //cout<< fk_matrix[12]<<"  "<<xyzrpya[0]<<"  1  2  "<<fk_matrix[13]<<"  "<<xyzrpya[1]<<"  2  3  "<<fk_matrix[14]<<"  "<<xyzrpya[2]<<endl;
                //然后进行障碍检测
                if (collisionCheck2(r,ik_joint_11)) {
                    key++;
                    for (int hk = 0; hk < 7; hk++)
                {
                    ik_joint_2[hk]=ik_joint_11[hk];
                    //cout<<ik_joint_11[hk]<<"   ";
                }
                //cout<<endl;
                ik_joint_.push_back(ik_joint_2);
                }
                //}
                //     for (int hk = 0; hk < 7; hk++)
                // {
                //     //ik_joint_2[hk]=ik_joint_11[hk];
                //     cout<<ik_joint_11[hk]<<"   ";
                // }
                // cout<<endl;
            }

            //cout<<"                                              "<<endl;
            //cout<<ik_joint_.size()<<endl;
            //从八组解中选出可行且最优的那一组
            double sum[ik_joint_.size()];
                for (int hk = abc; hk < ik_joint_.size(); hk++)
                {
                    //cout<<hk<<endl;
                    sum[hk]=abs(ik_joint_[hk][0])+abs(ik_joint_[hk][1])+abs(ik_joint_[hk][2])+abs(ik_joint_[hk][3])+abs(ik_joint_[hk][4])+abs(ik_joint_[hk][5])+abs(ik_joint_[hk][6]);
                    //cout<<sum[hk]<<"   ";
                }
                //cout<<endl;
                double sort1;
                for (int hk = abc; hk < ik_joint_.size()-1; hk++)
                {
                    for(int k=hk+1;k<ik_joint_.size();k++){
                        if(sum[hk]>sum[k]){
                            sort1=sum[hk];
                            sum[hk]=sum[k];
                            sum[k]=sort1;
                            index[hk]=k;
                        }
                    }
                    //cout<<index[hk]<<"   ";
                }
                //cout<<endl;
                //cout<<1<<endl;
                //for (int hi = 0; hi < ik_joint_.size(); hi++)
                //{
                    // for (int i3 = 0; i3 < 7; i3++)
                    // {
                    //     cout<<ik_joint_[index[0]-1][i3]<<"  "<<endl;
                    // }
                    
                    //if(sum[0]<10){
                        //cout<<index[hi]<<ik_joint_.size()<<endl;
                        joint.push_back(ik_joint_[index[0]-1]);

                    //}
                    for (int ic = 0; ic <= ik_joint_.size(); ic++)
                    {
                        ik_joint_.pop_back();
                    }
                    abc=ik_joint_.size();
                    //cout<<abc<<endl;
                    //cout<<"erw"<<endl;
                    //cout<<"   "<<index[hi]<<endl;
                //}
               // cout<<"123456789"<<endl;
            // for (int ig = 0; ig< 7; ig++)
            // {
            //    cout<<joint[j][ig]<<" ";
            // }
            //cout<<"123456789"<<endl;
              //cout<<endl;
               //cout<<key<<"  "<<j<<"  "<<pathlength<<endl;
            //cout<<ik_joint_.size()<<" out k "<<endl;
            if (key<j+1&&j==pathlength-1)
            {
                //cout<<key<<"  "<<j<<endl;
                break;
            }
        //cout<<j<<"   "<<endl;                  
        }
        
        //cout<<key<<"  "<<pathlength<<endl;
        if (key>=pathlength)
            {
                return joint;
            }
        
    }
    
    vector<vector<double>> joint1;
    cout<<"error"<<endl;
    return joint1;
}
 bool RRT::collisionCheck2(double r,double ik_joint_[7]) {
//     //D-H,根据运动学正解,用关节角计算各关节位置
    // for (int i = 0; i < 7; i++)
    // {

    //     if(abs(ik_joint_[i])>2)
    //     return false;
    // }
    // // for (int i = 0; i < 7; i++)
    // // {
    // //     cout<<ik_joint_[i]<<"     ";
    // // }
    // // cout<<endl;

    // double D[7]={0.08+0.0895,0.125+0.0895,0,0.1225+0.0856,0,0.1225+0.0041,0};
    // //double D[7]={0.1678,0,0.2042,0,0.2041,0,0.1621};
    // double apl[7]={0,M_PI/2,M_PI/2,M_PI/2,M_PI/2,M_PI/2,0};
    // //double apl[7]={-M_PI/2,M_PI/2,M_PI/2,-M_PI/2,-M_PI/2,M_PI/2,0};
    // double a[7]={0,0,0,0,0,0,0};
    //  vector<vector<double>> T1;
    //  vector<double> t11;
    //  t11={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    // T1.push_back(t11);T1.push_back(t11);T1.push_back(t11);T1.push_back(t11);T1.push_back(t11);T1.push_back(t11);T1.push_back(t11);
    // vector<double> miniT;
    // vector<vector<double>> T;
    // for (int i = 0; i < 7; i++)
    // {
    //     miniT={cos(ik_joint_[i]),-sin(ik_joint_[i]),0,a[0],
    //             sin(ik_joint_[i])*cos(apl[i]),cos(ik_joint_[i])*cos(apl[i]),-sin(apl[i]),-D[i]*sin(apl[i]),
    //             sin(ik_joint_[i])*sin(apl[i]),cos(ik_joint_[i])*sin(apl[i]),cos(apl[i]),D[i]*cos(apl[i]),
    //             0,0,0,1};
    //     T.push_back(miniT);
    // }
    // //vector<double>Tt;
    // for (int i = 6; i >=0; i--)
    // {
    //     for (int j = 0; j <=i; j++)
    //     {
    //         vector<double> Tt={T1[i][0]*T[j][0]+T1[i][1]*T[j][4]+T1[i][2]*T[j][8]+T1[i][3]*T[j][12],T1[i][0]*T[j][1]+T1[i][1]*T[j][5]+T1[i][2]*T[j][9]+T1[i][3]*T[j][13],T1[i][0]*T[j][2]+T1[i][1]*T[j][6]+T1[i][2]*T[j][10]+T1[i][3]*T[j][14],T1[i][0]*T[j][3]+T1[i][1]*T[j][7]+T1[i][2]*T[j][11]+T1[i][3]*T[j][15],
    //         T1[i][4]*T[j][0]+T1[i][5]*T[j][4]+T1[i][6]*T[j][8]+T1[i][7]*T[j][12],T1[i][4]*T[j][1]+T1[i][5]*T[j][5]+T1[i][6]*T[j][9]+T1[i][7]*T[j][13],T1[i][4]*T[j][2]+T1[i][5]*T[j][6]+T1[i][6]*T[j][10]+T1[i][7]*T[j][14],T1[i][4]*T[j][3]+T1[i][5]*T[j][7]+T1[i][6]*T[j][11]+T1[i][7]*T[j][15],
    //         T1[i][8]*T[j][0]+T1[i][9]*T[j][4]+T1[i][10]*T[j][8]+T1[i][11]*T[j][12],T1[i][8]*T[j][1]+T1[i][9]*T[j][5]+T1[i][10]*T[j][9]+T1[i][11]*T[j][13],T1[i][8]*T[j][2]+T1[i][9]*T[j][6]+T1[i][10]*T[j][10]+T1[i][11]*T[j][14],T1[i][8]*T[j][3]+T1[i][9]*T[j][7]+T1[i][10]*T[j][11]+T1[i][11]*T[j][15],
    //         T1[i][12]*T[j][0]+T1[i][13]*T[j][4]+T1[i][14]*T[j][8]+T1[i][15]*T[j][12],T1[i][12]*T[j][1]+T1[i][13]*T[j][5]+T1[i][14]*T[j][9]+T1[i][15]*T[j][13],T1[i][12]*T[j][2]+T1[i][13]*T[j][6]+T1[i][14]*T[j][10]+T1[i][15]*T[j][14],T1[i][12]*T[j][3]+T1[i][13]*T[j][7]+T1[i][14]*T[j][11]+T1[i][15]*T[j][15]};   
    //         for (int k = 0; k < 16; k++)
    //                     {
    //                         T1[i][k]=Tt[k];
    //                     }

    //     }
        
        
    // }
    //根据dh参数与关节角信息计算每个关节的位置
    Eigen::Matrix4d A0_;
    Eigen::Matrix4d B0_;
    Eigen::Matrix4d C0_;
    Eigen::Matrix4d D0_;
    Eigen::Matrix4d E0_;
    Eigen::Matrix4d F0_;
    Eigen::Matrix4d G0_;
    Eigen::Matrix4d H0_;

    Eigen::Matrix4d A0T_;
    Eigen::Matrix4d B0T_;
    Eigen::Matrix4d C0T_;
    Eigen::Matrix4d D0T_;
    Eigen::Matrix4d E0T_;
    Eigen::Matrix4d F0T_;
    Eigen::Matrix4d G0T_;
    Eigen::Matrix4d H0T_;
    double joint_lower[7];
    double joint_upper[7];
    A0_ << 1,  0,  0, 0,
           0,  1,  0, 0,
           0,  0,  1, 0.0825,
           0,  0,  0, 1;

    A0T_ << 1,  0,  0, 0,
            0,  1,  0, 0,
            0,  0,  1, 0.0825,
            0,  0,  0, 1;

    B0_ <<1, 0, 0, 0,
          0, 0, 1, 0.0275,
          0, -1, 0, 0.0853,
          0, 0, 0, 1;

    B0T_ <<1, 0, 0, 0,
            0, 0, 1, 0,
            0, -1, 0, 0.0853,
            0, 0, 0, 1;

    C0_ << 1, 0, 0, 0,
            0, 0, -1, -0.1289,
            0, 1, 0, -0.0275,
            0, 0, 0, 1;

    C0T_ << 1, 0, 0, 0,
            0, 0, -1, -0.1289,
            0, 1, 0, 0,
            0, 0, 0, 1;

    D0_ << 1, 0, 0, 0,
            0, 0, 1, -0.0275,
            0, -1, 0, 0.0853,
            0, 0, 0, 1;

    D0T_ << 1, 0, 0, 0,
            0, 0, 1,  0,
            0, -1, 0, 0.0853,
            0, 0, 0, 1;

    E0_ << 1, 0, 0, 0,
            0, 0, -1, -0.1251,
            0, 1, 0, 0.0275,
            0, 0, 0, 1;

    E0T_ << 1, 0, 0, 0,
            0, 0, -1, -0.1251,
            0, 1, 0, 0,
            0, 0, 0, 1;

    F0_ << 1, 0, 0, 0,
            0, 0, 1, 0.0251,
            0, -1, 0, 0.079,
            0, 0, 0, 1;

    F0T_ << 1, 0, 0, 0,
            0, 0, 1, 0,
            0, -1, 0, 0.079,
            0, 0, 0, 1;

    G0_ << 1, 0, 0, 0,
            0, 0, -1, -0.1236,
            0, 1, 0, -0.0251,
            0, 0, 0, 1;

    G0T_ << 1, 0, 0, 0,
            0, 0, -1, -0.1236,
            0, 1, 0, 0,
            0, 0, 0, 1;

    H0_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.0385,
            0, 0, 0, 1;
    Eigen::Matrix4d J1;
    J1<< cos(ik_joint_[0]), -sin(ik_joint_[0]),  0, 0,
            sin(ik_joint_[0]),  cos(ik_joint_[0]),  0, 0,
            0,              0,  1, 0,
            0,              0,  0, 1;
    Eigen::Matrix4d J2;
    J2<< cos(ik_joint_[1]), -sin(ik_joint_[1]),  0, 0,
            sin(ik_joint_[1]),  cos(ik_joint_[1]),  0, 0,
            0,              0,  1, 0,
            0,              0,  0, 1;
    Eigen::Matrix4d J3;
    J3<< cos(ik_joint_[2]), -sin(ik_joint_[2]),  0, 0,
            sin(ik_joint_[2]),  cos(ik_joint_[2]),  0, 0,
            0,              0,  1, 0,
            0,              0,  0, 1;
    Eigen::Matrix4d J4;
    J4<< cos(ik_joint_[3]), -sin(ik_joint_[3]),  0, 0,
            sin(ik_joint_[3]),  cos(ik_joint_[3]),  0, 0,
            0,              0,  1, 0,
            0,              0,  0, 1;
    Eigen::Matrix4d J5;
    J5<< cos(ik_joint_[4]), -sin(ik_joint_[4]),  0, 0,
            sin(ik_joint_[4]),  cos(ik_joint_[4]),  0, 0,
            0,              0,  1, 0,
            0,              0,  0, 1;
    Eigen::Matrix4d J6;
    J6<< cos(ik_joint_[5]), -sin(ik_joint_[5]),  0, 0,
            sin(ik_joint_[5]),  cos(ik_joint_[5]),  0, 0,
            0,              0,  1, 0,
            0,              0,  0, 1;

    Eigen::Matrix4d J7;
    J7<< cos(ik_joint_[6]), -sin(ik_joint_[6]),  0, 0,
            sin(ik_joint_[6]),  cos(ik_joint_[6]),  0, 0,
            0,              0,  1, 0,
            0,              0,  0, 1;
    vector<Eigen::Vector3d> XYZ;
    Eigen::Matrix4d J_Zero;
    J_Zero<< 1, 0,  0, 0,
             0, 1,  0, 0,
             0, 0,  1, 0,
             0, 0,  0, 1;
    Eigen::Vector3d xyz_;
    Eigen::Matrix4d toe= A0_*J1;
    xyz_ = toe.block(0, 3, 3, 1);XYZ.push_back(xyz_);
     toe= A0_*J1*B0_*J2;
    xyz_ = toe.block(0, 3, 3, 1);XYZ.push_back(xyz_);
     toe= A0_*J1*B0_*J2*C0_*J3;
    xyz_ = toe.block(0, 3, 3, 1);XYZ.push_back(xyz_);
    toe= A0_*J1*B0_*J2*C0_*J3*D0_*J4;
    xyz_ = toe.block(0, 3, 3, 1);XYZ.push_back(xyz_);
     toe= A0_*J1*B0_*J2*C0_*J3*D0_*J4*E0_*J5;
    xyz_ = toe.block(0, 3, 3, 1);XYZ.push_back(xyz_);
    toe= A0_*J1*B0_*J2*C0_*J3*D0_*J4*E0_*J5*F0_*J6;
    xyz_ = toe.block(0, 3, 3, 1);XYZ.push_back(xyz_);
     toe= A0_*J1*B0_*J2*C0_*J3*D0_*J4*E0_*J5*F0_*J6*G0_*J7*H0_;
    xyz_ = toe.block(0, 3, 3, 1);XYZ.push_back(xyz_);

    

    double H[3]={0,0,0};
    vector<vector<double>> xyzhm;
    for (int i = 0; i < 7; i++)
    {
        
        vector<double> thisxyzhm;//x,y,z,Hx,Hy,Hz,maxx,maxy,maxz,minx,miny,minz
        // thisxyzhm.push_back(T1[i][3]);thisxyzhm.push_back(T1[i][7]);thisxyzhm.push_back(T1[i][11]);       
        // thisxyzhm.push_back(T1[i][3]-H[0]);thisxyzhm.push_back(T1[i][7]-H[1]);thisxyzhm.push_back(T1[i][11]-H[2]);
        thisxyzhm.push_back(XYZ[i](0));thisxyzhm.push_back(XYZ[i](1));thisxyzhm.push_back(XYZ[i](2));       
        thisxyzhm.push_back(XYZ[i](0)-H[0]);thisxyzhm.push_back(XYZ[i](1)-H[1]);thisxyzhm.push_back(XYZ[i](2)-H[2]);
        thisxyzhm.push_back(max(XYZ[i](0),H[0]));thisxyzhm.push_back(max(XYZ[i](1),H[1]));thisxyzhm.push_back(max(XYZ[i](2),H[2]));
        thisxyzhm.push_back(min(XYZ[i](0),H[0]));thisxyzhm.push_back(min(XYZ[i](1),H[1]));thisxyzhm.push_back(min(XYZ[i](2),H[2]));
        xyzhm.push_back(thisxyzhm);
        H[0]=thisxyzhm[0],H[1]=thisxyzhm[1],H[2]=thisxyzhm[2];
        //cout<<thisxyzhm[0]<<"     "<<thisxyzhm[1]<<"     "<<thisxyzhm[2]<<"     "<<ik_joint_[i]<<endl;
        //cout<<thisxyzhm[2]<<"     "<<H[2]<<"     ";
        
    }
    
    double X1,X2,X3,X4,X5,X6,X7,X8;
    double Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8;
    double Z1,Z2,Z3,Z4,Z5,Z6,Z7,Z8;

    for (auto item : obstacleList){
        //八个顶点
        X1=item[0],Y1=item[1],Z1=item[2];
        X2=X1+item[3],Y2=Y1,Z2=Z1,X3=X1+item[3],Y3=Y1+item[4],Z3=Z1;
        X4=X1,Y4=Y1+item[4],Z4=Z1,X5=X1,Y5=Y1,Z5=Z1+item[5];
        X6=X1+item[3],Y6=Y1,Z6=Z1+item[5],X7=X1+item[3],Y7=Y1+item[4],Z7=Z1+item[5];
        // for (int i = 0; i < 6; i++)
        // {
        //     cout<<item[i]<<"   ";
        // }
        // cout<<endl;
        X8=X1,Y8=Y1+item[4],Z8=Z1+item[5];
        // cout<<X1<<"  "<<Y1<<"  "<<Z1<<endl;
        // cout<<X2<<"  "<<Y2<<"  "<<Z2<<endl;
        // cout<<X3<<"  "<<Y3<<"  "<<Z3<<endl;
        // cout<<X4<<"  "<<Y4<<"  "<<Z4<<endl;
        // cout<<X5<<"  "<<Y5<<"  "<<Z5<<endl;
        // cout<<X6<<"  "<<Y6<<"  "<<Z6<<endl;
        // cout<<X7<<"  "<<Y7<<"  "<<Z7<<endl;
        // cout<<X8<<"  "<<Y8<<"  "<<Z8<<endl;
        vector<vector<double>>  P;
        //向xyz轴正方向膨胀
        double KX1[3]={X1-X2, Y1-Y2 ,Z1-Z2};double NK1=(r/sqrt(pow(KX1[0],2)+pow(KX1[1],2)+pow(KX1[2],2)));double NX1[3]={KX1[0]*NK1,KX1[1]*NK1,KX1[2]*NK1};
        double KY1[3]={X1-X4, Y1-Y4 ,Z1-Z4};double NY11=(r/sqrt(pow(KY1[0],2)+pow(KY1[1],2)+pow(KY1[2],2)));double NY1[3]={KY1[0]*NY11,KY1[1]*NY11,KY1[2]*NY11};
        double KZ1[3]={X1-X5, Y1-Y5 ,Z1-Z5};double NZ11=(r/sqrt(pow(KZ1[0],2)+pow(KZ1[1],2)+pow(KZ1[2],2)));double NZ1[3]={KZ1[0]*NZ11,KZ1[1]*NZ11,KZ1[2]*NZ11};
        vector<double> P1={X1+NX1[0]+NY1[0]+NZ1[0],Y1+NX1[1]+NY1[1]+NZ1[1],Z1+NX1[2]+NY1[2]+NZ1[2]};

        double KX2[3]={X2-X1, Y2-Y1 ,Z2-Z1};double NX22=(r/sqrt(pow(KX2[0],2)+pow(KX2[1],2)+pow(KX2[2],2)));double NX2[3]={KX2[0]*NX22,KX2[1]*NX22,KX2[2]*NX22};
        double KY2[3]={X2-X3, Y1-Y3 ,Z1-Z3};double NY22=(r/sqrt(pow(KY2[0],2)+pow(KY2[1],2)+pow(KY2[2],2)));double NY2[3]={KY2[0]*NY22,KY2[1]*NY22,KY2[2]*NY22};
        double KZ2[3]={X2-X6, Y1-Y6 ,Z1-Z6};double NZ22=(r/sqrt(pow(KZ2[0],2)+pow(KZ2[1],2)+pow(KZ2[2],2)));double NZ2[3]={KZ2[0]*NZ22,KZ2[1]*NZ22,KZ2[2]*NZ22};
        vector<double> P2={X2+NX2[0]+NY2[0]+NZ2[0],Y2+NX2[1]+NY2[1]+NZ2[1],Z2+NX2[2]+NY2[2]+NZ2[2]};

        double KX3[3]={X3-X4, Y3-Y4 ,Z3-Z4};double NX33=(r/sqrt(pow(KX3[0],2)+pow(KX3[1],2)+pow(KX3[2],2)));double NX3[3]={KX3[0]*NX33,KX3[1]*NX33,KX3[2]*NX33};
        double KY3[3]={X3-X2, Y3-Y2 ,Z3-Z2};double NY33=(r/sqrt(pow(KY3[0],2)+pow(KY3[1],2)+pow(KY3[2],2)));double NY3[3]={KY3[0]*NY33,KY3[1]*NY33,KY3[2]*NY33};
        double KZ3[3]={X3-X7, Y3-Y7 ,Z3-Z7};double NZ33=(r/sqrt(pow(KZ3[0],2)+pow(KZ3[1],2)+pow(KZ3[2],2)));double NZ3[3]={KZ3[0]*NZ33,KZ3[1]*NZ33,KZ3[2]*NZ33};
        vector<double> P3={X3+NX3[0]+NY3[0]+NZ3[0],Y3+NX3[1]+NY3[1]+NZ3[1],Z3+NX3[2]+NY3[2]+NZ3[2]};

        double KX4[3]={X4-X3, Y4-Y3 ,Z4-Z3};double NX44=(r/sqrt(pow(KX4[0],2)+pow(KX4[1],2)+pow(KX4[2],2)));double NX4[3]={KX4[0]*NX44,KX4[1]*NX44,KX4[2]*NX44};
        double KY4[3]={X4-X1, Y4-Y1 ,Z4-Z1};double NY44=(r/sqrt(pow(KY4[0],2)+pow(KY4[1],2)+pow(KY4[2],2)));double NY4[3]={KY4[0]*NY44,KY4[1]*NY44,KY4[2]*NY44};
        double KZ4[3]={X4-X8, Y4-Y8 ,Z4-Z8};double NZ44=(r/sqrt(pow(KZ4[0],2)+pow(KZ4[1],2)+pow(KZ4[2],2)));double NZ4[3]={KZ4[0]*NZ44,KZ4[1]*NZ44,KZ4[2]*NZ44};
        vector<double> P4={X4+NX4[0]+NY4[0]+NZ4[0],Y4+NX4[1]+NY4[1]+NZ4[1],Z4+NX4[2]+NY4[2]+NZ4[2]};

        double KX5[3]={X5-X6, Y5-Y6 ,Z5-Z6};double NX55=(r/sqrt(pow(KX5[0],2)+pow(KX5[1],2)+pow(KX5[2],2)));double NX5[3]={KX5[0]*NX55,KX5[1]*NX55,KX5[2]*NX55};
        double KY5[3]={X5-X8, Y5-Y8 ,Z5-Z8};double NY55=(r/sqrt(pow(KY5[0],2)+pow(KY5[1],2)+pow(KY5[2],2)));double NY5[3]={KY5[0]*NY55,KY5[1]*NY55,KY5[2]*NY55};
        double KZ5[3]={X5-X1, Y5-Y1 ,Z5-Z1};double NZ55=(r/sqrt(pow(KZ5[0],2)+pow(KZ5[1],2)+pow(KZ5[2],2)));double NZ5[3]={KZ5[0]*NZ55,KZ5[1]*NZ55,KZ5[2]*NZ55};
        vector<double> P5={X5+NX5[0]+NY5[0]+NZ5[0],Y5+NX5[1]+NY5[1]+NZ5[1],Z5+NX5[2]+NY5[2]+NZ5[2]};

        double KX6[3]={X6-X5, Y6-Y5 ,Z6-Z5};double NX66=(r/sqrt(pow(KX6[0],2)+pow(KX6[1],2)+pow(KX6[2],2)));double NX6[3]={KX6[0]*NX66,KX6[1]*NX66,KX6[2]*NX66};
        double KY6[3]={X6-X7, Y6-Y7 ,Z6-Z7};double NY66=(r/sqrt(pow(KY6[0],2)+pow(KY6[1],2)+pow(KY6[2],2)));double NY6[3]={KY6[0]*NY66,KY6[1]*NY66,KY6[2]*NY66};
        double KZ6[3]={X6-X2, Y6-Y2 ,Z6-Z2};double NZ66=(r/sqrt(pow(KZ6[0],2)+pow(KZ6[1],2)+pow(KZ6[2],2)));double NZ6[3]={KZ6[0]*NZ66,KZ6[1]*NZ66,KZ6[2]*NZ66};
        vector<double> P6={X6+NX6[0]+NY6[0]+NZ6[0],Y6+NX6[1]+NY6[1]+NZ6[1],Z6+NX6[2]+NY6[2]+NZ6[2]};

        double KX7[3]={X7-X8, Y7-Y8 ,Z7-Z8};double NX77=(r/sqrt(pow(KX7[0],2)+pow(KX7[1],2)+pow(KX7[2],2)));double NX7[3]={KX7[0]*NX77,KX7[1]*NX77,KX7[2]*NX77};
        double KY7[3]={X7-X6, Y7-Y6 ,Z7-Z6};double NY77=(r/sqrt(pow(KY7[0],2)+pow(KY7[1],2)+pow(KY7[2],2)));double NY7[3]={KY7[0]*NY77,KY7[1]*NY77,KY7[2]*NY77};
        double KZ7[3]={X7-X3, Y7-Y3 ,Z7-Z3};double NZ77=(r/sqrt(pow(KZ7[0],2)+pow(KZ7[1],2)+pow(KZ7[2],2)));double NZ7[3]={KZ7[0]*NZ77,KZ7[1]*NZ77,KZ7[2]*NZ77};
        vector<double> P7={X7+NX7[0]+NY7[0]+NZ7[0],Y7+NX7[1]+NY7[1]+NZ7[1],Z7+NX7[2]+NY7[2]+NZ7[2]};

        double KX8[3]={X8-X7, Y8-Y7 ,Z8-Z7};double NX88=(r/sqrt(pow(KX8[0],2)+pow(KX8[1],2)+pow(KX8[2],2)));double NX8[3]={KX8[0]*NX88,KX8[1]*NX88,KX8[2]*NX88};
        double KY8[3]={X8-X5, Y8-Y5 ,Z8-Z5};double NY88=(r/sqrt(pow(KY8[0],2)+pow(KY8[1],2)+pow(KY8[2],2)));double NY8[3]={KY8[0]*NY88,KY8[1]*NY88,KY8[2]*NY88};
        double KZ8[3]={X8-X4, Y8-Y4 ,Z8-Z4};double NZ88=(r/sqrt(pow(KZ8[0],2)+pow(KZ8[1],2)+pow(KZ8[2],2)));double NZ8[3]={KZ8[0]*NZ88,KZ8[1]*NZ88,KZ8[2]*NZ88};
        vector<double> P8={X8+NX8[0]+NY8[0]+NZ8[0],Y8+NX8[1]+NY8[1]+NZ8[1],Z8+NX8[2]+NY8[2]+NZ8[2]};

        // cout<<P1[0]<<"   "<<P1[1]<<"   "<<P1[2]<<"   "<<endl;
        // cout<<P2[0]<<"   "<<P2[1]<<"   "<<P2[2]<<"   "<<endl;
        // cout<<P3[0]<<"   "<<P3[1]<<"   "<<P3[2]<<"   "<<endl;
        // cout<<P4[0]<<"   "<<P4[1]<<"   "<<P4[2]<<"   "<<endl;
        // cout<<P5[0]<<"   "<<P5[1]<<"   "<<P5[2]<<"   "<<endl;
        // cout<<P6[0]<<"   "<<P6[1]<<"   "<<P6[2]<<"   "<<endl;
        // cout<<P7[0]<<"   "<<P7[1]<<"   "<<P7[2]<<"   "<<endl;
        // cout<<P8[0]<<"   "<<P8[1]<<"   "<<P8[2]<<"   "<<endl;
        // a=new node(-0.0651,0,0.6499);
        // c=new node(0,0.0651,0.6499);
        // b=new node(-0.0351,0.0351,0.7);


        //信息整合  计算杆的位置
        P.push_back(P1);P.push_back(P2);P.push_back(P3);P.push_back(P4);P.push_back(P5);P.push_back(P6);P.push_back(P7);P.push_back(P8);
        vector<vector<double>> M;
        vector<vector<int>> C;
        vector<int> c1={1,1,1,1,1,1};
        double K26[3]={X2-X6, Y1-Y6 ,Z1-Z6},K21[3]={X1-X2, Y1-Y2 ,Z1-Z2},K23[3]={X2-X3, Y1-Y3 ,Z1-Z3};
        double K34[3]={X3-X4, Y3-Y4 ,Z3-Z4},K48[3]={X8-X4, Y8-Y4 ,Z8-Z4},K14[3]={X4-X1, Y4-Y1 ,Z4-Z1},K56[3]={X5-X6, Y5-Y6 ,Z5-Z6},K58[3]={X8-X5, Y8-Y5 ,Z8-Z5};
        vector<double> M1={K23[0]-K21[0],K23[1]-K21[1],K23[2]-K21[2]},M2={K21[0]-K26[0],K21[1]-K26[1],K21[2]-K26[2]},M3={K26[0]-K23[0],K26[1]-K23[1],K26[2]-K23[2]};
        vector<double> M4={K34[0]-K48[0],K34[1]-K48[1],K34[2]-K48[2]},M5={K14[0]-K48[0],K14[1]-K48[1],K14[2]-K48[2]},M6={K56[0]-K58[0],K56[1]-K58[1],K56[2]-K58[2]};
        M.push_back(M1);M.push_back(M2);M.push_back(M3);M.push_back(M4);M.push_back(M5);M.push_back(M6);
        double MAXX=X2+NX2[0]+NY2[0]+NZ2[0],MINX=X1+NX1[0]+NY1[0]+NZ1[0],MAXY=Y3+NX3[1]+NY3[1]+NZ3[1],MINY=Y1+NX1[1]+NY1[1]+NZ1[1],MAXZ=Z5+NX5[2]+NY5[2]+NZ5[2],MINZ=Z1+NX1[2]+NY1[2]+NZ1[2];
        //判断每个杆是否与障碍物的六个平面相碰
        for(int i=0;i<7;i++){
            C.push_back(c1);
        }
        for(int i=0;i<7;i++){
            for(int j=0;j<6;j++){
                if((xyzhm[i][3]*M[j][0]+xyzhm[i][4]*M[j][1]+xyzhm[i][5]*M[j][2])==0){
                    C[i][j]=1;
                }
                else {//x,y,z,Hx,Hy,Hz,maxx,maxy,maxz,minx,miny,minz
                    double t=((P[j][0]-xyzhm[i][0])*M[j][0]+(P[j][1]-xyzhm[i][1])*M[j][1]+(P[j][2]-xyzhm[i][2])*M[j][2])/(xyzhm[i][3]*M[j][0]+xyzhm[i][4]*M[j][1]+xyzhm[i][5]*M[j][2]);
                    double x=xyzhm[i][0]+xyzhm[i][3]*t;double y=xyzhm[i][1]+xyzhm[i][4]*t;double z=xyzhm[i][2]+xyzhm[i][5]*t;
                    if((xyzhm[i][9]<=x&&xyzhm[i][6]>=x)&&(MINX<=x&&x<=MAXX)&&(xyzhm[i][10]<=y&&xyzhm[i][7]>=y)&&(MINY<=y&&y<=MAXY)&&(xyzhm[i][11]<=z&&xyzhm[i][8]>=z)&&(MINZ<=z&&z<=MAXZ)){
                        C[i][j]=0;
                        cout<<xyzhm[i][0]<<"   "<<xyzhm[i][1]<<"   "<<xyzhm[i][2]<<"   "<<xyzhm[i][6]<<"   "<<xyzhm[i][7]<<"   "<<xyzhm[i][8]<<"   "<<xyzhm[i][9]<<"   "<<xyzhm[i][10]<<"   "<<xyzhm[i][11]<<"   "<<endl;
                        cout<<i<<"   "<<j<<endl;
                        cout<<x<<"   "<<y<<"   "<<z<<"   "<<endl;
                        cout<<M[j][0]<<"   "<<M[j][1]<<"   "<<M[j][2]<<"   "<<endl<<endl;
                        break;
                    }else C[i][j]=1;
                }

            }
        }
        int a1=1;
        for (int i = 0; i<7;i++)
        {
           for(int j=0;j<6;j++){
            a1*=C[i][j];
           }
        }
        if(a1==0) return false;
    }
        
    return true;
}

bool RRT::collisionCheck1(double r,vector<node*>Node1) {

    double X1,X2,X3,X4,X5,X6,X7,X8;
    double Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8;
    double Z1,Z2,Z3,Z4,Z5,Z6,Z7,Z8;
    double H[3]={0,0,0};
    vector<vector<double>> xyzhm;
    for (int i = 0; i < Node1.size(); i++)
    {
        
        vector<double> thisxyzhm;//x,y,z,Hx,Hy,Hz,maxx,maxy,maxz,minx,miny,minz
        thisxyzhm.push_back(Node1[i]->getX());thisxyzhm.push_back(Node1[i]->getY());thisxyzhm.push_back(Node1[i]->getZ());
        thisxyzhm.push_back(Node1[i]->getX()-H[0]);thisxyzhm.push_back(Node1[i]->getY()-H[1]);thisxyzhm.push_back(Node1[i]->getZ()-H[2]);
        thisxyzhm.push_back(max(Node1[i]->getX(),H[0]));thisxyzhm.push_back(max(Node1[i]->getY(),H[1]));thisxyzhm.push_back(max(Node1[i]->getZ(),H[2]));
        thisxyzhm.push_back(min(Node1[i]->getX(),H[0]));thisxyzhm.push_back(min(Node1[i]->getY(),H[1]));thisxyzhm.push_back(min(Node1[i]->getZ(),H[2]));
        xyzhm.push_back(thisxyzhm);
        H[0]=Node1[i]->getX(),H[1]=Node1[i]->getY(),H[2]=Node1[i]->getZ();
        
    }
    

    for (auto item : obstacleList){
        //八个顶点
        X1=item[0],Y1=item[1],Z1=item[2];
        X2=X1+item[3],Y2=Y1,Z2=Z1,X3=X1+item[3],Y3=Y1+item[4],Z3=Z1;
        X4=X1,Y4=Y1+item[4],Z4=Z1,X5=X1,Y5=Y1,Z5=Z1+item[5];
        X6=X1+item[3],Y6=Y1,Z6=Z1+item[5],X7=X1+item[3],Y7=Y1+item[4],Z7=Z1+item[5];
        X8=X1,Y8=Y1+item[4],Z8=Z1+item[5];
        vector<vector<double>>  P;
        //向xyz轴正方向膨胀
        double KX1[3]={X1-X2, Y1-Y2 ,Z1-Z2};double NK1=(r/sqrt(pow(KX1[0],2)+pow(KX1[1],2)+pow(KX1[2],2)));double NX1[3]={KX1[0]*NK1,KX1[1]*NK1,KX1[2]*NK1};
        double KY1[3]={X1-X4, Y1-Y4 ,Z1-Z4};double NY11=(r/sqrt(pow(KY1[0],2)+pow(KY1[1],2)+pow(KY1[2],2)));double NY1[3]={KY1[0]*NY11,KY1[1]*NY11,KY1[2]*NY11};
        double KZ1[3]={X1-X5, Y1-Y5 ,Z1-Z5};double NZ11=(r/sqrt(pow(KZ1[0],2)+pow(KZ1[1],2)+pow(KZ1[2],2)));double NZ1[3]={KZ1[0]*NZ11,KZ1[1]*NZ11,KZ1[2]*NZ11};
        vector<double> P1={X1+NX1[0]+NY1[0]+NZ1[0],Y1+NX1[1]+NY1[1]+NZ1[1],Z1+NX1[2]+NY1[2]+NZ1[2]};

        double KX2[3]={X2-X1, Y2-Y1 ,Z2-Z1};double NX22=(r/sqrt(pow(KX2[0],2)+pow(KX2[1],2)+pow(KX2[2],2)));double NX2[3]={KX2[0]*NX22,KX2[1]*NX22,KX2[2]*NX22};
        double KY2[3]={X2-X3, Y1-Y3 ,Z1-Z3};double NY22=(r/sqrt(pow(KY2[0],2)+pow(KY2[1],2)+pow(KY2[2],2)));double NY2[3]={KY2[0]*NY22,KY2[1]*NY22,KY2[2]*NY22};
        double KZ2[3]={X2-X6, Y1-Y6 ,Z1-Z6};double NZ22=(r/sqrt(pow(KZ2[0],2)+pow(KZ2[1],2)+pow(KZ2[2],2)));double NZ2[3]={KZ2[0]*NZ22,KZ2[1]*NZ22,KZ2[2]*NZ22};
        vector<double> P2={X2+NX2[0]+NY2[0]+NZ2[0],Y2+NX2[1]+NY2[1]+NZ2[1],Z2+NX2[2]+NY2[2]+NZ2[2]};

        double KX3[3]={X3-X4, Y3-Y4 ,Z3-Z4};double NX33=(r/sqrt(pow(KX3[0],2)+pow(KX3[1],2)+pow(KX3[2],2)));double NX3[3]={KX3[0]*NX33,KX3[1]*NX33,KX3[2]*NX33};
        double KY3[3]={X3-X2, Y3-Y2 ,Z3-Z2};double NY33=(r/sqrt(pow(KY3[0],2)+pow(KY3[1],2)+pow(KY3[2],2)));double NY3[3]={KY3[0]*NY33,KY3[1]*NY33,KY3[2]*NY33};
        double KZ3[3]={X3-X7, Y3-Y7 ,Z3-Z7};double NZ33=(r/sqrt(pow(KZ3[0],2)+pow(KZ3[1],2)+pow(KZ3[2],2)));double NZ3[3]={KZ3[0]*NZ33,KZ3[1]*NZ33,KZ3[2]*NZ33};
        vector<double> P3={X3+NX3[0]+NY3[0]+NZ3[0],Y3+NX3[1]+NY3[1]+NZ3[1],Z3+NX3[2]+NY3[2]+NZ3[2]};

        double KX4[3]={X4-X3, Y4-Y3 ,Z4-Z3};double NX44=(r/sqrt(pow(KX4[0],2)+pow(KX4[1],2)+pow(KX4[2],2)));double NX4[3]={KX4[0]*NX44,KX4[1]*NX44,KX4[2]*NX44};
        double KY4[3]={X4-X1, Y4-Y1 ,Z4-Z1};double NY44=(r/sqrt(pow(KY4[0],2)+pow(KY4[1],2)+pow(KY4[2],2)));double NY4[3]={KY4[0]*NY44,KY4[1]*NY44,KY4[2]*NY44};
        double KZ4[3]={X4-X8, Y4-Y8 ,Z4-Z8};double NZ44=(r/sqrt(pow(KZ4[0],2)+pow(KZ4[1],2)+pow(KZ4[2],2)));double NZ4[3]={KZ4[0]*NZ44,KZ4[1]*NZ44,KZ4[2]*NZ44};
        vector<double> P4={X4+NX4[0]+NY4[0]+NZ4[0],Y4+NX4[1]+NY4[1]+NZ4[1],Z4+NX4[2]+NY4[2]+NZ4[2]};

        double KX5[3]={X5-X6, Y5-Y6 ,Z5-Z6};double NX55=(r/sqrt(pow(KX5[0],2)+pow(KX5[1],2)+pow(KX5[2],2)));double NX5[3]={KX5[0]*NX55,KX5[1]*NX55,KX5[2]*NX55};
        double KY5[3]={X5-X8, Y5-Y8 ,Z5-Z8};double NY55=(r/sqrt(pow(KY5[0],2)+pow(KY5[1],2)+pow(KY5[2],2)));double NY5[3]={KY5[0]*NY55,KY5[1]*NY55,KY5[2]*NY55};
        double KZ5[3]={X5-X1, Y5-Y1 ,Z5-Z1};double NZ55=(r/sqrt(pow(KZ5[0],2)+pow(KZ5[1],2)+pow(KZ5[2],2)));double NZ5[3]={KZ5[0]*NZ55,KZ5[1]*NZ55,KZ5[2]*NZ55};
        vector<double> P5={X5+NX5[0]+NY5[0]+NZ5[0],Y5+NX5[1]+NY5[1]+NZ5[1],Z5+NX5[2]+NY5[2]+NZ5[2]};

        double KX6[3]={X6-X5, Y6-Y5 ,Z6-Z5};double NX66=(r/sqrt(pow(KX6[0],2)+pow(KX6[1],2)+pow(KX6[2],2)));double NX6[3]={KX6[0]*NX66,KX6[1]*NX66,KX6[2]*NX66};
        double KY6[3]={X6-X7, Y6-Y7 ,Z6-Z7};double NY66=(r/sqrt(pow(KY6[0],2)+pow(KY6[1],2)+pow(KY6[2],2)));double NY6[3]={KY6[0]*NY66,KY6[1]*NY66,KY6[2]*NY66};
        double KZ6[3]={X6-X2, Y6-Y2 ,Z6-Z2};double NZ66=(r/sqrt(pow(KZ6[0],2)+pow(KZ6[1],2)+pow(KZ6[2],2)));double NZ6[3]={KZ6[0]*NZ66,KZ6[1]*NZ66,KZ6[2]*NZ66};
        vector<double> P6={X6+NX6[0]+NY6[0]+NZ6[0],Y6+NX6[1]+NY6[1]+NZ6[1],Z6+NX6[2]+NY6[2]+NZ6[2]};

        double KX7[3]={X7-X8, Y7-Y8 ,Z7-Z8};double NX77=(r/sqrt(pow(KX7[0],2)+pow(KX7[1],2)+pow(KX7[2],2)));double NX7[3]={KX7[0]*NX77,KX7[1]*NX77,KX7[2]*NX77};
        double KY7[3]={X7-X6, Y7-Y6 ,Z7-Z6};double NY77=(r/sqrt(pow(KY7[0],2)+pow(KY7[1],2)+pow(KY7[2],2)));double NY7[3]={KY7[0]*NY77,KY7[1]*NY77,KY7[2]*NY77};
        double KZ7[3]={X7-X3, Y7-Y3 ,Z7-Z3};double NZ77=(r/sqrt(pow(KZ7[0],2)+pow(KZ7[1],2)+pow(KZ7[2],2)));double NZ7[3]={KZ7[0]*NZ77,KZ7[1]*NZ77,KZ7[2]*NZ77};
        vector<double> P7={X7+NX7[0]+NY7[0]+NZ7[0],Y7+NX7[1]+NY7[1]+NZ7[1],Z7+NX7[2]+NY7[2]+NZ7[2]};

        double KX8[3]={X8-X7, Y8-Y7 ,Z8-Z7};double NX88=(r/sqrt(pow(KX8[0],2)+pow(KX8[1],2)+pow(KX8[2],2)));double NX8[3]={KX8[0]*NX88,KX8[1]*NX88,KX8[2]*NX88};
        double KY8[3]={X8-X5, Y8-Y5 ,Z8-Z5};double NY88=(r/sqrt(pow(KY8[0],2)+pow(KY8[1],2)+pow(KY8[2],2)));double NY8[3]={KY8[0]*NY88,KY8[1]*NY88,KY8[2]*NY88};
        double KZ8[3]={X8-X4, Y8-Y4 ,Z8-Z4};double NZ88=(r/sqrt(pow(KZ8[0],2)+pow(KZ8[1],2)+pow(KZ8[2],2)));double NZ8[3]={KZ8[0]*NZ88,KZ8[1]*NZ88,KZ8[2]*NZ88};
        vector<double> P8={X8+NX8[0]+NY8[0]+NZ8[0],Y8+NX8[1]+NY8[1]+NZ8[1],Z8+NX8[2]+NY8[2]+NZ8[2]};
        
        P.push_back(P1);P.push_back(P2);P.push_back(P3);P.push_back(P4);P.push_back(P5);P.push_back(P6);P.push_back(P7);P.push_back(P8);
        vector<vector<double>> M;
        vector<vector<int>> C;
        vector<int> c1={1,1,1,1,1,1};

        double K26[3]={X2-X6, Y1-Y6 ,Z1-Z6},K21[3]={X1-X2, Y1-Y2 ,Z1-Z2},K23[3]={X2-X3, Y1-Y3 ,Z1-Z3};
        double K34[3]={X3-X4, Y3-Y4 ,Z3-Z4},K48[3]={X8-X4, Y8-Y4 ,Z8-Z4},K14[3]={X4-X1, Y4-Y1 ,Z4-Z1},K56[3]={X5-X6, Y5-Y6 ,Z5-Z6},K58[3]={X8-X5, Y8-Y5 ,Z8-Z5};
        vector<double> M1={K23[0]-K21[0],K23[1]-K21[1],K23[2]-K21[2]},M2={K21[0]-K26[0],K21[1]-K26[1],K21[2]-K26[2]},M3={K26[0]-K23[0],K26[1]-K23[1],K26[2]-K23[2]};
        vector<double> M4={K34[0]-K48[0],K34[1]-K48[1],K34[2]-K48[2]},M5={K14[0]-K48[0],K14[1]-K48[1],K14[2]-K48[2]},M6={K56[0]-K58[0],K56[1]-K58[1],K56[2]-K58[2]};
        M.push_back(M1);M.push_back(M2);M.push_back(M3);M.push_back(M4);M.push_back(M5);M.push_back(M6);
        double MAXX=X2+NX2[0]+NY2[0]+NZ2[0],MINX=X1+NX1[0]+NY1[0]+NZ1[0],MAXY=Y3+NX3[1]+NY3[1]+NZ3[1],MINY=Y1+NX1[1]+NY1[1]+NZ1[1],MAXZ=Z5+NX5[2]+NY5[2]+NZ5[2],MINZ=Z1+NX1[2]+NY1[2]+NZ1[2];
        for(int i=0;i<Node1.size();i++){
            C.push_back(c1);
        }
        for(int i=0;i<Node1.size();i++){
            for(int j=0;j<6;j++){
                if((xyzhm[i][3]*M[j][0]+xyzhm[i][4]*M[j][1]+xyzhm[i][5]*M[j][2])==0){
                    C[i][j]=1;
                }
                else {//x,y,z,Hx,Hy,Hz,maxx,maxy,maxz,minx,miny,minz
                    double t=((P[j][0]-xyzhm[i][0])*M[j][0]+(P[j][1]-xyzhm[i][1])*M[j][1]+(P[j][2]-xyzhm[i][2])*M[j][2])/(xyzhm[i][3]*M[j][0]+xyzhm[i][4]*M[j][1]+xyzhm[i][5]*M[j][2]);
                    double x=xyzhm[i][0]+xyzhm[i][3]*t;double y=xyzhm[i][1]+xyzhm[i][4]*t;double z=xyzhm[i][2]+xyzhm[i][5]*t;
                    if((xyzhm[i][9]<=x&&xyzhm[i][6]>=x)&&(MINX<=x&&x<=MAXX)&&(xyzhm[i][10]<=y&&xyzhm[i][7]>=y)&&(MINY<=y&&y<=MAXY)&&(xyzhm[i][11]<=z&&xyzhm[i][8]>=z)&&(MINZ<=z&&z<=MAXZ)){
                        C[i][j]=0;
                        break;
                    }else C[i][j]=1;
                }

            }
        }
        int a1=1;
        for (int i = 0; i<Node1.size();i++)
        {
           for(int j=0;j<6;j++){
            a1*=C[i][j];
           }
        }
        if(a1==0) return false;
    }
        
    return true;
}


//       static const double b_dh[28] = {0.0,
//                                   0.0,
//                                   0.0,
//                                   0.0,
//                                   0.0,
//                                   0.0,
//                                   0.0,
//                                   -1.5707963267948966,
//                                   1.5707963267948966,
//                                   1.5707963267948966,
//                                   -1.5707963267948966,
//                                   -1.5707963267948966,
//                                   1.5707963267948966,
//                                   0.0,
//                                   0.1678,
//                                   0.0,
//                                   0.2042,
//                                   0.0,
//                                   0.2041,
//                                   0.0,
//                                   0.1621,
//                                   0.0,
//                                   0.0,
//                                   0.0,
//                                   0.0,
//                                   0.0,
//                                   0.0,
//                                   0.0};
//   static const double xs0[3] = {0.0, 0.0, 0.1678};
//   double tr[112];
//   double dh[28];
//   double b_tr[16];
//   double c_T_tmp[16];
//   double tmp[16];
//   double xsw[3];
//   double T_tmp;
//   double a_tmp;
//   double absxk;
//   double b_T_tmp;
//   double b_n;
//   double cos_ns;
//   double n;
//   double scale;
//   double t;
//   double v1_idx_0;
//   double v1_idx_1;
//   double v1_idx_2;
//   double v_idx_0;
//   double v_idx_1;
//   double v_idx_2;
//   double vc_idx_0;
//   double vc_idx_1;
//   double vc_idx_2;
//   double xsw_idx_0;
//   double xsw_idx_1;
//   double xsw_idx_2;
//   int b_i;
//   int b_tr_tmp;
//   int elbow;
//   int i;
//   int i1;
//   int tr_tmp;
//   /* Tolerance */
//   /* Robot parameters */
//   /* Link length */
//   /* Denavit-Hartenberg parameters 7 DoF */
//   /* DH: [a, alpha,    d, theta]  */
//   memcpy(&dh[0], &b_dh[0], 28U * sizeof(double));
//   /* Number of joints */
//   /* Robot configuration */
//   /* RCONF Summary of this function goes here */
//   /*    Detailed explanation goes here */
//   elbow = 1;
//   if (((((ik_joint_[1] < 0.0) + ((unsigned long long)(ik_joint_[3] < 0.0) << 1)) +
//         ((unsigned long long)(ik_joint_[5] < 0.0) << 2)) &
//        2ULL) != 0.0) {
//     elbow = -1;
//   }
//   /* Assign joint values to the theta column of the DH parameters */
//   for (i = 0; i < 7; i++) {
//     dh[i + 21] = ik_joint_[i];
//   }
//   /* Store transformations from the base reference frame to the index joint */
//   /*  e.g: tr(:,:,2) is the T02 -> transformation from base to joint 2 (DH
//    * table) */
//   memset(&tr[0], 0, 112U * sizeof(double));
//   /* Rotation Matrix applied with Denavit-Hartenberg parameters [same as (3)] */
//   /* R = [Xx,Yx,Zx,   --  Xx = cos(theta), Yx = -sin(theta) * cos(alpha), Zx =
//    * sin(theta) * sin(alpha) */
//   /*      Xy,YY,Zy,   --  Xy = sin(theta), Yy =  cos(theta) * cos(alpha), Zy =
//    * -cos(theta) * sin(alpha) */
//   /*      Xz,Yz,Zz];  --  Xz = 0.0,        Yz =  sin(alpha),              Zz =
//    * cos(alpha)    */
//   tmp[2] = 0.0;
//   tmp[3] = 0.0;
//   tmp[7] = 0.0;
//   tmp[11] = 0.0;
//   tmp[15] = 1.0;
//   for (b_i = 0; b_i < 7; b_i++) {
//     n = dh[b_i + 21];
//     v_idx_1 = cos(n);
//     b_n = sin(n);
//     n = dh[b_i + 7];
//     v_idx_0 = cos(n);
//     n = sin(n);
//     tmp[0] = v_idx_1;
//     tmp[4] = -b_n * v_idx_0;
//     tmp[8] = b_n * n;
//     v1_idx_2 = dh[b_i];
//     tmp[12] = v1_idx_2 * v_idx_1;
//     tmp[1] = b_n;
//     tmp[5] = v_idx_1 * v_idx_0;
//     tmp[9] = -v_idx_1 * n;
//     tmp[13] = v1_idx_2 * b_n;
//     tmp[6] = n;
//     tmp[10] = v_idx_0;
//     tmp[14] = dh[b_i + 14];
//     if (b_i + 1 == 1) {
//       for (i = 0; i < 4; i++) {
//         tr_tmp = i << 2;
//         tr[tr_tmp] = tmp[tr_tmp];
//         tr[tr_tmp + 1] = tmp[tr_tmp + 1];
//         tr[tr_tmp + 2] = tmp[tr_tmp + 2];
//         tr[tr_tmp + 3] = tmp[tr_tmp + 3];
//       }
//     } else {
//       for (i = 0; i < 4; i++) {
//         tr_tmp = i + ((b_i - 1) << 4);
//         for (b_tr_tmp = 0; b_tr_tmp < 4; b_tr_tmp++) {
//           i1 = b_tr_tmp << 2;
//           b_tr[i + i1] =
//               ((tr[tr_tmp] * tmp[i1] + tr[tr_tmp + 4] * tmp[i1 + 1]) +
//                tr[tr_tmp + 8] * tmp[i1 + 2]) +
//               tr[tr_tmp + 12] * tmp[i1 + 3];
//         }
//       }
//       for (i = 0; i < 4; i++) {
//         tr_tmp = i << 2;
//         b_tr_tmp = tr_tmp + (b_i << 4);
//         tr[b_tr_tmp] = b_tr[tr_tmp];
//         tr[b_tr_tmp + 1] = b_tr[tr_tmp + 1];
//         tr[b_tr_tmp + 2] = b_tr[tr_tmp + 2];
//         tr[b_tr_tmp + 3] = b_tr[tr_tmp + 3];
//         //cout<<tr[b_tr_tmp]<<"    "<<tr[b_tr_tmp + 1]<<"    "<<tr[b_tr_tmp + 2]<<"    "<<tr[b_tr_tmp + 3]<<"    "<<endl;
//       }
//     }
//   }
//   for (i = 0; i < 7; i++) {
//     vector<double> t11;
//     t11={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//     for (int j = 0; j < 4; j++)
//     {      
//         tr_tmp = j << 2;
//     t11[tr_tmp] = tr[tr_tmp + 16*i];
//     t11[tr_tmp + 1] = tr[tr_tmp +1+ 16*i];
//     t11[tr_tmp + 2] = tr[tr_tmp +2+ 16*i];
//     t11[tr_tmp + 3] = tr[tr_tmp +3+ 16*i];        
//     }
//     for (int j = 0; j < 4; j++)
//     {      
//         cout<<t11[j]<<"    "  <<t11[j+1]<<"    "  <<t11[j+2]<<"    "  <<t11[j+3]<<"    "  <<endl;   
//     }
//     T1.push_back(t11);
//   }