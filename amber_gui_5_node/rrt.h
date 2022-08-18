#ifndef _RRT_H
#define _RRT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <random>

using namespace std;
    // 0.0825
    // 0.1678   0.0853
    // 0.2967   0.1289
    // 0.382    0.0853
    // 0.5071   0.1251
    // 0.5861   0.0790
    // 0.7097   0.1236
class node {
private:
    double x, y,z;                // �ڵ�����
    vector<double> pathX, pathY, pathZ;// ·��
    node* parent;              // ���ڵ�
    double cost;
public:
    node(double _x, double _y, double _z);
    double getX();
    double getY();
    double getZ();
    void setX(double);
    void setY(double);
    void setZ(double);
    void setParent(node*);
    node* getParent();
};

class RRT {
private:
    node* startNode, * goalNode;        // ��ʼ�ڵ��Ŀ��ڵ�
    vector<vector<double>> obstacleList; // 障碍物距离基坐标系最近的坐标+x长+y宽+z高
    vector<node*> nodeList;             // 
    double stepSize;                     // ����步长
    double searchsize;//机械臂总长
    double goal_sample_rate;

    random_device goal_rd;
    mt19937 goal_gen;
    uniform_int_distribution<int> goal_dis;

    random_device area_rd;
    mt19937 area_gen;
    uniform_real_distribution<double> area_dis;
public:
    RRT(node*, node*, const vector<vector<double>>&, double , int,double);
    node* getNearestNode(const vector<double>&);
    bool collisionCheck2(double ,double[7]);
    bool collisionCheck1(double ,vector<node*>);
    vector<node*> planning();
    vector<vector<double>> check();
};

#endif
