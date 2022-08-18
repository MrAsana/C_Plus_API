#include <iostream>
#include <vector>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"
#include <cmath>
#include "udp.h"
#include "signal.h"
using namespace std;
        typedef struct{
            int number = 0;
        double LineLength = 0;
        double dx, dy, dz = 0;
            vector<double> x_start ;
            vector<double> y_start ;
            vector<double> z_start ;
        } LinearPathnumbers;       
            void clear();
            void Plan_path(double xs,double ys,double zs,double xe,double ye,double ze,double speed);
            double* Circelcenter_or_Rad(double[] ,double[] ,double[] );
