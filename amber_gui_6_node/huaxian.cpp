#include <iostream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "string.h"
#include <cmath>
#include "huaxian.h"
#include "signal.h"


LinearPathnumbers LinearPathnumberss;
double* Circelcenter_or_Rad(double p1[] ,double p2[] ,double p3[] )//形参输入采用字符串分割方法" ";
        {
            double v1[3],v2[3],v1n[3],v2n[3],nv[3],U[3],cW[3],W[3],V[3];
            double bx,cx,cy,h,m_rad;
            double* m_center=new double [3];
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
                }
                m_rad = sqrt((m_center[0] - p1[0]) * (m_center[0] - p1[0]) + (m_center[1] - p1[1]) * (m_center[1] - p1[1]) + (m_center[2] - p1[2]) * (m_center[2] - p1[2]));
            }
            return m_center;
        }
         void Clear()
        {
            LinearPathnumberss.x_start.clear();
            LinearPathnumberss.y_start.clear();
            LinearPathnumberss.z_start.clear();
        };
        void Plan_path(double xs,double ys,double zs,double xe,double ye,double ze,double speed)
        {
            Clear();      
            LinearPathnumberss.x_start.push_back(xs);    
            LinearPathnumberss.y_start.push_back(ys);
            LinearPathnumberss.z_start.push_back(zs);
            LinearPathnumberss.LineLength = sqrt((xe - xs) * (xe - xs) + (ye - ys) * (ye - ys) + (ze - zs) * (ze - zs));
            LinearPathnumberss.number = (int)LinearPathnumberss.LineLength / speed;
            LinearPathnumberss.dx = (xe - xs) / LinearPathnumberss.number;
            LinearPathnumberss.dy = (ye - ys) / LinearPathnumberss.number;
            LinearPathnumberss.dz = (ze - zs) / LinearPathnumberss.number;
            for(int i = 0; i < LinearPathnumberss.number; i++)
            {
                LinearPathnumberss.x_start.push_back(LinearPathnumberss.x_start[i] + LinearPathnumberss.dx);
                LinearPathnumberss.y_start.push_back(LinearPathnumberss.y_start[i] + LinearPathnumberss.dy);
                LinearPathnumberss.z_start.push_back(LinearPathnumberss.z_start[i] + LinearPathnumberss.dz);
            }
        }