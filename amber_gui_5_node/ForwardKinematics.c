/*
 * File: ForwardKinematics.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 25-Jan-2022 17:39:36
 */

/* Include Files */
#include "ForwardKinematics.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * sqrt(y * y + 1.0);
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

/*
 * FORWARDKINEMATICS
 *
 *  Calculates the forward kinematics for a KUKA LBR iiwa manipulator.
 *
 *  Input:    joints  - Joint values size(1,7).
 *  Output:   pose    - Homogeneous Matrix size(4,4)
 *            nsparam - Arm Angle
 *            rconf   - Robot Configuration 8-bit number
 *
 * Arguments    : const double joints[7]
 *                double pose[16]
 *                double *nsparam
 * Return Type  : void
 */
void ForwardKinematics(const double joints[7], double pose[16], double *nsparam)
{
  static const double b_dh[28] = {0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  -1.5707963267948966,
                                  1.5707963267948966,
                                  1.5707963267948966,
                                  -1.5707963267948966,
                                  -1.5707963267948966,
                                  1.5707963267948966,
                                  0.0,
                                  0.1678,
                                  0.0,
                                  0.2042,
                                  0.0,
                                  0.2041,
                                  0.0,
                                  0.1621,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0};
  static const double b[16] = {1.0,
                               0.0,
                               0.0,
                               0.0,
                               -0.0,
                               6.123233995736766E-17,
                               1.0,
                               0.0,
                               0.0,
                               -1.0,
                               6.123233995736766E-17,
                               0.0,
                               0.0,
                               0.0,
                               0.2042,
                               1.0};
  static const double xs0[3] = {0.0, 0.0, 0.1678};
  double tr[112];
  double dh[28];
  double b_tr[16];
  double c_T_tmp[16];
  double tmp[16];
  double xsw[3];
  double T_tmp;
  double a_tmp;
  double absxk;
  double b_T_tmp;
  double b_n;
  double cos_ns;
  double n;
  double scale;
  double t;
  double v1_idx_0;
  double v1_idx_1;
  double v1_idx_2;
  double v_idx_0;
  double v_idx_1;
  double v_idx_2;
  double vc_idx_0;
  double vc_idx_1;
  double vc_idx_2;
  double xsw_idx_0;
  double xsw_idx_1;
  double xsw_idx_2;
  int b_i;
  int b_tr_tmp;
  int elbow;
  int i;
  int i1;
  int tr_tmp;
  /* Tolerance */
  /* Robot parameters */
  /* Link length */
  /* Denavit-Hartenberg parameters 7 DoF */
  /* DH: [a, alpha,    d, theta]  */
  memcpy(&dh[0], &b_dh[0], 28U * sizeof(double));
  /* Number of joints */
  /* Robot configuration */
  /* RCONF Summary of this function goes here */
  /*    Detailed explanation goes here */
  elbow = 1;
  if (((((joints[1] < 0.0) + ((unsigned long long)(joints[3] < 0.0) << 1)) +
        ((unsigned long long)(joints[5] < 0.0) << 2)) &
       2ULL) != 0.0) {
    elbow = -1;
  }
  /* Assign joint values to the theta column of the DH parameters */
  for (i = 0; i < 7; i++) {
    dh[i + 21] = joints[i];
  }
  /* Store transformations from the base reference frame to the index joint */
  /*  e.g: tr(:,:,2) is the T02 -> transformation from base to joint 2 (DH
   * table) */
  memset(&tr[0], 0, 112U * sizeof(double));
  /* Rotation Matrix applied with Denavit-Hartenberg parameters [same as (3)] */
  /* R = [Xx,Yx,Zx,   --  Xx = cos(theta), Yx = -sin(theta) * cos(alpha), Zx =
   * sin(theta) * sin(alpha) */
  /*      Xy,YY,Zy,   --  Xy = sin(theta), Yy =  cos(theta) * cos(alpha), Zy =
   * -cos(theta) * sin(alpha) */
  /*      Xz,Yz,Zz];  --  Xz = 0.0,        Yz =  sin(alpha),              Zz =
   * cos(alpha)    */
  tmp[2] = 0.0;
  tmp[3] = 0.0;
  tmp[7] = 0.0;
  tmp[11] = 0.0;
  tmp[15] = 1.0;
  for (b_i = 0; b_i < 7; b_i++) {
    n = dh[b_i + 21];
    v_idx_1 = cos(n);
    b_n = sin(n);
    n = dh[b_i + 7];
    v_idx_0 = cos(n);
    n = sin(n);
    tmp[0] = v_idx_1;
    tmp[4] = -b_n * v_idx_0;
    tmp[8] = b_n * n;
    v1_idx_2 = dh[b_i];
    tmp[12] = v1_idx_2 * v_idx_1;
    tmp[1] = b_n;
    tmp[5] = v_idx_1 * v_idx_0;
    tmp[9] = -v_idx_1 * n;
    tmp[13] = v1_idx_2 * b_n;
    tmp[6] = n;
    tmp[10] = v_idx_0;
    tmp[14] = dh[b_i + 14];
    if (b_i + 1 == 1) {
      for (i = 0; i < 4; i++) {
        tr_tmp = i << 2;
        tr[tr_tmp] = tmp[tr_tmp];
        tr[tr_tmp + 1] = tmp[tr_tmp + 1];
        tr[tr_tmp + 2] = tmp[tr_tmp + 2];
        tr[tr_tmp + 3] = tmp[tr_tmp + 3];
      }
    } else {
      for (i = 0; i < 4; i++) {
        tr_tmp = i + ((b_i - 1) << 4);
        for (b_tr_tmp = 0; b_tr_tmp < 4; b_tr_tmp++) {
          i1 = b_tr_tmp << 2;
          b_tr[i + i1] =
              ((tr[tr_tmp] * tmp[i1] + tr[tr_tmp + 4] * tmp[i1 + 1]) +
               tr[tr_tmp + 8] * tmp[i1 + 2]) +
              tr[tr_tmp + 12] * tmp[i1 + 3];
        }
      }
      for (i = 0; i < 4; i++) {
        tr_tmp = i << 2;
        b_tr_tmp = tr_tmp + (b_i << 4);
        tr[b_tr_tmp] = b_tr[tr_tmp];
        tr[b_tr_tmp + 1] = b_tr[tr_tmp + 1];
        tr[b_tr_tmp + 2] = b_tr[tr_tmp + 2];
        tr[b_tr_tmp + 3] = b_tr[tr_tmp + 3];
      }
    }
  }
  /*  shoulder position from base */
  /*  elbow position from base */
  /*  wrist position from base */
  xsw_idx_0 = tr[92] - tr[12];
  xsw_idx_1 = tr[93] - tr[13];
  xsw_idx_2 = tr[94] - tr[14];
  /*  wrist position from shoulder */
  for (i = 0; i < 4; i++) {
    tr_tmp = i << 2;
    pose[tr_tmp] = tr[tr_tmp + 96];
    pose[tr_tmp + 1] = tr[tr_tmp + 97];
    pose[tr_tmp + 2] = tr[tr_tmp + 98];
    pose[tr_tmp + 3] = tr[tr_tmp + 99];
  }
  /* end-effector transformation from base */
  /* Calculate the nsparam - Arm Angle */
  /*  The arm plane is defined by the plane formed by the xs, xe and xw points
   */
  /*  The reference plane is defined by the xs and xw and a xe0 (explained
   * below) */
  /*  . A virtual robotic manipulator is created from the KUKA LBR iiwa
   * manipulator */
  /*    structure. Equal in everything least the 3rd joint, which is fixed as 0.
   */
  /*  . Now we compute the Inverse Kinematics to place the virtual robot in the
   */
  /*    same pose as the real robot. */
  /*  . The elbow position of this virtual robot is xe0. Thus, with xs, xw and
   */
  /*    xe0 we form the reference plane */
  /*  REFERENCEPLANE */
  /*   */
  /*  Calculates the vector normal to the reference plane. */
  /*  A virtual robotic manipulator is created with the same structure as the */
  /*  real robot, but keeping its joint3 = 0. */
  /*  */
  /*  From the current end-effector position and robot configuration, it */
  /*  computes the set of joint positions for the virtual manipulator (i.e.
   * theta_3=0) */
  /*  With this we compute the plane of the virtual Shoulder-Elbow-Wrist, which
   */
  /*  is the reference plane. The NSParam is calculated from the angle between
   */
  /*  the actual robot Shoulder-Elbow-Wrist plane and the reference plane. */
  /*  */
  /*  Input:  pose            - Homogeneous Matrix size(4,4) of current robot
   * pose */
  /*          rconf           - Robot Configuration 8-bit number */
  /*  Output: ref_plan_vector - Vector normal to the reference plane. */
  /*          rot_base_elbow  - Rotation Matrix size(3,3) from base to elbow */
  /*          joints          - Joint values of the virtual robot (debug) */
  /*  */
  /*  The robot configuration parameter is considered because there is usually
   */
  /*  2 possible solutions: elbow 'up' and 'down'. Therefore and as the author
   */
  /*  suggested we are selecting the configuration that matches the current */
  /*  robot. */
  /*  Calculations tolerance */
  /* Robot parameters */
  /* Link length */
  /* Denavit-Hartenberg parameters 7 DoF */
  /* DH: [a, alpha,    d, theta]  */
  /* theta3 == 0 */
  /*  Joint values of virtual manipulator */
  /*  end-effector position from base     */
  /*  shoulder position from base  */
  /*  end-effector position from wrist */
  /*  wrist position from base */
  /*  shoulder to wrist vector */
  /*  upper arm length (shoulder to elbow) */
  /*  lower arm length (elbow to wrist) */
  /* Check if pose is within arm+forearm reach */
  /*  Cosine law */
  a_tmp = 0.0;
  scale = 3.3121686421112381E-170;
  for (tr_tmp = 0; tr_tmp < 3; tr_tmp++) {
    v1_idx_2 =
        (tr[tr_tmp + 108] - ((tr[tr_tmp + 96] * 0.0 + tr[tr_tmp + 100] * 0.0) +
                             tr[tr_tmp + 104] * 0.1621)) -
        xs0[tr_tmp];
    xsw[tr_tmp] = v1_idx_2;
    absxk = fabs(v1_idx_2);
    if (absxk > scale) {
      t = scale / absxk;
      a_tmp = a_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      a_tmp += t * t;
    }
  }
  a_tmp = scale * sqrt(a_tmp);
  /*  Shoulder Joints */
  /*  These are the vectors corresponding to our DH parameters */
  /*  m = member between parentisis. Check equation (14) */
  /*  -- Joint 1 -- */
  /*  Since joint3 is locked as 0, the only joint to define the orientation of
   */
  /*  the xsw vector in the xy-plane is joint 1. Therefore and since we are */
  /*  only interested in the transformation T03 (disregarding joint limits), we
   */
  /*  chose to simply set joint 1 as the atan of xsw y and x coordinates  */
  /*  (even if if goes beyond the joint limit). */
  /* Cannot be this because if x and y are 0, then it is not defined. */
  scale = 3.3121686421112381E-170;
  absxk = fabs(xsw[1] - xsw[2] * 0.0);
  if (absxk > 3.3121686421112381E-170) {
    n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n = t * t;
  }
  absxk = fabs(xsw[2] * 0.0 - xsw[0]);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  t = (xsw[0] * 0.0 - xsw[1] * 0.0) / scale;
  n += t * t;
  n = scale * sqrt(n);
  if (n > 1.0E-6) {
    n = rt_atan2d_snf(xsw[1], xsw[0]);
  } else {
    n = 0.0;
  }
  /*  -- Joint 2 -- */
  /*  Can be found through geometric relations */
  /*  Let phi be the angle E-S-W, and theta2 the angle (z-axis)-S-E. */
  /*  Then, theta2 = atan2(r,xsw(3)) -/+ phi. */
  /*  phi can be calculated, as a function of theta3: */
  /*    atan2(lew*sin(theta4),lse+lew*cos(theta4)) */
  /*  z-axis */
  /*    ^ */
  /*    |  E O------------O W */
  /*    |   /        .   */
  /*    |  /      . */
  /*    | /    .    xsw */
  /*    |/  . */
  /*  S O___________________ r-axis */
  /*  */
  /*  Lower arm transformation */
  /*  With T03 we can calculate the reference elbow position and with it the */
  /*  vector normal to the reference plane. */
  /*  reference elbow position */
  /* DH Summary of this function goes here */
  /*    Detailed explanation goes here */
  v1_idx_2 = sin(n);
  v_idx_0 = cos(n);
  n = a_tmp * a_tmp;
  b_n = rt_atan2d_snf(rt_hypotd_snf(xsw[0], xsw[1]), xsw[2]) +
        (double)elbow *
            acos(((n + 0.041697639999999994) - 0.04165681) / (0.4084 * a_tmp));
  /* DH Summary of this function goes here */
  /*    Detailed explanation goes here */
  v_idx_1 = sin(b_n);
  v_idx_2 = cos(b_n);
  b_n = (double)elbow *
        acos(((n - 0.041697639999999994) - 0.04165681) / 0.08335444);
  /* DH Summary of this function goes here */
  /*    Detailed explanation goes here */
  T_tmp = sin(b_n);
  b_T_tmp = cos(b_n);
  tmp[0] = v_idx_0;
  tmp[4] = -v1_idx_2 * 6.123233995736766E-17;
  tmp[8] = -v1_idx_2;
  tmp[12] = 0.0 * v_idx_0;
  tmp[1] = v1_idx_2;
  tmp[5] = v_idx_0 * 6.123233995736766E-17;
  tmp[9] = v_idx_0;
  tmp[13] = 0.0 * v1_idx_2;
  tmp[2] = 0.0;
  tmp[6] = -1.0;
  tmp[10] = 6.123233995736766E-17;
  tmp[14] = 0.1678;
  b_tr[0] = v_idx_2;
  b_tr[4] = -v_idx_1 * 6.123233995736766E-17;
  b_tr[8] = v_idx_1;
  b_tr[12] = 0.0 * v_idx_2;
  b_tr[1] = v_idx_1;
  b_tr[5] = v_idx_2 * 6.123233995736766E-17;
  b_tr[9] = -v_idx_2;
  b_tr[13] = 0.0 * v_idx_1;
  b_tr[2] = 0.0;
  b_tr[6] = 1.0;
  b_tr[10] = 6.123233995736766E-17;
  b_tr[14] = 0.0;
  tmp[3] = 0.0;
  b_tr[3] = 0.0;
  tmp[7] = 0.0;
  b_tr[7] = 0.0;
  tmp[11] = 0.0;
  b_tr[11] = 0.0;
  tmp[15] = 1.0;
  b_tr[15] = 1.0;
  for (i = 0; i < 4; i++) {
    v1_idx_2 = tmp[i];
    n = tmp[i + 4];
    b_n = tmp[i + 8];
    v_idx_0 = tmp[i + 12];
    for (tr_tmp = 0; tr_tmp < 4; tr_tmp++) {
      b_tr_tmp = tr_tmp << 2;
      c_T_tmp[i + b_tr_tmp] =
          ((v1_idx_2 * b_tr[b_tr_tmp] + n * b_tr[b_tr_tmp + 1]) +
           b_n * b_tr[b_tr_tmp + 2]) +
          v_idx_0 * b_tr[b_tr_tmp + 3];
    }
    v1_idx_2 = c_T_tmp[i];
    n = c_T_tmp[i + 4];
    b_n = c_T_tmp[i + 8];
    v_idx_0 = c_T_tmp[i + 12];
    for (tr_tmp = 0; tr_tmp < 4; tr_tmp++) {
      b_tr_tmp = tr_tmp << 2;
      tmp[i + b_tr_tmp] = ((v1_idx_2 * b[b_tr_tmp] + n * b[b_tr_tmp + 1]) +
                           b_n * b[b_tr_tmp + 2]) +
                          v_idx_0 * b[b_tr_tmp + 3];
    }
  }
  b_tr[0] = b_T_tmp;
  b_tr[4] = -T_tmp * 6.123233995736766E-17;
  b_tr[8] = -T_tmp;
  b_tr[12] = 0.0 * b_T_tmp;
  b_tr[1] = T_tmp;
  b_tr[5] = b_T_tmp * 6.123233995736766E-17;
  b_tr[9] = b_T_tmp;
  b_tr[13] = 0.0 * T_tmp;
  b_tr[2] = 0.0;
  b_tr[6] = -1.0;
  b_tr[10] = 6.123233995736766E-17;
  b_tr[14] = 0.0;
  b_tr[3] = 0.0;
  b_tr[7] = 0.0;
  b_tr[11] = 0.0;
  b_tr[15] = 1.0;
  for (i = 0; i < 4; i++) {
    v1_idx_2 = tmp[i];
    n = tmp[i + 4];
    b_n = tmp[i + 8];
    v_idx_0 = tmp[i + 12];
    for (tr_tmp = 0; tr_tmp < 4; tr_tmp++) {
      b_tr_tmp = tr_tmp << 2;
      c_T_tmp[i + b_tr_tmp] =
          ((v1_idx_2 * b_tr[b_tr_tmp] + n * b_tr[b_tr_tmp + 1]) +
           b_n * b_tr[b_tr_tmp + 2]) +
          v_idx_0 * b_tr[b_tr_tmp + 3];
    }
  }
  /* UNIT Unitize a vector */
  /*  */
  /*  VN = UNIT(V) is a unit-vector parallel to V. */
  /*  */
  /*  Note:: */
  /*  - Reports error for the case where norm(V) is zero. */
  /*  Copyright (C) 1993-2015, by Peter I. Corke */
  /*  */
  /*  This file is part of The Robotics Toolbox for MATLAB (RTB). */
  /*   */
  /*  RTB is free software: you can redistribute it and/or modify */
  /*  it under the terms of the GNU Lesser General Public License as published
   * by */
  /*  the Free Software Foundation, either version 3 of the License, or */
  /*  (at your option) any later version. */
  /*   */
  /*  RTB is distributed in the hope that it will be useful, */
  /*  but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  GNU Lesser General Public License for more details. */
  /*   */
  /*  You should have received a copy of the GNU Leser General Public License */
  /*  along with RTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www.petercorke.com */
  scale = 3.3121686421112381E-170;
  absxk = fabs(c_T_tmp[12]);
  if (absxk > 3.3121686421112381E-170) {
    n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n = t * t;
  }
  absxk = fabs(c_T_tmp[13]);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  absxk = fabs(c_T_tmp[14] - 0.1678);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  n = scale * sqrt(n);
  /*  unit vector from shoulder to elbow */
  /* UNIT Unitize a vector */
  /*  */
  /*  VN = UNIT(V) is a unit-vector parallel to V. */
  /*  */
  /*  Note:: */
  /*  - Reports error for the case where norm(V) is zero. */
  /*  Copyright (C) 1993-2015, by Peter I. Corke */
  /*  */
  /*  This file is part of The Robotics Toolbox for MATLAB (RTB). */
  /*   */
  /*  RTB is free software: you can redistribute it and/or modify */
  /*  it under the terms of the GNU Lesser General Public License as published
   * by */
  /*  the Free Software Foundation, either version 3 of the License, or */
  /*  (at your option) any later version. */
  /*   */
  /*  RTB is distributed in the hope that it will be useful, */
  /*  but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  GNU Lesser General Public License for more details. */
  /*   */
  /*  You should have received a copy of the GNU Leser General Public License */
  /*  along with RTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www.petercorke.com */
  v1_idx_0 = c_T_tmp[12] / n;
  xsw[0] /= a_tmp;
  v1_idx_1 = c_T_tmp[13] / n;
  xsw[1] /= a_tmp;
  v1_idx_2 = (c_T_tmp[14] - 0.1678) / n;
  xsw[2] /= a_tmp;
  /*  unit vector from shoulder to wrist */
  b_T_tmp = v1_idx_1 * xsw[2] - xsw[1] * v1_idx_2;
  T_tmp = xsw[0] * v1_idx_2 - v1_idx_0 * xsw[2];
  a_tmp = v1_idx_0 * xsw[1] - xsw[0] * v1_idx_1;
  /*  vv is the vector normal to the reference plane: xs-xe0-xw */
  /*  vc is the vector normal to the current plane:   xs-xe-xw */
  /* UNIT Unitize a vector */
  /*  */
  /*  VN = UNIT(V) is a unit-vector parallel to V. */
  /*  */
  /*  Note:: */
  /*  - Reports error for the case where norm(V) is zero. */
  /*  Copyright (C) 1993-2015, by Peter I. Corke */
  /*  */
  /*  This file is part of The Robotics Toolbox for MATLAB (RTB). */
  /*   */
  /*  RTB is free software: you can redistribute it and/or modify */
  /*  it under the terms of the GNU Lesser General Public License as published
   * by */
  /*  the Free Software Foundation, either version 3 of the License, or */
  /*  (at your option) any later version. */
  /*   */
  /*  RTB is distributed in the hope that it will be useful, */
  /*  but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  GNU Lesser General Public License for more details. */
  /*   */
  /*  You should have received a copy of the GNU Leser General Public License */
  /*  along with RTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www.petercorke.com */
  scale = 3.3121686421112381E-170;
  v1_idx_2 = tr[60] - tr[12];
  v1_idx_0 = v1_idx_2;
  absxk = fabs(v1_idx_2);
  if (absxk > 3.3121686421112381E-170) {
    n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n = t * t;
  }
  v1_idx_2 = tr[61] - tr[13];
  v1_idx_1 = v1_idx_2;
  absxk = fabs(v1_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  v1_idx_2 = tr[62] - tr[14];
  absxk = fabs(v1_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  n = scale * sqrt(n);
  /* UNIT Unitize a vector */
  /*  */
  /*  VN = UNIT(V) is a unit-vector parallel to V. */
  /*  */
  /*  Note:: */
  /*  - Reports error for the case where norm(V) is zero. */
  /*  Copyright (C) 1993-2015, by Peter I. Corke */
  /*  */
  /*  This file is part of The Robotics Toolbox for MATLAB (RTB). */
  /*   */
  /*  RTB is free software: you can redistribute it and/or modify */
  /*  it under the terms of the GNU Lesser General Public License as published
   * by */
  /*  the Free Software Foundation, either version 3 of the License, or */
  /*  (at your option) any later version. */
  /*   */
  /*  RTB is distributed in the hope that it will be useful, */
  /*  but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  GNU Lesser General Public License for more details. */
  /*   */
  /*  You should have received a copy of the GNU Leser General Public License */
  /*  along with RTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www.petercorke.com */
  scale = 3.3121686421112381E-170;
  v1_idx_0 /= n;
  absxk = fabs(xsw_idx_0);
  if (absxk > 3.3121686421112381E-170) {
    b_n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_n = t * t;
  }
  v1_idx_1 /= n;
  absxk = fabs(xsw_idx_1);
  if (absxk > scale) {
    t = scale / absxk;
    b_n = b_n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_n += t * t;
  }
  v1_idx_2 /= n;
  absxk = fabs(xsw_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    b_n = b_n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_n += t * t;
  }
  b_n = scale * sqrt(b_n);
  /* UNIT Unitize a vector */
  /*  */
  /*  VN = UNIT(V) is a unit-vector parallel to V. */
  /*  */
  /*  Note:: */
  /*  - Reports error for the case where norm(V) is zero. */
  /*  Copyright (C) 1993-2015, by Peter I. Corke */
  /*  */
  /*  This file is part of The Robotics Toolbox for MATLAB (RTB). */
  /*   */
  /*  RTB is free software: you can redistribute it and/or modify */
  /*  it under the terms of the GNU Lesser General Public License as published
   * by */
  /*  the Free Software Foundation, either version 3 of the License, or */
  /*  (at your option) any later version. */
  /*   */
  /*  RTB is distributed in the hope that it will be useful, */
  /*  but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  GNU Lesser General Public License for more details. */
  /*   */
  /*  You should have received a copy of the GNU Leser General Public License */
  /*  along with RTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www.petercorke.com */
  scale = 3.3121686421112381E-170;
  v_idx_0 = xsw_idx_0 / b_n;
  absxk = fabs(b_T_tmp);
  if (absxk > 3.3121686421112381E-170) {
    n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n = t * t;
  }
  v_idx_1 = xsw_idx_1 / b_n;
  absxk = fabs(T_tmp);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  v_idx_2 = xsw_idx_2 / b_n;
  absxk = fabs(a_tmp);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  vc_idx_0 = v1_idx_1 * v_idx_2 - v_idx_1 * v1_idx_2;
  vc_idx_1 = v_idx_0 * v1_idx_2 - v1_idx_0 * v_idx_2;
  vc_idx_2 = v1_idx_0 * v_idx_1 - v_idx_0 * v1_idx_1;
  n = scale * sqrt(n);
  /* UNIT Unitize a vector */
  /*  */
  /*  VN = UNIT(V) is a unit-vector parallel to V. */
  /*  */
  /*  Note:: */
  /*  - Reports error for the case where norm(V) is zero. */
  /*  Copyright (C) 1993-2015, by Peter I. Corke */
  /*  */
  /*  This file is part of The Robotics Toolbox for MATLAB (RTB). */
  /*   */
  /*  RTB is free software: you can redistribute it and/or modify */
  /*  it under the terms of the GNU Lesser General Public License as published
   * by */
  /*  the Free Software Foundation, either version 3 of the License, or */
  /*  (at your option) any later version. */
  /*   */
  /*  RTB is distributed in the hope that it will be useful, */
  /*  but WITHOUT ANY WARRANTY; without even the implied warranty of */
  /*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
  /*  GNU Lesser General Public License for more details. */
  /*   */
  /*  You should have received a copy of the GNU Leser General Public License */
  /*  along with RTB.  If not, see <http://www.gnu.org/licenses/>. */
  /*  */
  /*  http://www.petercorke.com */
  scale = 3.3121686421112381E-170;
  v_idx_0 = b_T_tmp / n;
  absxk = fabs(vc_idx_0);
  if (absxk > 3.3121686421112381E-170) {
    b_n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_n = t * t;
  }
  v_idx_1 = T_tmp / n;
  absxk = fabs(vc_idx_1);
  if (absxk > scale) {
    t = scale / absxk;
    b_n = b_n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_n += t * t;
  }
  v_idx_2 = a_tmp / n;
  absxk = fabs(vc_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    b_n = b_n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_n += t * t;
  }
  b_n = scale * sqrt(b_n);
  v1_idx_2 = vc_idx_0 / b_n;
  xsw[0] = v1_idx_2;
  cos_ns = v_idx_0 * v1_idx_2;
  v1_idx_2 = vc_idx_1 / b_n;
  xsw[1] = v1_idx_2;
  cos_ns += v_idx_1 * v1_idx_2;
  v1_idx_2 = vc_idx_2 / b_n;
  cos_ns += v_idx_2 * v1_idx_2;
  if (fabs(cos_ns) > 1.0) {
    if (cos_ns < 0.0) {
      cos_ns = -1.0;
    } else if (cos_ns > 0.0) {
      cos_ns = 1.0;
    } else if (cos_ns == 0.0) {
      cos_ns = 0.0;
    }
  }
  /* this vector will give the sign of the nsparam */
  v1_idx_0 = v_idx_1 * v1_idx_2 - xsw[1] * v_idx_2;
  v1_idx_1 = xsw[0] * v_idx_2 - v_idx_0 * v1_idx_2;
  v1_idx_2 = v_idx_0 * xsw[1] - xsw[0] * v_idx_1;
  scale = 3.3121686421112381E-170;
  absxk = fabs(v1_idx_0);
  if (absxk > 3.3121686421112381E-170) {
    n = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n = t * t;
  }
  absxk = fabs(v1_idx_1);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  absxk = fabs(v1_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    n = n * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n += t * t;
  }
  n = scale * sqrt(n);
  if (n > 1.0E-8) {
    n = (v1_idx_0 * xsw_idx_0 + v1_idx_1 * xsw_idx_1) + v1_idx_2 * xsw_idx_2;
    if (n < 0.0) {
      n = -1.0;
    } else if (n > 0.0) {
      n = 1.0;
    } else if (n == 0.0) {
      n = 0.0;
    }
    *nsparam = n * acos(cos_ns);
  } else {
    scale = 3.3121686421112381E-170;
    absxk = fabs(b_T_tmp - vc_idx_0);
    if (absxk > 3.3121686421112381E-170) {
      n = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      n = t * t;
    }
    absxk = fabs(T_tmp - vc_idx_1);
    if (absxk > scale) {
      t = scale / absxk;
      n = n * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      n += t * t;
    }
    absxk = fabs(a_tmp - vc_idx_2);
    if (absxk > scale) {
      t = scale / absxk;
      n = n * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      n += t * t;
    }
    n = scale * sqrt(n);
    /*if (n < 1.0E-8) {
      *nsparam = 3.1415926;
    } else {
      *nsparam = 0.0;
    }*/
    *nsparam = 0.0;
  }
}

/*
 * File trailer for ForwardKinematics.c
 *
 * [EOF]
 */
