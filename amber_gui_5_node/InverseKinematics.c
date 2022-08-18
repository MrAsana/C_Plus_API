/*
 * File: InverseKinematics.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 25-Jan-2022 17:36:15
 */

/* Include Files */
#include "InverseKinematics.h"
#include "InverseKinematics_types.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void Double2MultiWord(double u1, unsigned int y[], int n);

static void MultiWordAnd(const unsigned int u1[], const unsigned int u2[],
                         unsigned int y[], int n);

static double rt_atan2d_snf(double u0, double u1);

static double rt_hypotd_snf(double u0, double u1);

static double uMultiWord2Double(const unsigned int u1[], int n1, int e1);

/* Function Definitions */
/*
 * Arguments    : double u1
 *                unsigned int y[]
 *                int n
 * Return Type  : void
 */
static void Double2MultiWord(double u1, unsigned int y[], int n)
{
  double b_yn;
  double yd;
  int cb;
  int currExp;
  int msl;
  int prevExp;
  unsigned int u1i;
  unsigned int yi;
  bool isNegative;
  isNegative = (u1 < 0.0);
  b_yn = frexp(u1, &currExp);
  if (currExp <= 0) {
    msl = -1;
  } else {
    msl = (currExp - 1) / 32;
  }
  cb = 1;
  if (msl + 1 <= n - 1) {
    memset(&y[msl + 1], 0, ((n - msl) + -1) * sizeof(unsigned int));
  }
  if (isNegative) {
    b_yn = -b_yn;
  }
  prevExp = 32 * msl;
  while (msl >= 0) {
    b_yn = ldexp(b_yn, currExp - prevExp);
    yd = floor(b_yn);
    b_yn -= yd;
    if (msl < n) {
      y[msl] = (unsigned int)yd;
    }
    currExp = prevExp;
    prevExp -= 32;
    msl--;
  }
  if (isNegative) {
    for (msl = 0; msl < n; msl++) {
      u1i = ~y[msl];
      yi = u1i + cb;
      y[msl] = yi;
      cb = (yi < u1i);
    }
  }
}

/*
 * Arguments    : const unsigned int u1[]
 *                const unsigned int u2[]
 *                unsigned int y[]
 *                int n
 * Return Type  : void
 */
static void MultiWordAnd(const unsigned int u1[], const unsigned int u2[],
                         unsigned int y[], int n)
{
  int i;
  for (i = 0; i < n; i++) {
    y[i] = u1[i] & u2[i];
  }
}

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
 * Arguments    : const unsigned int u1[]
 *                int n1
 *                int e1
 * Return Type  : double
 */
static double uMultiWord2Double(const unsigned int u1[], int n1, int e1)
{
  double y;
  int b_exp;
  int i;
  y = 0.0;
  b_exp = e1;
  for (i = 0; i < n1; i++) {
    y += ldexp(u1[i], b_exp);
    b_exp += 32;
  }
  return y;
}

/*
 * INVERSEKINEMATICS
 *
 *  Calculates the inverse kinematics for a KUKA LBR iiwa manipulator.
 *
 *  Input:  pose    - Homogeneous Matrix size(4,4)
 *          nsparam - Arm Angle
 *          rconf   - Robot Configuration 8-bit number
 *  Output: joints  - Joint values size(1,7).
 *          s_mat   - Shoulder joint matrices As, Bs and Cs
 *          w_mat   - Wrist joint matrices Aw, Bw and Cw
 *
 * Arguments    : const double pose[16]
 *                double nsparam
 *                double rconf
 *                double joints[7]
 * Return Type  : void
 */
void InverseKinematics(const double pose[16], double nsparam, double rconf,
                       double joints[7])
{
  static const uint64m_T r2 = {
      {1U, 0U} /* chunks */
  };
  static const uint64m_T r4 = {
      {2U, 0U} /* chunks */
  };
  static const uint64m_T r5 = {
      {4U, 0U} /* chunks */
  };
  static const double b[9] = {1.0,
                              0.0,
                              0.0,
                              -0.0,
                              6.123233995736766E-17,
                              1.0,
                              0.0,
                              -1.0,
                              6.123233995736766E-17};
  static const double xs[3] = {0.0, 0.0, 0.1678};
  static const double xs0[3] = {0.0, 0.0, 0.1678};
  uint64m_T r;
  uint64m_T r1;
  uint64m_T r3;
  double T[16];
  double T_tmp[16];
  double b_T_tmp[16];
  double As[9];
  double Bs[9];
  double Cs[9];
  double R03_o[9];
  double c_T_tmp[9];
  double skew_usw[9];
  double b_xsw[3];
  double xsw[3];
  double R03_tmp;
  double a_tmp;
  double absxk;
  double b_R03_tmp;
  double b_scale;
  double d;
  double d1;
  double d2;
  double n_tmp;
  double scale;
  double t;
  int As_tmp;
  int aoffset;
  int arm;
  int b_As_tmp;
  int coffset;
  int coffset_tmp;
  int i;
  int wrist;
  /* RCONF Summary of this function goes here */
  /*    Detailed explanation goes here */
  arm = 1;
  coffset = 1;
  wrist = 1;
  Double2MultiWord(rconf, (unsigned int *)&r.chunks[0U], 2);
  r1 = r2;
  MultiWordAnd((unsigned int *)&r.chunks[0U], (unsigned int *)&r2.chunks[0U],
               (unsigned int *)&r3.chunks[0U], 2);
  if (uMultiWord2Double((unsigned int *)&r3.chunks[0U], 2, 0) != 0.0) {
    arm = -1;
  }
  Double2MultiWord(rconf, (unsigned int *)&r1.chunks[0U], 2);
  r3 = r4;
  MultiWordAnd((unsigned int *)&r1.chunks[0U], (unsigned int *)&r4.chunks[0U],
               (unsigned int *)&r.chunks[0U], 2);
  if (uMultiWord2Double((unsigned int *)&r.chunks[0U], 2, 0) != 0.0) {
    coffset = -1;
  }
  Double2MultiWord(rconf, (unsigned int *)&r3.chunks[0U], 2);
  MultiWordAnd((unsigned int *)&r3.chunks[0U], (unsigned int *)&r5.chunks[0U],
               (unsigned int *)&r1.chunks[0U], 2);
  if (uMultiWord2Double((unsigned int *)&r1.chunks[0U], 2, 0) != 0.0) {
    wrist = -1;
  }
  /* Tolerance */
  /* Robot parameters */
  /* Link length */
  /* Denavit-Hartenberg parameters 7 DoF */
  /* DH: [a, alpha,    d, theta]  */
  /* Number of joints */
  /*  Joint values of virtual manipulator */
  /*  end-effector position from base     */
  /*  shoulder position from base  */
  /*  end-effector position from wrist */
  /*  wrist position from base */
  /*  shoulder to wrist vector */
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
  n_tmp = 0.0;
  scale = 3.3121686421112381E-170;
  /*  upper arm length (shoulder to elbow) */
  /*  lower arm length (elbow to wrist) */
  /* Check if pose is within arm+forearm reach */
  /*  Cosine law - According to our robot, joint 4 rotates backwards */
  /* Added  */
  /* DH Summary of this function goes here */
  /*    Detailed explanation goes here */
  T[2] = 0.0;
  T[6] = -1.0;
  T[10] = 6.123233995736766E-17;
  T[14] = 0.0;
  T[3] = 0.0;
  T[7] = 0.0;
  T[11] = 0.0;
  T[15] = 1.0;
  /*  Shoulder Joints */
  /*  First compute the reference joint angles when the arm angle is zero. */
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
  a_tmp = 0.0;
  b_scale = 3.3121686421112381E-170;
  for (aoffset = 0; aoffset < 3; aoffset++) {
    d = pose[aoffset + 12] - ((pose[aoffset] * 0.0 + pose[aoffset + 4] * 0.0) +
                              pose[aoffset + 8] * 0.1621);
    d1 = d - xs[aoffset];
    xsw[aoffset] = d1;
    absxk = fabs(d1);
    if (absxk > scale) {
      t = scale / absxk;
      n_tmp = n_tmp * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      n_tmp += t * t;
    }
    d -= xs0[aoffset];
    b_xsw[aoffset] = d;
    absxk = fabs(d);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      a_tmp = a_tmp * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      a_tmp += t * t;
    }
  }
  n_tmp = scale * sqrt(n_tmp);
  xsw[0] /= n_tmp;
  xsw[1] /= n_tmp;
  xsw[2] /= n_tmp;
  joints[3] =
      (double)coffset *
      acos(((n_tmp * n_tmp - 0.041697639999999994) - 0.04165681) / 0.08335444);
  scale = sin(joints[3]);
  t = cos(joints[3]);
  T[0] = t;
  T[4] = -scale * 6.123233995736766E-17;
  T[8] = -scale;
  T[12] = 0.0 * t;
  T[1] = scale;
  T[5] = t * 6.123233995736766E-17;
  T[9] = t;
  T[13] = 0.0 * scale;
  a_tmp = b_scale * sqrt(a_tmp);
  /*  Cosine law */
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
  absxk = fabs(b_xsw[1] - b_xsw[2] * 0.0);
  if (absxk > 3.3121686421112381E-170) {
    n_tmp = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n_tmp = t * t;
  }
  absxk = fabs(b_xsw[2] * 0.0 - b_xsw[0]);
  if (absxk > scale) {
    t = scale / absxk;
    n_tmp = n_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n_tmp += t * t;
  }
  t = (b_xsw[0] * 0.0 - b_xsw[1] * 0.0) / scale;
  n_tmp += t * t;
  n_tmp = scale * sqrt(n_tmp);
  if (n_tmp > 1.0E-6) {
    n_tmp = rt_atan2d_snf(b_xsw[1], b_xsw[0]);
  } else {
    n_tmp = 0.0;
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
  /* DH Summary of this function goes here */
  /*    Detailed explanation goes here */
  scale = sin(n_tmp);
  t = cos(n_tmp);
  n_tmp = rt_atan2d_snf(rt_hypotd_snf(b_xsw[0], b_xsw[1]), b_xsw[2]) +
          (double)coffset *
              acos(((a_tmp * a_tmp + 0.041697639999999994) - 0.04165681) /
                   (0.4084 * a_tmp));
  /* DH Summary of this function goes here */
  /*    Detailed explanation goes here */
  absxk = sin(n_tmp);
  n_tmp = cos(n_tmp);
  T_tmp[0] = t;
  T_tmp[4] = -scale * 6.123233995736766E-17;
  T_tmp[8] = -scale;
  T_tmp[12] = 0.0 * t;
  T_tmp[1] = scale;
  T_tmp[5] = t * 6.123233995736766E-17;
  T_tmp[9] = t;
  T_tmp[13] = 0.0 * scale;
  T_tmp[2] = 0.0;
  T_tmp[6] = -1.0;
  T_tmp[10] = 6.123233995736766E-17;
  T_tmp[14] = 0.1678;
  b_T_tmp[0] = n_tmp;
  b_T_tmp[4] = -absxk * 6.123233995736766E-17;
  b_T_tmp[8] = absxk;
  b_T_tmp[12] = 0.0 * n_tmp;
  b_T_tmp[1] = absxk;
  b_T_tmp[5] = n_tmp * 6.123233995736766E-17;
  b_T_tmp[9] = -n_tmp;
  b_T_tmp[13] = 0.0 * absxk;
  b_T_tmp[2] = 0.0;
  b_T_tmp[6] = 1.0;
  b_T_tmp[10] = 6.123233995736766E-17;
  b_T_tmp[14] = 0.0;
  T_tmp[3] = 0.0;
  b_T_tmp[3] = 0.0;
  T_tmp[7] = 0.0;
  b_T_tmp[7] = 0.0;
  T_tmp[11] = 0.0;
  b_T_tmp[11] = 0.0;
  T_tmp[15] = 1.0;
  b_T_tmp[15] = 1.0;
  for (coffset_tmp = 0; coffset_tmp < 3; coffset_tmp++) {
    d = T_tmp[coffset_tmp];
    d1 = T_tmp[coffset_tmp + 4];
    n_tmp = T_tmp[coffset_tmp + 8];
    for (As_tmp = 0; As_tmp < 3; As_tmp++) {
      coffset = As_tmp << 2;
      c_T_tmp[coffset_tmp + 3 * As_tmp] =
          (d * b_T_tmp[coffset] + d1 * b_T_tmp[coffset + 1]) +
          n_tmp * b_T_tmp[coffset + 2];
    }
    d = c_T_tmp[coffset_tmp];
    d1 = c_T_tmp[coffset_tmp + 3];
    n_tmp = c_T_tmp[coffset_tmp + 6];
    for (As_tmp = 0; As_tmp < 3; As_tmp++) {
      R03_o[coffset_tmp + 3 * As_tmp] =
          (d * b[3 * As_tmp] + d1 * b[3 * As_tmp + 1]) +
          n_tmp * b[3 * As_tmp + 2];
    }
  }
  /*  With T03 we can calculate the reference elbow position and with it the */
  /*  vector normal to the reference plane. */
  /*  reference elbow position */
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
  /*  unit vector from shoulder to wrist */
  /* SKEW Summary of this function goes here */
  /*    Detailed explanation goes here */
  skew_usw[0] = 0.0;
  skew_usw[3] = -xsw[2];
  skew_usw[6] = xsw[1];
  skew_usw[1] = xsw[2];
  skew_usw[4] = 0.0;
  skew_usw[7] = -xsw[0];
  skew_usw[2] = -xsw[1];
  skew_usw[5] = xsw[0];
  skew_usw[8] = 0.0;
  /*  Following eq. (15), the auxiliary matrixes As Bs and Cs can be calculated
   */
  /*  by substituting eq. (6) into (9).  */
  /*  R0psi = I3 + sin(psi)*skew_usw + (1-cos(psi))*skew_usw²    (6) */
  /*  R03 = R0psi * R03_o                                         (9) */
  /*  Substituting (distributive prop.) we get: */
  /*  R03 = R03_o*skew_usw*sin(psi) + R03_o*(-skew_usw²)*cos(psi) + R03_o(I3 +
   * skew_usw²) */
  /*  R03 =      As       *sin(psi) +        Bs         *cos(psi) +          Cs
   */
  for (coffset_tmp = 0; coffset_tmp < 3; coffset_tmp++) {
    d = skew_usw[coffset_tmp + 3];
    d1 = skew_usw[coffset_tmp + 6];
    for (As_tmp = 0; As_tmp < 3; As_tmp++) {
      coffset = 3 * As_tmp + 1;
      aoffset = 3 * As_tmp + 2;
      b_As_tmp = coffset_tmp + 3 * As_tmp;
      As[b_As_tmp] =
          (skew_usw[coffset_tmp] * R03_o[3 * As_tmp] + d * R03_o[coffset]) +
          d1 * R03_o[aoffset];
      c_T_tmp[b_As_tmp] = -((skew_usw[coffset_tmp] * skew_usw[3 * As_tmp] +
                             d * skew_usw[coffset]) +
                            d1 * skew_usw[aoffset]);
    }
    d = c_T_tmp[coffset_tmp];
    d1 = c_T_tmp[coffset_tmp + 3];
    n_tmp = c_T_tmp[coffset_tmp + 6];
    for (As_tmp = 0; As_tmp < 3; As_tmp++) {
      Bs[coffset_tmp + 3 * As_tmp] =
          (d * R03_o[3 * As_tmp] + d1 * R03_o[3 * As_tmp + 1]) +
          n_tmp * R03_o[3 * As_tmp + 2];
    }
  }
  for (coffset_tmp = 0; coffset_tmp < 3; coffset_tmp++) {
    c_T_tmp[3 * coffset_tmp] = xsw[0] * xsw[coffset_tmp];
    c_T_tmp[3 * coffset_tmp + 1] = xsw[1] * xsw[coffset_tmp];
    c_T_tmp[3 * coffset_tmp + 2] = xsw[2] * xsw[coffset_tmp];
  }
  for (coffset_tmp = 0; coffset_tmp < 3; coffset_tmp++) {
    d = c_T_tmp[coffset_tmp];
    d1 = c_T_tmp[coffset_tmp + 3];
    n_tmp = c_T_tmp[coffset_tmp + 6];
    for (As_tmp = 0; As_tmp < 3; As_tmp++) {
      Cs[coffset_tmp + 3 * As_tmp] =
          (d * R03_o[3 * As_tmp] + d1 * R03_o[3 * As_tmp + 1]) +
          n_tmp * R03_o[3 * As_tmp + 2];
    }
  }
  R03_tmp = sin(nsparam);
  b_R03_tmp = cos(nsparam);
  for (coffset_tmp = 0; coffset_tmp < 9; coffset_tmp++) {
    skew_usw[coffset_tmp] =
        (As[coffset_tmp] * R03_tmp + Bs[coffset_tmp] * b_R03_tmp) +
        Cs[coffset_tmp];
  }
  /*  T03 transformation matrix (DH parameters) */
  /* [ cos(j1)*cos(j2)*cos(j3) - sin(j1)*sin(j3), cos(j1)*sin(j2),
   * cos(j3)*sin(j1) + cos(j1)*cos(j2)*sin(j3), 0.4*cos(j1)*sin(j2)] */
  /* [ cos(j1)*sin(j3) + cos(j2)*cos(j3)*sin(j1), sin(j1)*sin(j2),
   * cos(j2)*sin(j1)*sin(j3) - cos(j1)*cos(j3), 0.4*sin(j1)*sin(j2)] */
  /* [                          -cos(j3)*sin(j2),         cos(j2),
   * -sin(j2)*sin(j3),  0.4*cos(j2) + 0.34] */
  /* [                                         0,               0, 0, 1] */
  joints[0] =
      rt_atan2d_snf((double)arm * skew_usw[4], (double)arm * skew_usw[3]);
  joints[1] = (double)arm * acos(skew_usw[5]);
  joints[2] =
      rt_atan2d_snf((double)arm * -skew_usw[8], (double)arm * -skew_usw[2]);
  /* DH Summary of this function goes here */
  /*    Detailed explanation goes here */
  scale = sin(joints[3]);
  t = cos(joints[3]);
  T_tmp[0] = t;
  T_tmp[4] = -scale * 6.123233995736766E-17;
  T_tmp[8] = -scale;
  T_tmp[12] = 0.0 * t;
  T_tmp[1] = scale;
  T_tmp[5] = t * 6.123233995736766E-17;
  T_tmp[9] = t;
  T_tmp[13] = 0.0 * scale;
  T_tmp[2] = 0.0;
  T_tmp[6] = -1.0;
  T_tmp[10] = 6.123233995736766E-17;
  T_tmp[14] = 0.0;
  T_tmp[3] = 0.0;
  T_tmp[7] = 0.0;
  T_tmp[11] = 0.0;
  T_tmp[15] = 1.0;
  for (arm = 0; arm < 3; arm++) {
    coffset = arm * 3;
    d = As[arm];
    d1 = As[arm + 3];
    n_tmp = As[arm + 6];
    for (i = 0; i < 3; i++) {
      aoffset = i * 3;
      c_T_tmp[coffset + i] =
          (T[aoffset % 3 + ((aoffset / 3) << 2)] * d +
           T[(aoffset + 1) % 3 + (((aoffset + 1) / 3) << 2)] * d1) +
          T[(aoffset + 2) % 3 + (((aoffset + 2) / 3) << 2)] * n_tmp;
    }
  }
  for (arm = 0; arm < 3; arm++) {
    coffset_tmp = arm * 3;
    d = Bs[arm];
    d1 = Bs[arm + 3];
    n_tmp = Bs[arm + 6];
    absxk = Cs[arm];
    t = Cs[arm + 3];
    scale = Cs[arm + 6];
    for (i = 0; i < 3; i++) {
      coffset = i * 3;
      b_As_tmp = coffset % 3 + ((coffset / 3) << 2);
      As_tmp = (coffset + 1) % 3 + (((coffset + 1) / 3) << 2);
      coffset = (coffset + 2) % 3 + (((coffset + 2) / 3) << 2);
      aoffset = coffset_tmp + i;
      As[aoffset] = (T[b_As_tmp] * d + T[As_tmp] * d1) + T[coffset] * n_tmp;
      R03_o[aoffset] = (T_tmp[b_As_tmp] * absxk + T_tmp[As_tmp] * t) +
                       T_tmp[coffset] * scale;
    }
  }
  for (coffset_tmp = 0; coffset_tmp < 3; coffset_tmp++) {
    d = c_T_tmp[coffset_tmp];
    d1 = c_T_tmp[coffset_tmp + 3];
    n_tmp = c_T_tmp[coffset_tmp + 6];
    for (As_tmp = 0; As_tmp < 3; As_tmp++) {
      coffset = As_tmp << 2;
      Cs[coffset_tmp + 3 * As_tmp] =
          (d * pose[coffset] + d1 * pose[coffset + 1]) +
          n_tmp * pose[coffset + 2];
    }
    d = As[coffset_tmp];
    d1 = As[coffset_tmp + 3];
    n_tmp = As[coffset_tmp + 6];
    absxk = R03_o[coffset_tmp];
    t = R03_o[coffset_tmp + 3];
    scale = R03_o[coffset_tmp + 6];
    for (As_tmp = 0; As_tmp < 3; As_tmp++) {
      coffset = As_tmp << 2;
      b_scale = pose[coffset];
      a_tmp = d * b_scale;
      d2 = absxk * b_scale;
      b_scale = pose[coffset + 1];
      a_tmp += d1 * b_scale;
      d2 += t * b_scale;
      b_scale = pose[coffset + 2];
      a_tmp += n_tmp * b_scale;
      d2 += scale * b_scale;
      coffset = coffset_tmp + 3 * As_tmp;
      c_T_tmp[coffset] = a_tmp;
      skew_usw[coffset] = (Cs[coffset] * R03_tmp + a_tmp * b_R03_tmp) + d2;
    }
  }
  /*  T47 transformation matrix (DH parameters) */
  /* [ cos(j5)*cos(j6)*cos(j7) - sin(j5)*sin(j7), - cos(j7)*sin(j5) -
   * cos(j5)*cos(j6)*sin(j7), cos(j5)*sin(j6), (63*cos(j5)*sin(j6))/500] */
  /* [ cos(j5)*sin(j7) + cos(j6)*cos(j7)*sin(j5),   cos(j5)*cos(j7) -
   * cos(j6)*sin(j5)*sin(j7), sin(j5)*sin(j6), (63*sin(j5)*sin(j6))/500] */
  /* [                          -cos(j7)*sin(j6), sin(j6)*sin(j7), cos(j6),
   * (63*cos(j6))/500 + 2/5] */
  /* [                                         0, 0,               0, 1] */
  joints[4] =
      rt_atan2d_snf((double)wrist * skew_usw[7], (double)wrist * skew_usw[6]);
  joints[5] = (double)wrist * acos(skew_usw[8]);
  joints[6] =
      rt_atan2d_snf((double)wrist * skew_usw[5], (double)wrist * -skew_usw[2]);
}

/*
 * File trailer for InverseKinematics.c
 *
 * [EOF]
 */
