#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7453460125519006838) {
   out_7453460125519006838[0] = delta_x[0] + nom_x[0];
   out_7453460125519006838[1] = delta_x[1] + nom_x[1];
   out_7453460125519006838[2] = delta_x[2] + nom_x[2];
   out_7453460125519006838[3] = delta_x[3] + nom_x[3];
   out_7453460125519006838[4] = delta_x[4] + nom_x[4];
   out_7453460125519006838[5] = delta_x[5] + nom_x[5];
   out_7453460125519006838[6] = delta_x[6] + nom_x[6];
   out_7453460125519006838[7] = delta_x[7] + nom_x[7];
   out_7453460125519006838[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5088499265255808698) {
   out_5088499265255808698[0] = -nom_x[0] + true_x[0];
   out_5088499265255808698[1] = -nom_x[1] + true_x[1];
   out_5088499265255808698[2] = -nom_x[2] + true_x[2];
   out_5088499265255808698[3] = -nom_x[3] + true_x[3];
   out_5088499265255808698[4] = -nom_x[4] + true_x[4];
   out_5088499265255808698[5] = -nom_x[5] + true_x[5];
   out_5088499265255808698[6] = -nom_x[6] + true_x[6];
   out_5088499265255808698[7] = -nom_x[7] + true_x[7];
   out_5088499265255808698[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1035652654272596140) {
   out_1035652654272596140[0] = 1.0;
   out_1035652654272596140[1] = 0.0;
   out_1035652654272596140[2] = 0.0;
   out_1035652654272596140[3] = 0.0;
   out_1035652654272596140[4] = 0.0;
   out_1035652654272596140[5] = 0.0;
   out_1035652654272596140[6] = 0.0;
   out_1035652654272596140[7] = 0.0;
   out_1035652654272596140[8] = 0.0;
   out_1035652654272596140[9] = 0.0;
   out_1035652654272596140[10] = 1.0;
   out_1035652654272596140[11] = 0.0;
   out_1035652654272596140[12] = 0.0;
   out_1035652654272596140[13] = 0.0;
   out_1035652654272596140[14] = 0.0;
   out_1035652654272596140[15] = 0.0;
   out_1035652654272596140[16] = 0.0;
   out_1035652654272596140[17] = 0.0;
   out_1035652654272596140[18] = 0.0;
   out_1035652654272596140[19] = 0.0;
   out_1035652654272596140[20] = 1.0;
   out_1035652654272596140[21] = 0.0;
   out_1035652654272596140[22] = 0.0;
   out_1035652654272596140[23] = 0.0;
   out_1035652654272596140[24] = 0.0;
   out_1035652654272596140[25] = 0.0;
   out_1035652654272596140[26] = 0.0;
   out_1035652654272596140[27] = 0.0;
   out_1035652654272596140[28] = 0.0;
   out_1035652654272596140[29] = 0.0;
   out_1035652654272596140[30] = 1.0;
   out_1035652654272596140[31] = 0.0;
   out_1035652654272596140[32] = 0.0;
   out_1035652654272596140[33] = 0.0;
   out_1035652654272596140[34] = 0.0;
   out_1035652654272596140[35] = 0.0;
   out_1035652654272596140[36] = 0.0;
   out_1035652654272596140[37] = 0.0;
   out_1035652654272596140[38] = 0.0;
   out_1035652654272596140[39] = 0.0;
   out_1035652654272596140[40] = 1.0;
   out_1035652654272596140[41] = 0.0;
   out_1035652654272596140[42] = 0.0;
   out_1035652654272596140[43] = 0.0;
   out_1035652654272596140[44] = 0.0;
   out_1035652654272596140[45] = 0.0;
   out_1035652654272596140[46] = 0.0;
   out_1035652654272596140[47] = 0.0;
   out_1035652654272596140[48] = 0.0;
   out_1035652654272596140[49] = 0.0;
   out_1035652654272596140[50] = 1.0;
   out_1035652654272596140[51] = 0.0;
   out_1035652654272596140[52] = 0.0;
   out_1035652654272596140[53] = 0.0;
   out_1035652654272596140[54] = 0.0;
   out_1035652654272596140[55] = 0.0;
   out_1035652654272596140[56] = 0.0;
   out_1035652654272596140[57] = 0.0;
   out_1035652654272596140[58] = 0.0;
   out_1035652654272596140[59] = 0.0;
   out_1035652654272596140[60] = 1.0;
   out_1035652654272596140[61] = 0.0;
   out_1035652654272596140[62] = 0.0;
   out_1035652654272596140[63] = 0.0;
   out_1035652654272596140[64] = 0.0;
   out_1035652654272596140[65] = 0.0;
   out_1035652654272596140[66] = 0.0;
   out_1035652654272596140[67] = 0.0;
   out_1035652654272596140[68] = 0.0;
   out_1035652654272596140[69] = 0.0;
   out_1035652654272596140[70] = 1.0;
   out_1035652654272596140[71] = 0.0;
   out_1035652654272596140[72] = 0.0;
   out_1035652654272596140[73] = 0.0;
   out_1035652654272596140[74] = 0.0;
   out_1035652654272596140[75] = 0.0;
   out_1035652654272596140[76] = 0.0;
   out_1035652654272596140[77] = 0.0;
   out_1035652654272596140[78] = 0.0;
   out_1035652654272596140[79] = 0.0;
   out_1035652654272596140[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1273096831272741265) {
   out_1273096831272741265[0] = state[0];
   out_1273096831272741265[1] = state[1];
   out_1273096831272741265[2] = state[2];
   out_1273096831272741265[3] = state[3];
   out_1273096831272741265[4] = state[4];
   out_1273096831272741265[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1273096831272741265[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1273096831272741265[7] = state[7];
   out_1273096831272741265[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4155690995932901086) {
   out_4155690995932901086[0] = 1;
   out_4155690995932901086[1] = 0;
   out_4155690995932901086[2] = 0;
   out_4155690995932901086[3] = 0;
   out_4155690995932901086[4] = 0;
   out_4155690995932901086[5] = 0;
   out_4155690995932901086[6] = 0;
   out_4155690995932901086[7] = 0;
   out_4155690995932901086[8] = 0;
   out_4155690995932901086[9] = 0;
   out_4155690995932901086[10] = 1;
   out_4155690995932901086[11] = 0;
   out_4155690995932901086[12] = 0;
   out_4155690995932901086[13] = 0;
   out_4155690995932901086[14] = 0;
   out_4155690995932901086[15] = 0;
   out_4155690995932901086[16] = 0;
   out_4155690995932901086[17] = 0;
   out_4155690995932901086[18] = 0;
   out_4155690995932901086[19] = 0;
   out_4155690995932901086[20] = 1;
   out_4155690995932901086[21] = 0;
   out_4155690995932901086[22] = 0;
   out_4155690995932901086[23] = 0;
   out_4155690995932901086[24] = 0;
   out_4155690995932901086[25] = 0;
   out_4155690995932901086[26] = 0;
   out_4155690995932901086[27] = 0;
   out_4155690995932901086[28] = 0;
   out_4155690995932901086[29] = 0;
   out_4155690995932901086[30] = 1;
   out_4155690995932901086[31] = 0;
   out_4155690995932901086[32] = 0;
   out_4155690995932901086[33] = 0;
   out_4155690995932901086[34] = 0;
   out_4155690995932901086[35] = 0;
   out_4155690995932901086[36] = 0;
   out_4155690995932901086[37] = 0;
   out_4155690995932901086[38] = 0;
   out_4155690995932901086[39] = 0;
   out_4155690995932901086[40] = 1;
   out_4155690995932901086[41] = 0;
   out_4155690995932901086[42] = 0;
   out_4155690995932901086[43] = 0;
   out_4155690995932901086[44] = 0;
   out_4155690995932901086[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4155690995932901086[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4155690995932901086[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4155690995932901086[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4155690995932901086[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4155690995932901086[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4155690995932901086[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4155690995932901086[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4155690995932901086[53] = -9.8000000000000007*dt;
   out_4155690995932901086[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4155690995932901086[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4155690995932901086[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4155690995932901086[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4155690995932901086[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4155690995932901086[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4155690995932901086[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4155690995932901086[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4155690995932901086[62] = 0;
   out_4155690995932901086[63] = 0;
   out_4155690995932901086[64] = 0;
   out_4155690995932901086[65] = 0;
   out_4155690995932901086[66] = 0;
   out_4155690995932901086[67] = 0;
   out_4155690995932901086[68] = 0;
   out_4155690995932901086[69] = 0;
   out_4155690995932901086[70] = 1;
   out_4155690995932901086[71] = 0;
   out_4155690995932901086[72] = 0;
   out_4155690995932901086[73] = 0;
   out_4155690995932901086[74] = 0;
   out_4155690995932901086[75] = 0;
   out_4155690995932901086[76] = 0;
   out_4155690995932901086[77] = 0;
   out_4155690995932901086[78] = 0;
   out_4155690995932901086[79] = 0;
   out_4155690995932901086[80] = 1;
}
void h_25(double *state, double *unused, double *out_7529324327931565469) {
   out_7529324327931565469[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8157535571639641097) {
   out_8157535571639641097[0] = 0;
   out_8157535571639641097[1] = 0;
   out_8157535571639641097[2] = 0;
   out_8157535571639641097[3] = 0;
   out_8157535571639641097[4] = 0;
   out_8157535571639641097[5] = 0;
   out_8157535571639641097[6] = 1;
   out_8157535571639641097[7] = 0;
   out_8157535571639641097[8] = 0;
}
void h_24(double *state, double *unused, double *out_854981700122849040) {
   out_854981700122849040[0] = state[4];
   out_854981700122849040[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8116558903064410953) {
   out_8116558903064410953[0] = 0;
   out_8116558903064410953[1] = 0;
   out_8116558903064410953[2] = 0;
   out_8116558903064410953[3] = 0;
   out_8116558903064410953[4] = 1;
   out_8116558903064410953[5] = 0;
   out_8116558903064410953[6] = 0;
   out_8116558903064410953[7] = 0;
   out_8116558903064410953[8] = 0;
   out_8116558903064410953[9] = 0;
   out_8116558903064410953[10] = 0;
   out_8116558903064410953[11] = 0;
   out_8116558903064410953[12] = 0;
   out_8116558903064410953[13] = 0;
   out_8116558903064410953[14] = 1;
   out_8116558903064410953[15] = 0;
   out_8116558903064410953[16] = 0;
   out_8116558903064410953[17] = 0;
}
void h_30(double *state, double *unused, double *out_7254130265647059580) {
   out_7254130265647059580[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5639202613132392470) {
   out_5639202613132392470[0] = 0;
   out_5639202613132392470[1] = 0;
   out_5639202613132392470[2] = 0;
   out_5639202613132392470[3] = 0;
   out_5639202613132392470[4] = 1;
   out_5639202613132392470[5] = 0;
   out_5639202613132392470[6] = 0;
   out_5639202613132392470[7] = 0;
   out_5639202613132392470[8] = 0;
}
void h_26(double *state, double *unused, double *out_855088748233210434) {
   out_855088748233210434[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6547705183195854295) {
   out_6547705183195854295[0] = 0;
   out_6547705183195854295[1] = 0;
   out_6547705183195854295[2] = 0;
   out_6547705183195854295[3] = 0;
   out_6547705183195854295[4] = 0;
   out_6547705183195854295[5] = 0;
   out_6547705183195854295[6] = 0;
   out_6547705183195854295[7] = 1;
   out_6547705183195854295[8] = 0;
}
void h_27(double *state, double *unused, double *out_8313014424969288496) {
   out_8313014424969288496[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3415608541948449253) {
   out_3415608541948449253[0] = 0;
   out_3415608541948449253[1] = 0;
   out_3415608541948449253[2] = 0;
   out_3415608541948449253[3] = 1;
   out_3415608541948449253[4] = 0;
   out_3415608541948449253[5] = 0;
   out_3415608541948449253[6] = 0;
   out_3415608541948449253[7] = 0;
   out_3415608541948449253[8] = 0;
}
void h_29(double *state, double *unused, double *out_2252456997986213935) {
   out_2252456997986213935[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5128971268818000286) {
   out_5128971268818000286[0] = 0;
   out_5128971268818000286[1] = 1;
   out_5128971268818000286[2] = 0;
   out_5128971268818000286[3] = 0;
   out_5128971268818000286[4] = 0;
   out_5128971268818000286[5] = 0;
   out_5128971268818000286[6] = 0;
   out_5128971268818000286[7] = 0;
   out_5128971268818000286[8] = 0;
}
void h_28(double *state, double *unused, double *out_7369504572067626061) {
   out_7369504572067626061[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8235373787822020756) {
   out_8235373787822020756[0] = 1;
   out_8235373787822020756[1] = 0;
   out_8235373787822020756[2] = 0;
   out_8235373787822020756[3] = 0;
   out_8235373787822020756[4] = 0;
   out_8235373787822020756[5] = 0;
   out_8235373787822020756[6] = 0;
   out_8235373787822020756[7] = 0;
   out_8235373787822020756[8] = 0;
}
void h_31(double *state, double *unused, double *out_4761972549937326018) {
   out_4761972549937326018[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5921497080962502819) {
   out_5921497080962502819[0] = 0;
   out_5921497080962502819[1] = 0;
   out_5921497080962502819[2] = 0;
   out_5921497080962502819[3] = 0;
   out_5921497080962502819[4] = 0;
   out_5921497080962502819[5] = 0;
   out_5921497080962502819[6] = 0;
   out_5921497080962502819[7] = 0;
   out_5921497080962502819[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7453460125519006838) {
  err_fun(nom_x, delta_x, out_7453460125519006838);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5088499265255808698) {
  inv_err_fun(nom_x, true_x, out_5088499265255808698);
}
void car_H_mod_fun(double *state, double *out_1035652654272596140) {
  H_mod_fun(state, out_1035652654272596140);
}
void car_f_fun(double *state, double dt, double *out_1273096831272741265) {
  f_fun(state,  dt, out_1273096831272741265);
}
void car_F_fun(double *state, double dt, double *out_4155690995932901086) {
  F_fun(state,  dt, out_4155690995932901086);
}
void car_h_25(double *state, double *unused, double *out_7529324327931565469) {
  h_25(state, unused, out_7529324327931565469);
}
void car_H_25(double *state, double *unused, double *out_8157535571639641097) {
  H_25(state, unused, out_8157535571639641097);
}
void car_h_24(double *state, double *unused, double *out_854981700122849040) {
  h_24(state, unused, out_854981700122849040);
}
void car_H_24(double *state, double *unused, double *out_8116558903064410953) {
  H_24(state, unused, out_8116558903064410953);
}
void car_h_30(double *state, double *unused, double *out_7254130265647059580) {
  h_30(state, unused, out_7254130265647059580);
}
void car_H_30(double *state, double *unused, double *out_5639202613132392470) {
  H_30(state, unused, out_5639202613132392470);
}
void car_h_26(double *state, double *unused, double *out_855088748233210434) {
  h_26(state, unused, out_855088748233210434);
}
void car_H_26(double *state, double *unused, double *out_6547705183195854295) {
  H_26(state, unused, out_6547705183195854295);
}
void car_h_27(double *state, double *unused, double *out_8313014424969288496) {
  h_27(state, unused, out_8313014424969288496);
}
void car_H_27(double *state, double *unused, double *out_3415608541948449253) {
  H_27(state, unused, out_3415608541948449253);
}
void car_h_29(double *state, double *unused, double *out_2252456997986213935) {
  h_29(state, unused, out_2252456997986213935);
}
void car_H_29(double *state, double *unused, double *out_5128971268818000286) {
  H_29(state, unused, out_5128971268818000286);
}
void car_h_28(double *state, double *unused, double *out_7369504572067626061) {
  h_28(state, unused, out_7369504572067626061);
}
void car_H_28(double *state, double *unused, double *out_8235373787822020756) {
  H_28(state, unused, out_8235373787822020756);
}
void car_h_31(double *state, double *unused, double *out_4761972549937326018) {
  h_31(state, unused, out_4761972549937326018);
}
void car_H_31(double *state, double *unused, double *out_5921497080962502819) {
  H_31(state, unused, out_5921497080962502819);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
