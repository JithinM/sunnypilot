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
void err_fun(double *nom_x, double *delta_x, double *out_2020592376761707488) {
   out_2020592376761707488[0] = delta_x[0] + nom_x[0];
   out_2020592376761707488[1] = delta_x[1] + nom_x[1];
   out_2020592376761707488[2] = delta_x[2] + nom_x[2];
   out_2020592376761707488[3] = delta_x[3] + nom_x[3];
   out_2020592376761707488[4] = delta_x[4] + nom_x[4];
   out_2020592376761707488[5] = delta_x[5] + nom_x[5];
   out_2020592376761707488[6] = delta_x[6] + nom_x[6];
   out_2020592376761707488[7] = delta_x[7] + nom_x[7];
   out_2020592376761707488[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6228169048164514030) {
   out_6228169048164514030[0] = -nom_x[0] + true_x[0];
   out_6228169048164514030[1] = -nom_x[1] + true_x[1];
   out_6228169048164514030[2] = -nom_x[2] + true_x[2];
   out_6228169048164514030[3] = -nom_x[3] + true_x[3];
   out_6228169048164514030[4] = -nom_x[4] + true_x[4];
   out_6228169048164514030[5] = -nom_x[5] + true_x[5];
   out_6228169048164514030[6] = -nom_x[6] + true_x[6];
   out_6228169048164514030[7] = -nom_x[7] + true_x[7];
   out_6228169048164514030[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2059008125999644243) {
   out_2059008125999644243[0] = 1.0;
   out_2059008125999644243[1] = 0.0;
   out_2059008125999644243[2] = 0.0;
   out_2059008125999644243[3] = 0.0;
   out_2059008125999644243[4] = 0.0;
   out_2059008125999644243[5] = 0.0;
   out_2059008125999644243[6] = 0.0;
   out_2059008125999644243[7] = 0.0;
   out_2059008125999644243[8] = 0.0;
   out_2059008125999644243[9] = 0.0;
   out_2059008125999644243[10] = 1.0;
   out_2059008125999644243[11] = 0.0;
   out_2059008125999644243[12] = 0.0;
   out_2059008125999644243[13] = 0.0;
   out_2059008125999644243[14] = 0.0;
   out_2059008125999644243[15] = 0.0;
   out_2059008125999644243[16] = 0.0;
   out_2059008125999644243[17] = 0.0;
   out_2059008125999644243[18] = 0.0;
   out_2059008125999644243[19] = 0.0;
   out_2059008125999644243[20] = 1.0;
   out_2059008125999644243[21] = 0.0;
   out_2059008125999644243[22] = 0.0;
   out_2059008125999644243[23] = 0.0;
   out_2059008125999644243[24] = 0.0;
   out_2059008125999644243[25] = 0.0;
   out_2059008125999644243[26] = 0.0;
   out_2059008125999644243[27] = 0.0;
   out_2059008125999644243[28] = 0.0;
   out_2059008125999644243[29] = 0.0;
   out_2059008125999644243[30] = 1.0;
   out_2059008125999644243[31] = 0.0;
   out_2059008125999644243[32] = 0.0;
   out_2059008125999644243[33] = 0.0;
   out_2059008125999644243[34] = 0.0;
   out_2059008125999644243[35] = 0.0;
   out_2059008125999644243[36] = 0.0;
   out_2059008125999644243[37] = 0.0;
   out_2059008125999644243[38] = 0.0;
   out_2059008125999644243[39] = 0.0;
   out_2059008125999644243[40] = 1.0;
   out_2059008125999644243[41] = 0.0;
   out_2059008125999644243[42] = 0.0;
   out_2059008125999644243[43] = 0.0;
   out_2059008125999644243[44] = 0.0;
   out_2059008125999644243[45] = 0.0;
   out_2059008125999644243[46] = 0.0;
   out_2059008125999644243[47] = 0.0;
   out_2059008125999644243[48] = 0.0;
   out_2059008125999644243[49] = 0.0;
   out_2059008125999644243[50] = 1.0;
   out_2059008125999644243[51] = 0.0;
   out_2059008125999644243[52] = 0.0;
   out_2059008125999644243[53] = 0.0;
   out_2059008125999644243[54] = 0.0;
   out_2059008125999644243[55] = 0.0;
   out_2059008125999644243[56] = 0.0;
   out_2059008125999644243[57] = 0.0;
   out_2059008125999644243[58] = 0.0;
   out_2059008125999644243[59] = 0.0;
   out_2059008125999644243[60] = 1.0;
   out_2059008125999644243[61] = 0.0;
   out_2059008125999644243[62] = 0.0;
   out_2059008125999644243[63] = 0.0;
   out_2059008125999644243[64] = 0.0;
   out_2059008125999644243[65] = 0.0;
   out_2059008125999644243[66] = 0.0;
   out_2059008125999644243[67] = 0.0;
   out_2059008125999644243[68] = 0.0;
   out_2059008125999644243[69] = 0.0;
   out_2059008125999644243[70] = 1.0;
   out_2059008125999644243[71] = 0.0;
   out_2059008125999644243[72] = 0.0;
   out_2059008125999644243[73] = 0.0;
   out_2059008125999644243[74] = 0.0;
   out_2059008125999644243[75] = 0.0;
   out_2059008125999644243[76] = 0.0;
   out_2059008125999644243[77] = 0.0;
   out_2059008125999644243[78] = 0.0;
   out_2059008125999644243[79] = 0.0;
   out_2059008125999644243[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5517399123552293061) {
   out_5517399123552293061[0] = state[0];
   out_5517399123552293061[1] = state[1];
   out_5517399123552293061[2] = state[2];
   out_5517399123552293061[3] = state[3];
   out_5517399123552293061[4] = state[4];
   out_5517399123552293061[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5517399123552293061[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5517399123552293061[7] = state[7];
   out_5517399123552293061[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1816212949820204834) {
   out_1816212949820204834[0] = 1;
   out_1816212949820204834[1] = 0;
   out_1816212949820204834[2] = 0;
   out_1816212949820204834[3] = 0;
   out_1816212949820204834[4] = 0;
   out_1816212949820204834[5] = 0;
   out_1816212949820204834[6] = 0;
   out_1816212949820204834[7] = 0;
   out_1816212949820204834[8] = 0;
   out_1816212949820204834[9] = 0;
   out_1816212949820204834[10] = 1;
   out_1816212949820204834[11] = 0;
   out_1816212949820204834[12] = 0;
   out_1816212949820204834[13] = 0;
   out_1816212949820204834[14] = 0;
   out_1816212949820204834[15] = 0;
   out_1816212949820204834[16] = 0;
   out_1816212949820204834[17] = 0;
   out_1816212949820204834[18] = 0;
   out_1816212949820204834[19] = 0;
   out_1816212949820204834[20] = 1;
   out_1816212949820204834[21] = 0;
   out_1816212949820204834[22] = 0;
   out_1816212949820204834[23] = 0;
   out_1816212949820204834[24] = 0;
   out_1816212949820204834[25] = 0;
   out_1816212949820204834[26] = 0;
   out_1816212949820204834[27] = 0;
   out_1816212949820204834[28] = 0;
   out_1816212949820204834[29] = 0;
   out_1816212949820204834[30] = 1;
   out_1816212949820204834[31] = 0;
   out_1816212949820204834[32] = 0;
   out_1816212949820204834[33] = 0;
   out_1816212949820204834[34] = 0;
   out_1816212949820204834[35] = 0;
   out_1816212949820204834[36] = 0;
   out_1816212949820204834[37] = 0;
   out_1816212949820204834[38] = 0;
   out_1816212949820204834[39] = 0;
   out_1816212949820204834[40] = 1;
   out_1816212949820204834[41] = 0;
   out_1816212949820204834[42] = 0;
   out_1816212949820204834[43] = 0;
   out_1816212949820204834[44] = 0;
   out_1816212949820204834[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1816212949820204834[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1816212949820204834[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1816212949820204834[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1816212949820204834[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1816212949820204834[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1816212949820204834[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1816212949820204834[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1816212949820204834[53] = -9.8000000000000007*dt;
   out_1816212949820204834[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1816212949820204834[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1816212949820204834[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1816212949820204834[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1816212949820204834[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1816212949820204834[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1816212949820204834[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1816212949820204834[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1816212949820204834[62] = 0;
   out_1816212949820204834[63] = 0;
   out_1816212949820204834[64] = 0;
   out_1816212949820204834[65] = 0;
   out_1816212949820204834[66] = 0;
   out_1816212949820204834[67] = 0;
   out_1816212949820204834[68] = 0;
   out_1816212949820204834[69] = 0;
   out_1816212949820204834[70] = 1;
   out_1816212949820204834[71] = 0;
   out_1816212949820204834[72] = 0;
   out_1816212949820204834[73] = 0;
   out_1816212949820204834[74] = 0;
   out_1816212949820204834[75] = 0;
   out_1816212949820204834[76] = 0;
   out_1816212949820204834[77] = 0;
   out_1816212949820204834[78] = 0;
   out_1816212949820204834[79] = 0;
   out_1816212949820204834[80] = 1;
}
void h_25(double *state, double *unused, double *out_4403706546784777641) {
   out_4403706546784777641[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4620687993836660668) {
   out_4620687993836660668[0] = 0;
   out_4620687993836660668[1] = 0;
   out_4620687993836660668[2] = 0;
   out_4620687993836660668[3] = 0;
   out_4620687993836660668[4] = 0;
   out_4620687993836660668[5] = 0;
   out_4620687993836660668[6] = 1;
   out_4620687993836660668[7] = 0;
   out_4620687993836660668[8] = 0;
}
void h_24(double *state, double *unused, double *out_3318196902836985848) {
   out_3318196902836985848[0] = state[4];
   out_3318196902836985848[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5978702068998868886) {
   out_5978702068998868886[0] = 0;
   out_5978702068998868886[1] = 0;
   out_5978702068998868886[2] = 0;
   out_5978702068998868886[3] = 0;
   out_5978702068998868886[4] = 1;
   out_5978702068998868886[5] = 0;
   out_5978702068998868886[6] = 0;
   out_5978702068998868886[7] = 0;
   out_5978702068998868886[8] = 0;
   out_5978702068998868886[9] = 0;
   out_5978702068998868886[10] = 0;
   out_5978702068998868886[11] = 0;
   out_5978702068998868886[12] = 0;
   out_5978702068998868886[13] = 0;
   out_5978702068998868886[14] = 1;
   out_5978702068998868886[15] = 0;
   out_5978702068998868886[16] = 0;
   out_5978702068998868886[17] = 0;
}
void h_30(double *state, double *unused, double *out_6161764876170724790) {
   out_6161764876170724790[0] = state[4];
}
void H_30(double *state, double *unused, double *out_92991663709052470) {
   out_92991663709052470[0] = 0;
   out_92991663709052470[1] = 0;
   out_92991663709052470[2] = 0;
   out_92991663709052470[3] = 0;
   out_92991663709052470[4] = 1;
   out_92991663709052470[5] = 0;
   out_92991663709052470[6] = 0;
   out_92991663709052470[7] = 0;
   out_92991663709052470[8] = 0;
}
void h_26(double *state, double *unused, double *out_7283305929877981181) {
   out_7283305929877981181[0] = state[7];
}
void H_26(double *state, double *unused, double *out_879184674962604444) {
   out_879184674962604444[0] = 0;
   out_879184674962604444[1] = 0;
   out_879184674962604444[2] = 0;
   out_879184674962604444[3] = 0;
   out_879184674962604444[4] = 0;
   out_879184674962604444[5] = 0;
   out_879184674962604444[6] = 0;
   out_879184674962604444[7] = 1;
   out_879184674962604444[8] = 0;
}
void h_27(double *state, double *unused, double *out_1081377484556054814) {
   out_1081377484556054814[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2081771648091372441) {
   out_2081771648091372441[0] = 0;
   out_2081771648091372441[1] = 0;
   out_2081771648091372441[2] = 0;
   out_2081771648091372441[3] = 1;
   out_2081771648091372441[4] = 0;
   out_2081771648091372441[5] = 0;
   out_2081771648091372441[6] = 0;
   out_2081771648091372441[7] = 0;
   out_2081771648091372441[8] = 0;
}
void h_29(double *state, double *unused, double *out_8541936248804943139) {
   out_8541936248804943139[0] = state[1];
}
void H_29(double *state, double *unused, double *out_603223008023444654) {
   out_603223008023444654[0] = 0;
   out_603223008023444654[1] = 1;
   out_603223008023444654[2] = 0;
   out_603223008023444654[3] = 0;
   out_603223008023444654[4] = 0;
   out_603223008023444654[5] = 0;
   out_603223008023444654[6] = 0;
   out_603223008023444654[7] = 0;
   out_603223008023444654[8] = 0;
}
void h_28(double *state, double *unused, double *out_7337202206515102068) {
   out_7337202206515102068[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2566853279588770905) {
   out_2566853279588770905[0] = 1;
   out_2566853279588770905[1] = 0;
   out_2566853279588770905[2] = 0;
   out_2566853279588770905[3] = 0;
   out_2566853279588770905[4] = 0;
   out_2566853279588770905[5] = 0;
   out_2566853279588770905[6] = 0;
   out_2566853279588770905[7] = 0;
   out_2566853279588770905[8] = 0;
}
void h_31(double *state, double *unused, double *out_766257018530755291) {
   out_766257018530755291[0] = state[8];
}
void H_31(double *state, double *unused, double *out_252976572729252968) {
   out_252976572729252968[0] = 0;
   out_252976572729252968[1] = 0;
   out_252976572729252968[2] = 0;
   out_252976572729252968[3] = 0;
   out_252976572729252968[4] = 0;
   out_252976572729252968[5] = 0;
   out_252976572729252968[6] = 0;
   out_252976572729252968[7] = 0;
   out_252976572729252968[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2020592376761707488) {
  err_fun(nom_x, delta_x, out_2020592376761707488);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6228169048164514030) {
  inv_err_fun(nom_x, true_x, out_6228169048164514030);
}
void car_H_mod_fun(double *state, double *out_2059008125999644243) {
  H_mod_fun(state, out_2059008125999644243);
}
void car_f_fun(double *state, double dt, double *out_5517399123552293061) {
  f_fun(state,  dt, out_5517399123552293061);
}
void car_F_fun(double *state, double dt, double *out_1816212949820204834) {
  F_fun(state,  dt, out_1816212949820204834);
}
void car_h_25(double *state, double *unused, double *out_4403706546784777641) {
  h_25(state, unused, out_4403706546784777641);
}
void car_H_25(double *state, double *unused, double *out_4620687993836660668) {
  H_25(state, unused, out_4620687993836660668);
}
void car_h_24(double *state, double *unused, double *out_3318196902836985848) {
  h_24(state, unused, out_3318196902836985848);
}
void car_H_24(double *state, double *unused, double *out_5978702068998868886) {
  H_24(state, unused, out_5978702068998868886);
}
void car_h_30(double *state, double *unused, double *out_6161764876170724790) {
  h_30(state, unused, out_6161764876170724790);
}
void car_H_30(double *state, double *unused, double *out_92991663709052470) {
  H_30(state, unused, out_92991663709052470);
}
void car_h_26(double *state, double *unused, double *out_7283305929877981181) {
  h_26(state, unused, out_7283305929877981181);
}
void car_H_26(double *state, double *unused, double *out_879184674962604444) {
  H_26(state, unused, out_879184674962604444);
}
void car_h_27(double *state, double *unused, double *out_1081377484556054814) {
  h_27(state, unused, out_1081377484556054814);
}
void car_H_27(double *state, double *unused, double *out_2081771648091372441) {
  H_27(state, unused, out_2081771648091372441);
}
void car_h_29(double *state, double *unused, double *out_8541936248804943139) {
  h_29(state, unused, out_8541936248804943139);
}
void car_H_29(double *state, double *unused, double *out_603223008023444654) {
  H_29(state, unused, out_603223008023444654);
}
void car_h_28(double *state, double *unused, double *out_7337202206515102068) {
  h_28(state, unused, out_7337202206515102068);
}
void car_H_28(double *state, double *unused, double *out_2566853279588770905) {
  H_28(state, unused, out_2566853279588770905);
}
void car_h_31(double *state, double *unused, double *out_766257018530755291) {
  h_31(state, unused, out_766257018530755291);
}
void car_H_31(double *state, double *unused, double *out_252976572729252968) {
  H_31(state, unused, out_252976572729252968);
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
