#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2020592376761707488);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6228169048164514030);
void car_H_mod_fun(double *state, double *out_2059008125999644243);
void car_f_fun(double *state, double dt, double *out_5517399123552293061);
void car_F_fun(double *state, double dt, double *out_1816212949820204834);
void car_h_25(double *state, double *unused, double *out_4403706546784777641);
void car_H_25(double *state, double *unused, double *out_4620687993836660668);
void car_h_24(double *state, double *unused, double *out_3318196902836985848);
void car_H_24(double *state, double *unused, double *out_5978702068998868886);
void car_h_30(double *state, double *unused, double *out_6161764876170724790);
void car_H_30(double *state, double *unused, double *out_92991663709052470);
void car_h_26(double *state, double *unused, double *out_7283305929877981181);
void car_H_26(double *state, double *unused, double *out_879184674962604444);
void car_h_27(double *state, double *unused, double *out_1081377484556054814);
void car_H_27(double *state, double *unused, double *out_2081771648091372441);
void car_h_29(double *state, double *unused, double *out_8541936248804943139);
void car_H_29(double *state, double *unused, double *out_603223008023444654);
void car_h_28(double *state, double *unused, double *out_7337202206515102068);
void car_H_28(double *state, double *unused, double *out_2566853279588770905);
void car_h_31(double *state, double *unused, double *out_766257018530755291);
void car_H_31(double *state, double *unused, double *out_252976572729252968);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}