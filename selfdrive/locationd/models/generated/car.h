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
void car_err_fun(double *nom_x, double *delta_x, double *out_7453460125519006838);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5088499265255808698);
void car_H_mod_fun(double *state, double *out_1035652654272596140);
void car_f_fun(double *state, double dt, double *out_1273096831272741265);
void car_F_fun(double *state, double dt, double *out_4155690995932901086);
void car_h_25(double *state, double *unused, double *out_7529324327931565469);
void car_H_25(double *state, double *unused, double *out_8157535571639641097);
void car_h_24(double *state, double *unused, double *out_854981700122849040);
void car_H_24(double *state, double *unused, double *out_8116558903064410953);
void car_h_30(double *state, double *unused, double *out_7254130265647059580);
void car_H_30(double *state, double *unused, double *out_5639202613132392470);
void car_h_26(double *state, double *unused, double *out_855088748233210434);
void car_H_26(double *state, double *unused, double *out_6547705183195854295);
void car_h_27(double *state, double *unused, double *out_8313014424969288496);
void car_H_27(double *state, double *unused, double *out_3415608541948449253);
void car_h_29(double *state, double *unused, double *out_2252456997986213935);
void car_H_29(double *state, double *unused, double *out_5128971268818000286);
void car_h_28(double *state, double *unused, double *out_7369504572067626061);
void car_H_28(double *state, double *unused, double *out_8235373787822020756);
void car_h_31(double *state, double *unused, double *out_4761972549937326018);
void car_H_31(double *state, double *unused, double *out_5921497080962502819);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}