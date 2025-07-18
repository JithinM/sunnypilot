#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2763597537072923646);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2518170697447024659);
void pose_H_mod_fun(double *state, double *out_4694189082729724861);
void pose_f_fun(double *state, double dt, double *out_793777746767564627);
void pose_F_fun(double *state, double dt, double *out_713869417857701794);
void pose_h_4(double *state, double *unused, double *out_7965232407703054575);
void pose_H_4(double *state, double *unused, double *out_6966467242834448328);
void pose_h_10(double *state, double *unused, double *out_3313691075049490186);
void pose_H_10(double *state, double *unused, double *out_4634573071595528693);
void pose_h_13(double *state, double *unused, double *out_3184703805746577303);
void pose_H_13(double *state, double *unused, double *out_3132711779531924304);
void pose_h_14(double *state, double *unused, double *out_5549991908937183113);
void pose_H_14(double *state, double *unused, double *out_3883678810539076032);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}