#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6091666443681126136);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3296932361415467485);
void pose_H_mod_fun(double *state, double *out_4756587579081525558);
void pose_f_fun(double *state, double dt, double *out_356369068438679194);
void pose_F_fun(double *state, double dt, double *out_8347244080238777649);
void pose_h_4(double *state, double *unused, double *out_639306389444674144);
void pose_H_4(double *state, double *unused, double *out_141960542152209194);
void pose_h_10(double *state, double *unused, double *out_4208130342329524840);
void pose_H_10(double *state, double *unused, double *out_3882092915448509152);
void pose_h_13(double *state, double *unused, double *out_1415975790652254250);
void pose_H_13(double *state, double *unused, double *out_3070313283180123607);
void pose_h_14(double *state, double *unused, double *out_8443724961933214887);
void pose_H_14(double *state, double *unused, double *out_3821280314187275335);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}