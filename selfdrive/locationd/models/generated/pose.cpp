#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2763597537072923646) {
   out_2763597537072923646[0] = delta_x[0] + nom_x[0];
   out_2763597537072923646[1] = delta_x[1] + nom_x[1];
   out_2763597537072923646[2] = delta_x[2] + nom_x[2];
   out_2763597537072923646[3] = delta_x[3] + nom_x[3];
   out_2763597537072923646[4] = delta_x[4] + nom_x[4];
   out_2763597537072923646[5] = delta_x[5] + nom_x[5];
   out_2763597537072923646[6] = delta_x[6] + nom_x[6];
   out_2763597537072923646[7] = delta_x[7] + nom_x[7];
   out_2763597537072923646[8] = delta_x[8] + nom_x[8];
   out_2763597537072923646[9] = delta_x[9] + nom_x[9];
   out_2763597537072923646[10] = delta_x[10] + nom_x[10];
   out_2763597537072923646[11] = delta_x[11] + nom_x[11];
   out_2763597537072923646[12] = delta_x[12] + nom_x[12];
   out_2763597537072923646[13] = delta_x[13] + nom_x[13];
   out_2763597537072923646[14] = delta_x[14] + nom_x[14];
   out_2763597537072923646[15] = delta_x[15] + nom_x[15];
   out_2763597537072923646[16] = delta_x[16] + nom_x[16];
   out_2763597537072923646[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2518170697447024659) {
   out_2518170697447024659[0] = -nom_x[0] + true_x[0];
   out_2518170697447024659[1] = -nom_x[1] + true_x[1];
   out_2518170697447024659[2] = -nom_x[2] + true_x[2];
   out_2518170697447024659[3] = -nom_x[3] + true_x[3];
   out_2518170697447024659[4] = -nom_x[4] + true_x[4];
   out_2518170697447024659[5] = -nom_x[5] + true_x[5];
   out_2518170697447024659[6] = -nom_x[6] + true_x[6];
   out_2518170697447024659[7] = -nom_x[7] + true_x[7];
   out_2518170697447024659[8] = -nom_x[8] + true_x[8];
   out_2518170697447024659[9] = -nom_x[9] + true_x[9];
   out_2518170697447024659[10] = -nom_x[10] + true_x[10];
   out_2518170697447024659[11] = -nom_x[11] + true_x[11];
   out_2518170697447024659[12] = -nom_x[12] + true_x[12];
   out_2518170697447024659[13] = -nom_x[13] + true_x[13];
   out_2518170697447024659[14] = -nom_x[14] + true_x[14];
   out_2518170697447024659[15] = -nom_x[15] + true_x[15];
   out_2518170697447024659[16] = -nom_x[16] + true_x[16];
   out_2518170697447024659[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4694189082729724861) {
   out_4694189082729724861[0] = 1.0;
   out_4694189082729724861[1] = 0.0;
   out_4694189082729724861[2] = 0.0;
   out_4694189082729724861[3] = 0.0;
   out_4694189082729724861[4] = 0.0;
   out_4694189082729724861[5] = 0.0;
   out_4694189082729724861[6] = 0.0;
   out_4694189082729724861[7] = 0.0;
   out_4694189082729724861[8] = 0.0;
   out_4694189082729724861[9] = 0.0;
   out_4694189082729724861[10] = 0.0;
   out_4694189082729724861[11] = 0.0;
   out_4694189082729724861[12] = 0.0;
   out_4694189082729724861[13] = 0.0;
   out_4694189082729724861[14] = 0.0;
   out_4694189082729724861[15] = 0.0;
   out_4694189082729724861[16] = 0.0;
   out_4694189082729724861[17] = 0.0;
   out_4694189082729724861[18] = 0.0;
   out_4694189082729724861[19] = 1.0;
   out_4694189082729724861[20] = 0.0;
   out_4694189082729724861[21] = 0.0;
   out_4694189082729724861[22] = 0.0;
   out_4694189082729724861[23] = 0.0;
   out_4694189082729724861[24] = 0.0;
   out_4694189082729724861[25] = 0.0;
   out_4694189082729724861[26] = 0.0;
   out_4694189082729724861[27] = 0.0;
   out_4694189082729724861[28] = 0.0;
   out_4694189082729724861[29] = 0.0;
   out_4694189082729724861[30] = 0.0;
   out_4694189082729724861[31] = 0.0;
   out_4694189082729724861[32] = 0.0;
   out_4694189082729724861[33] = 0.0;
   out_4694189082729724861[34] = 0.0;
   out_4694189082729724861[35] = 0.0;
   out_4694189082729724861[36] = 0.0;
   out_4694189082729724861[37] = 0.0;
   out_4694189082729724861[38] = 1.0;
   out_4694189082729724861[39] = 0.0;
   out_4694189082729724861[40] = 0.0;
   out_4694189082729724861[41] = 0.0;
   out_4694189082729724861[42] = 0.0;
   out_4694189082729724861[43] = 0.0;
   out_4694189082729724861[44] = 0.0;
   out_4694189082729724861[45] = 0.0;
   out_4694189082729724861[46] = 0.0;
   out_4694189082729724861[47] = 0.0;
   out_4694189082729724861[48] = 0.0;
   out_4694189082729724861[49] = 0.0;
   out_4694189082729724861[50] = 0.0;
   out_4694189082729724861[51] = 0.0;
   out_4694189082729724861[52] = 0.0;
   out_4694189082729724861[53] = 0.0;
   out_4694189082729724861[54] = 0.0;
   out_4694189082729724861[55] = 0.0;
   out_4694189082729724861[56] = 0.0;
   out_4694189082729724861[57] = 1.0;
   out_4694189082729724861[58] = 0.0;
   out_4694189082729724861[59] = 0.0;
   out_4694189082729724861[60] = 0.0;
   out_4694189082729724861[61] = 0.0;
   out_4694189082729724861[62] = 0.0;
   out_4694189082729724861[63] = 0.0;
   out_4694189082729724861[64] = 0.0;
   out_4694189082729724861[65] = 0.0;
   out_4694189082729724861[66] = 0.0;
   out_4694189082729724861[67] = 0.0;
   out_4694189082729724861[68] = 0.0;
   out_4694189082729724861[69] = 0.0;
   out_4694189082729724861[70] = 0.0;
   out_4694189082729724861[71] = 0.0;
   out_4694189082729724861[72] = 0.0;
   out_4694189082729724861[73] = 0.0;
   out_4694189082729724861[74] = 0.0;
   out_4694189082729724861[75] = 0.0;
   out_4694189082729724861[76] = 1.0;
   out_4694189082729724861[77] = 0.0;
   out_4694189082729724861[78] = 0.0;
   out_4694189082729724861[79] = 0.0;
   out_4694189082729724861[80] = 0.0;
   out_4694189082729724861[81] = 0.0;
   out_4694189082729724861[82] = 0.0;
   out_4694189082729724861[83] = 0.0;
   out_4694189082729724861[84] = 0.0;
   out_4694189082729724861[85] = 0.0;
   out_4694189082729724861[86] = 0.0;
   out_4694189082729724861[87] = 0.0;
   out_4694189082729724861[88] = 0.0;
   out_4694189082729724861[89] = 0.0;
   out_4694189082729724861[90] = 0.0;
   out_4694189082729724861[91] = 0.0;
   out_4694189082729724861[92] = 0.0;
   out_4694189082729724861[93] = 0.0;
   out_4694189082729724861[94] = 0.0;
   out_4694189082729724861[95] = 1.0;
   out_4694189082729724861[96] = 0.0;
   out_4694189082729724861[97] = 0.0;
   out_4694189082729724861[98] = 0.0;
   out_4694189082729724861[99] = 0.0;
   out_4694189082729724861[100] = 0.0;
   out_4694189082729724861[101] = 0.0;
   out_4694189082729724861[102] = 0.0;
   out_4694189082729724861[103] = 0.0;
   out_4694189082729724861[104] = 0.0;
   out_4694189082729724861[105] = 0.0;
   out_4694189082729724861[106] = 0.0;
   out_4694189082729724861[107] = 0.0;
   out_4694189082729724861[108] = 0.0;
   out_4694189082729724861[109] = 0.0;
   out_4694189082729724861[110] = 0.0;
   out_4694189082729724861[111] = 0.0;
   out_4694189082729724861[112] = 0.0;
   out_4694189082729724861[113] = 0.0;
   out_4694189082729724861[114] = 1.0;
   out_4694189082729724861[115] = 0.0;
   out_4694189082729724861[116] = 0.0;
   out_4694189082729724861[117] = 0.0;
   out_4694189082729724861[118] = 0.0;
   out_4694189082729724861[119] = 0.0;
   out_4694189082729724861[120] = 0.0;
   out_4694189082729724861[121] = 0.0;
   out_4694189082729724861[122] = 0.0;
   out_4694189082729724861[123] = 0.0;
   out_4694189082729724861[124] = 0.0;
   out_4694189082729724861[125] = 0.0;
   out_4694189082729724861[126] = 0.0;
   out_4694189082729724861[127] = 0.0;
   out_4694189082729724861[128] = 0.0;
   out_4694189082729724861[129] = 0.0;
   out_4694189082729724861[130] = 0.0;
   out_4694189082729724861[131] = 0.0;
   out_4694189082729724861[132] = 0.0;
   out_4694189082729724861[133] = 1.0;
   out_4694189082729724861[134] = 0.0;
   out_4694189082729724861[135] = 0.0;
   out_4694189082729724861[136] = 0.0;
   out_4694189082729724861[137] = 0.0;
   out_4694189082729724861[138] = 0.0;
   out_4694189082729724861[139] = 0.0;
   out_4694189082729724861[140] = 0.0;
   out_4694189082729724861[141] = 0.0;
   out_4694189082729724861[142] = 0.0;
   out_4694189082729724861[143] = 0.0;
   out_4694189082729724861[144] = 0.0;
   out_4694189082729724861[145] = 0.0;
   out_4694189082729724861[146] = 0.0;
   out_4694189082729724861[147] = 0.0;
   out_4694189082729724861[148] = 0.0;
   out_4694189082729724861[149] = 0.0;
   out_4694189082729724861[150] = 0.0;
   out_4694189082729724861[151] = 0.0;
   out_4694189082729724861[152] = 1.0;
   out_4694189082729724861[153] = 0.0;
   out_4694189082729724861[154] = 0.0;
   out_4694189082729724861[155] = 0.0;
   out_4694189082729724861[156] = 0.0;
   out_4694189082729724861[157] = 0.0;
   out_4694189082729724861[158] = 0.0;
   out_4694189082729724861[159] = 0.0;
   out_4694189082729724861[160] = 0.0;
   out_4694189082729724861[161] = 0.0;
   out_4694189082729724861[162] = 0.0;
   out_4694189082729724861[163] = 0.0;
   out_4694189082729724861[164] = 0.0;
   out_4694189082729724861[165] = 0.0;
   out_4694189082729724861[166] = 0.0;
   out_4694189082729724861[167] = 0.0;
   out_4694189082729724861[168] = 0.0;
   out_4694189082729724861[169] = 0.0;
   out_4694189082729724861[170] = 0.0;
   out_4694189082729724861[171] = 1.0;
   out_4694189082729724861[172] = 0.0;
   out_4694189082729724861[173] = 0.0;
   out_4694189082729724861[174] = 0.0;
   out_4694189082729724861[175] = 0.0;
   out_4694189082729724861[176] = 0.0;
   out_4694189082729724861[177] = 0.0;
   out_4694189082729724861[178] = 0.0;
   out_4694189082729724861[179] = 0.0;
   out_4694189082729724861[180] = 0.0;
   out_4694189082729724861[181] = 0.0;
   out_4694189082729724861[182] = 0.0;
   out_4694189082729724861[183] = 0.0;
   out_4694189082729724861[184] = 0.0;
   out_4694189082729724861[185] = 0.0;
   out_4694189082729724861[186] = 0.0;
   out_4694189082729724861[187] = 0.0;
   out_4694189082729724861[188] = 0.0;
   out_4694189082729724861[189] = 0.0;
   out_4694189082729724861[190] = 1.0;
   out_4694189082729724861[191] = 0.0;
   out_4694189082729724861[192] = 0.0;
   out_4694189082729724861[193] = 0.0;
   out_4694189082729724861[194] = 0.0;
   out_4694189082729724861[195] = 0.0;
   out_4694189082729724861[196] = 0.0;
   out_4694189082729724861[197] = 0.0;
   out_4694189082729724861[198] = 0.0;
   out_4694189082729724861[199] = 0.0;
   out_4694189082729724861[200] = 0.0;
   out_4694189082729724861[201] = 0.0;
   out_4694189082729724861[202] = 0.0;
   out_4694189082729724861[203] = 0.0;
   out_4694189082729724861[204] = 0.0;
   out_4694189082729724861[205] = 0.0;
   out_4694189082729724861[206] = 0.0;
   out_4694189082729724861[207] = 0.0;
   out_4694189082729724861[208] = 0.0;
   out_4694189082729724861[209] = 1.0;
   out_4694189082729724861[210] = 0.0;
   out_4694189082729724861[211] = 0.0;
   out_4694189082729724861[212] = 0.0;
   out_4694189082729724861[213] = 0.0;
   out_4694189082729724861[214] = 0.0;
   out_4694189082729724861[215] = 0.0;
   out_4694189082729724861[216] = 0.0;
   out_4694189082729724861[217] = 0.0;
   out_4694189082729724861[218] = 0.0;
   out_4694189082729724861[219] = 0.0;
   out_4694189082729724861[220] = 0.0;
   out_4694189082729724861[221] = 0.0;
   out_4694189082729724861[222] = 0.0;
   out_4694189082729724861[223] = 0.0;
   out_4694189082729724861[224] = 0.0;
   out_4694189082729724861[225] = 0.0;
   out_4694189082729724861[226] = 0.0;
   out_4694189082729724861[227] = 0.0;
   out_4694189082729724861[228] = 1.0;
   out_4694189082729724861[229] = 0.0;
   out_4694189082729724861[230] = 0.0;
   out_4694189082729724861[231] = 0.0;
   out_4694189082729724861[232] = 0.0;
   out_4694189082729724861[233] = 0.0;
   out_4694189082729724861[234] = 0.0;
   out_4694189082729724861[235] = 0.0;
   out_4694189082729724861[236] = 0.0;
   out_4694189082729724861[237] = 0.0;
   out_4694189082729724861[238] = 0.0;
   out_4694189082729724861[239] = 0.0;
   out_4694189082729724861[240] = 0.0;
   out_4694189082729724861[241] = 0.0;
   out_4694189082729724861[242] = 0.0;
   out_4694189082729724861[243] = 0.0;
   out_4694189082729724861[244] = 0.0;
   out_4694189082729724861[245] = 0.0;
   out_4694189082729724861[246] = 0.0;
   out_4694189082729724861[247] = 1.0;
   out_4694189082729724861[248] = 0.0;
   out_4694189082729724861[249] = 0.0;
   out_4694189082729724861[250] = 0.0;
   out_4694189082729724861[251] = 0.0;
   out_4694189082729724861[252] = 0.0;
   out_4694189082729724861[253] = 0.0;
   out_4694189082729724861[254] = 0.0;
   out_4694189082729724861[255] = 0.0;
   out_4694189082729724861[256] = 0.0;
   out_4694189082729724861[257] = 0.0;
   out_4694189082729724861[258] = 0.0;
   out_4694189082729724861[259] = 0.0;
   out_4694189082729724861[260] = 0.0;
   out_4694189082729724861[261] = 0.0;
   out_4694189082729724861[262] = 0.0;
   out_4694189082729724861[263] = 0.0;
   out_4694189082729724861[264] = 0.0;
   out_4694189082729724861[265] = 0.0;
   out_4694189082729724861[266] = 1.0;
   out_4694189082729724861[267] = 0.0;
   out_4694189082729724861[268] = 0.0;
   out_4694189082729724861[269] = 0.0;
   out_4694189082729724861[270] = 0.0;
   out_4694189082729724861[271] = 0.0;
   out_4694189082729724861[272] = 0.0;
   out_4694189082729724861[273] = 0.0;
   out_4694189082729724861[274] = 0.0;
   out_4694189082729724861[275] = 0.0;
   out_4694189082729724861[276] = 0.0;
   out_4694189082729724861[277] = 0.0;
   out_4694189082729724861[278] = 0.0;
   out_4694189082729724861[279] = 0.0;
   out_4694189082729724861[280] = 0.0;
   out_4694189082729724861[281] = 0.0;
   out_4694189082729724861[282] = 0.0;
   out_4694189082729724861[283] = 0.0;
   out_4694189082729724861[284] = 0.0;
   out_4694189082729724861[285] = 1.0;
   out_4694189082729724861[286] = 0.0;
   out_4694189082729724861[287] = 0.0;
   out_4694189082729724861[288] = 0.0;
   out_4694189082729724861[289] = 0.0;
   out_4694189082729724861[290] = 0.0;
   out_4694189082729724861[291] = 0.0;
   out_4694189082729724861[292] = 0.0;
   out_4694189082729724861[293] = 0.0;
   out_4694189082729724861[294] = 0.0;
   out_4694189082729724861[295] = 0.0;
   out_4694189082729724861[296] = 0.0;
   out_4694189082729724861[297] = 0.0;
   out_4694189082729724861[298] = 0.0;
   out_4694189082729724861[299] = 0.0;
   out_4694189082729724861[300] = 0.0;
   out_4694189082729724861[301] = 0.0;
   out_4694189082729724861[302] = 0.0;
   out_4694189082729724861[303] = 0.0;
   out_4694189082729724861[304] = 1.0;
   out_4694189082729724861[305] = 0.0;
   out_4694189082729724861[306] = 0.0;
   out_4694189082729724861[307] = 0.0;
   out_4694189082729724861[308] = 0.0;
   out_4694189082729724861[309] = 0.0;
   out_4694189082729724861[310] = 0.0;
   out_4694189082729724861[311] = 0.0;
   out_4694189082729724861[312] = 0.0;
   out_4694189082729724861[313] = 0.0;
   out_4694189082729724861[314] = 0.0;
   out_4694189082729724861[315] = 0.0;
   out_4694189082729724861[316] = 0.0;
   out_4694189082729724861[317] = 0.0;
   out_4694189082729724861[318] = 0.0;
   out_4694189082729724861[319] = 0.0;
   out_4694189082729724861[320] = 0.0;
   out_4694189082729724861[321] = 0.0;
   out_4694189082729724861[322] = 0.0;
   out_4694189082729724861[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_793777746767564627) {
   out_793777746767564627[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_793777746767564627[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_793777746767564627[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_793777746767564627[3] = dt*state[12] + state[3];
   out_793777746767564627[4] = dt*state[13] + state[4];
   out_793777746767564627[5] = dt*state[14] + state[5];
   out_793777746767564627[6] = state[6];
   out_793777746767564627[7] = state[7];
   out_793777746767564627[8] = state[8];
   out_793777746767564627[9] = state[9];
   out_793777746767564627[10] = state[10];
   out_793777746767564627[11] = state[11];
   out_793777746767564627[12] = state[12];
   out_793777746767564627[13] = state[13];
   out_793777746767564627[14] = state[14];
   out_793777746767564627[15] = state[15];
   out_793777746767564627[16] = state[16];
   out_793777746767564627[17] = state[17];
}
void F_fun(double *state, double dt, double *out_713869417857701794) {
   out_713869417857701794[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_713869417857701794[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_713869417857701794[2] = 0;
   out_713869417857701794[3] = 0;
   out_713869417857701794[4] = 0;
   out_713869417857701794[5] = 0;
   out_713869417857701794[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_713869417857701794[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_713869417857701794[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_713869417857701794[9] = 0;
   out_713869417857701794[10] = 0;
   out_713869417857701794[11] = 0;
   out_713869417857701794[12] = 0;
   out_713869417857701794[13] = 0;
   out_713869417857701794[14] = 0;
   out_713869417857701794[15] = 0;
   out_713869417857701794[16] = 0;
   out_713869417857701794[17] = 0;
   out_713869417857701794[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_713869417857701794[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_713869417857701794[20] = 0;
   out_713869417857701794[21] = 0;
   out_713869417857701794[22] = 0;
   out_713869417857701794[23] = 0;
   out_713869417857701794[24] = 0;
   out_713869417857701794[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_713869417857701794[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_713869417857701794[27] = 0;
   out_713869417857701794[28] = 0;
   out_713869417857701794[29] = 0;
   out_713869417857701794[30] = 0;
   out_713869417857701794[31] = 0;
   out_713869417857701794[32] = 0;
   out_713869417857701794[33] = 0;
   out_713869417857701794[34] = 0;
   out_713869417857701794[35] = 0;
   out_713869417857701794[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_713869417857701794[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_713869417857701794[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_713869417857701794[39] = 0;
   out_713869417857701794[40] = 0;
   out_713869417857701794[41] = 0;
   out_713869417857701794[42] = 0;
   out_713869417857701794[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_713869417857701794[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_713869417857701794[45] = 0;
   out_713869417857701794[46] = 0;
   out_713869417857701794[47] = 0;
   out_713869417857701794[48] = 0;
   out_713869417857701794[49] = 0;
   out_713869417857701794[50] = 0;
   out_713869417857701794[51] = 0;
   out_713869417857701794[52] = 0;
   out_713869417857701794[53] = 0;
   out_713869417857701794[54] = 0;
   out_713869417857701794[55] = 0;
   out_713869417857701794[56] = 0;
   out_713869417857701794[57] = 1;
   out_713869417857701794[58] = 0;
   out_713869417857701794[59] = 0;
   out_713869417857701794[60] = 0;
   out_713869417857701794[61] = 0;
   out_713869417857701794[62] = 0;
   out_713869417857701794[63] = 0;
   out_713869417857701794[64] = 0;
   out_713869417857701794[65] = 0;
   out_713869417857701794[66] = dt;
   out_713869417857701794[67] = 0;
   out_713869417857701794[68] = 0;
   out_713869417857701794[69] = 0;
   out_713869417857701794[70] = 0;
   out_713869417857701794[71] = 0;
   out_713869417857701794[72] = 0;
   out_713869417857701794[73] = 0;
   out_713869417857701794[74] = 0;
   out_713869417857701794[75] = 0;
   out_713869417857701794[76] = 1;
   out_713869417857701794[77] = 0;
   out_713869417857701794[78] = 0;
   out_713869417857701794[79] = 0;
   out_713869417857701794[80] = 0;
   out_713869417857701794[81] = 0;
   out_713869417857701794[82] = 0;
   out_713869417857701794[83] = 0;
   out_713869417857701794[84] = 0;
   out_713869417857701794[85] = dt;
   out_713869417857701794[86] = 0;
   out_713869417857701794[87] = 0;
   out_713869417857701794[88] = 0;
   out_713869417857701794[89] = 0;
   out_713869417857701794[90] = 0;
   out_713869417857701794[91] = 0;
   out_713869417857701794[92] = 0;
   out_713869417857701794[93] = 0;
   out_713869417857701794[94] = 0;
   out_713869417857701794[95] = 1;
   out_713869417857701794[96] = 0;
   out_713869417857701794[97] = 0;
   out_713869417857701794[98] = 0;
   out_713869417857701794[99] = 0;
   out_713869417857701794[100] = 0;
   out_713869417857701794[101] = 0;
   out_713869417857701794[102] = 0;
   out_713869417857701794[103] = 0;
   out_713869417857701794[104] = dt;
   out_713869417857701794[105] = 0;
   out_713869417857701794[106] = 0;
   out_713869417857701794[107] = 0;
   out_713869417857701794[108] = 0;
   out_713869417857701794[109] = 0;
   out_713869417857701794[110] = 0;
   out_713869417857701794[111] = 0;
   out_713869417857701794[112] = 0;
   out_713869417857701794[113] = 0;
   out_713869417857701794[114] = 1;
   out_713869417857701794[115] = 0;
   out_713869417857701794[116] = 0;
   out_713869417857701794[117] = 0;
   out_713869417857701794[118] = 0;
   out_713869417857701794[119] = 0;
   out_713869417857701794[120] = 0;
   out_713869417857701794[121] = 0;
   out_713869417857701794[122] = 0;
   out_713869417857701794[123] = 0;
   out_713869417857701794[124] = 0;
   out_713869417857701794[125] = 0;
   out_713869417857701794[126] = 0;
   out_713869417857701794[127] = 0;
   out_713869417857701794[128] = 0;
   out_713869417857701794[129] = 0;
   out_713869417857701794[130] = 0;
   out_713869417857701794[131] = 0;
   out_713869417857701794[132] = 0;
   out_713869417857701794[133] = 1;
   out_713869417857701794[134] = 0;
   out_713869417857701794[135] = 0;
   out_713869417857701794[136] = 0;
   out_713869417857701794[137] = 0;
   out_713869417857701794[138] = 0;
   out_713869417857701794[139] = 0;
   out_713869417857701794[140] = 0;
   out_713869417857701794[141] = 0;
   out_713869417857701794[142] = 0;
   out_713869417857701794[143] = 0;
   out_713869417857701794[144] = 0;
   out_713869417857701794[145] = 0;
   out_713869417857701794[146] = 0;
   out_713869417857701794[147] = 0;
   out_713869417857701794[148] = 0;
   out_713869417857701794[149] = 0;
   out_713869417857701794[150] = 0;
   out_713869417857701794[151] = 0;
   out_713869417857701794[152] = 1;
   out_713869417857701794[153] = 0;
   out_713869417857701794[154] = 0;
   out_713869417857701794[155] = 0;
   out_713869417857701794[156] = 0;
   out_713869417857701794[157] = 0;
   out_713869417857701794[158] = 0;
   out_713869417857701794[159] = 0;
   out_713869417857701794[160] = 0;
   out_713869417857701794[161] = 0;
   out_713869417857701794[162] = 0;
   out_713869417857701794[163] = 0;
   out_713869417857701794[164] = 0;
   out_713869417857701794[165] = 0;
   out_713869417857701794[166] = 0;
   out_713869417857701794[167] = 0;
   out_713869417857701794[168] = 0;
   out_713869417857701794[169] = 0;
   out_713869417857701794[170] = 0;
   out_713869417857701794[171] = 1;
   out_713869417857701794[172] = 0;
   out_713869417857701794[173] = 0;
   out_713869417857701794[174] = 0;
   out_713869417857701794[175] = 0;
   out_713869417857701794[176] = 0;
   out_713869417857701794[177] = 0;
   out_713869417857701794[178] = 0;
   out_713869417857701794[179] = 0;
   out_713869417857701794[180] = 0;
   out_713869417857701794[181] = 0;
   out_713869417857701794[182] = 0;
   out_713869417857701794[183] = 0;
   out_713869417857701794[184] = 0;
   out_713869417857701794[185] = 0;
   out_713869417857701794[186] = 0;
   out_713869417857701794[187] = 0;
   out_713869417857701794[188] = 0;
   out_713869417857701794[189] = 0;
   out_713869417857701794[190] = 1;
   out_713869417857701794[191] = 0;
   out_713869417857701794[192] = 0;
   out_713869417857701794[193] = 0;
   out_713869417857701794[194] = 0;
   out_713869417857701794[195] = 0;
   out_713869417857701794[196] = 0;
   out_713869417857701794[197] = 0;
   out_713869417857701794[198] = 0;
   out_713869417857701794[199] = 0;
   out_713869417857701794[200] = 0;
   out_713869417857701794[201] = 0;
   out_713869417857701794[202] = 0;
   out_713869417857701794[203] = 0;
   out_713869417857701794[204] = 0;
   out_713869417857701794[205] = 0;
   out_713869417857701794[206] = 0;
   out_713869417857701794[207] = 0;
   out_713869417857701794[208] = 0;
   out_713869417857701794[209] = 1;
   out_713869417857701794[210] = 0;
   out_713869417857701794[211] = 0;
   out_713869417857701794[212] = 0;
   out_713869417857701794[213] = 0;
   out_713869417857701794[214] = 0;
   out_713869417857701794[215] = 0;
   out_713869417857701794[216] = 0;
   out_713869417857701794[217] = 0;
   out_713869417857701794[218] = 0;
   out_713869417857701794[219] = 0;
   out_713869417857701794[220] = 0;
   out_713869417857701794[221] = 0;
   out_713869417857701794[222] = 0;
   out_713869417857701794[223] = 0;
   out_713869417857701794[224] = 0;
   out_713869417857701794[225] = 0;
   out_713869417857701794[226] = 0;
   out_713869417857701794[227] = 0;
   out_713869417857701794[228] = 1;
   out_713869417857701794[229] = 0;
   out_713869417857701794[230] = 0;
   out_713869417857701794[231] = 0;
   out_713869417857701794[232] = 0;
   out_713869417857701794[233] = 0;
   out_713869417857701794[234] = 0;
   out_713869417857701794[235] = 0;
   out_713869417857701794[236] = 0;
   out_713869417857701794[237] = 0;
   out_713869417857701794[238] = 0;
   out_713869417857701794[239] = 0;
   out_713869417857701794[240] = 0;
   out_713869417857701794[241] = 0;
   out_713869417857701794[242] = 0;
   out_713869417857701794[243] = 0;
   out_713869417857701794[244] = 0;
   out_713869417857701794[245] = 0;
   out_713869417857701794[246] = 0;
   out_713869417857701794[247] = 1;
   out_713869417857701794[248] = 0;
   out_713869417857701794[249] = 0;
   out_713869417857701794[250] = 0;
   out_713869417857701794[251] = 0;
   out_713869417857701794[252] = 0;
   out_713869417857701794[253] = 0;
   out_713869417857701794[254] = 0;
   out_713869417857701794[255] = 0;
   out_713869417857701794[256] = 0;
   out_713869417857701794[257] = 0;
   out_713869417857701794[258] = 0;
   out_713869417857701794[259] = 0;
   out_713869417857701794[260] = 0;
   out_713869417857701794[261] = 0;
   out_713869417857701794[262] = 0;
   out_713869417857701794[263] = 0;
   out_713869417857701794[264] = 0;
   out_713869417857701794[265] = 0;
   out_713869417857701794[266] = 1;
   out_713869417857701794[267] = 0;
   out_713869417857701794[268] = 0;
   out_713869417857701794[269] = 0;
   out_713869417857701794[270] = 0;
   out_713869417857701794[271] = 0;
   out_713869417857701794[272] = 0;
   out_713869417857701794[273] = 0;
   out_713869417857701794[274] = 0;
   out_713869417857701794[275] = 0;
   out_713869417857701794[276] = 0;
   out_713869417857701794[277] = 0;
   out_713869417857701794[278] = 0;
   out_713869417857701794[279] = 0;
   out_713869417857701794[280] = 0;
   out_713869417857701794[281] = 0;
   out_713869417857701794[282] = 0;
   out_713869417857701794[283] = 0;
   out_713869417857701794[284] = 0;
   out_713869417857701794[285] = 1;
   out_713869417857701794[286] = 0;
   out_713869417857701794[287] = 0;
   out_713869417857701794[288] = 0;
   out_713869417857701794[289] = 0;
   out_713869417857701794[290] = 0;
   out_713869417857701794[291] = 0;
   out_713869417857701794[292] = 0;
   out_713869417857701794[293] = 0;
   out_713869417857701794[294] = 0;
   out_713869417857701794[295] = 0;
   out_713869417857701794[296] = 0;
   out_713869417857701794[297] = 0;
   out_713869417857701794[298] = 0;
   out_713869417857701794[299] = 0;
   out_713869417857701794[300] = 0;
   out_713869417857701794[301] = 0;
   out_713869417857701794[302] = 0;
   out_713869417857701794[303] = 0;
   out_713869417857701794[304] = 1;
   out_713869417857701794[305] = 0;
   out_713869417857701794[306] = 0;
   out_713869417857701794[307] = 0;
   out_713869417857701794[308] = 0;
   out_713869417857701794[309] = 0;
   out_713869417857701794[310] = 0;
   out_713869417857701794[311] = 0;
   out_713869417857701794[312] = 0;
   out_713869417857701794[313] = 0;
   out_713869417857701794[314] = 0;
   out_713869417857701794[315] = 0;
   out_713869417857701794[316] = 0;
   out_713869417857701794[317] = 0;
   out_713869417857701794[318] = 0;
   out_713869417857701794[319] = 0;
   out_713869417857701794[320] = 0;
   out_713869417857701794[321] = 0;
   out_713869417857701794[322] = 0;
   out_713869417857701794[323] = 1;
}
void h_4(double *state, double *unused, double *out_7965232407703054575) {
   out_7965232407703054575[0] = state[6] + state[9];
   out_7965232407703054575[1] = state[7] + state[10];
   out_7965232407703054575[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6966467242834448328) {
   out_6966467242834448328[0] = 0;
   out_6966467242834448328[1] = 0;
   out_6966467242834448328[2] = 0;
   out_6966467242834448328[3] = 0;
   out_6966467242834448328[4] = 0;
   out_6966467242834448328[5] = 0;
   out_6966467242834448328[6] = 1;
   out_6966467242834448328[7] = 0;
   out_6966467242834448328[8] = 0;
   out_6966467242834448328[9] = 1;
   out_6966467242834448328[10] = 0;
   out_6966467242834448328[11] = 0;
   out_6966467242834448328[12] = 0;
   out_6966467242834448328[13] = 0;
   out_6966467242834448328[14] = 0;
   out_6966467242834448328[15] = 0;
   out_6966467242834448328[16] = 0;
   out_6966467242834448328[17] = 0;
   out_6966467242834448328[18] = 0;
   out_6966467242834448328[19] = 0;
   out_6966467242834448328[20] = 0;
   out_6966467242834448328[21] = 0;
   out_6966467242834448328[22] = 0;
   out_6966467242834448328[23] = 0;
   out_6966467242834448328[24] = 0;
   out_6966467242834448328[25] = 1;
   out_6966467242834448328[26] = 0;
   out_6966467242834448328[27] = 0;
   out_6966467242834448328[28] = 1;
   out_6966467242834448328[29] = 0;
   out_6966467242834448328[30] = 0;
   out_6966467242834448328[31] = 0;
   out_6966467242834448328[32] = 0;
   out_6966467242834448328[33] = 0;
   out_6966467242834448328[34] = 0;
   out_6966467242834448328[35] = 0;
   out_6966467242834448328[36] = 0;
   out_6966467242834448328[37] = 0;
   out_6966467242834448328[38] = 0;
   out_6966467242834448328[39] = 0;
   out_6966467242834448328[40] = 0;
   out_6966467242834448328[41] = 0;
   out_6966467242834448328[42] = 0;
   out_6966467242834448328[43] = 0;
   out_6966467242834448328[44] = 1;
   out_6966467242834448328[45] = 0;
   out_6966467242834448328[46] = 0;
   out_6966467242834448328[47] = 1;
   out_6966467242834448328[48] = 0;
   out_6966467242834448328[49] = 0;
   out_6966467242834448328[50] = 0;
   out_6966467242834448328[51] = 0;
   out_6966467242834448328[52] = 0;
   out_6966467242834448328[53] = 0;
}
void h_10(double *state, double *unused, double *out_3313691075049490186) {
   out_3313691075049490186[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3313691075049490186[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3313691075049490186[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4634573071595528693) {
   out_4634573071595528693[0] = 0;
   out_4634573071595528693[1] = 9.8100000000000005*cos(state[1]);
   out_4634573071595528693[2] = 0;
   out_4634573071595528693[3] = 0;
   out_4634573071595528693[4] = -state[8];
   out_4634573071595528693[5] = state[7];
   out_4634573071595528693[6] = 0;
   out_4634573071595528693[7] = state[5];
   out_4634573071595528693[8] = -state[4];
   out_4634573071595528693[9] = 0;
   out_4634573071595528693[10] = 0;
   out_4634573071595528693[11] = 0;
   out_4634573071595528693[12] = 1;
   out_4634573071595528693[13] = 0;
   out_4634573071595528693[14] = 0;
   out_4634573071595528693[15] = 1;
   out_4634573071595528693[16] = 0;
   out_4634573071595528693[17] = 0;
   out_4634573071595528693[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4634573071595528693[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4634573071595528693[20] = 0;
   out_4634573071595528693[21] = state[8];
   out_4634573071595528693[22] = 0;
   out_4634573071595528693[23] = -state[6];
   out_4634573071595528693[24] = -state[5];
   out_4634573071595528693[25] = 0;
   out_4634573071595528693[26] = state[3];
   out_4634573071595528693[27] = 0;
   out_4634573071595528693[28] = 0;
   out_4634573071595528693[29] = 0;
   out_4634573071595528693[30] = 0;
   out_4634573071595528693[31] = 1;
   out_4634573071595528693[32] = 0;
   out_4634573071595528693[33] = 0;
   out_4634573071595528693[34] = 1;
   out_4634573071595528693[35] = 0;
   out_4634573071595528693[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4634573071595528693[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4634573071595528693[38] = 0;
   out_4634573071595528693[39] = -state[7];
   out_4634573071595528693[40] = state[6];
   out_4634573071595528693[41] = 0;
   out_4634573071595528693[42] = state[4];
   out_4634573071595528693[43] = -state[3];
   out_4634573071595528693[44] = 0;
   out_4634573071595528693[45] = 0;
   out_4634573071595528693[46] = 0;
   out_4634573071595528693[47] = 0;
   out_4634573071595528693[48] = 0;
   out_4634573071595528693[49] = 0;
   out_4634573071595528693[50] = 1;
   out_4634573071595528693[51] = 0;
   out_4634573071595528693[52] = 0;
   out_4634573071595528693[53] = 1;
}
void h_13(double *state, double *unused, double *out_3184703805746577303) {
   out_3184703805746577303[0] = state[3];
   out_3184703805746577303[1] = state[4];
   out_3184703805746577303[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3132711779531924304) {
   out_3132711779531924304[0] = 0;
   out_3132711779531924304[1] = 0;
   out_3132711779531924304[2] = 0;
   out_3132711779531924304[3] = 1;
   out_3132711779531924304[4] = 0;
   out_3132711779531924304[5] = 0;
   out_3132711779531924304[6] = 0;
   out_3132711779531924304[7] = 0;
   out_3132711779531924304[8] = 0;
   out_3132711779531924304[9] = 0;
   out_3132711779531924304[10] = 0;
   out_3132711779531924304[11] = 0;
   out_3132711779531924304[12] = 0;
   out_3132711779531924304[13] = 0;
   out_3132711779531924304[14] = 0;
   out_3132711779531924304[15] = 0;
   out_3132711779531924304[16] = 0;
   out_3132711779531924304[17] = 0;
   out_3132711779531924304[18] = 0;
   out_3132711779531924304[19] = 0;
   out_3132711779531924304[20] = 0;
   out_3132711779531924304[21] = 0;
   out_3132711779531924304[22] = 1;
   out_3132711779531924304[23] = 0;
   out_3132711779531924304[24] = 0;
   out_3132711779531924304[25] = 0;
   out_3132711779531924304[26] = 0;
   out_3132711779531924304[27] = 0;
   out_3132711779531924304[28] = 0;
   out_3132711779531924304[29] = 0;
   out_3132711779531924304[30] = 0;
   out_3132711779531924304[31] = 0;
   out_3132711779531924304[32] = 0;
   out_3132711779531924304[33] = 0;
   out_3132711779531924304[34] = 0;
   out_3132711779531924304[35] = 0;
   out_3132711779531924304[36] = 0;
   out_3132711779531924304[37] = 0;
   out_3132711779531924304[38] = 0;
   out_3132711779531924304[39] = 0;
   out_3132711779531924304[40] = 0;
   out_3132711779531924304[41] = 1;
   out_3132711779531924304[42] = 0;
   out_3132711779531924304[43] = 0;
   out_3132711779531924304[44] = 0;
   out_3132711779531924304[45] = 0;
   out_3132711779531924304[46] = 0;
   out_3132711779531924304[47] = 0;
   out_3132711779531924304[48] = 0;
   out_3132711779531924304[49] = 0;
   out_3132711779531924304[50] = 0;
   out_3132711779531924304[51] = 0;
   out_3132711779531924304[52] = 0;
   out_3132711779531924304[53] = 0;
}
void h_14(double *state, double *unused, double *out_5549991908937183113) {
   out_5549991908937183113[0] = state[6];
   out_5549991908937183113[1] = state[7];
   out_5549991908937183113[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3883678810539076032) {
   out_3883678810539076032[0] = 0;
   out_3883678810539076032[1] = 0;
   out_3883678810539076032[2] = 0;
   out_3883678810539076032[3] = 0;
   out_3883678810539076032[4] = 0;
   out_3883678810539076032[5] = 0;
   out_3883678810539076032[6] = 1;
   out_3883678810539076032[7] = 0;
   out_3883678810539076032[8] = 0;
   out_3883678810539076032[9] = 0;
   out_3883678810539076032[10] = 0;
   out_3883678810539076032[11] = 0;
   out_3883678810539076032[12] = 0;
   out_3883678810539076032[13] = 0;
   out_3883678810539076032[14] = 0;
   out_3883678810539076032[15] = 0;
   out_3883678810539076032[16] = 0;
   out_3883678810539076032[17] = 0;
   out_3883678810539076032[18] = 0;
   out_3883678810539076032[19] = 0;
   out_3883678810539076032[20] = 0;
   out_3883678810539076032[21] = 0;
   out_3883678810539076032[22] = 0;
   out_3883678810539076032[23] = 0;
   out_3883678810539076032[24] = 0;
   out_3883678810539076032[25] = 1;
   out_3883678810539076032[26] = 0;
   out_3883678810539076032[27] = 0;
   out_3883678810539076032[28] = 0;
   out_3883678810539076032[29] = 0;
   out_3883678810539076032[30] = 0;
   out_3883678810539076032[31] = 0;
   out_3883678810539076032[32] = 0;
   out_3883678810539076032[33] = 0;
   out_3883678810539076032[34] = 0;
   out_3883678810539076032[35] = 0;
   out_3883678810539076032[36] = 0;
   out_3883678810539076032[37] = 0;
   out_3883678810539076032[38] = 0;
   out_3883678810539076032[39] = 0;
   out_3883678810539076032[40] = 0;
   out_3883678810539076032[41] = 0;
   out_3883678810539076032[42] = 0;
   out_3883678810539076032[43] = 0;
   out_3883678810539076032[44] = 1;
   out_3883678810539076032[45] = 0;
   out_3883678810539076032[46] = 0;
   out_3883678810539076032[47] = 0;
   out_3883678810539076032[48] = 0;
   out_3883678810539076032[49] = 0;
   out_3883678810539076032[50] = 0;
   out_3883678810539076032[51] = 0;
   out_3883678810539076032[52] = 0;
   out_3883678810539076032[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_2763597537072923646) {
  err_fun(nom_x, delta_x, out_2763597537072923646);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2518170697447024659) {
  inv_err_fun(nom_x, true_x, out_2518170697447024659);
}
void pose_H_mod_fun(double *state, double *out_4694189082729724861) {
  H_mod_fun(state, out_4694189082729724861);
}
void pose_f_fun(double *state, double dt, double *out_793777746767564627) {
  f_fun(state,  dt, out_793777746767564627);
}
void pose_F_fun(double *state, double dt, double *out_713869417857701794) {
  F_fun(state,  dt, out_713869417857701794);
}
void pose_h_4(double *state, double *unused, double *out_7965232407703054575) {
  h_4(state, unused, out_7965232407703054575);
}
void pose_H_4(double *state, double *unused, double *out_6966467242834448328) {
  H_4(state, unused, out_6966467242834448328);
}
void pose_h_10(double *state, double *unused, double *out_3313691075049490186) {
  h_10(state, unused, out_3313691075049490186);
}
void pose_H_10(double *state, double *unused, double *out_4634573071595528693) {
  H_10(state, unused, out_4634573071595528693);
}
void pose_h_13(double *state, double *unused, double *out_3184703805746577303) {
  h_13(state, unused, out_3184703805746577303);
}
void pose_H_13(double *state, double *unused, double *out_3132711779531924304) {
  H_13(state, unused, out_3132711779531924304);
}
void pose_h_14(double *state, double *unused, double *out_5549991908937183113) {
  h_14(state, unused, out_5549991908937183113);
}
void pose_H_14(double *state, double *unused, double *out_3883678810539076032) {
  H_14(state, unused, out_3883678810539076032);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
