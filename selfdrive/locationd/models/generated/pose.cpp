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
void err_fun(double *nom_x, double *delta_x, double *out_6091666443681126136) {
   out_6091666443681126136[0] = delta_x[0] + nom_x[0];
   out_6091666443681126136[1] = delta_x[1] + nom_x[1];
   out_6091666443681126136[2] = delta_x[2] + nom_x[2];
   out_6091666443681126136[3] = delta_x[3] + nom_x[3];
   out_6091666443681126136[4] = delta_x[4] + nom_x[4];
   out_6091666443681126136[5] = delta_x[5] + nom_x[5];
   out_6091666443681126136[6] = delta_x[6] + nom_x[6];
   out_6091666443681126136[7] = delta_x[7] + nom_x[7];
   out_6091666443681126136[8] = delta_x[8] + nom_x[8];
   out_6091666443681126136[9] = delta_x[9] + nom_x[9];
   out_6091666443681126136[10] = delta_x[10] + nom_x[10];
   out_6091666443681126136[11] = delta_x[11] + nom_x[11];
   out_6091666443681126136[12] = delta_x[12] + nom_x[12];
   out_6091666443681126136[13] = delta_x[13] + nom_x[13];
   out_6091666443681126136[14] = delta_x[14] + nom_x[14];
   out_6091666443681126136[15] = delta_x[15] + nom_x[15];
   out_6091666443681126136[16] = delta_x[16] + nom_x[16];
   out_6091666443681126136[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3296932361415467485) {
   out_3296932361415467485[0] = -nom_x[0] + true_x[0];
   out_3296932361415467485[1] = -nom_x[1] + true_x[1];
   out_3296932361415467485[2] = -nom_x[2] + true_x[2];
   out_3296932361415467485[3] = -nom_x[3] + true_x[3];
   out_3296932361415467485[4] = -nom_x[4] + true_x[4];
   out_3296932361415467485[5] = -nom_x[5] + true_x[5];
   out_3296932361415467485[6] = -nom_x[6] + true_x[6];
   out_3296932361415467485[7] = -nom_x[7] + true_x[7];
   out_3296932361415467485[8] = -nom_x[8] + true_x[8];
   out_3296932361415467485[9] = -nom_x[9] + true_x[9];
   out_3296932361415467485[10] = -nom_x[10] + true_x[10];
   out_3296932361415467485[11] = -nom_x[11] + true_x[11];
   out_3296932361415467485[12] = -nom_x[12] + true_x[12];
   out_3296932361415467485[13] = -nom_x[13] + true_x[13];
   out_3296932361415467485[14] = -nom_x[14] + true_x[14];
   out_3296932361415467485[15] = -nom_x[15] + true_x[15];
   out_3296932361415467485[16] = -nom_x[16] + true_x[16];
   out_3296932361415467485[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4756587579081525558) {
   out_4756587579081525558[0] = 1.0;
   out_4756587579081525558[1] = 0.0;
   out_4756587579081525558[2] = 0.0;
   out_4756587579081525558[3] = 0.0;
   out_4756587579081525558[4] = 0.0;
   out_4756587579081525558[5] = 0.0;
   out_4756587579081525558[6] = 0.0;
   out_4756587579081525558[7] = 0.0;
   out_4756587579081525558[8] = 0.0;
   out_4756587579081525558[9] = 0.0;
   out_4756587579081525558[10] = 0.0;
   out_4756587579081525558[11] = 0.0;
   out_4756587579081525558[12] = 0.0;
   out_4756587579081525558[13] = 0.0;
   out_4756587579081525558[14] = 0.0;
   out_4756587579081525558[15] = 0.0;
   out_4756587579081525558[16] = 0.0;
   out_4756587579081525558[17] = 0.0;
   out_4756587579081525558[18] = 0.0;
   out_4756587579081525558[19] = 1.0;
   out_4756587579081525558[20] = 0.0;
   out_4756587579081525558[21] = 0.0;
   out_4756587579081525558[22] = 0.0;
   out_4756587579081525558[23] = 0.0;
   out_4756587579081525558[24] = 0.0;
   out_4756587579081525558[25] = 0.0;
   out_4756587579081525558[26] = 0.0;
   out_4756587579081525558[27] = 0.0;
   out_4756587579081525558[28] = 0.0;
   out_4756587579081525558[29] = 0.0;
   out_4756587579081525558[30] = 0.0;
   out_4756587579081525558[31] = 0.0;
   out_4756587579081525558[32] = 0.0;
   out_4756587579081525558[33] = 0.0;
   out_4756587579081525558[34] = 0.0;
   out_4756587579081525558[35] = 0.0;
   out_4756587579081525558[36] = 0.0;
   out_4756587579081525558[37] = 0.0;
   out_4756587579081525558[38] = 1.0;
   out_4756587579081525558[39] = 0.0;
   out_4756587579081525558[40] = 0.0;
   out_4756587579081525558[41] = 0.0;
   out_4756587579081525558[42] = 0.0;
   out_4756587579081525558[43] = 0.0;
   out_4756587579081525558[44] = 0.0;
   out_4756587579081525558[45] = 0.0;
   out_4756587579081525558[46] = 0.0;
   out_4756587579081525558[47] = 0.0;
   out_4756587579081525558[48] = 0.0;
   out_4756587579081525558[49] = 0.0;
   out_4756587579081525558[50] = 0.0;
   out_4756587579081525558[51] = 0.0;
   out_4756587579081525558[52] = 0.0;
   out_4756587579081525558[53] = 0.0;
   out_4756587579081525558[54] = 0.0;
   out_4756587579081525558[55] = 0.0;
   out_4756587579081525558[56] = 0.0;
   out_4756587579081525558[57] = 1.0;
   out_4756587579081525558[58] = 0.0;
   out_4756587579081525558[59] = 0.0;
   out_4756587579081525558[60] = 0.0;
   out_4756587579081525558[61] = 0.0;
   out_4756587579081525558[62] = 0.0;
   out_4756587579081525558[63] = 0.0;
   out_4756587579081525558[64] = 0.0;
   out_4756587579081525558[65] = 0.0;
   out_4756587579081525558[66] = 0.0;
   out_4756587579081525558[67] = 0.0;
   out_4756587579081525558[68] = 0.0;
   out_4756587579081525558[69] = 0.0;
   out_4756587579081525558[70] = 0.0;
   out_4756587579081525558[71] = 0.0;
   out_4756587579081525558[72] = 0.0;
   out_4756587579081525558[73] = 0.0;
   out_4756587579081525558[74] = 0.0;
   out_4756587579081525558[75] = 0.0;
   out_4756587579081525558[76] = 1.0;
   out_4756587579081525558[77] = 0.0;
   out_4756587579081525558[78] = 0.0;
   out_4756587579081525558[79] = 0.0;
   out_4756587579081525558[80] = 0.0;
   out_4756587579081525558[81] = 0.0;
   out_4756587579081525558[82] = 0.0;
   out_4756587579081525558[83] = 0.0;
   out_4756587579081525558[84] = 0.0;
   out_4756587579081525558[85] = 0.0;
   out_4756587579081525558[86] = 0.0;
   out_4756587579081525558[87] = 0.0;
   out_4756587579081525558[88] = 0.0;
   out_4756587579081525558[89] = 0.0;
   out_4756587579081525558[90] = 0.0;
   out_4756587579081525558[91] = 0.0;
   out_4756587579081525558[92] = 0.0;
   out_4756587579081525558[93] = 0.0;
   out_4756587579081525558[94] = 0.0;
   out_4756587579081525558[95] = 1.0;
   out_4756587579081525558[96] = 0.0;
   out_4756587579081525558[97] = 0.0;
   out_4756587579081525558[98] = 0.0;
   out_4756587579081525558[99] = 0.0;
   out_4756587579081525558[100] = 0.0;
   out_4756587579081525558[101] = 0.0;
   out_4756587579081525558[102] = 0.0;
   out_4756587579081525558[103] = 0.0;
   out_4756587579081525558[104] = 0.0;
   out_4756587579081525558[105] = 0.0;
   out_4756587579081525558[106] = 0.0;
   out_4756587579081525558[107] = 0.0;
   out_4756587579081525558[108] = 0.0;
   out_4756587579081525558[109] = 0.0;
   out_4756587579081525558[110] = 0.0;
   out_4756587579081525558[111] = 0.0;
   out_4756587579081525558[112] = 0.0;
   out_4756587579081525558[113] = 0.0;
   out_4756587579081525558[114] = 1.0;
   out_4756587579081525558[115] = 0.0;
   out_4756587579081525558[116] = 0.0;
   out_4756587579081525558[117] = 0.0;
   out_4756587579081525558[118] = 0.0;
   out_4756587579081525558[119] = 0.0;
   out_4756587579081525558[120] = 0.0;
   out_4756587579081525558[121] = 0.0;
   out_4756587579081525558[122] = 0.0;
   out_4756587579081525558[123] = 0.0;
   out_4756587579081525558[124] = 0.0;
   out_4756587579081525558[125] = 0.0;
   out_4756587579081525558[126] = 0.0;
   out_4756587579081525558[127] = 0.0;
   out_4756587579081525558[128] = 0.0;
   out_4756587579081525558[129] = 0.0;
   out_4756587579081525558[130] = 0.0;
   out_4756587579081525558[131] = 0.0;
   out_4756587579081525558[132] = 0.0;
   out_4756587579081525558[133] = 1.0;
   out_4756587579081525558[134] = 0.0;
   out_4756587579081525558[135] = 0.0;
   out_4756587579081525558[136] = 0.0;
   out_4756587579081525558[137] = 0.0;
   out_4756587579081525558[138] = 0.0;
   out_4756587579081525558[139] = 0.0;
   out_4756587579081525558[140] = 0.0;
   out_4756587579081525558[141] = 0.0;
   out_4756587579081525558[142] = 0.0;
   out_4756587579081525558[143] = 0.0;
   out_4756587579081525558[144] = 0.0;
   out_4756587579081525558[145] = 0.0;
   out_4756587579081525558[146] = 0.0;
   out_4756587579081525558[147] = 0.0;
   out_4756587579081525558[148] = 0.0;
   out_4756587579081525558[149] = 0.0;
   out_4756587579081525558[150] = 0.0;
   out_4756587579081525558[151] = 0.0;
   out_4756587579081525558[152] = 1.0;
   out_4756587579081525558[153] = 0.0;
   out_4756587579081525558[154] = 0.0;
   out_4756587579081525558[155] = 0.0;
   out_4756587579081525558[156] = 0.0;
   out_4756587579081525558[157] = 0.0;
   out_4756587579081525558[158] = 0.0;
   out_4756587579081525558[159] = 0.0;
   out_4756587579081525558[160] = 0.0;
   out_4756587579081525558[161] = 0.0;
   out_4756587579081525558[162] = 0.0;
   out_4756587579081525558[163] = 0.0;
   out_4756587579081525558[164] = 0.0;
   out_4756587579081525558[165] = 0.0;
   out_4756587579081525558[166] = 0.0;
   out_4756587579081525558[167] = 0.0;
   out_4756587579081525558[168] = 0.0;
   out_4756587579081525558[169] = 0.0;
   out_4756587579081525558[170] = 0.0;
   out_4756587579081525558[171] = 1.0;
   out_4756587579081525558[172] = 0.0;
   out_4756587579081525558[173] = 0.0;
   out_4756587579081525558[174] = 0.0;
   out_4756587579081525558[175] = 0.0;
   out_4756587579081525558[176] = 0.0;
   out_4756587579081525558[177] = 0.0;
   out_4756587579081525558[178] = 0.0;
   out_4756587579081525558[179] = 0.0;
   out_4756587579081525558[180] = 0.0;
   out_4756587579081525558[181] = 0.0;
   out_4756587579081525558[182] = 0.0;
   out_4756587579081525558[183] = 0.0;
   out_4756587579081525558[184] = 0.0;
   out_4756587579081525558[185] = 0.0;
   out_4756587579081525558[186] = 0.0;
   out_4756587579081525558[187] = 0.0;
   out_4756587579081525558[188] = 0.0;
   out_4756587579081525558[189] = 0.0;
   out_4756587579081525558[190] = 1.0;
   out_4756587579081525558[191] = 0.0;
   out_4756587579081525558[192] = 0.0;
   out_4756587579081525558[193] = 0.0;
   out_4756587579081525558[194] = 0.0;
   out_4756587579081525558[195] = 0.0;
   out_4756587579081525558[196] = 0.0;
   out_4756587579081525558[197] = 0.0;
   out_4756587579081525558[198] = 0.0;
   out_4756587579081525558[199] = 0.0;
   out_4756587579081525558[200] = 0.0;
   out_4756587579081525558[201] = 0.0;
   out_4756587579081525558[202] = 0.0;
   out_4756587579081525558[203] = 0.0;
   out_4756587579081525558[204] = 0.0;
   out_4756587579081525558[205] = 0.0;
   out_4756587579081525558[206] = 0.0;
   out_4756587579081525558[207] = 0.0;
   out_4756587579081525558[208] = 0.0;
   out_4756587579081525558[209] = 1.0;
   out_4756587579081525558[210] = 0.0;
   out_4756587579081525558[211] = 0.0;
   out_4756587579081525558[212] = 0.0;
   out_4756587579081525558[213] = 0.0;
   out_4756587579081525558[214] = 0.0;
   out_4756587579081525558[215] = 0.0;
   out_4756587579081525558[216] = 0.0;
   out_4756587579081525558[217] = 0.0;
   out_4756587579081525558[218] = 0.0;
   out_4756587579081525558[219] = 0.0;
   out_4756587579081525558[220] = 0.0;
   out_4756587579081525558[221] = 0.0;
   out_4756587579081525558[222] = 0.0;
   out_4756587579081525558[223] = 0.0;
   out_4756587579081525558[224] = 0.0;
   out_4756587579081525558[225] = 0.0;
   out_4756587579081525558[226] = 0.0;
   out_4756587579081525558[227] = 0.0;
   out_4756587579081525558[228] = 1.0;
   out_4756587579081525558[229] = 0.0;
   out_4756587579081525558[230] = 0.0;
   out_4756587579081525558[231] = 0.0;
   out_4756587579081525558[232] = 0.0;
   out_4756587579081525558[233] = 0.0;
   out_4756587579081525558[234] = 0.0;
   out_4756587579081525558[235] = 0.0;
   out_4756587579081525558[236] = 0.0;
   out_4756587579081525558[237] = 0.0;
   out_4756587579081525558[238] = 0.0;
   out_4756587579081525558[239] = 0.0;
   out_4756587579081525558[240] = 0.0;
   out_4756587579081525558[241] = 0.0;
   out_4756587579081525558[242] = 0.0;
   out_4756587579081525558[243] = 0.0;
   out_4756587579081525558[244] = 0.0;
   out_4756587579081525558[245] = 0.0;
   out_4756587579081525558[246] = 0.0;
   out_4756587579081525558[247] = 1.0;
   out_4756587579081525558[248] = 0.0;
   out_4756587579081525558[249] = 0.0;
   out_4756587579081525558[250] = 0.0;
   out_4756587579081525558[251] = 0.0;
   out_4756587579081525558[252] = 0.0;
   out_4756587579081525558[253] = 0.0;
   out_4756587579081525558[254] = 0.0;
   out_4756587579081525558[255] = 0.0;
   out_4756587579081525558[256] = 0.0;
   out_4756587579081525558[257] = 0.0;
   out_4756587579081525558[258] = 0.0;
   out_4756587579081525558[259] = 0.0;
   out_4756587579081525558[260] = 0.0;
   out_4756587579081525558[261] = 0.0;
   out_4756587579081525558[262] = 0.0;
   out_4756587579081525558[263] = 0.0;
   out_4756587579081525558[264] = 0.0;
   out_4756587579081525558[265] = 0.0;
   out_4756587579081525558[266] = 1.0;
   out_4756587579081525558[267] = 0.0;
   out_4756587579081525558[268] = 0.0;
   out_4756587579081525558[269] = 0.0;
   out_4756587579081525558[270] = 0.0;
   out_4756587579081525558[271] = 0.0;
   out_4756587579081525558[272] = 0.0;
   out_4756587579081525558[273] = 0.0;
   out_4756587579081525558[274] = 0.0;
   out_4756587579081525558[275] = 0.0;
   out_4756587579081525558[276] = 0.0;
   out_4756587579081525558[277] = 0.0;
   out_4756587579081525558[278] = 0.0;
   out_4756587579081525558[279] = 0.0;
   out_4756587579081525558[280] = 0.0;
   out_4756587579081525558[281] = 0.0;
   out_4756587579081525558[282] = 0.0;
   out_4756587579081525558[283] = 0.0;
   out_4756587579081525558[284] = 0.0;
   out_4756587579081525558[285] = 1.0;
   out_4756587579081525558[286] = 0.0;
   out_4756587579081525558[287] = 0.0;
   out_4756587579081525558[288] = 0.0;
   out_4756587579081525558[289] = 0.0;
   out_4756587579081525558[290] = 0.0;
   out_4756587579081525558[291] = 0.0;
   out_4756587579081525558[292] = 0.0;
   out_4756587579081525558[293] = 0.0;
   out_4756587579081525558[294] = 0.0;
   out_4756587579081525558[295] = 0.0;
   out_4756587579081525558[296] = 0.0;
   out_4756587579081525558[297] = 0.0;
   out_4756587579081525558[298] = 0.0;
   out_4756587579081525558[299] = 0.0;
   out_4756587579081525558[300] = 0.0;
   out_4756587579081525558[301] = 0.0;
   out_4756587579081525558[302] = 0.0;
   out_4756587579081525558[303] = 0.0;
   out_4756587579081525558[304] = 1.0;
   out_4756587579081525558[305] = 0.0;
   out_4756587579081525558[306] = 0.0;
   out_4756587579081525558[307] = 0.0;
   out_4756587579081525558[308] = 0.0;
   out_4756587579081525558[309] = 0.0;
   out_4756587579081525558[310] = 0.0;
   out_4756587579081525558[311] = 0.0;
   out_4756587579081525558[312] = 0.0;
   out_4756587579081525558[313] = 0.0;
   out_4756587579081525558[314] = 0.0;
   out_4756587579081525558[315] = 0.0;
   out_4756587579081525558[316] = 0.0;
   out_4756587579081525558[317] = 0.0;
   out_4756587579081525558[318] = 0.0;
   out_4756587579081525558[319] = 0.0;
   out_4756587579081525558[320] = 0.0;
   out_4756587579081525558[321] = 0.0;
   out_4756587579081525558[322] = 0.0;
   out_4756587579081525558[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_356369068438679194) {
   out_356369068438679194[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_356369068438679194[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_356369068438679194[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_356369068438679194[3] = dt*state[12] + state[3];
   out_356369068438679194[4] = dt*state[13] + state[4];
   out_356369068438679194[5] = dt*state[14] + state[5];
   out_356369068438679194[6] = state[6];
   out_356369068438679194[7] = state[7];
   out_356369068438679194[8] = state[8];
   out_356369068438679194[9] = state[9];
   out_356369068438679194[10] = state[10];
   out_356369068438679194[11] = state[11];
   out_356369068438679194[12] = state[12];
   out_356369068438679194[13] = state[13];
   out_356369068438679194[14] = state[14];
   out_356369068438679194[15] = state[15];
   out_356369068438679194[16] = state[16];
   out_356369068438679194[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8347244080238777649) {
   out_8347244080238777649[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8347244080238777649[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8347244080238777649[2] = 0;
   out_8347244080238777649[3] = 0;
   out_8347244080238777649[4] = 0;
   out_8347244080238777649[5] = 0;
   out_8347244080238777649[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8347244080238777649[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8347244080238777649[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8347244080238777649[9] = 0;
   out_8347244080238777649[10] = 0;
   out_8347244080238777649[11] = 0;
   out_8347244080238777649[12] = 0;
   out_8347244080238777649[13] = 0;
   out_8347244080238777649[14] = 0;
   out_8347244080238777649[15] = 0;
   out_8347244080238777649[16] = 0;
   out_8347244080238777649[17] = 0;
   out_8347244080238777649[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8347244080238777649[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8347244080238777649[20] = 0;
   out_8347244080238777649[21] = 0;
   out_8347244080238777649[22] = 0;
   out_8347244080238777649[23] = 0;
   out_8347244080238777649[24] = 0;
   out_8347244080238777649[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8347244080238777649[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8347244080238777649[27] = 0;
   out_8347244080238777649[28] = 0;
   out_8347244080238777649[29] = 0;
   out_8347244080238777649[30] = 0;
   out_8347244080238777649[31] = 0;
   out_8347244080238777649[32] = 0;
   out_8347244080238777649[33] = 0;
   out_8347244080238777649[34] = 0;
   out_8347244080238777649[35] = 0;
   out_8347244080238777649[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8347244080238777649[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8347244080238777649[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8347244080238777649[39] = 0;
   out_8347244080238777649[40] = 0;
   out_8347244080238777649[41] = 0;
   out_8347244080238777649[42] = 0;
   out_8347244080238777649[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8347244080238777649[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8347244080238777649[45] = 0;
   out_8347244080238777649[46] = 0;
   out_8347244080238777649[47] = 0;
   out_8347244080238777649[48] = 0;
   out_8347244080238777649[49] = 0;
   out_8347244080238777649[50] = 0;
   out_8347244080238777649[51] = 0;
   out_8347244080238777649[52] = 0;
   out_8347244080238777649[53] = 0;
   out_8347244080238777649[54] = 0;
   out_8347244080238777649[55] = 0;
   out_8347244080238777649[56] = 0;
   out_8347244080238777649[57] = 1;
   out_8347244080238777649[58] = 0;
   out_8347244080238777649[59] = 0;
   out_8347244080238777649[60] = 0;
   out_8347244080238777649[61] = 0;
   out_8347244080238777649[62] = 0;
   out_8347244080238777649[63] = 0;
   out_8347244080238777649[64] = 0;
   out_8347244080238777649[65] = 0;
   out_8347244080238777649[66] = dt;
   out_8347244080238777649[67] = 0;
   out_8347244080238777649[68] = 0;
   out_8347244080238777649[69] = 0;
   out_8347244080238777649[70] = 0;
   out_8347244080238777649[71] = 0;
   out_8347244080238777649[72] = 0;
   out_8347244080238777649[73] = 0;
   out_8347244080238777649[74] = 0;
   out_8347244080238777649[75] = 0;
   out_8347244080238777649[76] = 1;
   out_8347244080238777649[77] = 0;
   out_8347244080238777649[78] = 0;
   out_8347244080238777649[79] = 0;
   out_8347244080238777649[80] = 0;
   out_8347244080238777649[81] = 0;
   out_8347244080238777649[82] = 0;
   out_8347244080238777649[83] = 0;
   out_8347244080238777649[84] = 0;
   out_8347244080238777649[85] = dt;
   out_8347244080238777649[86] = 0;
   out_8347244080238777649[87] = 0;
   out_8347244080238777649[88] = 0;
   out_8347244080238777649[89] = 0;
   out_8347244080238777649[90] = 0;
   out_8347244080238777649[91] = 0;
   out_8347244080238777649[92] = 0;
   out_8347244080238777649[93] = 0;
   out_8347244080238777649[94] = 0;
   out_8347244080238777649[95] = 1;
   out_8347244080238777649[96] = 0;
   out_8347244080238777649[97] = 0;
   out_8347244080238777649[98] = 0;
   out_8347244080238777649[99] = 0;
   out_8347244080238777649[100] = 0;
   out_8347244080238777649[101] = 0;
   out_8347244080238777649[102] = 0;
   out_8347244080238777649[103] = 0;
   out_8347244080238777649[104] = dt;
   out_8347244080238777649[105] = 0;
   out_8347244080238777649[106] = 0;
   out_8347244080238777649[107] = 0;
   out_8347244080238777649[108] = 0;
   out_8347244080238777649[109] = 0;
   out_8347244080238777649[110] = 0;
   out_8347244080238777649[111] = 0;
   out_8347244080238777649[112] = 0;
   out_8347244080238777649[113] = 0;
   out_8347244080238777649[114] = 1;
   out_8347244080238777649[115] = 0;
   out_8347244080238777649[116] = 0;
   out_8347244080238777649[117] = 0;
   out_8347244080238777649[118] = 0;
   out_8347244080238777649[119] = 0;
   out_8347244080238777649[120] = 0;
   out_8347244080238777649[121] = 0;
   out_8347244080238777649[122] = 0;
   out_8347244080238777649[123] = 0;
   out_8347244080238777649[124] = 0;
   out_8347244080238777649[125] = 0;
   out_8347244080238777649[126] = 0;
   out_8347244080238777649[127] = 0;
   out_8347244080238777649[128] = 0;
   out_8347244080238777649[129] = 0;
   out_8347244080238777649[130] = 0;
   out_8347244080238777649[131] = 0;
   out_8347244080238777649[132] = 0;
   out_8347244080238777649[133] = 1;
   out_8347244080238777649[134] = 0;
   out_8347244080238777649[135] = 0;
   out_8347244080238777649[136] = 0;
   out_8347244080238777649[137] = 0;
   out_8347244080238777649[138] = 0;
   out_8347244080238777649[139] = 0;
   out_8347244080238777649[140] = 0;
   out_8347244080238777649[141] = 0;
   out_8347244080238777649[142] = 0;
   out_8347244080238777649[143] = 0;
   out_8347244080238777649[144] = 0;
   out_8347244080238777649[145] = 0;
   out_8347244080238777649[146] = 0;
   out_8347244080238777649[147] = 0;
   out_8347244080238777649[148] = 0;
   out_8347244080238777649[149] = 0;
   out_8347244080238777649[150] = 0;
   out_8347244080238777649[151] = 0;
   out_8347244080238777649[152] = 1;
   out_8347244080238777649[153] = 0;
   out_8347244080238777649[154] = 0;
   out_8347244080238777649[155] = 0;
   out_8347244080238777649[156] = 0;
   out_8347244080238777649[157] = 0;
   out_8347244080238777649[158] = 0;
   out_8347244080238777649[159] = 0;
   out_8347244080238777649[160] = 0;
   out_8347244080238777649[161] = 0;
   out_8347244080238777649[162] = 0;
   out_8347244080238777649[163] = 0;
   out_8347244080238777649[164] = 0;
   out_8347244080238777649[165] = 0;
   out_8347244080238777649[166] = 0;
   out_8347244080238777649[167] = 0;
   out_8347244080238777649[168] = 0;
   out_8347244080238777649[169] = 0;
   out_8347244080238777649[170] = 0;
   out_8347244080238777649[171] = 1;
   out_8347244080238777649[172] = 0;
   out_8347244080238777649[173] = 0;
   out_8347244080238777649[174] = 0;
   out_8347244080238777649[175] = 0;
   out_8347244080238777649[176] = 0;
   out_8347244080238777649[177] = 0;
   out_8347244080238777649[178] = 0;
   out_8347244080238777649[179] = 0;
   out_8347244080238777649[180] = 0;
   out_8347244080238777649[181] = 0;
   out_8347244080238777649[182] = 0;
   out_8347244080238777649[183] = 0;
   out_8347244080238777649[184] = 0;
   out_8347244080238777649[185] = 0;
   out_8347244080238777649[186] = 0;
   out_8347244080238777649[187] = 0;
   out_8347244080238777649[188] = 0;
   out_8347244080238777649[189] = 0;
   out_8347244080238777649[190] = 1;
   out_8347244080238777649[191] = 0;
   out_8347244080238777649[192] = 0;
   out_8347244080238777649[193] = 0;
   out_8347244080238777649[194] = 0;
   out_8347244080238777649[195] = 0;
   out_8347244080238777649[196] = 0;
   out_8347244080238777649[197] = 0;
   out_8347244080238777649[198] = 0;
   out_8347244080238777649[199] = 0;
   out_8347244080238777649[200] = 0;
   out_8347244080238777649[201] = 0;
   out_8347244080238777649[202] = 0;
   out_8347244080238777649[203] = 0;
   out_8347244080238777649[204] = 0;
   out_8347244080238777649[205] = 0;
   out_8347244080238777649[206] = 0;
   out_8347244080238777649[207] = 0;
   out_8347244080238777649[208] = 0;
   out_8347244080238777649[209] = 1;
   out_8347244080238777649[210] = 0;
   out_8347244080238777649[211] = 0;
   out_8347244080238777649[212] = 0;
   out_8347244080238777649[213] = 0;
   out_8347244080238777649[214] = 0;
   out_8347244080238777649[215] = 0;
   out_8347244080238777649[216] = 0;
   out_8347244080238777649[217] = 0;
   out_8347244080238777649[218] = 0;
   out_8347244080238777649[219] = 0;
   out_8347244080238777649[220] = 0;
   out_8347244080238777649[221] = 0;
   out_8347244080238777649[222] = 0;
   out_8347244080238777649[223] = 0;
   out_8347244080238777649[224] = 0;
   out_8347244080238777649[225] = 0;
   out_8347244080238777649[226] = 0;
   out_8347244080238777649[227] = 0;
   out_8347244080238777649[228] = 1;
   out_8347244080238777649[229] = 0;
   out_8347244080238777649[230] = 0;
   out_8347244080238777649[231] = 0;
   out_8347244080238777649[232] = 0;
   out_8347244080238777649[233] = 0;
   out_8347244080238777649[234] = 0;
   out_8347244080238777649[235] = 0;
   out_8347244080238777649[236] = 0;
   out_8347244080238777649[237] = 0;
   out_8347244080238777649[238] = 0;
   out_8347244080238777649[239] = 0;
   out_8347244080238777649[240] = 0;
   out_8347244080238777649[241] = 0;
   out_8347244080238777649[242] = 0;
   out_8347244080238777649[243] = 0;
   out_8347244080238777649[244] = 0;
   out_8347244080238777649[245] = 0;
   out_8347244080238777649[246] = 0;
   out_8347244080238777649[247] = 1;
   out_8347244080238777649[248] = 0;
   out_8347244080238777649[249] = 0;
   out_8347244080238777649[250] = 0;
   out_8347244080238777649[251] = 0;
   out_8347244080238777649[252] = 0;
   out_8347244080238777649[253] = 0;
   out_8347244080238777649[254] = 0;
   out_8347244080238777649[255] = 0;
   out_8347244080238777649[256] = 0;
   out_8347244080238777649[257] = 0;
   out_8347244080238777649[258] = 0;
   out_8347244080238777649[259] = 0;
   out_8347244080238777649[260] = 0;
   out_8347244080238777649[261] = 0;
   out_8347244080238777649[262] = 0;
   out_8347244080238777649[263] = 0;
   out_8347244080238777649[264] = 0;
   out_8347244080238777649[265] = 0;
   out_8347244080238777649[266] = 1;
   out_8347244080238777649[267] = 0;
   out_8347244080238777649[268] = 0;
   out_8347244080238777649[269] = 0;
   out_8347244080238777649[270] = 0;
   out_8347244080238777649[271] = 0;
   out_8347244080238777649[272] = 0;
   out_8347244080238777649[273] = 0;
   out_8347244080238777649[274] = 0;
   out_8347244080238777649[275] = 0;
   out_8347244080238777649[276] = 0;
   out_8347244080238777649[277] = 0;
   out_8347244080238777649[278] = 0;
   out_8347244080238777649[279] = 0;
   out_8347244080238777649[280] = 0;
   out_8347244080238777649[281] = 0;
   out_8347244080238777649[282] = 0;
   out_8347244080238777649[283] = 0;
   out_8347244080238777649[284] = 0;
   out_8347244080238777649[285] = 1;
   out_8347244080238777649[286] = 0;
   out_8347244080238777649[287] = 0;
   out_8347244080238777649[288] = 0;
   out_8347244080238777649[289] = 0;
   out_8347244080238777649[290] = 0;
   out_8347244080238777649[291] = 0;
   out_8347244080238777649[292] = 0;
   out_8347244080238777649[293] = 0;
   out_8347244080238777649[294] = 0;
   out_8347244080238777649[295] = 0;
   out_8347244080238777649[296] = 0;
   out_8347244080238777649[297] = 0;
   out_8347244080238777649[298] = 0;
   out_8347244080238777649[299] = 0;
   out_8347244080238777649[300] = 0;
   out_8347244080238777649[301] = 0;
   out_8347244080238777649[302] = 0;
   out_8347244080238777649[303] = 0;
   out_8347244080238777649[304] = 1;
   out_8347244080238777649[305] = 0;
   out_8347244080238777649[306] = 0;
   out_8347244080238777649[307] = 0;
   out_8347244080238777649[308] = 0;
   out_8347244080238777649[309] = 0;
   out_8347244080238777649[310] = 0;
   out_8347244080238777649[311] = 0;
   out_8347244080238777649[312] = 0;
   out_8347244080238777649[313] = 0;
   out_8347244080238777649[314] = 0;
   out_8347244080238777649[315] = 0;
   out_8347244080238777649[316] = 0;
   out_8347244080238777649[317] = 0;
   out_8347244080238777649[318] = 0;
   out_8347244080238777649[319] = 0;
   out_8347244080238777649[320] = 0;
   out_8347244080238777649[321] = 0;
   out_8347244080238777649[322] = 0;
   out_8347244080238777649[323] = 1;
}
void h_4(double *state, double *unused, double *out_639306389444674144) {
   out_639306389444674144[0] = state[6] + state[9];
   out_639306389444674144[1] = state[7] + state[10];
   out_639306389444674144[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_141960542152209194) {
   out_141960542152209194[0] = 0;
   out_141960542152209194[1] = 0;
   out_141960542152209194[2] = 0;
   out_141960542152209194[3] = 0;
   out_141960542152209194[4] = 0;
   out_141960542152209194[5] = 0;
   out_141960542152209194[6] = 1;
   out_141960542152209194[7] = 0;
   out_141960542152209194[8] = 0;
   out_141960542152209194[9] = 1;
   out_141960542152209194[10] = 0;
   out_141960542152209194[11] = 0;
   out_141960542152209194[12] = 0;
   out_141960542152209194[13] = 0;
   out_141960542152209194[14] = 0;
   out_141960542152209194[15] = 0;
   out_141960542152209194[16] = 0;
   out_141960542152209194[17] = 0;
   out_141960542152209194[18] = 0;
   out_141960542152209194[19] = 0;
   out_141960542152209194[20] = 0;
   out_141960542152209194[21] = 0;
   out_141960542152209194[22] = 0;
   out_141960542152209194[23] = 0;
   out_141960542152209194[24] = 0;
   out_141960542152209194[25] = 1;
   out_141960542152209194[26] = 0;
   out_141960542152209194[27] = 0;
   out_141960542152209194[28] = 1;
   out_141960542152209194[29] = 0;
   out_141960542152209194[30] = 0;
   out_141960542152209194[31] = 0;
   out_141960542152209194[32] = 0;
   out_141960542152209194[33] = 0;
   out_141960542152209194[34] = 0;
   out_141960542152209194[35] = 0;
   out_141960542152209194[36] = 0;
   out_141960542152209194[37] = 0;
   out_141960542152209194[38] = 0;
   out_141960542152209194[39] = 0;
   out_141960542152209194[40] = 0;
   out_141960542152209194[41] = 0;
   out_141960542152209194[42] = 0;
   out_141960542152209194[43] = 0;
   out_141960542152209194[44] = 1;
   out_141960542152209194[45] = 0;
   out_141960542152209194[46] = 0;
   out_141960542152209194[47] = 1;
   out_141960542152209194[48] = 0;
   out_141960542152209194[49] = 0;
   out_141960542152209194[50] = 0;
   out_141960542152209194[51] = 0;
   out_141960542152209194[52] = 0;
   out_141960542152209194[53] = 0;
}
void h_10(double *state, double *unused, double *out_4208130342329524840) {
   out_4208130342329524840[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4208130342329524840[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4208130342329524840[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3882092915448509152) {
   out_3882092915448509152[0] = 0;
   out_3882092915448509152[1] = 9.8100000000000005*cos(state[1]);
   out_3882092915448509152[2] = 0;
   out_3882092915448509152[3] = 0;
   out_3882092915448509152[4] = -state[8];
   out_3882092915448509152[5] = state[7];
   out_3882092915448509152[6] = 0;
   out_3882092915448509152[7] = state[5];
   out_3882092915448509152[8] = -state[4];
   out_3882092915448509152[9] = 0;
   out_3882092915448509152[10] = 0;
   out_3882092915448509152[11] = 0;
   out_3882092915448509152[12] = 1;
   out_3882092915448509152[13] = 0;
   out_3882092915448509152[14] = 0;
   out_3882092915448509152[15] = 1;
   out_3882092915448509152[16] = 0;
   out_3882092915448509152[17] = 0;
   out_3882092915448509152[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3882092915448509152[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3882092915448509152[20] = 0;
   out_3882092915448509152[21] = state[8];
   out_3882092915448509152[22] = 0;
   out_3882092915448509152[23] = -state[6];
   out_3882092915448509152[24] = -state[5];
   out_3882092915448509152[25] = 0;
   out_3882092915448509152[26] = state[3];
   out_3882092915448509152[27] = 0;
   out_3882092915448509152[28] = 0;
   out_3882092915448509152[29] = 0;
   out_3882092915448509152[30] = 0;
   out_3882092915448509152[31] = 1;
   out_3882092915448509152[32] = 0;
   out_3882092915448509152[33] = 0;
   out_3882092915448509152[34] = 1;
   out_3882092915448509152[35] = 0;
   out_3882092915448509152[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3882092915448509152[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3882092915448509152[38] = 0;
   out_3882092915448509152[39] = -state[7];
   out_3882092915448509152[40] = state[6];
   out_3882092915448509152[41] = 0;
   out_3882092915448509152[42] = state[4];
   out_3882092915448509152[43] = -state[3];
   out_3882092915448509152[44] = 0;
   out_3882092915448509152[45] = 0;
   out_3882092915448509152[46] = 0;
   out_3882092915448509152[47] = 0;
   out_3882092915448509152[48] = 0;
   out_3882092915448509152[49] = 0;
   out_3882092915448509152[50] = 1;
   out_3882092915448509152[51] = 0;
   out_3882092915448509152[52] = 0;
   out_3882092915448509152[53] = 1;
}
void h_13(double *state, double *unused, double *out_1415975790652254250) {
   out_1415975790652254250[0] = state[3];
   out_1415975790652254250[1] = state[4];
   out_1415975790652254250[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3070313283180123607) {
   out_3070313283180123607[0] = 0;
   out_3070313283180123607[1] = 0;
   out_3070313283180123607[2] = 0;
   out_3070313283180123607[3] = 1;
   out_3070313283180123607[4] = 0;
   out_3070313283180123607[5] = 0;
   out_3070313283180123607[6] = 0;
   out_3070313283180123607[7] = 0;
   out_3070313283180123607[8] = 0;
   out_3070313283180123607[9] = 0;
   out_3070313283180123607[10] = 0;
   out_3070313283180123607[11] = 0;
   out_3070313283180123607[12] = 0;
   out_3070313283180123607[13] = 0;
   out_3070313283180123607[14] = 0;
   out_3070313283180123607[15] = 0;
   out_3070313283180123607[16] = 0;
   out_3070313283180123607[17] = 0;
   out_3070313283180123607[18] = 0;
   out_3070313283180123607[19] = 0;
   out_3070313283180123607[20] = 0;
   out_3070313283180123607[21] = 0;
   out_3070313283180123607[22] = 1;
   out_3070313283180123607[23] = 0;
   out_3070313283180123607[24] = 0;
   out_3070313283180123607[25] = 0;
   out_3070313283180123607[26] = 0;
   out_3070313283180123607[27] = 0;
   out_3070313283180123607[28] = 0;
   out_3070313283180123607[29] = 0;
   out_3070313283180123607[30] = 0;
   out_3070313283180123607[31] = 0;
   out_3070313283180123607[32] = 0;
   out_3070313283180123607[33] = 0;
   out_3070313283180123607[34] = 0;
   out_3070313283180123607[35] = 0;
   out_3070313283180123607[36] = 0;
   out_3070313283180123607[37] = 0;
   out_3070313283180123607[38] = 0;
   out_3070313283180123607[39] = 0;
   out_3070313283180123607[40] = 0;
   out_3070313283180123607[41] = 1;
   out_3070313283180123607[42] = 0;
   out_3070313283180123607[43] = 0;
   out_3070313283180123607[44] = 0;
   out_3070313283180123607[45] = 0;
   out_3070313283180123607[46] = 0;
   out_3070313283180123607[47] = 0;
   out_3070313283180123607[48] = 0;
   out_3070313283180123607[49] = 0;
   out_3070313283180123607[50] = 0;
   out_3070313283180123607[51] = 0;
   out_3070313283180123607[52] = 0;
   out_3070313283180123607[53] = 0;
}
void h_14(double *state, double *unused, double *out_8443724961933214887) {
   out_8443724961933214887[0] = state[6];
   out_8443724961933214887[1] = state[7];
   out_8443724961933214887[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3821280314187275335) {
   out_3821280314187275335[0] = 0;
   out_3821280314187275335[1] = 0;
   out_3821280314187275335[2] = 0;
   out_3821280314187275335[3] = 0;
   out_3821280314187275335[4] = 0;
   out_3821280314187275335[5] = 0;
   out_3821280314187275335[6] = 1;
   out_3821280314187275335[7] = 0;
   out_3821280314187275335[8] = 0;
   out_3821280314187275335[9] = 0;
   out_3821280314187275335[10] = 0;
   out_3821280314187275335[11] = 0;
   out_3821280314187275335[12] = 0;
   out_3821280314187275335[13] = 0;
   out_3821280314187275335[14] = 0;
   out_3821280314187275335[15] = 0;
   out_3821280314187275335[16] = 0;
   out_3821280314187275335[17] = 0;
   out_3821280314187275335[18] = 0;
   out_3821280314187275335[19] = 0;
   out_3821280314187275335[20] = 0;
   out_3821280314187275335[21] = 0;
   out_3821280314187275335[22] = 0;
   out_3821280314187275335[23] = 0;
   out_3821280314187275335[24] = 0;
   out_3821280314187275335[25] = 1;
   out_3821280314187275335[26] = 0;
   out_3821280314187275335[27] = 0;
   out_3821280314187275335[28] = 0;
   out_3821280314187275335[29] = 0;
   out_3821280314187275335[30] = 0;
   out_3821280314187275335[31] = 0;
   out_3821280314187275335[32] = 0;
   out_3821280314187275335[33] = 0;
   out_3821280314187275335[34] = 0;
   out_3821280314187275335[35] = 0;
   out_3821280314187275335[36] = 0;
   out_3821280314187275335[37] = 0;
   out_3821280314187275335[38] = 0;
   out_3821280314187275335[39] = 0;
   out_3821280314187275335[40] = 0;
   out_3821280314187275335[41] = 0;
   out_3821280314187275335[42] = 0;
   out_3821280314187275335[43] = 0;
   out_3821280314187275335[44] = 1;
   out_3821280314187275335[45] = 0;
   out_3821280314187275335[46] = 0;
   out_3821280314187275335[47] = 0;
   out_3821280314187275335[48] = 0;
   out_3821280314187275335[49] = 0;
   out_3821280314187275335[50] = 0;
   out_3821280314187275335[51] = 0;
   out_3821280314187275335[52] = 0;
   out_3821280314187275335[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6091666443681126136) {
  err_fun(nom_x, delta_x, out_6091666443681126136);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3296932361415467485) {
  inv_err_fun(nom_x, true_x, out_3296932361415467485);
}
void pose_H_mod_fun(double *state, double *out_4756587579081525558) {
  H_mod_fun(state, out_4756587579081525558);
}
void pose_f_fun(double *state, double dt, double *out_356369068438679194) {
  f_fun(state,  dt, out_356369068438679194);
}
void pose_F_fun(double *state, double dt, double *out_8347244080238777649) {
  F_fun(state,  dt, out_8347244080238777649);
}
void pose_h_4(double *state, double *unused, double *out_639306389444674144) {
  h_4(state, unused, out_639306389444674144);
}
void pose_H_4(double *state, double *unused, double *out_141960542152209194) {
  H_4(state, unused, out_141960542152209194);
}
void pose_h_10(double *state, double *unused, double *out_4208130342329524840) {
  h_10(state, unused, out_4208130342329524840);
}
void pose_H_10(double *state, double *unused, double *out_3882092915448509152) {
  H_10(state, unused, out_3882092915448509152);
}
void pose_h_13(double *state, double *unused, double *out_1415975790652254250) {
  h_13(state, unused, out_1415975790652254250);
}
void pose_H_13(double *state, double *unused, double *out_3070313283180123607) {
  H_13(state, unused, out_3070313283180123607);
}
void pose_h_14(double *state, double *unused, double *out_8443724961933214887) {
  h_14(state, unused, out_8443724961933214887);
}
void pose_H_14(double *state, double *unused, double *out_3821280314187275335) {
  H_14(state, unused, out_3821280314187275335);
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
