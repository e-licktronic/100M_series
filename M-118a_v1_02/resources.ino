// Copyright 2012 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// -----------------------------------------------------------------------------
//
// Resources definitions.
//
// Automatically generated with:
// make resources


#include "resources.h"

namespace grids {

static const prog_char str_res_dummy[] PROGMEM = "dummy";


const prog_char* const string_table[] = {
  str_res_dummy,
};



const prog_uint16_t* const lookup_table_table[] = {
};

const prog_uint32_t lut_res_euclidean[] PROGMEM = {
       0,      0,      0,      0,      0,      0,      0,      0,
       0,      0,      0,      0,      0,      0,      0,      0,
       1,      1,      1,      1,      1,      1,      1,      1,
       1,      1,      1,      1,      1,      1,      1,      1,
       0,      0,      0,      0,      0,      0,      0,      0,
       1,      1,      1,      1,      1,      1,      1,      1,
       1,      1,      1,      1,      1,      1,      1,      1,
       3,      3,      3,      3,      3,      3,      3,      3,
       0,      0,      0,      0,      0,      0,      1,      1,
       1,      1,      1,      1,      1,      1,      1,      1,
       5,      5,      5,      5,      5,      5,      5,      5,
       5,      5,      7,      7,      7,      7,      7,      7,
       0,      0,      0,      0,      1,      1,      1,      1,
       1,      1,      1,      1,      5,      5,      5,      5,
       5,      5,      5,      5,     13,     13,     13,     13,
      13,     13,     13,     13,     15,     15,     15,     15,
       0,      0,      0,      0,      1,      1,      1,      1,
       1,      1,      9,      9,      9,      9,      9,      9,
      13,     13,     13,     13,     13,     13,     29,     29,
      29,     29,     29,     29,     31,     31,     31,     31,
       0,      0,      0,      1,      1,      1,      1,      1,
       9,      9,      9,      9,      9,     21,     21,     21,
      21,     21,     21,     45,     45,     45,     45,     45,
      61,     61,     61,     61,     61,     63,     63,     63,
       0,      0,      0,      1,      1,      1,      1,     17,
      17,     17,     17,     17,     41,     41,     41,     41,
      45,     45,     45,     45,     93,     93,     93,     93,
      93,    125,    125,    125,    125,    127,    127,    127,
       0,      0,      1,      1,      1,      1,     17,     17,
      17,     17,     41,     41,     41,     41,     85,     85,
      85,     85,    173,    173,    173,    173,    221,    221,
     221,    221,    253,    253,    253,    253,    255,    255,
       0,      0,      1,      1,      1,      1,     33,     33,
      33,     73,     73,     73,     73,    169,    169,    169,
     173,    173,    173,    365,    365,    365,    365,    445,
     445,    445,    509,    509,    509,    509,    511,    511,
       0,      0,      1,      1,      1,     33,     33,     33,
     145,    145,    145,    297,    297,    297,    341,    341,
     341,    341,    429,    429,    429,    733,    733,    733,
     957,    957,    957,   1021,   1021,   1021,   1023,   1023,
       0,      0,      1,      1,      1,     65,     65,     65,
     145,    145,    297,    297,    297,    681,    681,    681,
     685,    685,    685,   1453,   1453,   1453,   1757,   1757,
    1917,   1917,   1917,   2045,   2045,   2045,   2047,   2047,
       0,      0,      1,      1,     65,     65,     65,    273,
     273,    273,    585,    585,   1193,   1193,   1193,   1365,
    1365,   1709,   1709,   1709,   2925,   2925,   3549,   3549,
    3549,   3965,   3965,   3965,   4093,   4093,   4095,   4095,
       0,      0,      1,      1,    129,    129,    545,    545,
     545,   1169,   1169,   2345,   2345,   2345,   2729,   2729,
    2733,   2733,   3501,   3501,   3501,   5853,   5853,   7101,
    7101,   7101,   7933,   7933,   8189,   8189,   8191,   8191,
       0,      0,      1,      1,    129,    129,    545,    545,
    2193,   2193,   2345,   2345,   2345,   5289,   5289,   5461,
    5461,   5805,   5805,  11693,  11693,  11693,  11997,  11997,
   15293,  15293,  16125,  16125,  16381,  16381,  16383,  16383,
       0,      0,      1,      1,    257,    257,   1057,   1057,
    2193,   2193,   4681,   4681,   9513,   9513,  10921,  10921,
   10925,  10925,  13741,  13741,  23405,  23405,  28381,  28381,
   30653,  30653,  32253,  32253,  32765,  32765,  32767,  32767,
       0,      1,      1,    257,    257,   2113,   2113,   4369,
    4369,   9361,   9361,  10537,  10537,  21161,  21161,  21845,
   21845,  23213,  23213,  44461,  44461,  46813,  46813,  56797,
   56797,  61309,  61309,  65021,  65021,  65533,  65533,  65535,
       0,      1,      1,    513,    513,   2113,   2113,   8737,
    8737,  17553,  17553,  18729,  38057,  38057,  43689,  43689,
   43693,  43693,  54957,  54957,  93613,  95965,  95965, 113597,
  113597, 126845, 126845, 130045, 130045, 131069, 131069, 131071,
       0,      1,      1,    513,    513,   4161,   4161,  16929,
   34961,  34961,  37449,  37449,  76073,  86697,  86697,  87381,
   87381,  88749,  88749, 109997, 187245, 187245, 192221, 192221,
  228285, 253821, 253821, 261117, 261117, 262141, 262141, 262143,
       0,      1,      1,   1025,   1025,   8321,  16929,  16929,
   34961,  74897,  74897,  84265,  84265, 169129, 174761, 174761,
  174765, 174765, 186029, 355757, 355757, 374493, 374493, 454365,
  490429, 490429, 507645, 522237, 522237, 524285, 524285, 524287,
       0,      1,      1,   1025,   8321,   8321,  33825,  69905,
   69905, 148625, 148625, 149801, 304425, 304425, 346793, 349525,
  349525, 354989, 439725, 439725, 748973, 751325, 751325, 908765,
  908765, 980925, 1031933, 1031933, 1046525, 1048573, 1048573, 1048575,
       0,      1,      1,   2049,  16513,  16513,  67649, 139809,
  139809, 280721, 299593, 299593, 338217, 677033, 677033, 699049,
  699053, 743085, 743085, 1420717, 1497965, 1497965, 1535709, 1817533,
  1817533, 1961853, 2064125, 2064125, 2093053, 2097149, 2097149, 2097151,
       0,      1,      1,   2049,  33025, 133185, 133185, 270881,
  297105, 297105, 599185, 608553, 1217705, 1217705, 1395369, 1398101,
  1398101, 1403565, 1758893, 1758893, 2977197, 2995933, 3600093, 3600093,
  3652541, 3927933, 3927933, 4128253, 4190205, 4194301, 4194301, 4194303,
       0,      1,      1,   4097,  33025, 133185, 133185, 541217,
  559249, 1189009, 1189009, 1198377, 2435369, 2708137, 2708137, 2796201,
  2796205, 2972333, 2972333, 3517869, 5991853, 6010589, 6010589, 7270109,
  7306173, 8122237, 8122237, 8322557, 8380413, 8388605, 8388605, 8388607,
       0,      1,   4097,   4097,  65793, 266305, 541217, 541217,
  1118481, 2245777, 2396745, 2697513, 2697513, 4887721, 5581481, 5592405,
  5592405, 5614253, 7001773, 11382189, 11382189, 11983725, 12285661, 14540253,
  15694781, 15694781, 16244605, 16645629, 16769021, 16769021, 16777213, 16777215,
       0,      1,   8193,   8193, 131585, 532609, 1082401, 2236961,
  2236961, 4491409, 4793489, 4868393, 9741609, 9741609, 11096745, 11184809,
  11184813, 11360941, 14071213, 14071213, 23817645, 23967453, 24571613, 29080509,
  29080509, 31389629, 32489213, 33291261, 33538045, 33538045, 33554429, 33554431,
       0,      1,   8193, 131585, 131585, 1056897, 2164801, 4465185,
  4753553, 9577617, 9577617, 9586985, 19212585, 21664937, 22358697, 22369621,
  22369621, 22391469, 23778989, 28683693, 47934893, 47953629, 47953629, 57601757,
  58178493, 62779261, 64995069, 66845693, 66845693, 67092477, 67108861, 67108863,
       0,      1,  16385, 262657, 262657, 1056897, 4261953, 8667681,
  8947857, 19022993, 19173961, 21580073, 21580073, 38966441, 44389033, 44739241,
  44739245, 45439661, 56284845, 91057581, 91057581, 95869805, 96171741, 116322013,
  116882365, 125693821, 132103933, 133692413, 133692413, 134184957, 134217725, 134217727,
       0,      1,  16385, 525313, 2113665, 8521793, 8521793, 8929825,
  17895697, 35932305, 38347921, 38422825, 77932841, 86660265, 89434793, 89478485,
  89478485, 89565869, 95114925, 112569773, 191589805, 191739613, 196570845, 232644061,
  250575805, 251391869, 251391869, 264208125, 267384829, 268402685, 268435453, 268435455,
       0,      1,  32769, 525313, 4227329, 8521793, 17318433, 35791393,
  35791393, 38045841, 76620945, 76695849, 86321449, 156406953, 177556137, 178956969,
  178956973, 181758637, 224057005, 364228013, 383479213, 383629021, 460779229, 465288125,
  465288125, 502234045, 519827325, 528416253, 535820285, 536805373, 536870909, 536870911,
       0,      1,  32769, 1049601, 8421633, 17043521, 34636833, 71442977,
  71862417, 152192145, 153391689, 155797801, 311731497, 346641065, 357870249, 357913941,
  357913941, 358001325, 380459693, 450278829, 762146221, 766958445, 769357533, 930016989,
  930855869, 1004468157, 1039654781, 1056898557, 1071642621, 1073676285, 1073741821, 1073741823,
       0,      1,  65537, 2099201, 8421633, 34087041, 69273665, 138682913,
  143165585, 287458449, 306783377, 307382569, 614803753, 625644713, 714427049, 715827881,
  715827885, 718629549, 896194221, 917876141, 1532718509, 1533916893, 1572566749, 1861152477,
  1870117821, 2008936317, 2079309565, 2130640381, 2143285245, 2147352573, 2147483645, 2147483647,
       0,      1,  65537, 2099201, 16843009, 67641473, 138479681, 277365281,
  286331153, 574916753, 613491857, 613566761, 690563369, 1246925993, 1386828457, 1431481001,
  1432005293, 1521310381, 1801115309, 2913840557, 3067833773, 3067983581, 3145133789, 3722304989,
  3740236733, 4018007933, 4159684349, 4261281277, 4290768893, 4294836221, 4294967293, 4294967295,
};
const prog_uint32_t lut_res_tempo_phase_increment[] PROGMEM = {
       0,  35791,  71582, 107374, 143165, 178956, 214748, 250539,
  286331, 322122, 357913, 393705, 429496, 465288, 501079, 536870,
  572662, 608453, 644245, 680036, 715827, 751619, 787410, 823202,
  858993, 894784, 930576, 966367, 1002159, 1037950, 1073741, 1109533,
  1145324, 1181116, 1216907, 1252698, 1288490, 1324281, 1360072, 1395864,
  1431655, 1467447, 1503238, 1539029, 1574821, 1610612, 1646404, 1682195,
  1717986, 1753778, 1789569, 1825361, 1861152, 1896943, 1932735, 1968526,
  2004318, 2040109, 2075900, 2111692, 2147483, 2183275, 2219066, 2254857,
  2290649, 2326440, 2362232, 2398023, 2433814, 2469606, 2505397, 2541188,
  2576980, 2612771, 2648563, 2684354, 2720145, 2755937, 2791728, 2827520,
  2863311, 2899102, 2934894, 2970685, 3006477, 3042268, 3078059, 3113851,
  3149642, 3185434, 3221225, 3257016, 3292808, 3328599, 3364391, 3400182,
  3435973, 3471765, 3507556, 3543348, 3579139, 3614930, 3650722, 3686513,
  3722304, 3758096, 3793887, 3829679, 3865470, 3901261, 3937053, 3972844,
  4008636, 4044427, 4080218, 4116010, 4151801, 4187593, 4223384, 4259175,
  4294967, 4330758, 4366550, 4402341, 4438132, 4473924, 4509715, 4545507,
  4581298, 4617089, 4652881, 4688672, 4724464, 4760255, 4796046, 4831838,
  4867629, 4903420, 4939212, 4975003, 5010795, 5046586, 5082377, 5118169,
  5153960, 5189752, 5225543, 5261334, 5297126, 5332917, 5368709, 5404500,
  5440291, 5476083, 5511874, 5547666, 5583457, 5619248, 5655040, 5690831,
  5726623, 5762414, 5798205, 5833997, 5869788, 5905580, 5941371, 5977162,
  6012954, 6048745, 6084537, 6120328, 6156119, 6191911, 6227702, 6263493,
  6299285, 6335076, 6370868, 6406659, 6442450, 6478242, 6514033, 6549825,
  6585616, 6621407, 6657199, 6692990, 6728782, 6764573, 6800364, 6836156,
  6871947, 6907739, 6943530, 6979321, 7015113, 7050904, 7086696, 7122487,
  7158278, 7194070, 7229861, 7265653, 7301444, 7337235, 7373027, 7408818,
  7444609, 7480401, 7516192, 7551984, 7587775, 7623566, 7659358, 7695149,
  7730941, 7766732, 7802523, 7838315, 7874106, 7909898, 7945689, 7981480,
  8017272, 8053063, 8088855, 8124646, 8160437, 8196229, 8232020, 8267812,
  8303603, 8339394, 8375186, 8410977, 8446769, 8482560, 8518351, 8554143,
  8589934, 8625725, 8661517, 8697308, 8733100, 8768891, 8804682, 8840474,
  8876265, 8912057, 8947848, 8983639, 9019431, 9055222, 9091014, 9126805,
  9162596, 9198388, 9234179, 9269971, 9305762, 9341553, 9377345, 9413136,
  9448928, 9484719, 9520510, 9556302, 9592093, 9627885, 9663676, 9699467,
  9735259, 9771050, 9806841, 9842633, 9878424, 9914216, 9950007, 9985798,
  10021590, 10057381, 10093173, 10128964, 10164755, 10200547, 10236338, 10272130,
  10307921, 10343712, 10379504, 10415295, 10451087, 10486878, 10522669, 10558461,
  10594252, 10630044, 10665835, 10701626, 10737418, 10773209, 10809001, 10844792,
  10880583, 10916375, 10952166, 10987957, 11023749, 11059540, 11095332, 11131123,
  11166914, 11202706, 11238497, 11274289, 11310080, 11345871, 11381663, 11417454,
  11453246, 11489037, 11524828, 11560620, 11596411, 11632203, 11667994, 11703785,
  11739577, 11775368, 11811160, 11846951, 11882742, 11918534, 11954325, 11990117,
  12025908, 12061699, 12097491, 12133282, 12169074, 12204865, 12240656, 12276448,
  12312239, 12348030, 12383822, 12419613, 12455405, 12491196, 12526987, 12562779,
  12598570, 12634362, 12670153, 12705944, 12741736, 12777527, 12813319, 12849110,
  12884901, 12920693, 12956484, 12992276, 13028067, 13063858, 13099650, 13135441,
  13171233, 13207024, 13242815, 13278607, 13314398, 13350190, 13385981, 13421772,
  13457564, 13493355, 13529146, 13564938, 13600729, 13636521, 13672312, 13708103,
  13743895, 13779686, 13815478, 13851269, 13887060, 13922852, 13958643, 13994435,
  14030226, 14066017, 14101809, 14137600, 14173392, 14209183, 14244974, 14280766,
  14316557, 14352349, 14388140, 14423931, 14459723, 14495514, 14531306, 14567097,
  14602888, 14638680, 14674471, 14710262, 14746054, 14781845, 14817637, 14853428,
  14889219, 14925011, 14960802, 14996594, 15032385, 15068176, 15103968, 15139759,
  15175551, 15211342, 15247133, 15282925, 15318716, 15354508, 15390299, 15426090,
  15461882, 15497673, 15533465, 15569256, 15605047, 15640839, 15676630, 15712422,
  15748213, 15784004, 15819796, 15855587, 15891378, 15927170, 15962961, 15998753,
  16034544, 16070335, 16106127, 16141918, 16177710, 16213501, 16249292, 16285084,
  16320875, 16356667, 16392458, 16428249, 16464041, 16499832, 16535624, 16571415,
  16607206, 16642998, 16678789, 16714581, 16750372, 16786163, 16821955, 16857746,
  16893538, 16929329, 16965120, 17000912, 17036703, 17072495, 17108286, 17144077,
  17179869, 17215660, 17251451, 17287243, 17323034, 17358826, 17394617, 17430408,
  17466200, 17501991, 17537783, 17573574, 17609365, 17645157, 17680948, 17716740,
  17752531, 17788322, 17824114, 17859905, 17895697, 17931488, 17967279, 18003071,
  18038862, 18074654, 18110445, 18146236, 18182028, 18217819, 18253611, 18289402,
};


const prog_uint32_t* const lookup_table32_table[] = {
  lut_res_euclidean,
  lut_res_tempo_phase_increment,
};

const prog_uint8_t node_0[] PROGMEM = {
     255,      0,      0,      0,      0,      0,    145,      0,
       0,      0,      0,      0,    218,      0,      0,      0,
      72,      0,     36,      0,    182,      0,      0,      0,
     109,      0,      0,      0,     72,      0,      0,      0,
      36,      0,    109,      0,      0,      0,      8,      0,
     255,      0,      0,      0,      0,      0,     72,      0,
       0,      0,    182,      0,      0,      0,     36,      0,
     218,      0,      0,      0,    145,      0,      0,      0,
     170,      0,    113,      0,    255,      0,     56,      0,
     170,      0,    141,      0,    198,      0,     56,      0,
     170,      0,    113,      0,    226,      0,     28,      0,
     170,      0,    113,      0,    198,      0,     85,      0,
};
const prog_uint8_t node_1[] PROGMEM = {
     229,      0,     25,      0,    102,      0,     25,      0,
     204,      0,     25,      0,     76,      0,      8,      0,
     255,      0,      8,      0,     51,      0,     25,      0,
     178,      0,     25,      0,    153,      0,    127,      0,
      28,      0,    198,      0,     56,      0,     56,      0,
     226,      0,     28,      0,    141,      0,     28,      0,
      28,      0,    170,      0,     28,      0,     28,      0,
     255,      0,    113,      0,     85,      0,     85,      0,
     159,      0,    159,      0,    255,      0,     63,      0,
     159,      0,    159,      0,    191,      0,     31,      0,
     159,      0,    127,      0,    255,      0,     31,      0,
     159,      0,    127,      0,    223,      0,     95,      0,
};
const prog_uint8_t node_2[] PROGMEM = {
     255,      0,      0,      0,    127,      0,      0,      0,
       0,      0,    102,      0,      0,      0,    229,      0,
       0,      0,    178,      0,    204,      0,      0,      0,
      76,      0,     51,      0,    153,      0,     25,      0,
       0,      0,    127,      0,      0,      0,      0,      0,
     255,      0,    191,      0,     31,      0,     63,      0,
       0,      0,     95,      0,      0,      0,      0,      0,
     223,      0,      0,      0,     31,      0,    159,      0,
     255,      0,     85,      0,    148,      0,     85,      0,
     127,      0,     85,      0,    106,      0,     63,      0,
     212,      0,    170,      0,    191,      0,    170,      0,
      85,      0,     42,      0,    233,      0,     21,      0,
};
const prog_uint8_t node_3[] PROGMEM = {
     255,      0,    212,      0,     63,      0,      0,      0,
     106,      0,    148,      0,     85,      0,    127,      0,
     191,      0,     21,      0,    233,      0,      0,      0,
      21,      0,    170,      0,      0,      0,     42,      0,
       0,      0,      0,      0,    141,      0,    113,      0,
     255,      0,    198,      0,      0,      0,     56,      0,
       0,      0,     85,      0,     56,      0,     28,      0,
     226,      0,     28,      0,    170,      0,     56,      0,
     255,      0,    231,      0,    255,      0,    208,      0,
     139,      0,     92,      0,    115,      0,     92,      0,
     185,      0,     69,      0,     46,      0,     46,      0,
     162,      0,     23,      0,    208,      0,     46,      0,
};
const prog_uint8_t node_4[] PROGMEM = {
     255,      0,     31,      0,     63,      0,     63,      0,
     127,      0,     95,      0,    191,      0,     63,      0,
     223,      0,     31,      0,    159,      0,     63,      0,
      31,      0,     63,      0,     95,      0,     31,      0,
       8,      0,      0,      0,     95,      0,     63,      0,
     255,      0,      0,      0,    127,      0,      0,      0,
       8,      0,      0,      0,    159,      0,     63,      0,
     255,      0,    223,      0,    191,      0,     31,      0,
      76,      0,     25,      0,    255,      0,    127,      0,
     153,      0,     51,      0,    204,      0,    102,      0,
      76,      0,     51,      0,    229,      0,    127,      0,
     153,      0,     51,      0,    178,      0,    102,      0,
};
const prog_uint8_t node_5[] PROGMEM = {
     255,      0,     51,      0,     25,      0,     76,      0,
       0,      0,      0,      0,    102,      0,      0,      0,
     204,      0,    229,      0,      0,      0,    178,      0,
       0,      0,    153,      0,    127,      0,      8,      0,
     178,      0,    127,      0,    153,      0,    204,      0,
     255,      0,      0,      0,     25,      0,     76,      0,
     102,      0,     51,      0,      0,      0,      0,      0,
     229,      0,     25,      0,     25,      0,    204,      0,
     178,      0,    102,      0,    255,      0,     76,      0,
     127,      0,     76,      0,    229,      0,     76,      0,
     153,      0,    102,      0,    255,      0,     25,      0,
     127,      0,     51,      0,    204,      0,     51,      0,
};
const prog_uint8_t node_6[] PROGMEM = {
     255,      0,      0,      0,    223,      0,      0,      0,
      31,      0,      8,      0,    127,      0,      0,      0,
      95,      0,      0,      0,    159,      0,      0,      0,
      95,      0,     63,      0,    191,      0,      0,      0,
      51,      0,    204,      0,      0,      0,    102,      0,
     255,      0,    127,      0,      8,      0,    178,      0,
      25,      0,    229,      0,      0,      0,     76,      0,
     204,      0,    153,      0,     51,      0,     25,      0,
     255,      0,    226,      0,    255,      0,    255,      0,
     198,      0,     28,      0,    141,      0,     56,      0,
     170,      0,     56,      0,     85,      0,     28,      0,
     170,      0,     28,      0,    113,      0,     56,      0,
};
const prog_uint8_t node_7[] PROGMEM = {
     223,      0,      0,      0,     63,      0,      0,      0,
      95,      0,      0,      0,    223,      0,     31,      0,
     255,      0,      0,      0,    159,      0,      0,      0,
     127,      0,     31,      0,    191,      0,     31,      0,
       0,      0,      0,      0,    109,      0,      0,      0,
     218,      0,      0,      0,    182,      0,     72,      0,
       8,      0,     36,      0,    145,      0,     36,      0,
     255,      0,      8,      0,    182,      0,     72,      0,
     255,      0,     72,      0,    218,      0,     36,      0,
     218,      0,      0,      0,    145,      0,      0,      0,
     255,      0,     36,      0,    182,      0,     36,      0,
     182,      0,      0,      0,    109,      0,      0,      0,
};
const prog_uint8_t node_8[] PROGMEM = {
     255,      0,      0,      0,    218,      0,      0,      0,
      36,      0,      0,      0,    218,      0,      0,      0,
     182,      0,    109,      0,    255,      0,      0,      0,
       0,      0,      0,      0,    145,      0,     72,      0,
     159,      0,      0,      0,     31,      0,    127,      0,
     255,      0,     31,      0,      0,      0,     95,      0,
       8,      0,      0,      0,    191,      0,     31,      0,
     255,      0,     31,      0,    223,      0,     63,      0,
     255,      0,     31,      0,     63,      0,     31,      0,
      95,      0,     31,      0,     63,      0,    127,      0,
     159,      0,     31,      0,     63,      0,     31,      0,
     223,      0,    223,      0,    191,      0,    191,      0,
};
const prog_uint8_t node_9[] PROGMEM = {
     226,      0,     28,      0,     28,      0,    141,      0,
       8,      0,      8,      0,    255,      0,      8,      0,
     113,      0,     28,      0,    198,      0,     85,      0,
      56,      0,    198,      0,    170,      0,     28,      0,
       8,      0,     95,      0,      8,      0,      8,      0,
     255,      0,     63,      0,     31,      0,    223,      0,
       8,      0,     31,      0,    191,      0,      8,      0,
     255,      0,    127,      0,    127,      0,    159,      0,
     115,      0,     46,      0,    255,      0,    185,      0,
     139,      0,     23,      0,    208,      0,    115,      0,
     231,      0,     69,      0,    255,      0,    162,      0,
     139,      0,    115,      0,    231,      0,     92,      0,
};
const prog_uint8_t node_10[] PROGMEM = {
     145,      0,      0,      0,      0,      0,    109,      0,
       0,      0,      0,      0,    255,      0,    109,      0,
      72,      0,    218,      0,      0,      0,      0,      0,
      36,      0,      0,      0,    182,      0,      0,      0,
       0,      0,    127,      0,    159,      0,    127,      0,
     159,      0,    191,      0,    223,      0,     63,      0,
     255,      0,     95,      0,     31,      0,     95,      0,
      31,      0,      8,      0,     63,      0,      8,      0,
     255,      0,      0,      0,    145,      0,      0,      0,
     182,      0,    109,      0,    109,      0,    109,      0,
     218,      0,      0,      0,     72,      0,      0,      0,
     182,      0,     72,      0,    182,      0,     36,      0,
};
const prog_uint8_t node_11[] PROGMEM = {
     255,      0,      0,      0,      0,      0,      0,      0,
       0,      0,      0,      0,      0,      0,      0,      0,
     255,      0,      0,      0,    218,      0,     72,     36,
       0,      0,    182,      0,      0,      0,    145,    109,
       0,      0,    127,      0,      0,      0,     42,      0,
     212,      0,      0,    212,      0,      0,    212,      0,
       0,      0,      0,      0,     42,      0,      0,      0,
     255,      0,      0,      0,    170,    170,    127,     85,
     145,      0,    109,    109,    218,    109,     72,      0,
     145,      0,     72,      0,    218,      0,    109,      0,
     182,      0,    109,      0,    255,      0,     72,      0,
     182,    109,     36,    109,    255,    109,    109,      0,
};
const prog_uint8_t node_12[] PROGMEM = {
     255,      0,      0,      0,    255,      0,    191,      0,
       0,      0,      0,      0,     95,      0,     63,      0,
      31,      0,      0,      0,    223,      0,    223,      0,
       0,      0,      8,      0,    159,      0,    127,      0,
       0,      0,     85,      0,     56,      0,     28,      0,
     255,      0,     28,      0,      0,      0,    226,      0,
       0,      0,    170,      0,     56,      0,    113,      0,
     198,      0,      0,      0,    113,      0,    141,      0,
     255,      0,     42,      0,    233,      0,     63,      0,
     212,      0,     85,      0,    191,      0,    106,      0,
     191,      0,     21,      0,    170,      0,      8,      0,
     170,      0,    127,      0,    148,      0,    148,      0,
};
const prog_uint8_t node_13[] PROGMEM = {
     255,      0,      0,      0,      0,      0,     63,      0,
     191,      0,     95,      0,     31,      0,    223,      0,
     255,      0,     63,      0,     95,      0,     63,      0,
     159,      0,      0,      0,      0,      0,    127,      0,
      72,      0,      0,      0,      0,      0,      0,      0,
     255,      0,      0,      0,      0,      0,      0,      0,
      72,      0,     72,      0,     36,      0,      8,      0,
     218,      0,    182,      0,    145,      0,    109,      0,
     255,      0,    162,      0,    231,      0,    162,      0,
     231,      0,    115,      0,    208,      0,    139,      0,
     185,      0,     92,      0,    185,      0,     46,      0,
     162,      0,     69,      0,    162,      0,     23,      0,
};
const prog_uint8_t node_14[] PROGMEM = {
     255,      0,      0,      0,     51,      0,      0,      0,
       0,      0,      0,      0,    102,      0,      0,      0,
     204,      0,      0,      0,    153,      0,      0,      0,
       0,      0,      0,      0,     51,      0,      0,      0,
       0,      0,      0,      0,      8,      0,     36,      0,
     255,      0,      0,      0,    182,      0,      8,      0,
       0,      0,      0,      0,     72,      0,    109,      0,
     145,      0,      0,      0,    255,      0,    218,      0,
     212,      0,      8,      0,    170,      0,      0,      0,
     127,      0,      0,      0,     85,      0,      8,      0,
     255,      0,      8,      0,    170,      0,      0,      0,
     127,      0,      0,      0,     42,      0,      8,      0,
};
const prog_uint8_t node_15[] PROGMEM = {
     255,      0,      0,      0,      0,      0,      0,      0,
      36,      0,      0,      0,    182,      0,      0,      0,
     218,      0,      0,      0,      0,      0,      0,      0,
      72,      0,      0,      0,    145,      0,    109,      0,
      36,      0,     36,      0,      0,      0,      0,      0,
     255,      0,      0,      0,    182,      0,      0,      0,
       0,      0,      0,      0,      0,      0,      0,    109,
     218,      0,      0,      0,    145,      0,     72,     72,
     255,      0,     28,      0,    226,      0,     56,      0,
     198,      0,      0,      0,      0,      0,     28,     28,
     170,      0,      0,      0,    141,      0,      0,      0,
     113,      0,      0,      0,     85,     85,     85,     85,
};
const prog_uint8_t node_16[] PROGMEM = {
     255,      0,      0,      0,      0,      0,     95,      0,
       0,      0,    127,      0,      0,      0,      0,      0,
     223,      0,     95,      0,     63,      0,     31,      0,
     191,      0,      0,      0,    159,      0,      0,      0,
       0,      0,     31,      0,    255,      0,      0,      0,
       0,      0,     95,      0,    223,      0,      0,      0,
       0,      0,     63,      0,    191,      0,      0,      0,
       0,      0,      0,      0,    159,      0,    127,      0,
     141,      0,     28,      0,     28,      0,     28,      0,
     113,      0,      8,      0,      8,      0,      8,      0,
     255,      0,      0,      0,    226,      0,      0,      0,
     198,      0,     56,      0,    170,      0,     85,      0,
};
const prog_uint8_t node_17[] PROGMEM = {
     255,      0,      0,      0,      8,      0,      0,      0,
     182,      0,      0,      0,     72,      0,      0,      0,
     218,      0,      0,      0,     36,      0,      0,      0,
     145,      0,      0,      0,    109,      0,      0,      0,
       0,      0,     51,     25,     76,     25,     25,      0,
     153,      0,      0,      0,    127,    102,    178,      0,
     204,      0,      0,      0,      0,      0,    255,      0,
       0,      0,    102,      0,    229,      0,     76,      0,
     113,      0,      0,      0,    141,      0,     85,      0,
       0,      0,      0,      0,    170,      0,      0,      0,
      56,     28,    255,      0,      0,      0,      0,      0,
     198,      0,      0,      0,    226,      0,      0,      0,
};
const prog_uint8_t node_18[] PROGMEM = {
     255,      0,      8,      0,     28,      0,     28,      0,
     198,      0,     56,      0,     56,      0,     85,      0,
     255,      0,     85,      0,    113,      0,    113,      0,
     226,      0,    141,      0,    170,      0,    141,      0,
       0,      0,      0,      0,      0,      0,      0,      0,
     255,      0,      0,      0,    127,      0,      0,      0,
       0,      0,      0,      0,      0,      0,      0,      0,
      63,      0,      0,      0,    191,      0,      0,      0,
     255,      0,      0,      0,    255,      0,    127,      0,
       0,      0,     85,      0,      0,      0,    212,      0,
       0,      0,    212,      0,     42,      0,    170,      0,
       0,      0,    127,      0,      0,      0,      0,      0,
};
const prog_uint8_t node_19[] PROGMEM = {
     255,      0,      0,      0,      0,      0,    218,      0,
     182,      0,      0,      0,      0,      0,    145,      0,
     145,      0,     36,      0,      0,      0,    109,      0,
     109,      0,      0,      0,     72,      0,     36,      0,
       0,      0,      0,      0,    109,      0,      8,      0,
      72,      0,      0,      0,    255,      0,    182,      0,
       0,      0,      0,      0,    145,      0,      8,      0,
      36,      0,      8,      0,    218,      0,    182,      0,
     255,      0,      0,      0,      0,      0,    226,      0,
      85,      0,      0,      0,    141,      0,      0,      0,
       0,      0,      0,      0,    170,      0,     56,      0,
     198,      0,      0,      0,    113,      0,     28,      0,
};
const prog_uint8_t node_20[] PROGMEM = {
     255,      0,      0,      0,    113,      0,      0,      0,
     198,      0,     56,      0,     85,      0,     28,      0,
     255,      0,      0,      0,    226,      0,      0,      0,
     170,      0,      0,      0,    141,      0,      0,      0,
       0,      0,      0,      0,      0,      0,      0,      0,
     255,      0,    145,      0,    109,      0,    218,      0,
      36,      0,    182,      0,     72,      0,     72,      0,
     255,      0,      0,      0,      0,      0,    109,      0,
      36,      0,     36,      0,    145,      0,      0,      0,
      72,      0,     72,      0,    182,      0,      0,      0,
      72,      0,     72,      0,    218,      0,      0,      0,
     109,      0,    109,      0,    255,      0,      0,      0,
};
const prog_uint8_t node_21[] PROGMEM = {
     255,      0,      0,      0,    218,      0,      0,      0,
     145,      0,      0,      0,     36,      0,      0,      0,
     218,      0,      0,      0,     36,      0,      0,      0,
     182,      0,     72,      0,      0,      0,    109,      0,
       0,      0,      0,      0,      8,      0,      0,      0,
     255,      0,     85,      0,    212,      0,     42,      0,
       0,      0,      0,      0,      8,      0,      0,      0,
      85,      0,    170,      0,    127,      0,     42,      0,
     109,      0,    109,      0,    255,      0,      0,      0,
      72,      0,     72,      0,    218,      0,      0,      0,
     145,      0,    182,      0,    255,      0,      0,      0,
      36,      0,     36,      0,    218,      0,      8,      0,
};
const prog_uint8_t node_22[] PROGMEM = {
     255,      0,      0,      0,     42,      0,      0,      0,
     212,      0,      0,      0,      8,      0,    212,      0,
     170,      0,      0,      0,     85,      0,      0,      0,
     212,      0,      8,      0,    127,      0,      8,      0,
     255,      0,     85,      0,      0,      0,      0,      0,
     226,      0,     85,      0,      0,      0,    198,      0,
       0,      0,    141,      0,     56,      0,      0,      0,
     170,      0,     28,      0,      0,      0,    113,      0,
     113,      0,     56,      0,    255,      0,      0,      0,
      85,      0,     56,      0,    226,      0,      0,      0,
       0,      0,    170,      0,      0,      0,    141,      0,
      28,      0,     28,      0,    198,      0,     28,      0,
};
const prog_uint8_t node_23[] PROGMEM = {
     255,      0,      0,      0,    229,      0,      0,      0,
     204,      0,    204,      0,      0,      0,     76,      0,
     178,      0,    153,      0,     51,      0,    178,      0,
     178,      0,    127,      0,    102,     51,     51,     25,
       0,      0,      0,      0,      0,      0,      0,     31,
       0,      0,      0,      0,    255,      0,      0,     31,
       0,      0,      8,      0,      0,      0,    191,    159,
     127,     95,     95,      0,    223,      0,     63,      0,
     255,      0,    255,      0,    204,    204,    204,    204,
       0,      0,     51,     51,     51,     51,      0,      0,
     204,      0,    204,      0,    153,    153,    153,    153,
     153,      0,      0,      0,    102,    102,    102,    102,
};
const prog_uint8_t node_24[] PROGMEM = {
     170,      0,      0,      0,      0,    255,      0,      0,
     198,      0,      0,      0,      0,     28,      0,      0,
     141,      0,      0,      0,      0,    226,      0,      0,
      56,      0,      0,    113,      0,     85,      0,      0,
     255,      0,      0,      0,      0,    113,      0,      0,
      85,      0,      0,      0,      0,    226,      0,      0,
     141,      0,      0,      8,      0,    170,     56,     56,
     198,      0,      0,     56,      0,    141,     28,      0,
     255,      0,      0,      0,      0,    191,      0,      0,
     159,      0,      0,      0,      0,    223,      0,      0,
      95,      0,      0,      0,      0,     63,      0,      0,
     127,      0,      0,      0,      0,     31,      0,      0,
};


const prog_uint8_t* const node_table[] = {
  node_0,
  node_1,
  node_2,
  node_3,
  node_4,
  node_5,
  node_6,
  node_7,
  node_8,
  node_9,
  node_10,
  node_11,
  node_12,
  node_13,
  node_14,
  node_15,
  node_16,
  node_17,
  node_18,
  node_19,
  node_20,
  node_21,
  node_22,
  node_23,
  node_24,
};


}  // namespace grids
