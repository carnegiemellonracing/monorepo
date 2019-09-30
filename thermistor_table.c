/**
 * @file thermistor_table.c
 * @brief thermistor resistance to temperature tables.
 *
 * @author carnegie mellon racing
 */

#include "thermistor_table.h"

/**
 * @brief Conversions from switch thermistor resistance to temperature (lookup NTCG103JF103FT1S for full table).
 *
 * @warning MUST be sorted in descending order of resistance!
 */
const thermistorTempConversion_t thermTempConvsSwitch[] = {{
    .resistance_Ohm = 27280,
    .temp_dC = 0
}, {
    .resistance_Ohm = 17960,
    .temp_dC = 100
}, {
    .resistance_Ohm = 12090,
    .temp_dC = 200
}, {
    .resistance_Ohm = 8310,
    .temp_dC = 300
}, {
    .resistance_Ohm = 5826,
    .temp_dC = 400
}, {
    .resistance_Ohm = 4158,
    .temp_dC = 500
}, {
    .resistance_Ohm = 3019,
    .temp_dC = 600
}, {
    .resistance_Ohm = 2227,
    .temp_dC = 700
}, {
    .resistance_Ohm = 1668,
    .temp_dC = 800
}, {
    .resistance_Ohm = 1267,
    .temp_dC = 900
}, {
    .resistance_Ohm = 975,
    .temp_dC = 1000
}, {
    .resistance_Ohm = 760,
    .temp_dC = 1100
}, {
    .resistance_Ohm = 599,
    .temp_dC = 1200
}, {
    .resistance_Ohm = 478,
    .temp_dC = 1300
}, {
    .resistance_Ohm = 385,
     .temp_dC = 1400
}, {
    .resistance_Ohm = 313,
    .temp_dC = 1500
}};

/**
 * @brief Conversions from rad thermistor resistance to temperature (lookup USP10978 for full table).
 *
 * @warning MUST be sorted in descending order of resistance!
 */
const thermistorTempConversion_t thermTempConvsRadiator[] = {{
    .resistance_Ohm = 32650,
    .temp_dC = 0
}, {
    .resistance_Ohm = 31029,
    .temp_dC = 10
}, {
    .resistance_Ohm = 29498,
    .temp_dC = 20
}, {
    .resistance_Ohm = 28052,
    .temp_dC = 30
}, {
    .resistance_Ohm = 26685,
    .temp_dC = 40
}, {
    .resistance_Ohm = 25392,
    .temp_dC = 50
}, {
    .resistance_Ohm = 24170,
    .temp_dC = 60
}, {
    .resistance_Ohm = 23013,
    .temp_dC = 70
}, {
    .resistance_Ohm = 21918,
    .temp_dC = 80
}, {
    .resistance_Ohm = 20882,
    .temp_dC = 90
}, {
    .resistance_Ohm = 19901,
    .temp_dC = 100
}, {
    .resistance_Ohm = 18971,
    .temp_dC = 110
}, {
    .resistance_Ohm = 18090,
    .temp_dC = 120
}, {
    .resistance_Ohm = 17255,
    .temp_dC = 130
}, {
    .resistance_Ohm = 16463,
    .temp_dC = 140
}, {
    .resistance_Ohm = 15712,
    .temp_dC = 150
}, {
    .resistance_Ohm = 14999,
    .temp_dC = 160
}, {
    .resistance_Ohm = 14323,
    .temp_dC = 170
}, {
    .resistance_Ohm = 13681,
    .temp_dC = 180
}, {
    .resistance_Ohm = 13072,
    .temp_dC = 190
}, {
    .resistance_Ohm = 12493,
    .temp_dC = 200
}, {
    .resistance_Ohm = 11942,
    .temp_dC = 210
}, {
    .resistance_Ohm = 11419,
    .temp_dC = 220
}, {
    .resistance_Ohm = 10922,
    .temp_dC = 230
}, {
    .resistance_Ohm = 10450,
    .temp_dC = 240
}, {
    .resistance_Ohm = 10000,
    .temp_dC = 250
}, {
    .resistance_Ohm = 9572,
    .temp_dC = 260
}, {
    .resistance_Ohm = 9165,
    .temp_dC = 270
}, {
    .resistance_Ohm = 8777,
    .temp_dC = 280
}, {
    .resistance_Ohm = 8408,
    .temp_dC = 290
}, {
    .resistance_Ohm = 8057,
    .temp_dC = 300
}, {
    .resistance_Ohm = 7722,
    .temp_dC = 310
}, {
    .resistance_Ohm = 7402,
    .temp_dC = 320
}, {
    .resistance_Ohm = 7098,
    .temp_dC = 330
}, {
    .resistance_Ohm = 6808,
    .temp_dC = 340
}, {
    .resistance_Ohm = 6531,
    .temp_dC = 350
}, {
    .resistance_Ohm = 6267,
    .temp_dC = 360
}, {
    .resistance_Ohm = 6015,
    .temp_dC = 370
}, {
    .resistance_Ohm = 5775,
    .temp_dC = 380
}, {
    .resistance_Ohm = 5545,
    .temp_dC = 390
}, {
    .resistance_Ohm = 5326,
    .temp_dC = 400
}, {
    .resistance_Ohm = 5117,
    .temp_dC = 410
}, {
    .resistance_Ohm = 4917,
    .temp_dC = 420
}, {
    .resistance_Ohm = 4725,
    .temp_dC = 430
}, {
    .resistance_Ohm = 4543,
    .temp_dC = 440
}, {
    .resistance_Ohm = 4368,
    .temp_dC = 450
}, {
    .resistance_Ohm = 4201,
    .temp_dC = 460
}, {
    .resistance_Ohm = 4041,
    .temp_dC = 470
}, {
    .resistance_Ohm = 3888,
    .temp_dC = 480
}, {
    .resistance_Ohm = 3742,
    .temp_dC = 490
}, {
    .resistance_Ohm = 3602,
    .temp_dC = 500
}, {
    .resistance_Ohm = 3468,
    .temp_dC = 510
}, {
    .resistance_Ohm = 3340,
    .temp_dC = 520
}, {
    .resistance_Ohm = 3217,
    .temp_dC = 530
}, {
    .resistance_Ohm = 3099,
    .temp_dC = 540
}, {
    .resistance_Ohm = 2986,
    .temp_dC = 550
}, {
    .resistance_Ohm = 2878,
    .temp_dC = 560
}, {
    .resistance_Ohm = 2774,
    .temp_dC = 570
}, {
    .resistance_Ohm = 2675,
    .temp_dC = 580
}, {
    .resistance_Ohm = 2579,
    .temp_dC = 590
}, {
    .resistance_Ohm = 2488,
    .temp_dC = 600
}, {
    .resistance_Ohm = 2400,
    .temp_dC = 610
}, {
    .resistance_Ohm = 2316,
    .temp_dC = 620
}, {
    .resistance_Ohm = 2235,
    .temp_dC = 630
}, {
    .resistance_Ohm = 2157,
    .temp_dC = 640
}, {
    .resistance_Ohm = 2083,
    .temp_dC = 650
}, {
    .resistance_Ohm = 2011,
    .temp_dC = 660
}, {
    .resistance_Ohm = 1942,
    .temp_dC = 670
}, {
    .resistance_Ohm = 1876,
    .temp_dC = 680
}, {
    .resistance_Ohm = 1813,
    .temp_dC = 690
}, {
    .resistance_Ohm = 1752,
    .temp_dC = 700
}, {
    .resistance_Ohm = 1693,
    .temp_dC = 710
}, {
    .resistance_Ohm = 1637,
    .temp_dC = 720
}, {
    .resistance_Ohm = 1582,
    .temp_dC = 730
}, {
    .resistance_Ohm = 1530,
    .temp_dC = 740
}, {
    .resistance_Ohm = 1480,
    .temp_dC = 750
}, {
    .resistance_Ohm = 1432,
    .temp_dC = 760
}, {
    .resistance_Ohm = 1385,
    .temp_dC = 770
}, {
    .resistance_Ohm = 1340,
    .temp_dC = 780
}, {
    .resistance_Ohm = 1297,
    .temp_dC = 790
}, {
    .resistance_Ohm = 1255,
    .temp_dC = 800
}, {
    .resistance_Ohm = 1215,
    .temp_dC = 810
}, {
    .resistance_Ohm = 1177,
    .temp_dC = 820
}, {
    .resistance_Ohm = 1140,
    .temp_dC = 830
}, {
    .resistance_Ohm = 1104,
    .temp_dC = 840
}, {
    .resistance_Ohm = 1070,
    .temp_dC = 850
}, {
    .resistance_Ohm = 1037,
    .temp_dC = 860
}, {
    .resistance_Ohm = 1005,
    .temp_dC = 870
}, {
    .resistance_Ohm = 973,
    .temp_dC = 880
}, {
    .resistance_Ohm = 944,
    .temp_dC = 890
}, {
    .resistance_Ohm = 915,
    .temp_dC = 900
}, {
    .resistance_Ohm = 887,
    .temp_dC = 910
}, {
    .resistance_Ohm = 861,
    .temp_dC = 920
}, {
    .resistance_Ohm = 835,
    .temp_dC = 930
}, {
    .resistance_Ohm = 810,
    .temp_dC = 940
}, {
    .resistance_Ohm = 786,
    .temp_dC = 950
}, {
    .resistance_Ohm = 763,
    .temp_dC = 960
}, {
    .resistance_Ohm = 741,
    .temp_dC = 970
}, {
    .resistance_Ohm = 719,
    .temp_dC = 980
}, {
    .resistance_Ohm = 698,
    .temp_dC = 990
}, {
    .resistance_Ohm = 678,
    .temp_dC = 1000
}};


const size_t radThermTempConvs_len = sizeof(thermTempConvsRadiator) / sizeof(thermTempConvsRadiator[0]);
const size_t switchThermTempConvs_len = sizeof(switchThermTempConvs) / sizeof(switchThermTempConvs[0]);



