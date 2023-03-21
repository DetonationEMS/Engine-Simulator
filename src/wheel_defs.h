/********************************************************************************************
* Standalone Engine Simulator
* Aaron S. (Detonation) 2023
******************************
  This file is (mostly) copy and paste from the official Ardustim code. The only modifications
    are spelling mistake fixes, modified and/or additional notes.
       https://github.com/speeduino/Ardu-Stim
********************************************************************************************/

/* Arbritray wheel pattern generator wheel definitions
 *
 * copyright 2014 David J. Andruczyk
 *
 * Ardu-Stim software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ArduStim software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with any ArduStim software.  If not, see http://www.gnu.org/licenses/
 */

/* Wheel patterns!
 *
 * Wheel patterns define the pin states and specific times. The ISR runs
 * at a constant speed related to the requested RPM. The request RPM is
 * scaled based on the LENGTH of each wheel's array.  The reference is
 * the 60-2 which was the first decoder designed which has 120 "edges"
 * (transitions" for each revolution of the wheel. Any other wheel that
 * also has 120 edges has and RPM scaling factor of 1.0. IF a wheel has
 * less edges needed to "describe" it, it's number of edges are divided by 120 to
 * get the scaling factor which is applied to the RPM calculation.
 * There is an enumeration (below) that lists the defined wheel types,
 * as well as an array listing the rpm_scaling factors with regards to
 * each pattern.
 *
 * NOTE: There is MORE THAN ONE WAY to define a wheel pattern.  You can
 * use more edges to get to 1 deg accuracy but the side effect is that
 * your maximum RPM is capped because of that. Currently 60-2 can run
 * up to about 60,000 RPM, 360and8 can only do about 10,000 RPM becasue
 * it has 6x the number of edges...  The less edges, the faster it can go... :)
 *
 * Using more edges allows you to do things like vary the dutycycle,
 * i.e. a simple non-missing tooth 50% duty cycle wheel can be defined
 * with only 2 entries if you really want, but I didn't do it that way
 * for some of the simple ones as it made it seem somewhat confusing
 * to look at as it required you to keep the rpm_scaler factor in mind.
 * Most/all patterns show the pulses you're receive for one revolution
 * of a REAL wheel on a real engine.
 */

/* Wheel types we know about...
 * This enumerations is the INDEX into the Wheels[] array of structures
 * defined in main file. That struct contains pointers to the following:
 * wheel name in a user friendly string
 * pointer to the wheel edge array used by the ISR
 * RPM scaling factor (num_edges/120 for crank wheels)
 * Number of edges in the edge array above, needed by the ISR
 */

#ifndef WHEEL_DEFS_H
#define WHEEL_DEFS_H

#include <Arduino.h>

typedef enum
{
  DIZZY_FOUR_CYLINDER,               /* 2 evenly spaced teeth */
  DIZZY_SIX_CYLINDER,                /* 3 evenly spaced teeth */
  DIZZY_EIGHT_CYLINDER,              /* 4 evenly spaced teeth */
  SIXTY_MINUS_TWO,                   /* 60-2 crank only */
  SIXTY_MINUS_TWO_WITH_CAM,          /* 60-2 with 2nd trigger on cam */
  SIXTY_MINUS_TWO_WITH_HALFMOON_CAM, /* 60-2 with "half moon" trigger on cam */
  THIRTY_SIX_MINUS_ONE,              /* 36-1 crank only */
  TWENTY_FOUR_MINUS_ONE,
  FOUR_MINUS_ONE_WITH_CAM,                                  /* 4-1 crank + cam */
  EIGHT_MINUS_ONE,                                          /* 8-1 crank only */
  SIX_MINUS_ONE_WITH_CAM,                                   /* 6-1 crank + cam */
  TWELVE_MINUS_ONE_WITH_CAM,                                /* 12-1 crank + cam */
  FOURTY_MINUS_ONE,                                         /* Ford V-10 40-1 crank only */
  DIZZY_FOUR_TRIGGER_RETURN,                                /* dizzy 4 cylinder signal, 40deg on 50 deg off */
  ODDFIRE_VR,                                               /* Oddfire V-twin */
  OPTISPARK_LT1,                                            /* Optispark 360 and 8 */
  TWELVE_MINUS_THREE,                                       /* 12-3 */
  THIRTY_SIX_MINUS_TWO_TWO_TWO,                             /* 36-2-2-2 crank only H4 */
  THIRTY_SIX_MINUS_TWO_TWO_TWO_H6,                          /* 36-2-2-2 crank only H6 */
  THIRTY_SIX_MINUS_TWO_TWO_TWO_WITH_CAM,                    /* 36-2-2-2 crank and cam */
  FOURTY_TWO_HUNDRED_WHEEL,                                 /* 4200 wheel */
  THIRTY_SIX_MINUS_ONE_WITH_CAM_FE3,                        /* Mazda F3 36-1 crank and cam */
  SIX_G_SEVENTY_TWO_WITH_CAM,                               /* Mitsubishi DOHC CAS and TCDS 6G72 */
  BUELL_ODDFIRE_CAM,                                        /* Buell 45 deg cam wheel */
  GM_LS1_CRANK_AND_CAM,                                     /* GM LS1 24 tooth with cam */
  LOTUS_THIRTY_SIX_MINUS_ONE_ONE_ONE_ONE,                   /* Lotus crank wheel 36-1-1-1-1 */
  HONDA_RC51_WITH_CAM,                                      /* Honda oddfire 90 deg V-twin */
  THIRTY_SIX_MINUS_ONE_WITH_SECOND_TRIGGER,                 /* From jimstim */
  CHRYSLER_NGC_THIRTY_SIX_PLUS_TWO_MINUS_TWO_WITH_NGC4_CAM, /* Chrysler NGC 36+2-2 crank with NGC 4 cylinder cam pattern */
  CHRYSLER_NGC_THIRTY_SIX_MINUS_TWO_PLUS_TWO_WITH_NGC6_CAM, /* Chrysler NGC 36-2+2 crank with NGC 6 cylinder cam pattern */
  CHRYSLER_NGC_THIRTY_SIX_MINUS_TWO_PLUS_TWO_WITH_NGC8_CAM, /* Chrysler NGC 36-2+2 crank with NGC 8 cylinder cam pattern */
  WEBER_IAW_WITH_CAM,                                       /* From jimstim IAW weber-marelli */
  FIAT_ONE_POINT_EIGHT_SIXTEEN_VALVE_WITH_CAM,              /* Fiat 1.8 16V from jimstim */
  THREE_SIXTY_NISSAN_CAS,                                   /*from jimstim 360 tooth cas with 6 slots */
  TWENTY_FOUR_MINUS_TWO_WITH_SECOND_TRIGGER,                /* Mazda CAS 24-1 inner ring single pulse outer ring */
  YAMAHA_EIGHT_TOOTH_WITH_CAM,                              /* 02-03 Yamaha R1, seank */
  GM_FOUR_TOOTH_WITH_CAM,                                   /* GM 4 even crank with half moon cam */
  GM_SIX_TOOTH_WITH_CAM,                                    /* GM 4 even crank with half moon cam */
  GM_EIGHT_TOOTH_WITH_CAM,                                  /* GM 4 even crank with half moon cam */
  VOLVO_D12ACD_WITH_CAM,                                    /* Volvo Diesel d12[acd] with cam (alex32 on forums.libreems.org */
  MAZDA_THIRTY_SIX_MINUS_TWO_TWO_TWO_WITH_SIX_TOOTH_CAM,
  MITSUBISH_4g63_4_2,
  AUDI_135_WITH_CAM,
  HONDA_D17_NO_CAM,
  MAZDA_323_AU,
  DAIHATSU_3CYL,
  MIATA_9905,
  TWELVE_WITH_CAM,                  // 12 evenly spaced crank teeth and a single cam tooth
  TWENTY_FOUR_WITH_CAM,             // 24 evenly spaced crank teeth and a single cam tooth
  SUBARU_SIX_SEVEN,                 /* Subaru 6 crank, 7 cam */
  GM_7X,                            /* GM 7X pattern. 6 even teeth with 1 extra uneven tooth */
  FOUR_TWENTY_A,                    /* DSM 420a */
  FORD_ST170,                       /* Ford ST170 */
  MITSUBISHI_3A92,                  /* Mitsubishi 3cylinder 3A92 */
  TOYOTA_4AGE_CAS,                  /*Toyota 4AGE CAS, 4 teeth and one cam tooth*/
  TOYOTA_4AGZE,                     /*Toyota 4AGZE, 24 teeth and one cam tooth*/
  SUZUKI_DRZ400,                    /* Suzuki DRZ-400 6 coil "tooths", 2 uneven crank tooths */
  JEEP2000,                         /* Jeep 4.0 6cyl aka jeep2000 */
  ROVER_K_MODE1_36_ONE_ONE,         /* early MEMS versions with 24-1-1 pattern (aka 12-1 at cam speed) */
  ROVER_K_MODE2_36_ONE_ONE_ONE_ONE, /* MEMS 1.9 of the 36 with 4 missing teeth */
  ROVER_K_MODE3_36_ONE_ONE_ONE_ONE, /* MEMS 2.0 of the 36 with 4 missing teeth at different places */
  ROVER_K_MODE4_36_ONE_ONE_ONE_ONE, /* MEMS 3.0 of the 36 with 4 missing teeth at different places */
  MAX_WHEELS,
} WheelType;

/* Name strings for EACH wheel type, for serial UI */
const char dizzy_four_cylinder_friendly_name[] PROGMEM = "4 cylinder dizzy";
const char dizzy_six_cylinder_friendly_name[] PROGMEM = "6 cylinder dizzy";
const char dizzy_eight_cylinder_friendly_name[] PROGMEM = "8 cylinder dizzy";
const char sixty_minus_two_friendly_name[] PROGMEM = "60-2 crank only";
const char sixty_minus_two_with_cam_friendly_name[] PROGMEM = "60-2 crank and cam";
const char sixty_minus_two_with_halfmoon_cam_friendly_name[] PROGMEM = "60-2 crank and 'half moon' cam";
const char thirty_six_minus_one_friendly_name[] PROGMEM = "36-1 crank only";
const char twenty_four_minus_one_friendly_name[] PROGMEM = "24-1 crank only";
const char four_minus_one_with_cam_friendly_name[] PROGMEM = "4-1 crank wheel with cam";
const char eight_minus_one_friendly_name[] PROGMEM = "8-1 crank only (R6)";
const char six_minus_one_with_cam_friendly_name[] PROGMEM = "6-1 crank with cam";
const char twelve_minus_one_with_cam_friendly_name[] PROGMEM = "12-1 crank with cam";
const char fourty_minus_one_friendly_name[] PROGMEM = "40-1 crank only (Ford V10)";
const char dizzy_four_trigger_return_friendly_name[] PROGMEM = "Distributor style 4 cyl 50deg off, 40 deg on";
const char oddfire_vr_friendly_name[] PROGMEM = "odd fire 90 deg pattern 0 and 135 pulses";
const char optispark_lt1_friendly_name[] PROGMEM = "GM OptiSpark LT1 360 and 8";
const char twelve_minus_three_friendly_name[] PROGMEM = "12-3 oddball";
const char thirty_six_minus_two_two_two_friendly_name[] PROGMEM = "36-2-2-2 H4 Crank only";
const char thirty_six_minus_two_two_two_h6_friendly_name[] PROGMEM = "36-2-2-2 H6 Crank only";
const char thirty_six_minus_two_two_two_with_cam_friendly_name[] PROGMEM = "36-2-2-2 Crank and cam";
const char fourty_two_hundred_wheel_friendly_name[] PROGMEM = "GM 4200 crank wheel";
const char thirty_six_minus_one_with_cam_fe3_friendly_name[] PROGMEM = "Mazda FE3 36-1 with cam";
const char six_g_seventy_two_with_cam_friendly_name[] PROGMEM = "Mitsubishi 6g72 with cam";
const char buell_oddfire_cam_friendly_name[] PROGMEM = "Buell Oddfire CAM wheel";
const char gm_ls1_crank_and_cam_friendly_name[] PROGMEM = "GM LS1 crank and cam";
const char lotus_thirty_six_minus_one_one_one_one_friendly_name[] PROGMEM = "Odd Lotus 36-1-1-1-1 flywheel";
const char honda_rc51_with_cam_friendly_name[] PROGMEM = "Honda RC51 with cam";
const char thirty_six_minus_one_with_second_trigger_friendly_name[] PROGMEM = "36-1 crank with 2nd trigger on teeth 33-34";
const char chrysler_ngc_thirty_six_plus_two_minus_two_with_ngc4_cam_friendly_name[] PROGMEM = "Chrysler NGC 36+2-2 crank, NGC 4-cyl cam";
const char chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc6_cam_friendly_name[] PROGMEM = "Chrysler NGC 36-2+2 crank, NGC 6-cyl cam";
const char chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc8_cam_friendly_name[] PROGMEM = "Chrysler NGC 36-2+2 crank, NGC 8-cyl cam";
const char weber_iaw_with_cam_friendly_name[] PROGMEM = "Weber-Marelli 8 crank+2 cam pattern";
const char fiat_one_point_eight_sixteen_valve_with_cam_friendly_name[] PROGMEM = "Fiat 1.8 16V crank and cam";
const char three_sixty_nissan_cas_friendly_name[] PROGMEM = "Nissan 360 CAS with 6 slots";
const char twenty_four_minus_two_with_second_trigger_friendly_name[] PROGMEM = "Mazda CAS 24-2 with single pulse outer ring";
const char yamaha_eight_tooth_with_cam_friendly_name[] PROGMEM = "Yamaha 2002-03 R1 8 even-tooth crank with 1 tooth cam";
const char gm_four_tooth_with_cam_friendly_name[] PROGMEM = "GM 4 even-tooth crank with 1 tooth cam";
const char gm_six_tooth_with_cam_friendly_name[] PROGMEM = "GM 6 even-tooth crank with 1 tooth cam";
const char gm_eight_tooth_with_cam_friendly_name[] PROGMEM = "GM 8 even-tooth crank with 1 tooth cam";
const char volvo_d12acd_with_cam_friendly_name[] PROGMEM = "Volvo d12[acd] crank with 7 tooth cam";
const char mazda_thirty_six_minus_two_two_two_with_six_tooth_cam_friendly_name[] PROGMEM = "Mazda 36-2-2-2 with 6 tooth cam";
const char mitsubishi_4g63_4_2_friendly_name[] PROGMEM = "Mitsubishi 4g63 aka 4/2 crank and cam";
const char audi_135_with_cam_friendly_name[] PROGMEM = "Audi 135 tooth crank and cam";
const char honda_d17_no_cam_friendly_name[] PROGMEM = "Honda D17 Crank (12+1)";
const char mazda_323_au_friendly_name[] PROGMEM = "Mazda 323 AU version";
const char daihatsu_3cyl_friendly_name[] PROGMEM = "Daihatsu 3+1 distributor (3 cylinders)";
const char miata_9905_friendly_name[] PROGMEM = "Miata 99-05";
const char twelve_with_cam_friendly_name[] PROGMEM = "12/1 (12 crank with cam)";
const char twenty_four_with_cam_friendly_name[] PROGMEM = "24/1 (24 crank with cam)";
const char subaru_six_seven_name_friendly_name[] PROGMEM = "Subaru 6/7 crank and cam";
const char gm_seven_x_friendly_name[] PROGMEM = "GM 7X";
const char four_twenty_a_friendly_name[] PROGMEM = "DSM 420a";
const char ford_st170_friendly_name[] PROGMEM = "Ford ST170";
const char mitsubishi_3A92_friendly_name[] PROGMEM = "Mitsubishi 3A92";
const char Toyota_4AGE_CAS_friendly_name[] PROGMEM = "Toyota 4AGE";
const char Toyota_4AGZE_friendly_name[] PROGMEM = "Toyota 4AGZE";
const char Suzuki_DRZ400_friendly_name[] PROGMEM = "Suzuki DRZ400";
const char Jeep_2000_friendly_name[] PROGMEM = "Jeep 2000";
const char rover_k_mode1_thirtysix_minus_one_one_friendly_name[] PROGMEM = "Rover k series 36-1-1 flywheel EARLY";
const char rover_k_mode2_thirtysix_minus_one_one_one_one_friendly_name[] PROGMEM = "Rover k series 36-1-1-1-1 flywheel MEMS 1.9";
const char rover_k_mode3_thirtysix_minus_one_one_one_one_friendly_name[] PROGMEM = "Rover k series 36-1-1-1-1 flywheel MEMS 2";
const char rover_k_mode4_thirtysix_minus_one_one_one_one_friendly_name[] PROGMEM = "Rover k series 36-1-1-1-1 flywheel MEMS 3";

extern const unsigned char dizzy_four_cylinder[] PROGMEM;
extern const unsigned char dizzy_six_cylinder[] PROGMEM;
extern const unsigned char dizzy_eight_cylinder[] PROGMEM;
extern const unsigned char sixty_minus_two[] PROGMEM;
extern const unsigned char sixty_minus_two_with_cam[] PROGMEM;
extern const unsigned char sixty_minus_two_with_halfmoon_cam[] PROGMEM;
extern const unsigned char thirty_six_minus_one[] PROGMEM;
extern const unsigned char twenty_four_minus_one[] PROGMEM;
extern const unsigned char four_minus_one_with_cam[] PROGMEM;
extern const unsigned char eight_minus_one[] PROGMEM;
extern const unsigned char six_minus_one_with_cam[] PROGMEM;
extern const unsigned char twelve_minus_one_with_cam[] PROGMEM;

// To be Generated
extern unsigned char fourty_minus_one[];
extern unsigned char dizzy_four_trigger_return[];
extern unsigned char oddfire_vr[];
extern unsigned char optispark_lt1[];

extern const unsigned char twelve_minus_three[] PROGMEM;
extern const unsigned char thirty_six_minus_two_two_two[] PROGMEM;
extern const unsigned char thirty_six_minus_two_two_two_h6[] PROGMEM;
extern const unsigned char thirty_six_minus_two_two_two_with_cam[] PROGMEM;
extern const unsigned char fourty_two_hundred_wheel[] PROGMEM;
extern const unsigned char thirty_six_minus_one_with_cam_fe3[] PROGMEM;
extern const unsigned char six_g_seventy_two_with_cam[] PROGMEM;
extern const unsigned char buell_oddfire_cam[] PROGMEM;
extern const unsigned char gm_ls1_crank_and_cam[] PROGMEM;
extern const unsigned char lotus_thirty_six_minus_one_one_one_one[] PROGMEM;
extern const unsigned char honda_rc51_with_cam[] PROGMEM;
extern const unsigned char thirty_six_minus_one_with_second_trigger[] PROGMEM;
extern const unsigned char chrysler_ngc_thirty_six_plus_two_minus_two_with_ngc4_cam[] PROGMEM;
extern const unsigned char chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc6_cam[] PROGMEM;
extern const unsigned char chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc8_cam[] PROGMEM;
extern const unsigned char weber_iaw_with_cam[] PROGMEM;
extern const unsigned char fiat_one_point_eight_sixteen_valve_with_cam[] PROGMEM;
extern const unsigned char three_sixty_nissan_cas[] PROGMEM;
extern const unsigned char twenty_four_minus_two_with_second_trigger[] PROGMEM;
extern const unsigned char yamaha_eight_tooth_with_cam[] PROGMEM;
extern const unsigned char gm_four_tooth_with_cam[] PROGMEM;
extern const unsigned char gm_six_tooth_with_cam[] PROGMEM;
extern const unsigned char gm_eight_tooth_with_cam[] PROGMEM;
extern const unsigned char volvo_d12acd_with_cam[] PROGMEM;
extern const unsigned char mazda_thirty_six_minus_two_two_two_with_six_tooth_cam[] PROGMEM;
extern const unsigned char mitsubishi_4g63_4_2[] PROGMEM;
extern const unsigned char audi_135_with_cam[] PROGMEM;
extern const unsigned char honda_d17_no_cam[] PROGMEM;
extern const unsigned char mazda_323_au[] PROGMEM;
extern const unsigned char daihatsu_3cyl[] PROGMEM;
extern const unsigned char miata_9905[] PROGMEM;
extern const unsigned char twelve_with_cam[] PROGMEM;
extern const unsigned char twenty_four_with_cam[] PROGMEM;
extern const unsigned char subaru_six_seven[] PROGMEM;
extern const unsigned char gm_seven_x[] PROGMEM;
extern const unsigned char four_twenty_a[] PROGMEM;
extern const unsigned char ford_st170[] PROGMEM;
extern const unsigned char mitsubishi_3A92[] PROGMEM;
extern const unsigned char toyota_4AGE_CAS[] PROGMEM;
extern const unsigned char toyota_4AGZE[] PROGMEM;
extern const unsigned char suzuki_DRZ400[] PROGMEM;
extern const unsigned char jeep_2000[] PROGMEM;
extern const unsigned char rover_k_mode1_thirtysix_minus_one_one[] PROGMEM;
extern const unsigned char rover_k_mode2_thirtysix_minus_one_one_one_one[] PROGMEM;
extern const unsigned char rover_k_mode3_thirtysix_minus_one_one_one_one[] PROGMEM;
extern const unsigned char rover_k_mode4_thirtysix_minus_one_one_one_one[] PROGMEM;

#endif
