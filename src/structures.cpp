#include <Arduino.h>
#include <stdint.h>
#include "trigger_arrays.h"
#include "structures.h"

#if defined(AVR328)
#include <avr/pgmspace.h>
#endif

// Access the specific pattern stored in Wheels using currentPattern as an index
// const char *selectedPatternName = Wheels[currentPattern].patternName;
// const unsigned char *selectedTriggerPattern = Wheels[currentPattern].selectedPattern;
// const float rpmScaler = Wheels[currentPattern].rpm_scaler;
// const uint16_t patternLength = Wheels[currentPattern].patternLength;
// const uint16_t wheelDegrees = Wheels[currentPattern].wheel_degrees;
// Use the selected trigger pattern, RPM scaler, pattern length, and wheel degrees as needed

/* Defined Structure for each wheel available. Copy & Pasted from Ardustim */
wheels Wheels[MAX_WHEELS] = {
    /* Pointer to friendly name string, pointer to edge array, RPM Scaler, Number of edges in the array */
    {dizzy_four_cylinder_friendly_name, dizzy_four_cylinder, 0.03333, 4, 360},
    {dizzy_six_cylinder_friendly_name, dizzy_six_cylinder, 0.05, 6, 360},
    {dizzy_eight_cylinder_friendly_name, dizzy_eight_cylinder, 0.06667, 8, 360},
    {sixty_minus_two_friendly_name, sixty_minus_two, 1.0, 120, 360},
    {sixty_minus_two_with_cam_friendly_name, sixty_minus_two_with_cam, 1.0, 240, 720},
    {sixty_minus_two_with_halfmoon_cam_friendly_name, sixty_minus_two_with_halfmoon_cam, 1.0, 240, 720},
    {thirty_six_minus_one_friendly_name, thirty_six_minus_one, 0.6, 72, 360},
    {twenty_four_minus_one_friendly_name, twenty_four_minus_one, 0.5, 48, 360},
    {four_minus_one_with_cam_friendly_name, four_minus_one_with_cam, 0.06667, 16, 720},
    {eight_minus_one_friendly_name, eight_minus_one, 0.13333, 16, 360},
    {six_minus_one_with_cam_friendly_name, six_minus_one_with_cam, 0.15, 36, 720},
    {twelve_minus_one_with_cam_friendly_name, twelve_minus_one_with_cam, 0.6, 144, 720},
    {fourty_minus_one_friendly_name, fourty_minus_one, 0.66667, 80, 360},
    {dizzy_four_trigger_return_friendly_name, dizzy_four_trigger_return, 0.15, 9, 720},
    {oddfire_vr_friendly_name, oddfire_vr, 0.2, 24, 360},
    {optispark_lt1_friendly_name, optispark_lt1, 3.0, 720, 720},
    {twelve_minus_three_friendly_name, twelve_minus_three, 0.4, 48, 360},
    {thirty_six_minus_two_two_two_friendly_name, thirty_six_minus_two_two_two, 0.6, 72, 360},
    {thirty_six_minus_two_two_two_h6_friendly_name, thirty_six_minus_two_two_two_h6, 0.6, 72, 360},
    {thirty_six_minus_two_two_two_with_cam_friendly_name, thirty_six_minus_two_two_two_with_cam, 0.6, 144, 720},
    {fourty_two_hundred_wheel_friendly_name, fourty_two_hundred_wheel, 0.6, 72, 360},
    {thirty_six_minus_one_with_cam_fe3_friendly_name, thirty_six_minus_one_with_cam_fe3, 0.6, 144, 720},
    {six_g_seventy_two_with_cam_friendly_name, six_g_seventy_two_with_cam, 0.6, 144, 720},
    {buell_oddfire_cam_friendly_name, buell_oddfire_cam, 0.33333, 80, 720},
    {gm_ls1_crank_and_cam_friendly_name, gm_ls1_crank_and_cam, 6.0, 720, 720},
    {lotus_thirty_six_minus_one_one_one_one_friendly_name, lotus_thirty_six_minus_one_one_one_one, 0.6, 72, 360},
    {honda_rc51_with_cam_friendly_name, honda_rc51_with_cam, 0.2, 48, 720},
    {thirty_six_minus_one_with_second_trigger_friendly_name, thirty_six_minus_one_with_second_trigger, 0.6, 144, 720},
    {chrysler_ngc_thirty_six_plus_two_minus_two_with_ngc4_cam_friendly_name, chrysler_ngc_thirty_six_plus_two_minus_two_with_ngc4_cam, 3.0, 720, 720},
    {chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc6_cam_friendly_name, chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc6_cam, 3.0, 720, 720},
    {chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc8_cam_friendly_name, chrysler_ngc_thirty_six_minus_two_plus_two_with_ngc8_cam, 3.0, 720, 720},
    {weber_iaw_with_cam_friendly_name, weber_iaw_with_cam, 1.2, 144, 720},
    {fiat_one_point_eight_sixteen_valve_with_cam_friendly_name, fiat_one_point_eight_sixteen_valve_with_cam, 3.0, 720, 720},
    {three_sixty_nissan_cas_friendly_name, three_sixty_nissan_cas, 3.0, 720, 720},
    {twenty_four_minus_two_with_second_trigger_friendly_name, twenty_four_minus_two_with_second_trigger, 0.3, 72, 720},
    {yamaha_eight_tooth_with_cam_friendly_name, yamaha_eight_tooth_with_cam, 0.26667, 64, 720},
    {gm_four_tooth_with_cam_friendly_name, gm_four_tooth_with_cam, 0.06666, 8, 720},
    {gm_six_tooth_with_cam_friendly_name, gm_six_tooth_with_cam, 0.1, 12, 720},
    {gm_eight_tooth_with_cam_friendly_name, gm_eight_tooth_with_cam, 0.13333, 16, 720},
    {volvo_d12acd_with_cam_friendly_name, volvo_d12acd_with_cam, 4.0, 480, 720},
    {mazda_thirty_six_minus_two_two_two_with_six_tooth_cam_friendly_name, mazda_thirty_six_minus_two_two_two_with_six_tooth_cam, 1.5, 360, 720},
    {mitsubishi_4g63_4_2_friendly_name, mitsubishi_4g63_4_2, 0.6, 144, 720},
    {audi_135_with_cam_friendly_name, audi_135_with_cam, 1.5, 1080, 720},
    {honda_d17_no_cam_friendly_name, honda_d17_no_cam, 0.6, 144, 720},
    {mazda_323_au_friendly_name, mazda_323_au, 1, 30, 720},
    {daihatsu_3cyl_friendly_name, daihatsu_3cyl, 0.8, 144, 360},
    {miata_9905_friendly_name, miata_9905, 0.6, 144, 720},
    {twelve_with_cam_friendly_name, twelve_with_cam, 0.6, 144, 720},
    {twenty_four_with_cam_friendly_name, twenty_four_with_cam, 0.6, 144, 720},
    {subaru_six_seven_name_friendly_name, subaru_six_seven, 3.0, 720, 720},
    {gm_seven_x_friendly_name, gm_seven_x, 1.502, 180, 720},
    {four_twenty_a_friendly_name, four_twenty_a, 0.6, 144, 720},
    {ford_st170_friendly_name, ford_st170, 3.0, 720, 720},  // Updated from https://github.com/speeduino/Ardu-Stim/issues/14
    {mitsubishi_3A92_friendly_name, mitsubishi_3A92, 0.6, 144, 720},  // Updated from https://github.com/speeduino/Ardu-Stim/issues/14
    {Toyota_4AGE_CAS_friendly_name, toyota_4AGE_CAS, 0.333, 144, 720},
    {Toyota_4AGZE_friendly_name, toyota_4AGZE, 0.333, 144, 720},
    {Suzuki_DRZ400_friendly_name, suzuki_DRZ400, 0.6, 72, 360},
    {Jeep_2000_friendly_name, jeep_2000, 1.5, 360, 720},
    {rover_k_mode1_thirtysix_minus_one_one_friendly_name, rover_k_mode1_thirtysix_minus_one_one, 0.6, 72, 360},
    {rover_k_mode2_thirtysix_minus_one_one_one_one_friendly_name, rover_k_mode2_thirtysix_minus_one_one_one_one, 0.6, 72, 360},
    {rover_k_mode3_thirtysix_minus_one_one_one_one_friendly_name, rover_k_mode3_thirtysix_minus_one_one_one_one, 0.6, 72, 360},
    {rover_k_mode4_thirtysix_minus_one_one_one_one_friendly_name, rover_k_mode4_thirtysix_minus_one_one_one_one, 0.6, 72, 360},
};
