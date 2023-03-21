#ifndef WHEEL_GEN_H
#define WHEEL_GEN_H
#include <arduino.h>

enum ArrayType {
  FourtyMinusOne,
  DizzyFourTriggerReturn,
  OddfireVR,
  OptisparkLT1
};

// Pattern to be generated (testing)
void generate_fourty_minus_one(unsigned char *arr);
void generate_dizzy_four_trigger_return(unsigned char *arr);
void generate_oddfire_vr(unsigned char *arr);
void generate_optispark_lt1(unsigned char *arr);


// Generation
void generate_array(ArrayType type, unsigned char* arr, int& size);

#endif