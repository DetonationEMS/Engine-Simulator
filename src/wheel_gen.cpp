
#include <Arduino.h>
#include "wheel_defs.h"
#include "wheel_gen.h"

/* Test code that will go into pattern selection functions  */
void arraySelectionCodeWontBeHere()
{
    // You can generate the array based on a selection parameter, e.g., from user input
    ArrayType selected_array_type = FourtyMinusOne; // Change this value based on user selection

    unsigned char generated_array[360]; // Allocate the maximum size needed for the largest array
    int array_size;

    generate_array(selected_array_type, generated_array, array_size);

    // Now you can use the generated array in your program
}

void generate_array(ArrayType type, unsigned char *arr, int &size)
{
    switch (type)
    {
    case FourtyMinusOne:
        size = 40;
        generate_fourty_minus_one(arr);
        break;
    case DizzyFourTriggerReturn:
        size = 9;
        generate_dizzy_four_trigger_return(arr);
        break;
    case OddfireVR:
        size = 24;
        generate_oddfire_vr(arr);
        break;
    case OptisparkLT1:
        size = 360;
        generate_optispark_lt1(arr);
        break;
    }
}

void generate_fourty_minus_one(unsigned char *arr)
{
    for (int i = 0; i < 39; i++)
    {
        arr[i] = i % 2;
    }
    arr[39] = 0;
}

void generate_dizzy_four_trigger_return(unsigned char *arr)
{
    for (int i = 0; i < 5; i++)
    {
        arr[i] = 0;
    }
    for (int i = 5; i < 9; i++)
    {
        arr[i] = 1;
    }
}

void generate_oddfire_vr(unsigned char *arr)
{
    arr[0] = 1;
    for (int i = 1; i < 9; i++)
    {
        arr[i] = 0;
    }
    arr[9] = 1;
    for (int i = 10; i < 24; i++)
    {
        arr[i] = 0;
    }
}

void generate_optispark_lt1(unsigned char *arr)
{
    for (int i = 0; i < 360; i++)
    {
        arr[i] = (i % 2) ? 1 : 0;
    }
    for (int i = 0; i < 8; i++)
    {
        arr[50 + i * 30] = 2;
        arr[50 + i * 30 + 1] = 3;
    }
}
