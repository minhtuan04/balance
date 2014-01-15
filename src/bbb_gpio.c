/*
Copyright (c) 2013 Adafruit

Original RPi.GPIO Author Ben Croston
Modified for BBIO Author Justin Cooper

This file incorporates work covered by the following copyright and 
permission notice, all modified code adopts the original license:

Copyright (c) 2013 Ben Croston

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "../inc/common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>       // for string cmp
#include "../inc/c_gpio.h"
 
// common.h
int init_gpio_module(void)
{
    int i;

   module_setup = 0;

    for (i=0; i<120; i++)
        gpio_direction[i] = -1;

    return 0;
}

// python function cleanup()
void c_gpio_cleanup()
{
    // clean up any /sys/class exports
    event_cleanup();

    // unexport the GPIO
    exports_cleanup();

}

// python function setup(channel, direction, pull_up_down=PUD_OFF, initial=None)
int c_setup_channel(char key[8], int direction, int pull_up_down)
{
   unsigned int gpio;
   int pud = PUD_OFF;
   pud = pull_up_down;

   if (direction != INPUT && direction != OUTPUT)
   {
      printf("An invalid direction was passed to setup()\n");
      return 0;
   }

   if (direction == OUTPUT)
      pud = PUD_OFF;

   if (pud != PUD_OFF && pud != PUD_DOWN && pud != PUD_UP)
   {
      printf("Invalid value for pull_up_down - should be either PUD_OFF, PUD_UP or PUD_DOWN\n");
      //return NULL;
      return 0;
   }
   if (get_gpio_number(key, &gpio))

       //return NULL;
       return 0;

   gpio_export(gpio);
   gpio_set_direction(gpio, direction);
   gpio_set_value(gpio, pud);

   gpio_direction[gpio] = direction;

 //  c_RETURN_NONE;
   return 0;
}

// python function output(channel, value)
// channel -> key name
int c_output_gpio(char key[8], int value)
{
    unsigned int gpio;

    if (get_gpio_number(key, &gpio))
        return 0;      

    if (gpio_direction[gpio] != OUTPUT)
    {
        printf("The GPIO channel has not been set up as an OUTPUT");
        return 0;
    }

    gpio_set_value(gpio, value);

    return 0;
}

// python function value = input(channel)
unsigned int c_input_gpio(char key[8])
{
    unsigned int gpio;
    unsigned int value;

    if (get_gpio_number(key, &gpio))
        return 0;

   // check channel is set up as an input or output
    if (gpio_direction[gpio] != INPUT && gpio_direction[gpio] != OUTPUT)
    {
        printf("You must setup() the GPIO channel first");
        return 0;
    }

    gpio_get_value(gpio, &value);

    return value;
}

