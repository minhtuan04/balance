#ifndef _BBB_GPIO_H_
#define _BBB_GPIO_H_
int init_gpio_module(void);
void c_gpio_cleanup();
int c_setup_channel(char key[8], int direction, int pull_up_down);
int c_output_gpio(char key[8], int value);
unsigned int c_input_gpio(char key[8]);
#endif
