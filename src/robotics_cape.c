#include <unistd.h>
#include <stdio.h>
#include "../inc/common.h"
#include "../inc/bbb_gpio.h"
#include "../inc/robotics_cape.h"

int initialized = 0;

// BLFNAR pins
#define GRN "P8_15"
#define RED "P8_16"
#define START_BTN  "P8_8"
#define SELECT_BTN "P8_9"

//Motor Values
#define MDIR1A    "P9_41"
#define MDIR1B    "P9_30"
#define MDIR3A    "P9_23"
#define MDIR3B    "P9_15"
#define MDIR4A    "P8_18"
#define MDIR4B    "P8_17"
#define PERIOD_NS    500000  //nanoseconds: 20khz
#define ENCODER_STEPS 300	// 6 steps per revolution times 50:1 gearbox


FILE *pwm0A_polarity;
FILE *pwm0A_period;
FILE *pwm0A_duty;
FILE *pwm1A_polarity;
FILE *pwm1A_period;
FILE *pwm1A_duty;
FILE *pwm1B_polarity;
FILE *pwm1B_period;
FILE *pwm1B_duty;

FILE *eqep0;
FILE *eqep1;
FILE *eqep2;

FILE *AIN6_file;
int millivolts;	

int initializeCape(){

	c_gpio_cleanup();
    	init_gpio_module();

	// c_setup_channel(channel, direction INPUT/OUTPUT, pull_up_down=PUD_OFF)
 	 c_setup_channel(GRN, OUTPUT, PUD_OFF);
 	 c_setup_channel(RED, OUTPUT, PUD_OFF);
	 c_setup_channel(START_BTN, INPUT, PUD_OFF);
 	 c_setup_channel(SELECT_BTN, INPUT, PUD_OFF);

    	printf("BLFNAR Initialized\n");

	c_setup_channel(MDIR1A, OUTPUT, PUD_OFF); 
	c_setup_channel(MDIR1B, OUTPUT, PUD_OFF); 
	c_setup_channel(MDIR3A, OUTPUT, PUD_OFF); 
	c_setup_channel(MDIR3B, OUTPUT, PUD_OFF); 
       c_setup_channel(MDIR4A, OUTPUT, PUD_OFF); 
	c_setup_channel(MDIR4B, OUTPUT, PUD_OFF); 
	c_output_gpio(MDIR1A,LOW);
       c_output_gpio(MDIR1B, LOW);
       c_output_gpio(MDIR3A, LOW);
       c_output_gpio(MDIR3B,LOW);
       c_output_gpio(MDIR4A, LOW);
       c_output_gpio(MDIR4B, LOW);

	pwm0A_polarity = fopen("/sys/devices/ocp.3/pwm_test_P9_31.12/polarity", "a");
	pwm0A_period   = fopen("/sys/devices/ocp.3/pwm_test_P9_31.12/period", "a");
	pwm0A_duty	= fopen("/sys/devices/ocp.3/pwm_test_P9_31.12/duty", "a");
	pwm1A_polarity = fopen("/sys/devices/ocp.3/pwm_test_P9_14.14/polarity", "a");
	pwm1A_period   = fopen("/sys/devices/ocp.3/pwm_test_P9_14.14/period", "a");
	pwm1A_duty	= fopen("/sys/devices/ocp.3/pwm_test_P9_14.14/duty", "a");
	pwm1B_polarity = fopen("/sys/devices/ocp.3/pwm_test_P9_16.13/polarity", "a");
	pwm1B_period   = fopen("/sys/devices/ocp.3/pwm_test_P9_16.13/period", "a");
	pwm1B_duty	= fopen("/sys/devices/ocp.3/pwm_test_P9_16.13/duty", "a");

	
	fprintf(pwm0A_polarity, "%c", '0');	fflush(pwm0A_polarity);
	fprintf(pwm0A_period, "%d", PERIOD_NS);	fflush(pwm0A_period);
	fprintf(pwm1A_polarity, "%c", '0');	fflush(pwm1A_polarity);
	fprintf(pwm1A_period, "%d", PERIOD_NS);	fflush(pwm1A_period);
	fprintf(pwm1B_polarity, "%c", '0');	fflush(pwm1B_polarity);
	fprintf(pwm1B_period, "%d", PERIOD_NS);	fflush(pwm1B_period);
		
	fprintf(pwm0A_duty, "%d", 0);		fflush(pwm0A_duty);
	fprintf(pwm1A_duty, "%d", 0);		fflush(pwm1A_duty);
	fprintf(pwm1B_duty, "%d", 0);		fflush(pwm1B_duty);
       printf("Motors Initialized\n");
	
	
	printf("eQep Encoders Initialized\n");

	float volts = getBattVoltage();
	printf("Battery Voltage = %fV\n", volts);
	initialized = 1;
	return 0;
}

int setMotors(float duty0A, float duty1A, float duty1B){

	if(initialized == 0){initializeCape();}

	if (duty0A>0){
		c_output_gpio(MDIR1A, LOW);
      		c_output_gpio(MDIR1B, HIGH);
	}
	else{
		
		c_output_gpio(MDIR1A, HIGH);
     		c_output_gpio(MDIR1B, LOW);
		duty0A = -duty0A;
	}

	if (duty1A>0){
	 	c_output_gpio(MDIR3A, HIGH);
     	 	c_output_gpio(MDIR3B, LOW);
	}
	else{
		c_output_gpio(MDIR3A, LOW);
      		c_output_gpio(MDIR3B, HIGH);
		duty1A = -duty1A;
	}
	
	if (duty1B>0){
		 c_output_gpio(MDIR4A, LOW);
     		 c_output_gpio(MDIR4B, HIGH);
	}
	else{
		c_output_gpio(MDIR4A, HIGH);
      		c_output_gpio(MDIR4B, LOW);
		duty1B = -duty1B;
	}


	if(duty0A>1){duty0A=1;}
	if(duty1A>1){duty1A=1;}
	if(duty1B>1){duty1B=1;}


	fprintf(pwm0A_duty, "%d", (int)(duty0A*PERIOD_NS));		fflush(pwm0A_duty);
	fprintf(pwm1A_duty, "%d", (int)(duty1A*PERIOD_NS));		fflush(pwm1A_duty);
	fprintf(pwm1B_duty, "%d", (int)(duty1B*PERIOD_NS));		fflush(pwm1B_duty);

	return 0;
}

int testMotors(){
	initializeCape();

	setMotors(.3,.3,.3);
	sleep(1);

	setMotors(-.3,-.3,-.3);
	sleep(1);
	
	cleanupCape();
	
	return 0;

}



long int getEncoder0(){
	long int j;
	eqep0 = fopen("/sys/devices/ocp.3/48300000.epwmss/48300180.eqep/position", "r");
	fscanf(eqep0, "%ld", &j);
	fclose(eqep0);
	return j;
}

long int getEncoder1(){
	long int j;
	eqep1 = fopen("/sys/devices/ocp.3/48302000.epwmss/48302180.eqep/position", "r");
	fscanf(eqep1, "%ld", &j);
	fclose(eqep1);
	return j;
}

long int getEncoder2(){
	long int j;
	eqep2 = fopen("/sys/devices/ocp.3/48304000.epwmss/48304180.eqep/position", "r");
	fscanf(eqep2, "%ld", &j);
	fclose(eqep2);
	return j;
}

/*  writing to encoder regster throws a fault if the driver is not loaded
int resetEncoder0(){
	eqep0 = fopen("/sys/devices/ocp.3/48300000.epwmss/48300180.eqep/position", "w");
	fprintf(eqep0, "%d", 0);
	fclose(eqep0);
	return 0;
}

int resetEncoder1(){
	eqep1 = fopen("/sys/devices/ocp.3/48302000.epwmss/48302180.eqep/position", "w");
	fprintf(eqep1, "%d", 0);
	fclose(eqep1);
	return 0;
}

int resetEncoder2(){
	eqep2 = fopen("/sys/devices/ocp.3/48304000.epwmss/48304180.eqep/position", "w");
	fprintf(eqep2, "%d", 0);
	fclose(eqep2);
	return 0;
}
*/


int getStartBTN(){
	return c_input_gpio(START_BTN);
}

int getSelectBTN(){
	return c_input_gpio(SELECT_BTN);
}

int setGRN(int i){
	return c_output_gpio(GRN, i);

}

int setRED(int i){
	return c_output_gpio(RED, i);
}

float getBattVoltage(){

	AIN6_file = fopen("/sys/devices/ocp.3/helper.15/AIN6", "r");
	fscanf(AIN6_file, "%i", &millivolts);
	fclose(AIN6_file);
	return (float)millivolts*11.0/1000.0; // time 11 for the voltage divider, divide by 1000 to go from mv to V
}

int cleanupCape(){

	setMotors(0,0,0);

	printf("\nClosing GPIO\n");
	c_output_gpio(MDIR1A, LOW);
	c_output_gpio(MDIR1B, LOW);
	c_output_gpio(MDIR3A, LOW);
	c_output_gpio(MDIR3B, LOW);
	c_output_gpio(MDIR4A, LOW);
	c_output_gpio(MDIR4B, LOW);
	
	setGRN(0);
	setRED(0);	

	printf("Closing PWM\n");
	fclose(pwm0A_polarity);
	fclose(pwm0A_period);
	fclose(pwm0A_duty);
	fclose(pwm1A_polarity);
	fclose(pwm1A_period);
	fclose(pwm1A_duty);
	fclose(pwm1B_polarity);
	fclose(pwm1B_period);
	fclose(pwm1B_duty);

/*	
	printf("Closing eQEP\n");
	fclose(eqep0);
	fclose(eqep1);
	fclose(eqep2);
*/	

	return 0;
}

