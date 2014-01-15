// BeagleMIP Balance - James Strawson 2013

#include <unistd.h>
#include <time.h>	//usleep, nanosleep
#include <stdio.h>
#include <stdlib.h>
#include <math.h>	//atan2 and fabs
#include <signal.h>	//capture ctrl-c
#include <pthread.h>   // multi-threading

#include "../inc/robotics_cape.h"
#include "../inc/c_i2c.h"
#include "../inc/MPU6050.h"	

#define DT 0.005       	//5ms loop (200hz)
#define WHEEL_RADIUS 0.035  // meters
#define TRACK_WIDTH 0.1 	//meters, width between contact patches
#define PI 3.14159265358

#define LEAN_THRESHOLD 0.8  //radians lean before killing motors
#define THETA_REF_MAX 0.6	// Maximum reference theta set point for inner loop
#define START_THRESHOLD 0.15 // how close to vertical before it will start balancing while self uprighting
#define LOW_BATT_VOLTAGE 7.0	//lowest battery voltage before it should be plugged in

int runTrue = 1;	//setting to 0 exists the program
int balancing = 0;	//setting to 1 starts the 200hz controller
int uprighting = 0;
int logging = 0;

// complementary high and low pass filter constants, plus integrator trim
#define THETA_MIX_TC  3   // t_seconds timeconstant on filter
const float HP_CONST = THETA_MIX_TC/(THETA_MIX_TC + DT);
const float LP_CONST = DT/(THETA_MIX_TC + DT);
float thetaTrim = 0;

// i2c declarations
static i2c_t MPU6050;
static i2c_t* pi2c = &MPU6050;

// Estimator variables
float xAccel, zAccel, yGyro, yGyroOffset, accLP, gyroHP, theta;

//enocder Variables
long int encoderCountsL, encoderCountsR;
long int encoderOffsetL, encoderOffsetR;

///Controller & State Variables ///
float prescaler = 0.7;
float theta, phi, phiRef; //system state
float oldTheta;

float eTheta[3];
float ePhi[2];
float u[3];
float thetaRef[2];

//Steering controller
float Gamma;
float gammaRef, torqueSplit;
float kTurn = 1.0;
float dutyLeft, dutyRight;

// Control constants
float numD1[] = {-6.0977, 11.6581, -5.5721};
float denD1[] = {1.0000,   -1.6663,    0.6663};
float numD2[] = {0.0987,   -0.0985};
float denD2[] = {1.0000,   -0.9719};
float kTrim = -0.1;  // outer loop integrator constant
float kInner = 1.8; //
float kOuter = 1.6; //

//datalogging stuff
#define N_DATAPOINTS 1024
#define LOGGED_VARIABLES 8
float data[LOGGED_VARIABLES][N_DATAPOINTS];
float path[N_DATAPOINTS];
int logcounter = 0;

// struct neccesary for the nanosleep function
typedef struct timespec	timespec;
timespec t1, t2, t2minust1, sleepRequest, deltaT;



//////////////////////////
// Function Definitions //
//////////////////////////

// gives the difference between two timespecs
// used to calcualte how long to sleep for
timespec diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000L+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

// start I2C communication with MPU-9150/6050
void i2cStart(){
	//printf("MPU6050 test\n\n");
	pi2c->bus = 1;
	i2c_init(pi2c, pi2c->bus, 0x68);
	openConnection(pi2c);
	bool mpu6050TestResult = MPU_testConnection(pi2c, pi2c->buffer);
	if(mpu6050TestResult) {
		printf("MPU6050 test passed \n");
	} else {
		printf("MPU6050 test failed \n");
	}
}

//returns theta after a fresh configuration and warmup
float initializeEstimator(){
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int i = 0;
	float xAccelCount, zAccelCount, yGyroCount;
	uint8_t buffer[14];
	
	MPU_setSleepEnabled(false, pi2c);  			  	// Wake up from sleep
	MPU_setClockSource(MPU6050_CLOCK_PLL_XGYRO, pi2c);		// setup MPU6050	
	MPU_setDLPFMode(0, pi2c); 					// as little filtering as possible	
	MPU_setFullScaleGyroRange(MPU6050_GYRO_FS_1000, pi2c); 	// GYRO_FS_1000  +-1000 deg/s range	
	MPU_setFullScaleAccelRange(MPU6050_ACCEL_FS_2, pi2c);	// MPU6050_ACCEL_FS_2    +- 2g range

	usleep(10000); // let the gyro settle

	// warm up loop, sample data 100 times
	for (i = 0; i < 100; i++){ 
		MPU_getMotion6(&ax, &ay, &az, &gx, &gy, &gz, pi2c, buffer); 
		xAccelCount += (float)ax; 
		zAccelCount += (float)az;
		yGyroCount += (float)gy;
		usleep(5000);
		//printf("gy: %f  ax: %f  az: %f\n", (float)gy, (float)ax, (float)ay);            
	}
		
	yGyroOffset = yGyroCount/100;		//offset to correct for steady-state gyro error
    	accLP = -atan2(zAccelCount, -xAccelCount); 		//initialize accLP so filter begins at correct theta
	theta=accLP;
	printf("yGyroOffset = %f\n", yGyroOffset);
	printf("Theta = %f\n", theta);
	return accLP;
}


//////////////////////////////////////////
// Complementary Filter		   //
// Returns the latest value for theta.  //
// You must call this function at 200hz //
//////////////////////////////////////////

float Complementary_Filter(){
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	MPU_getMotion6(&ax, &ay, &az, &gx, &gy, &gz, pi2c, pi2c->buffer); 
      
	// Apply preScale for +-2g range
	xAccel = (float)ax*2/32768;  
	zAccel = (float)az*2/32768; 

	//subtract the steady-state gyro offset error
	//then multiply by the gyro prescaler
	yGyro = (gy-yGyroOffset)*1000*2*PI/(360*32768);            

	//first order filters
	accLP = accLP + LP_CONST * (-atan2(zAccel,-xAccel) - accLP);
	gyroHP = HP_CONST*(gyroHP + .005*yGyro);

	// diagnostic print to console
	//printf("gyroHP: %f accelLP: %f theta: %f Phi: %f\n", gyroHP, accLP, gyroHP+accLP, phi); 

	return (gyroHP+accLP)+thetaTrim;
}


/////////////////////////////////////////
//Cleanup function captured on ctrl-C ///
/////////////////////////////////////////
void cleanup(int signo){
	if (signo == SIGINT){
		//MPU_setSleepEnabled(true, pi2c); 
		runTrue = 0;
		balancing = 0;
 	}
}


///////////////////////////////////////////////////////
// 10hz Loop checking battery, buttons, and tipover ///
///////////////////////////////////////////////////////
void* slow_loop_func(void* ptr){
	int i=0;// counter for timing button presses
	do{
		
		/////////////////////////////////////////
		/// check battery voltage and buttons ///
		/////////////////////////////////////////

		if(getBattVoltage()<LOW_BATT_VOLTAGE){
		
			printf("\n Battery voltage too low, please plug in charger\n");
			setMotors(0,0,0);
			balancing = 0; //stop the controller
			while(getBattVoltage()<LOW_BATT_VOLTAGE){
				setRED(1);
				usleep(100000);	//wait for the battery to be charged or plugged in
				setRED(0);
				usleep(100000);
			}			
			balancing = 1; //battery charged, back to balancing.
			printf("Battery Voltage OK, back to normal operation\n");
		}	

		// detect inputs while balancing
		if (balancing == 1){ 
	
			//watch for a pause button press
			// if(getStartBTN()==1){
				// FILE *fp = fopen("path.txt", "r"); //open path file for reading
					// if (fp == NULL){
						// printf("path file does not exist\n");
					// }
					// else{
						// int i;
						// for(i=0; i<N_DATAPOINTS;  i++){
							// fscanf(fp, "%f ", &path[i]);
							//printf("%f\n", path[i]);
						// }
						// fclose(fp);
						// printf("Path File Loaded\n");
					// }

				// logging = 1;
				// printf("Logging\n");
			// }
				
			// detect a tip-over
			if(fabs(theta)>LEAN_THRESHOLD){
				balancing=0;
				setMotors(0,0,0);
				printf("We have Tipped! Hold upright to balance.\n");
				setRED(1);
				setGRN(0);
			}	

			
		}
		
		//show if the select button has been pressed
		// holding for 3 seconds exits the program cleanly
		i=0;
		if(getSelectBTN()==1){
				uprighting = 1;
				usleep(500000);
				if(getSelectBTN()==0){				
					int sign;
					if(theta<0){
						sign=1;
					}
					else{
						sign=-1;
					}
					setMotors(sign,sign,0);
					usleep(250000);
					setMotors(-sign, -sign,0);
				}
				usleep(250000);
				uprighting = 0;
				if(balancing==0){
					setMotors(0,0,0);
				}
			
			printf("Select Button Pressed\nHold for 3 seconds to exit\n");
			while(getSelectBTN()==1 && i<=20){
				usleep(100000); // sleep 0.1 seconds
				i++;
				if(i>20){
					runTrue = 0; //close all threads
				}
			}
		}	

		//printf("theta: %f thetaTrim: %f thetaRef[0]: %f phi: %f\n", theta, thetaTrim, thetaRef[0], phi);
		//printf("theta: %f thetaTrim: %f thetaRef[0]: %f \n", theta, thetaTrim, thetaRef[0]);

		
		usleep(100000); //check buttons at roughly 10 hz,not very accurate)
	}while(runTrue == 1);
	return NULL;
}

//////////////////////////////////////////////////////////////////////////
/// running 200hz discrete estimator and controller in asychronous loop //
//////////////////////////////////////////////////////////////////////////
void* control_loop_func(void* ptr){
 	do{
		clock_gettime(CLOCK_MONOTONIC, &t1);  //record the time at the beginning.
		
		//////////////////////////////////
		//// Control Code Begins Here ////
		//////////////////////////////////

		//Average right(0) and left(1) enocders. 
		// encoder 1 is reversed to make both forward>positive
		encoderCountsL = -(getEncoder1()-encoderOffsetL);
		encoderCountsR = getEncoder0()-encoderOffsetR;
		phi = (encoderCountsL+encoderCountsR)*PI/352; //convert to radians, 352 ticks per revolution
		Gamma = WHEEL_RADIUS*2*(encoderCountsL-encoderCountsR)*PI/(352*TRACK_WIDTH);


		oldTheta=theta;
		theta = Complementary_Filter();
		//printf("%f\n", theta);
		if(balancing==0 && oldTheta*theta<0){ //check if it has crossed the equilibrium point while paused
			balancing = 1;
			setGRN(1);
			setRED(0);
			printf("Balancing\n");
		}
		
		if(uprighting==1){
			if(fabs(theta) < START_THRESHOLD){
				balancing = 1;
				uprighting = 0;
			}
		}			

		//printf("%d\n",balancing);	
		if(balancing && fabs(theta)<LEAN_THRESHOLD){	
		
			// step difference equation forward
			ePhi[1]=ePhi[0];
			thetaRef[1]=thetaRef[0];
			eTheta[2]=eTheta[1]; eTheta[1]=eTheta[0];
			u[2]=u[1]; u[1]=u[0];
  
			ePhi[0] = phiRef-phi;
			thetaRef[0] = kOuter*(numD2[0]*ePhi[0] + numD2[1]*ePhi[1]) - denD2[1]*thetaRef[1];
			
			//integrate the reference theta to correct for imbalance or sensor error
			thetaTrim += kTrim * thetaRef[0]*DT;

			if(thetaRef[0]>THETA_REF_MAX){thetaRef[0]=THETA_REF_MAX;}
			else if(thetaRef[0]<-THETA_REF_MAX){thetaRef[0]=-THETA_REF_MAX;}
		
			// if logging data, follow the prescribed reference path
			if(logging == 1){
				thetaRef[0] = path[logcounter]; 
			}

			eTheta[0] = (prescaler * thetaRef[0]) - theta;
			u[0] = kInner*(numD1[0]*eTheta[0]+numD1[1]*eTheta[1] + numD1[2]*eTheta[2]) - denD1[1]*u[1] - denD1[2]*u[2]; 
			
			if(u[0]>1){
				u[0]=1;
			}
			else if(u[0]<-1){
				u[0]=-1;
			}
   
			torqueSplit = kTurn*(gammaRef - Gamma);
			
			
			dutyLeft = u[0]+torqueSplit;
			dutyRight = u[0]-torqueSplit;			
			setMotors(dutyRight,dutyLeft, 0);
			
			// if(logging){
				// data[0][logcounter] = DT * logcounter; //time
				// data[1][logcounter] = u[0];
				// data[2][logcounter] = theta;
				// data[3][logcounter] = phi;
				// data[4][logcounter] = xAccel;
				// data[5][logcounter] = zAccel;
				// data[6][logcounter] = yGyro;
				// data[7][logcounter] = thetaRef[0];
				
				// logcounter ++;
				// if(logcounter >= N_DATAPOINTS){
					// printf("data collected, writing to file\n");

					// FILE *fp = fopen("data.txt", "w+"); //create file if it does not exist
					// if (fp == NULL){
						// printf("data.txt does not exist\n");
					// }
					// else{
						// // fprintf(fp, "time u theta phi xAccel ZAccel yGyro thetaRef\n\n");
						// int i,j;
						// for(i=0; i<N_DATAPOINTS;  i++){
							// for(j=0; j<LOGGED_VARIABLES; j++){
								// fprintf(fp, "%f ", data[j][i]);
							// }
							// fprintf(fp,"\n");
						// }
						// fclose(fp);
					// }
					// logging = 0;
					// ePhi[1]=0; ePhi[0]=0;
					// thetaRef[1]=0; thetaRef[0]=0;

				// }
				
			// }
		}

		//if not balancing, make sure the controller is reset and motors are off
		else{	
			encoderOffsetR = getEncoder0();
			encoderOffsetL = getEncoder1();
			ePhi[1]=0; ePhi[0]=0;
			thetaRef[1]=0; thetaRef[0]=0;
			eTheta[2]=0; eTheta[1]=0; eTheta[0]=0;
			u[2]=0; u[1]=0; u[0]=0;
		}

		////////////////////////////////
		//// Control Code ends Here ////
		////////////////////////////////

		//Sleep for the necessary time to maintain 200hz
		clock_gettime(CLOCK_MONOTONIC, &t2);
		t2minust1 = diff(t1, t2);
		sleepRequest = diff(t2minust1, deltaT);
		nanosleep(&sleepRequest, NULL);
	}while(runTrue == 1);
	return NULL;
}


//////////////////////////////////////////////////////////////////
/// Main function used for initializtion and thread management ///
//////////////////////////////////////////////////////////////////

int main(){
	
	printf("initializing\n");
	balancing=0;
	runTrue=1;
	
	deltaT.tv_sec = 0;		// create time struct for 5ms (200hz)
	deltaT.tv_nsec = 5000000;
	signal(SIGINT, cleanup);	//signal catcher calls cleanup function on exit
	
	/////////////////////////////////////////////////////////////
	/// Initialize Cape hardware, estimators, and controllers ///
	/////////////////////////////////////////////////////////////
	i2cStart();
	initializeCape();
	setMotors(0,0,0);
	initializeEstimator();
	setRED(1);
	setGRN(0);
	printf("\nHold your MIP upright to begin balancing\n");

	//////////////////////////////////////////
	/// Begin the control and slow threads ///
	//////////////////////////////////////////
	pthread_t control_thread, slow_thread;
	pthread_create(&control_thread, NULL, control_loop_func, (void*) NULL);
	pthread_create(&slow_thread, NULL, slow_loop_func, (void*) NULL);
	//pthread_join(control_thread, NULL);
	//pthread_join(slow_thread, NULL);

	while(runTrue==1){
	sleep(1);
	}

	//nothing get executed here until runTrue == 0 which terminates the threads
	//usleep(100000); //let other processes clean up
	cleanupCape();
	closeConnection(pi2c);
	printf("Exiting Cleanly\n");
	return 0;
}