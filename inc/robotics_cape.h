int initializeCape();

int setMotors(float duty1, float duty3, float duty4);
int testMotors();


long int getEncoder0();
long int getEncoder1();
long int getEncoder2();

/*
int	resetEncoder0();
int	resetEncoder1();
int	resetEncoder2();
*/

int getStartBTN();
int getSelectBTN();
int setGRN();
int setRED();

float getBattVoltage();

int cleanupCape();