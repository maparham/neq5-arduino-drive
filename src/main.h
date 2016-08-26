// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _main_H_
#define _main_H_
#include "Arduino.h"
#include "Wire.h"
#include "AccelStepper.h"

#include "../src/MPU6050_DMP6.h"

#define L LOW
#define H  HIGH
#define LEFT -1
#define RIGHT 1
struct mount_state_t {
	bool RA_isLeft = 0;bool DEC_isLeft = 0;
	float RA_speed = 0;
	float DEC_speed = 0;bool isInitialized() {
		return RA_speed > 0;
	}
	;
};

size_t println(const String &s);
size_t println(double num, int digits);
size_t println(float num, int digits);
//size_t println(long num);
size_t println(float num);
//size_t print(long num);
size_t print(float num);
size_t print(double n, int digits);
size_t print(const String &s);
size_t println(const char c[]);
size_t print(const char c[]);

void printGravity(triple_t gravity);
void setup();
void loop();

//Do not add code below this line
#endif /* _main_H_ */
