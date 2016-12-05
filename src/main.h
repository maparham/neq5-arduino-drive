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

struct mount_state_t {
	bool RA_isLeft = 0;bool DEC_isLeft = 0;
	float RA_speed = 0;
	float DEC_speed = 0;bool isInitialized() const {
		return RA_speed > 0;
	}
};

void setup();
void loop();

//Do not add code below this line
#endif /* _main_H_ */
