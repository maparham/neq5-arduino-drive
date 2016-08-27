#ifndef MPU6050_DMP6_H
#define MPU6050_DMP6_H

struct triple_t {
	float x;
	float y;
	float z;
	bool leftRA; // will RA tilt left for the target? it must be given by client.
	bool RA_sameTilt(bool RA_isLeft) {
		return leftRA == RA_isLeft; // is the target tilt the same as current RA tilt?
	}
};

void enableMPU();
void disableMPU();
triple_t getGravity();
void getGravity(triple_t* g);
void setCurrentGravity(triple_t* gr, bool fixedTime);
void setCurrentGravity(triple_t* gr);

void setupMPU();
bool loopMPU(bool verbose);

bool getMPULock();

void releaseMPULock();

#endif
