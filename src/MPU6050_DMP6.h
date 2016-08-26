#ifndef MPU6050_DMP6_H
#define MPU6050_DMP6_H

struct triple_t {
	float x;
	float y;
	float z;
} ;


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
