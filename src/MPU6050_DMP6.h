#ifndef MPU6050_DMP6_H
#define MPU6050_DMP6_H

struct triple_t {
	float x;
	float y;
	float z;
	float ya;
	float p;
	float r;

	bool RA_isLeft; // the title of RA is determined by user (because the x,y,z alone won't identify a unique position)
};

void enableMPU();
void disableMPU();
triple_t getGravity();
void getGravity(triple_t& g);
void setCurrentGravity(triple_t &myTriple, bool fixedTime);
void setCurrentGravity(triple_t &myTriple);
void setYPR(triple_t &myTriple, bool fixedTime);
void setYPR(triple_t &myTriple);

void setupMPU();
bool loopMPU(bool verbose);

bool getMPULock();

void releaseMPULock();

#endif
