#ifndef HELPERS_CPP
#define HELPERS_CPP
#include "Arduino.h"
#include "MPU6050_DMP6.h"
#define L LOW
#define H  HIGH
#define LEFT -1
#define RIGHT 1

size_t println(const String &s);
size_t println(double num, int digits);
size_t println(float num, int digits);
size_t println(float num);
size_t print(float num);
size_t print(double n, int digits);
size_t print(const String &s);
size_t println(const char c[]);
size_t print(const char c[]);
void printGravity(triple_t gravity);
void printGravity(triple_t gravity, String msg);

#endif
