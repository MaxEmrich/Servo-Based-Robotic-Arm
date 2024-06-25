#ifndef FUNCS
#define FUNCS

#include "defs.h"
#include <stdlib.h>
#include <stdio.h>

#include <assert.h>
#include <errno.h>
#include <string.h>

#include <math.h>
#include <Servo.h>

// Function declarations

struct Vector* newVec_FromPoint(struct R3Point* p1, struct R3Point* p2);    // make a new vector given two points in 3-space -- p1 is the tail, p2 is the head 

struct Vector* newVec_Comps(float x, float y, float z);    // make a new vector with COMPONENTS x, y, and z 

struct Vector* makeUnitVec(struct Vector* vector);

float getMag(struct Vector* vector); // returns the magnitude of the passed in vector

float getYComp(float mag, float angle); // get the y component of the vector (in an xy plane)

float getXComp(float mag, float angle); // get the y component of the vector (in an xy plane)

float getAngle(struct Vector* vector); // define later with magnitude of vector...

int rotate_servo(float num_degrees, Servo servo, float min_degrees = 0.0, float max_degrees = 180.0);

#endif // FUNCS