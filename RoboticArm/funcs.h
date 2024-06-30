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

struct Vector* newVec_FromPoints(struct R3Point* p1, struct R3Point* p2);  // make a new vector given two points in 3-space -- p1 will be the tail, p2 will be the head 

struct Vector* newVec_Comps(float x, float y, float z);  // make a new vector with COMPONENTS x, y, and z 

struct Vector* makeUnitVec(struct Vector* vector); // makes a unit vector from an existing vector

void scaleVec(struct Vector* vector, float scale_val); // scale an existing vector, takes a vector and a value to scale that vector by (use on unit vectors to get a reasonable effect)

float getMag(struct Vector* vector); // returns the magnitude of the passed in vector

float getYComp(float mag, float angle); // get the y component of the vector (in an xy plane)

float getXComp(float mag, float angle); // get the y component of the vector (in an xy plane)

float getAngle(struct Vector* vector); // define later with magnitude of vector...

int rotate_servo(float num_degrees, Servo servo, float min_degrees = 0.0, float max_degrees = 180.0); // rotates a servo smoothly, optional params for max and min degrees

#endif // FUNCS