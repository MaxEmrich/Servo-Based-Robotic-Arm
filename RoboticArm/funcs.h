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

// Function defintions and implementations


// VECTOR CREATION  --------------------------------------------------------------------------- 
// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------

struct Vector* newVec_FromPoint(struct R3Point* p1, struct R3Point* p2) {    // make a new vector given two points in 3-space -- p1 is the tail, p2 is the head 
  struct Vector* newVector = malloc(sizeof(struct Vector));

  // to make component vector, we use p2 - p1 = new vector

  float x_comp = (*p2).point_3d[0] - (*p1).point_3d[0];
  float y_comp = (*p2).point_3d[1] - (*p1).point_3d[1];
  float z_comp = (*p2).point_3d[2] - (*p1).point_3d[2];

  (*newVector).vectorComponents[0] = x_comp;
  (*newVector).vectorComponents[1] = y_comp;
  (*newVector).vectorComponents[2] = z_comp;

  // error checking....
  assert((*newVector).tailPoint.point_3d[0] == 0.0);
  assert((*newVector).tailPoint.point_3d[1] == 0.0);
  assert((*newVector).tailPoint.point_3d[2] == 0.0);
    
  return newVector;
}

struct Vector* newVec_Comps(float x, float y, float z) {    // make a new vector with COMPONENTS x, y, and z 
  struct Vector* newVector = malloc(sizeof(struct Vector));
  (*newVector).vectorComponents[0] = x;
  (*newVector).vectorComponents[1] = y;
  (*newVector).vectorComponents[2] = z;

  // establish head and tail R3 points for checking if vector is in component form...
  (*newVector).headPoint.point_3d[0] = x;
  (*newVector).headPoint.point_3d[1] = y;
  (*newVector).headPoint.point_3d[2] = z;
  
  (*newVector).tailPoint.point_3d[0] = 0.0; // set tail point as origin 
  (*newVector).tailPoint.point_3d[1] = 0.0;
  (*newVector).tailPoint.point_3d[2] = 0.0;
  // ------------------------------------------------

  return newVector;
}

// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------



// OPERATIONS ON VECTORS ----------------------------------------------------------------------
// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------

float getMag(struct Vector* vector) { // returns the magnitude of the passed in vector
  float x_val = (*vector).vectorComponents[0];
  float y_val = (*vector).vectorComponents[1];
  float z_val = (*vector).vectorComponents[2];
  float sumComponents = (x_val*x_val) + (y_val*y_val) + (z_val*z_val);
  float mag = sqrt(sumComponents);
  return mag;
}

struct Vector* makeUnitVec(struct Vector* vector) {

  struct Vector* unitVector = malloc(sizeof(struct Vector)); 
  float vectorMag = getMag(vector);

  for (int i = 0; i < 3; i++) {
    (*unitVector).vectorComponents[i] = ((*vector).vectorComponents[i])/(vectorMag); // unit vector formula is sqrt(original vector/magnitude of original vector)
  } 
  
  // error checking...

  float finalMag = getMag(vector);

  assert(finalMag == 1.0); // state that we assume the magnitude to be 1.0, if this changes, then there has been a logical error
  if (1.0 < finalMag || 1.0 > finalMag) {
    Serial.println("Error calculating magnitude...");
    exit(-1);
  }

  // -------
   
  return unitVector;
}  

float getYComp(float mag, float angle) { // get the y component of the vector (in an xy plane)
  return mag * sin(angle * (M_PI/180.0)); // accepts radians, so we need to convert the angle to radians 
}

float getXComp(float mag, float angle) { // get the y component of the vector (in an xy plane)
  return mag * cos(angle * (M_PI/180.0));
}

// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------


// SERVO MOVEMENTS ----------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------

int rotate_servo(float num_degrees, Servo servo, float min_degrees = 0.0, float max_degrees = 180.0) {
  float current_angle = servo.read();
  float desired_rotation = constrain(num_degrees, min_degrees, max_degrees); // default of min and max are the full rotation constraints of the servo, 0 to 180 degrees 
  float num_degrees_to_rotate = desired_rotation - current_angle; // how many degrees we are rotating from current angle

  if (num_degrees_to_rotate > 0.0) { // if num_degrees_to_rotate is positive, we rotate in a positive direction
      for (int i = current_angle; i < num_degrees_to_rotate; i++) { // for smooth rotation
      servo.write(i);
      delay(10); // smoothing rotation
    }
  } else if (num_degrees_to_rotate < 0.0) { // if num_degrees_to_rotate is negative, we rotate in a negative direction
      for (int i = current_angle; i > num_degrees_to_rotate; i--) {
      servo.write(i);
      delay(10);
    }
  }

  return 0;
}


#endif // FUNCS