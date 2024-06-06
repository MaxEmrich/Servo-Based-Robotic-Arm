// #include <Servo.h>
#include <stdlib.h>
#include <stdio.h>

// FABRIK == Forwards and Backwards Reaching Inverse Kinematics:
/*

(Vector library may be needed. Need cross product, 
dot product, vector subtraction and addition)

- Get total length of all arm segments (constants)
- Find the max length by adding up all lengths, find 
where the arm could reach at its maximum
- Set up xyz planes for the arm and end affector (tip of arm) 
- Get position of end goal
- Set up position of start (start == first joint of arm; 
this first joint position does not change)
- Draw vector from end affector line segment/vector to end goal
- Continue to draw from last drawn point to the next point                     
- ASSUMPTIONS: Suppose we have vectors L1, L2 and L3. Suppose we also have 5 points, 
which are P1, P2, P3, P4 and P5. P1 is the starting value, which is centered at 
P2 at the start of the calculation, and P2 is the tail of the L1. P3 is the tail of 
the vector L2, and P4 is the tail of L3. Assume also that the head of vector L1 is connected 
to the tail of L2, and that the head of L2 is connected to the tail of L3. P5, then, 
is where we want to place the end affector (the head of L3)
- Process for FABRICK:
  1) Make sure end point is within reach (make sure the goal point distance is
  not greater than the max arm length)
  2) Take 

*/

float getMaxLength(float arr[]);

typedef struct Vector {
  int tailPoint[3];
  int headPoint[3];
  float vectorComponents[3];
} Vector;

#define NUM_ARM_SEGMENTS 3
#define NUM_JOINTS 3

class EndAffector
{
  
};

const float segmentLengths[] = {2, 2, 4};
float maxDistance = getMaxLength(segmentLengths);  

void setup() {

}

void loop() {

}

void makeVector(Vector vector) {

}

float getMaxLength(float arr[]) {
  float maxLength = 0;
  for (int i = 0; i < NUM_ARM_SEGMENTS; i++) {
    maxLength = maxLength + arr[i];
  }
  return maxLength;
}


