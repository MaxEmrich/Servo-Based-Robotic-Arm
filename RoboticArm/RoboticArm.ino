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

#include <stdlib.h>
#include <stdio.h>

#include <assert.h>
#include <errno.h>
#include <string.h>

#include <math.h>
#include <Servo.h>

#include "defs.h"
#include "funcs.h"

extern int errno; // extern allows us to share errno variables to other files
// errno is set to 0 by default

// base servo determines orientation in 3d-space... adds z-dimension. 
// base servo rotates around the y-axis s
Servo servo_1;
Servo servo_2;
Servo servo_3;

struct Vector* my_vector;
struct Vector* my_unit_vector;

  // asssume all positions and rotations start at (0,0,0)
  // Define all 3d points by defining points RELATIVE to the (0,0,0) position of all points  
  // Make all servos rotate to be straight up, and then reset the rotation to 0

void setup() {  

  Serial.begin(9600);
  delay(200);

  servo_1.attach(3);
  servo_2.attach(4);
  servo_3.attach(5);

  struct R3Point* goalPoint; // this is a POINTER to an instance of the R3point structure  
  goalPoint = malloc(sizeof(struct R3Point)); // allocate memory for this pointer to a struct

  (*goalPoint).point_3d[0] = 3.0; // assign values to the point_3d array
  (*goalPoint).point_3d[1] = 3.0;
  (*goalPoint).point_3d[2] = 3.0;

  delay(500);

  Serial.println("Serial.begin with 9600 baud rate...");
}

void loop() {
  delay(100);
  // entry point - main loop logic 
   

  delay(100);
}





