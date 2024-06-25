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
  

  // main program loop:
  // 1. find goal point
  // 2. 

  delay(100);
}


// LINKED LIST --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------

struct node* getNewNode(struct Vector* vector, struct node* nextNode = NULL) {
  struct node* newNode = malloc(sizeof(struct node));

  if (newNode == NULL) { // if malloc didn't give us memory 
    // Serial.println("ERROR: Could not get memory for a new node...");
    // Serial.println(errno);
  }

  if (nextNode == NULL) { // if no nextNode was given...
    (*newNode).next = NULL;
  }

  (*newNode).vector = vector;

  return newNode;
}

struct Vector* myVector = malloc(sizeof(struct Vector));

struct node* myNode = getNewNode(myVector);

// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------



// VECTOR CREATION  --------------------------------------------------------------------------- 
// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------

struct Vector* newVec_FromPoint(struct R3Point* p1, struct R3Point* p2) {    // make a new vector given two points in 3-space -- p1 is the tail, p2 is the head 
  struct Vector* newVector = malloc(sizeof(struct Vector));

  // to make component vector, we use p2 - p1 = new vector

  float x_comp = (*p2).point_3d[0] - (*p1).point_3d[0]; // x_component
  float y_comp = (*p2).point_3d[1] - (*p1).point_3d[1]; // y_component
  float z_comp = (*p2).point_3d[2] - (*p1).point_3d[2]; // z_component

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
   
  return unitVector;
}  

float getYComp(float mag, float angle) { // get the y component of the vector (in an xy plane)
  return mag * sin(angle * (M_PI/180.0)); // accepts radians, so we need to convert the angle to radians 
}

float getXComp(float mag, float angle) { // get the y component of the vector (in an xy plane)
  return mag * cos(angle * (M_PI/180.0));
}

float getAngle(struct Vector* vector); // define later with magnitude of vector...

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



// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------





