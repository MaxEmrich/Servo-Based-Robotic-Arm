#ifndef DEFS
#define DEFS

// Strcture, enum, and constant definitions 

#define NUM_ARM_SEGMENTS 3

/*

 The arm/vector lengths are in milimeters, 
 arm lengths are measured from servo rotational axis 
 to the next servo's rotational axis 

*/

#define BASE_VEC_LENGTH 5.0
#define VEC1_LENGTH 60.5 
#define VEC2_LENGTH 76.1 
#define END_EFFECTOR_LENGTH 5.0 


struct R3Point {
  float point_3d[3];
};

struct Vector {
  R3Point tailPoint;
  R3Point headPoint;
  float magnitude = 0.0;
  float vectorComponents[3];
};

// all vectors should be in component form when being used for the FABRIK calculations

const int num_joints = 4;

enum Joints {
  BASE_JOINT = 1, // start enumeration at 1, bottom joint is 2, middle joint is 3, top joint is 4
  BOTTOM_JOINT,
  MIDDLE_JOINT,
  TOP_JOINT
};

#endif // DEFS