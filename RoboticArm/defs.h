#ifndef DEFS
#define DEFS

#define NUM_ARM_SEGMENTS 3

struct R3Point {
  float point_3d[3];
};

struct Vector {
  R3Point tailPoint;
  R3Point headPoint;
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