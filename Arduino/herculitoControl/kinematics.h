#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct {
  double x;
  double y;
} POINT;

typedef struct {
  double n1;
  double n2;
} TWONUMS;

typedef struct {
  double high;
  double low;
} RANGE;

double rad2deg(double radang);
double deg2rad(double degang);
POINT rotate_point(POINT p, POINT pc, double radang);
POINT angles2point(double lows, double highs);
POINT point2angles(double xd, double yd);
//POINT point2anglesV2(double xd, double yd);
POINT
circle_line_intersection(double x1, double y1,
                         double x2, double y2,
                         POINT pm, double r);
RANGE yrange(double x);
RANGE xrange(double y);
#endif
