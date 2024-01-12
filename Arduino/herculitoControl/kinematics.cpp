#include "config.h"
#include "kinematics.h"
#include "serial.h"
#include "StepperControl.h"

// Inverse kinematics: specifies the end-effector location and computes the associated joint angles.
// Forward kinematics: specifies the joint parameters and computes the configuration of the chain.

/* Arduino defines:
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
*/

extern StepperControl StepperA;
extern StepperControl StepperB;

double shank_length = SHANK_LENGTH;
double leg_length = LEG_LENGTH;

double sqr(double num) {
  return (num * num);
};

double rad2deg(double radang) {
  return (RAD_TO_DEG * radang);
}

double deg2rad(double degang) {
  return (DEG_TO_RAD * degang);
}

POINT rotate_point(POINT p, POINT pc, double radang) {
  POINT po;
  double s = sin(-radang);  // invert rotation
  double c = cos(-radang);
  po.x = p.x - pc.x;  // translate point back to origin:
  po.y = p.y - pc.y;
  double xnew = po.x * c - po.y * s;  // rotate point
  double ynew = po.x * s + po.y * c;
  po.x = xnew + pc.x;  // translate point back:
  po.y = ynew + pc.y;
  return po;
}

POINT angles2point(double lows, double highs) {
  POINT p1, p2, p3, p2r, p3R, p3r;
  p1.x = p1.y = p2.x = 0.0;
  p2.y = p3.x = p3.y = shank_length;
  p2r = rotate_point(p2, p1, lows);
  p3R = rotate_point(p3, p2, highs - lows);
  p3r = rotate_point(p3R, p1, lows);
  p3r.x = p3r.x - shank_length;
  p3r.y = p3r.y - shank_length;
  return (p3r);
}

// Simplified 2 circle intersection problem
POINT point2angles(double xd, double yd) {
  POINT po;
  double rx, xn, yn, beta, alpha, gamma;

  rx = hypot(xd, yd);
  beta = acos(0.5 * rx / shank_length);  // lower shank == upper shank
  if (yd > 0)
    alpha = HALF_PI - beta - acos(xd / rx);
  else
    alpha = HALF_PI - beta + acos(xd / rx);
  xn = shank_length * sin(alpha);
  yn = shank_length * cos(alpha);
  if (xd > xn)
    gamma = alpha + asin((yd - yn) / shank_length);
  else
    gamma = alpha - asin((yd - yn) / shank_length) - PI;
  if (gamma < -PI)
    gamma = gamma + TWO_PI;
  po.x = alpha;
  po.y = alpha - gamma;  // invert
  return (po);
}

/*
POINT point2anglesV2(double xd, double yd) {
  POINT po;
  double a, dx, dy, d, h, rx, ry;
  double pi2, xl, yl, xi, yi;

  pi2 = 0.5 * PI;
  po.x = po.y = -1000;  // indicate failure
  dx = xd + shank_length;
  dy = yd + shank_length;
  d = hypot(dx, dy);
  if (d > (shank_length + shank_length))  // no intersection possible
    return (po);
  if (d < 0)  // no solution. one circle is contained in the other
    return (po);
  a = (d * d) / (2.0 * d);
  xl = (dx * a / d) - shank_length;
  yl = (dy * a / d) - shank_length;
  h = sqrt((shank_length * shank_length) - (a * a));
  rx = -dy * (h / d);
  ry = dx * (h / d);
  if (ry > 0) {
    xi = xl + rx;
    yi = yl + ry;
  } else {
    xi = xl - rx;
    yi = yl - ry;
  }
  po.x = pi2 - atan((yi + shank_length) / (xi + shank_length));
  po.y = -atan((yi - yd) / (xi - xd));
  printf("PO %.4f %.4f\n", po.x, po.y);
  printf("A %d\n", StepperA.angle2steps(po.x, TWO_PI));
  printf("B %d\n", StepperB.angle2steps(po.y, TWO_PI));
  return (po);
}
*/

// Modified circle line intersection
// x1,y1: point on the line)
// x2,y2: point on the line)
// xm,ym: center of the circle and radius r

POINT
circle_line_intersection(double x1, double y1,
                         double x2, double y2,
                         POINT pm, double r) {
  POINT p1, p2;
  double x, y, a, b, c, mu, i;

  a = sqr(x2 - x1) + sqr(y2 - y1);
  b = 2 * ((x2 - x1) * (x1 - pm.x) + (y2 - y1) * (y1 - pm.y));
  c = sqr(pm.x) + sqr(pm.y) + sqr(x1) + sqr(y1)
      - 2 * (pm.x * x1 + pm.y * y1) - sqr(r);
  i = b * b - 4 * a * c;

  if (i < 0.0)  // no intersection
  {
    p1.x = p1.y = 1000.0;  // indicate no intersection
    return (p1);
  }
  if (i == 0.0)  // one intersection
  {
    mu = -b / (2 * a);
    p1.x = x1 + mu * (x2 - x1);
    p1.y = y1 + mu * (y2 - y1);
    return (p1);
  }
  if (i > 0.0)  // two intersections - take the "left" one
  {
    // first intersection
    mu = (-b + sqrt(sqr(b) - 4 * a * c)) / (2 * a);
    p1.x = x1 + mu * (x2 - x1);
    p1.y = y1 + mu * (y2 - y1);
    // second intersection
    mu = (-b - sqrt(sqr(b) - 4 * a * c)) / (2 * a);
    p2.x = x1 + mu * (x2 - x1);
    p2.y = y1 + mu * (y2 - y1);
    if (p2.x > p1.x)
      return (p2);
    else
      return (p1);
  }
}

RANGE yrange(double x) {
  RANGE ra;
  double ytmp, xposlim;

  xposlim = leg_length - shank_length;
  if (x > xposlim) x = xposlim;
  if (x < -100.0) x = -100.0;  // practical limit
  ytmp = sqrt(sqr(leg_length) - sqr(x + shank_length));
  ra.high = ytmp - shank_length;
  ra.low = -ytmp - shank_length;
  return (ra);
}

RANGE xrange(double y) {
  RANGE ra;
  POINT pa, pb, prot, plim;
  double xtmp;

  pa.x = pa.y = pb.x = -shank_length;
  pb.y = 0.0;
  prot = rotate_point(pb, pa, StepperA.steps2angle(STEPPER_A_MIN_STEPS, TWO_PI));
  plim = circle_line_intersection(-500.0, y, 500.0, y, prot, shank_length);
  if (plim.x == 1000.0)  // no intersection found
    plim.x = -100.0;     // practical limit
  xtmp = sqrt(sqr(leg_length) - sqr(y + shank_length));
  ra.high = xtmp - shank_length;
  ra.low = plim.x;  // pp.y is lower limit
  if ((ra.low < -80.0) && (y < -shank_length)) ra.low = -80.0;
  if (ra.high > (leg_length - shank_length)) ra.high = leg_length - shank_length;  // practical max.
  return (ra);
}
