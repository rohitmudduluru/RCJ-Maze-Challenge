#pragma once
#include <vector>
#include "bno.h"
#include "tof.h"
#include "motor.h"
#include <math.h>
#include "color.h"
#include "comm.h"

#ifdef NEW_BOT
#define WIDTH 12.6
#define ENC_PER_ROT 749
#define WHEELDIA 7.4
#define TOF_WIDTH 12.2
#define FRONTBACKTOF 19.9

#else
#define WIDTH 12.15
#define ENC_PER_ROT 749
#define WHEELDIA 7.4
#define TOF_WIDTH 12.55
#define FRONTBACKTOF 17.6
#endif

#define TILE_MOVE_DIST 32
#define TILE_LENGTH 30
extern int rampTilesForward;
extern bool incline;

#define FORWARD_MOVE_SPEED 40
#define RAMP_MOVE_SPEED 100
#define RAMP_TILT_THRESH 10

#define RAMP_ON

struct Point {
  int x;
  int y;
  int z;
};
struct PointCmp {
  bool operator()(const Point &lhs, const Point &rhs) const {
    return lhs.x == rhs.x ? (lhs.y == rhs.y ? lhs.z > rhs.z : lhs.y > rhs.y) : lhs.x > rhs.x;
  }
};
enum Direction {
  NORTH = 0,
  SOUTH = 2,
  EAST = 1,
  WEST = 3
};
enum Status {
  TRAVERSING = 0,
  DANGERZONE = 1,
  BACKTRACKING = 2,
  FINISH = 3,
  END = 4
};

enum ReturnError {
  GOOD = 0,
  BLACKTILE = 1,
  REDTILE = 2,
  RAMP = 3,
  NOMOVES = 4,
  BLUETILE = 5,
  SILVERTILE = 6,
  STAIRS = 7
};

#define BASE_TURN_SPEED 90
#define TURNKP 0

class Robot {
private:

public:
  Point pos;
  Direction facing;
  Status status;
  Robot();
  ReturnError moveDirections(std::vector<Direction> directions);
  ReturnError moveRobot(Direction dir);
  double sideAlignment();
  ReturnError robotForward(double cm);
  ReturnError wallTrace(int cm, int speed);
  void turn_to(int deg);
  void turn(int deg);
  void frontAlign();
  void backAlign();
  void print();
};
bool samePoint(Point p1, Point p2);
Point nextPoint(Point p, Direction d, int mag = 1);
int directionAngle(Direction d);
void printPoint(Point p);
void printDir(Direction d);
