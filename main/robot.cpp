#include "robot.h"

int rampTilesForward = 0;
bool incline = false;

bool samePoint(Point p1, Point p2) {
  return (p1.x == p2.x && p1.y == p2.y);
}
Point nextPoint(Point p, Direction d, int mag) {
  Point temp = p;
  switch (d) {
    case NORTH:
      temp.y += mag;
      break;
    case SOUTH:
      temp.y -= mag;
      break;
    case EAST:
      temp.x += mag;
      break;
    case WEST:
      temp.x -= mag;
      break;
  }
  return temp;
}

int directionAngle(Direction d) {
  switch (d) {
    case NORTH:
      return 0;
    case SOUTH:
      return 180;
    case EAST:
      return 90;
    case WEST:
      return 270;
  }
  return -1;
}

// Don't pay attention to this function it doesn't work well
ReturnError Robot::wallTrace(int cm, int speed) {
  encR = 0;
  double prevErr = 0, dist = 0, err = 0;
  double leftDist, rightDist;
  while (encToCm(encR) < cm) {
    leftDist = readTOF(LEFT_TOF);
    rightDist = readTOF(RIGHT_TOF);
    Serial.print("Left: ");
    Serial.print(leftDist);
    Serial.print("; Right: ");
    Serial.print(rightDist);
    if (leftDist < 15)
      dist = leftDist;
    else if (rightDist < 15)
      dist = 30 - WIDTH - rightDist;
    else {
      forward(speed);
      continue;
    }

    err = (30 - WIDTH) / 2 - dist;
    lmotors(speed + err * kP + (err - prevErr) * kD);
    rmotors(speed - err * kP - (err - prevErr) * kD);
    prevErr = err;
  }
  return GOOD;
}

// Radians to Angle
double rToA(double r) {
  return r * 180 / PI;
}
// Angle to Radians
double aToR(double a) {
  return a * PI / 180;
}

/*
 * Find angle to turn to in order to be aligned with the next tile
 *
 * @param None
 * @return Acute angle from the y-axis
 */
double Robot::sideAlignment() {
  double dist = readTOF(LEFT_TOF);  //bias - positive = goes closer to wall
  //dist*=dist/3; //exagerates movement - (dist/b)^a decrease a to exagerate less, b is focus of exageration
  dist = dist < 0 ? 0.2 : dist;
  if (dist < 15 && dist > 0.1) {
    double theta = atan(60 / (2 * dist - 30 + TOF_WIDTH)) >= 0 ? rToA(atan(60 / (2 * dist - 30 + TOF_WIDTH))) : rToA(atan(60 / (2 * dist - 30 + TOF_WIDTH))) + 180;  // math
    turn_to(roundAngle(getBNO()) - 90 + theta);
    stop_motors();
    delay(10);
    return theta;
  }
  dist = readTOF(RIGHT_TOF);  //bias - positive = goes closer to wall
  //dist*=dist/3; //exagerates movement - (dist/b)^a decrease a to exagerate less, b is focus of exageration
  dist = dist < 0 ? 0.2 : dist;
  if (dist < 15 && dist > 0.1) {
    double theta = atan(60 / (2 * dist - 30 + TOF_WIDTH)) >= 0 ? rToA(atan(60 / (2 * dist - 30 + TOF_WIDTH))) : rToA(atan(60 / (2 * dist - 30 + TOF_WIDTH))) + 180;  //math
    turn_to(roundAngle(getBNO()) + 90 - theta);
    stop_motors();
    delay(10);
    return theta;
  }
  return 90;
}


ReturnError Robot::rampCase() {
  bool blueTrigger = false;
  if (getTilt() > 0) {
    Serial.println("Going up ramp...");
    incline = true;
  } else {
    Serial.println("Going down ramp...");
    incline = false;
  }
  // stop_motors(); delay(500);// while(digitalRead(20)==HIGH);
  encR = 0;
  double distForward = 0;  // x dist forward
  int prevEnc = 0;
  float angle = getTilt();
  int calcIter = 0;
  double baseSpeed, tofError;
  while ((abs(getTilt())) > RAMP_TILT_THRESH) {
    if (interrupted) {
      interruptFunc();
    }
    if (!(calcIter++ % 10)) {
      baseSpeed = RAMP_MOVE_SPEED * (1 + 0.01 * (angle - 20));  //PID
      tofError = (double)(30 - TOF_WIDTH) / 2 - readTOF(LEFT_TOF);
      if (tofError > 15 || tofError < -15) tofError = 0;
      lmotors(baseSpeed + tofError * (incline ? 5 : 1));
      rmotors(baseSpeed - tofError * (incline ? 5 : 1));
      distForward += encToCm(encR - prevEnc) * cos((aToR(angle = getTilt())));
      Serial.print("Enc: ");
      Serial.print(encR);
      Serial.print(" Prev: ");
      Serial.print(prevEnc);
      Serial.print(" Angle: ");
      Serial.print(angle);
      Serial.print(" Cos: ");
      Serial.println(cos(aToR(angle)));
      prevEnc = encR;
    }
  }
  distForward *= .9;  // tested error
  if (!incline) distForward += 10;
  Serial.print("Dist forward: ");
  Serial.println(distForward);
  stop_motors();
  delay(500);
  switch (getColor()) {
    case BLUE:
      blueTrigger = true;
      tone(BUZZER, 400, 100);
      delay(200);
      tone(BUZZER, 500, 200);
      break;
    case BLACK:
      tone(BUZZER, 200, 500);
      backwardCm(incline ? 30 : 40, 5);
      while ((abs(getTilt())) > RAMP_TILT_THRESH) {
        baseSpeed = RAMP_MOVE_SPEED * (1 - 0.01 * (angle - 20));  //PID
        tofError = (double)(30 - TOF_WIDTH) / 2 - readTOF(LEFT_TOF);
        if (tofError > 15 || tofError < -15) tofError = 0;
        lmotors(-baseSpeed - tofError * (incline ? 1 : 5));
        rmotors(-baseSpeed + tofError * (incline ? 1 : 5));
      }
      backwardCm(incline ? 40 : 50, 5);
      return BLACKTILE;
  }

  rampTilesForward = distForward / 30;
  if ((int)distForward % TILE_LENGTH > 15) rampTilesForward++;
  while (abs(getTilt()) > 5) {
    forward(RAMP_MOVE_SPEED * (.75 + 0.04 * (angle)));
  }
  if (distForward < 20) {
    Serial.println("Stairs?");
    forwardCm(40, incline ? 20 : 10);
    Serial.println("Did adjust");
    stop_motors();
    delay(500);
    rampTilesForward = incline ? 1 : 0;
    return RAMP;
  }
  forwardCm(60, 5);
  stop_motors();
  delay(500 + blueTrigger ? 4500 : 0);
  return RAMP;
}

ReturnError Robot::colorCase(bool* blueTrigger, bool* silverTrigger, bool* redTrigger) {
  switch (getColor()) {
    case BLUE:
      if (*blueTrigger) break;  //encToCm(encR) < 20 ||
      stop_motors();
      delay(200);
      if (getColor() == BLUE) {
        *blueTrigger = true;
        tone(BUZZER, 400, 100);
        delay(200);
        tone(BUZZER, 500, 200);
      }
      break;
    case BLACK:
      if (*redTrigger) break;
      stop_motors();
      delay(200);
      if (getColor() == BLACK) {
        tone(BUZZER, 200, 500);
        backwardCm(FORWARD_MOVE_SPEED, encToCm(encR) + 1);
        restartPi = cmToEnc(5);
        if (*blueTrigger) {
          stop_motors();
          delay(5000);
        }
        return BLACKTILE;
      }
      break;
    case SILVER:
      // Serial.println("In silver...");
      if (encToCm(encR) < 20 || *silverTrigger) break;
      stop_motors();
      delay(200);
      if (getColor() == SILVER) {
        tone(BUZZER, 400, 200);
        Serial.print("Triggered Silver at angle ");
        Serial.println(getTilt());
        *silverTrigger = true;
      }
      break;
    case RED:
      if (_status == TRAVERSING) {
        stop_motors();
        delay(200);
        if (getColor() == RED) {
          backwardCm(FORWARD_MOVE_SPEED, encToCm(encR) + 1);
          return REDTILE;
        }
        break;
      } else if (!*redTrigger) {
        stop_motors();
        delay(200);
        if (getColor() == RED) {
          tone(BUZZER, 600, 500);
          *redTrigger = true;
        }
      }
    case WHITE:
    default:
      break;
  }
  return GOOD;
}

ReturnError Robot::objectCase(double cmLeft, double cmGoal) {
  forwardCm(80, 1);
  cmLeft--;
  bool lswitch = digitalRead(LEFT_LIMIT_SWITCH_PIN) == LOW, rswitch = digitalRead(RIGHT_LIMIT_SWITCH_PIN) == LOW;
  if (!lswitch && !rswitch) {
    encR = cmToEnc(cmGoal - cmLeft);
    return GOOD;
  }
  if (lswitch && rswitch) {
    if (cmLeft > cmGoal * .85) {  //close obstacle - count as wall
      return WALLOBSTACLE;
    }
    if (cmLeft < cmGoal * .15) {  //far obstacle - pretend it is wall
      backwardCm(60, 2);
      encR = encToCm(cmGoal) + 1;
      return GOOD;
    }
    backwardCm(FORWARD_MOVE_SPEED, cmGoal - cmLeft);  //middle obstacle - count as black tile
    return BLACKTILE;
  }
  backwardCm(60, 15);
  return GOOD;
}

/*
 * Move the robot forward while checking color sensor
 *
 * @param None
 * @return Whether or not the robot finds a special case (e.g. ramp, black tile)
 */
ReturnError Robot::robotForward(double cm) {
  goingForward = true;
  Serial.print("CM:");
  Serial.println(cm);
  encR = 0;
  unsigned long colorTimer = millis();
  bool blueTrigger = false, silverTrigger = false, redTrigger = false;
  // Serial.print("encR: ");
  // Serial.println(encR);
  while (encR < cmToEnc(cm)) {
    if (encToCm(encR) > (cm * .7) && readTOF(FRONT_TOF) < 8) break;  //sees wall

    if (digitalRead(LEFT_LIMIT_SWITCH_PIN) == LOW || digitalRead(RIGHT_LIMIT_SWITCH_PIN) == LOW) {
      backwardCm(FORWARD_MOVE_SPEED, encToCm(encR) + 1);
      return BLACKTILE;
      ReturnError obsReturn = objectCase(TILE_MOVE_DIST - encToCm(encR), cm);
      switch (obsReturn) {
        case GOOD: break;
        case WALLOBSTACLE:
        case BLACKTILE:
          return obsReturn;
      }
    }
    if (interrupted) {
      interruptFunc();
    }
    if (restartPi == 0) {
      restartPi--;
      Serial1.print("r");
    }
    // Serial.print("Ramp Tilt: "); Serial.println(abs(getTilt()));
    if (abs(getTilt()) > RAMP_TILT_THRESH) {
      return rampCase();
    }

    if (millis() - colorTimer > 100) {
      ReturnError colorReturn = colorCase(&blueTrigger, &silverTrigger, &redTrigger);
      switch (colorReturn) {
        case GOOD: break;
        default: return colorReturn;
      }
      colorTimer = millis();
    }
    forward(FORWARD_MOVE_SPEED);
  }
  // if(obstacle) {
  //   stop_motors();
  //   delay(500);
  //   backtrack();
  //   stop_motors();
  //   delay(100);
  // }
  encR = 0;
  stop_motors();
  delay(10);
  if (blueTrigger) return BLUETILE;
  if (silverTrigger) return SILVERTILE;
  return GOOD;
}

/*
 * Align the robot with the front TOF
 *
 * @param None
 * @return None
 */
void Robot::frontAlign() {
  Serial.println("Front Aligning");
  int dist = readTOF(FRONT_TOF);
  while (dist < 15 && dist > (30 - FRONTBACKTOF) / 2) {
    forward(30);
    dist = readTOF(FRONT_TOF);
  }
  while (dist < 15 && dist < (30 - FRONTBACKTOF) / 2) {
    backward(30);
    dist = readTOF(FRONT_TOF);
  }
  Serial.println("Front Aligning Done");
}
/*
 * Align the robot with the back TOF
 *
 * @param None
 * @return None
 */
void Robot::backAlign() {
  Serial.println("Back Aligning");
  int dist = readTOF(BACK_TOF);
  while (dist < 15 && dist > (30 - FRONTBACKTOF) / 2) {
    backward(30);
    dist = readTOF(BACK_TOF);
  }
  while (dist < 15 && dist < (30 - FRONTBACKTOF) / 2) {
    forward(30);
    dist = readTOF(BACK_TOF);
  }
  Serial.println("Back Aligning Done");
}


Robot::Robot() {
  _pos.x = 0;
  _pos.y = 0;
  _pos.z = 0;
  _facing = NORTH;
  _status = DANGERZONE;
  _coords = Vector2D(0, 0);
}

/*
 * Move the robot in the direction dir
 *
 * @param Direction dir to move towards
 * @return Error status of movement (e.g. black tile, ramp)
 */
ReturnError Robot::moveRobot(Direction dir) {
  _facing = dir;
  stop_motors();
  delay(200);

  turn_to(directionAngle(dir));
  stop_motors();
  delay(200);
  frontAlign();
  backAlign();
  turn_to(directionAngle(dir));
  stop_motors();
  delay(200);
  // Serial.println("rcj done");
  // Serial.println(abs(encR));
  // delay(10);
  switch (robotForward(TILE_MOVE_DIST / sin(aToR(sideAlignment())))) {
      // Serial.println(abs(encR));
      // Serial.println(abs(encR));
    case RAMP:
      goingForward = false;
      int back;
      while (readTOF(BACK_TOF) < 15) forward(60);
      stop_motors();
      delay(500);
      return RAMP;
    case BLACKTILE:
      goingForward = false;
      stop_motors();
      delay(500);
      return BLACKTILE;
    case REDTILE:
      goingForward = false;
      stop_motors();
      delay(500);
      return REDTILE;
    case SILVERTILE:
      goingForward = false;
      stop_motors();
      delay(200);
      turn_to(directionAngle(dir));
      stop_motors();
      delay(200);
      frontAlign();
      backAlign();
      return SILVERTILE;
    case BLUETILE:
      goingForward = false;
      stop_motors();
      delay(5000);
    case GOOD:
    default:
      goingForward = false;
      stop_motors();
      delay(200);
      turn_to(directionAngle(dir));
      stop_motors();
      delay(200);
      frontAlign();
      backAlign();
      return GOOD;
  }
}
/*
 * Move robot on the path denoted by the vector directions
 *
 * @param Vector of the directions to move in
 * @return Error status of movement (e.g. black tile, no moves left)
 */
ReturnError Robot::moveDirections(std::vector<Direction> directions) {
  if (directions.empty()) return NOMOVES;
  ReturnError moveStatus = GOOD;
  int numLeft = directions.size();
  bool trigger = false;
  for (Direction d : directions) {
    if (--numLeft) {
      doVictims = false;
      trigger = true;
    } else {
      doVictims = true;
      if (trigger) restartPi = cmToEnc(5);
    }
    printDir(d);
    _pos = nextPoint(_pos, d);
    switch ((moveStatus = moveRobot(d))) {
      case BLACKTILE:
        return BLACKTILE;
      case SILVERTILE:
        return SILVERTILE;
    }
    stop_motors();
    delay(200);
  }
  return moveStatus;
}

/*
 * PID turn the robot to specified degrees clockwise from North
 *
 * @param Degrees clockwise from North to turn to
 * @return None
 */

/*
void Robot::turn_to(int deg) {

  /*
lmotors(-80);
delay(175);
stop_motors();
delay(100);
rmotors(75);
delay(175);
stop_motors();
forward(60);
delay(20);
stop_motors();
delay(300);
  lmotors(-60);
  rmotors(-60);
  delay(50);
  stop_motors();
  int i = 0;
  deg %= 360;
  if (deg < 0) deg += 360;
  double err = deg - getBNO();
  if (err > 180) err -= 360;
  if (err < -180) err += 360;
  while (err > 2 || err < -2) {
#ifdef CAM_ON
    if (interrupted) interruptFunc();
#endif
    if (err > 0) {

      if (i == 0) {
        lmotors(-80);
        rmotors(0);
        delay(175);
      }
      if (i == 1) {
        stop_motors();
        delay(100);
      }

      if (i == 2) {
        rmotors(75);
        lmotors(0);
        delay(175);
      }

      if (i == 3) {
        stop_motors();
        forward(60);
        delay(35);
      }

      if (i == 4) {
        stop_motors();
        delay(300);
      }
    } else if (err <= 0) {
      if (i == 0) {
        lmotors(0);
        rmotors(-75);
        delay(175);
      }
      if (i == 1) {
        stop_motors();
        delay(100);
      }

      if (i == 2) {
        rmotors(0);
        lmotors(80);
        delay(175);
      }

      if (i == 3) {
        stop_motors();
        forward(60);
        delay(35);
      }

      if (i == 4) {
        stop_motors();
        delay(300);
      }
    }
    err = deg - getBNO();
    i++;
    if (i == 5) {
      i = 0;
    }
    if (err > 180) err -= 360;
    if (err < -180) err += 360;
    //Serial.print("Error: "); Serial.println(err);
  }
  //Serial.println("DONE");
}

*/

void Robot::turn_to(int deg) {
  deg %= 360;
  if (deg < 0) deg += 360;
  double err = 100;  // just to go in loop
  while (abs(err) > 1) {
    err = deg - getBNO();
    if (err > 180) err -= 360;
    if (err < -180) err += 360;
    if (interrupted) {
      stop_motors();
      interruptFunc();
    }
    lmotors((err > 0 ? BASE_TURN_SPEED : -BASE_TURN_SPEED) + err * TURNKP);
    rmotors((err > 0 ? -BASE_TURN_SPEED : BASE_TURN_SPEED) - err * TURNKP);

    //Serial.print("Error: "); Serial.println(err);
  }
  //Serial.println("DONE");
}




/*
 * Turn specified degrees clockwise from current angle
 *
 * @param Degrees to turn clockwise
 * @return None
 */
void Robot::turn(int deg) {

  turn_to(deg + getBNO());
}

void Robot::print() {
  switch (_status) {
    case TRAVERSING: Serial.print("Traversing... "); break;
    case DANGERZONE: Serial.print("In Danger Zone... "); break;
    case BACKTRACKING: Serial.print("Backtracking... "); break;
    case FINISH: Serial.print("Finished! "); break;
    case END: Serial.print("Run over... "); break;
  }
  Serial.print("Robot at (");
  Serial.print(_pos.x);
  Serial.print(",");
  Serial.print(_pos.y);
  Serial.print(",");
  Serial.print(_pos.z);
  Serial.print(") facing ");
  switch (_facing) {
    case NORTH: Serial.print("North"); break;
    case SOUTH: Serial.print("South"); break;
    case EAST: Serial.print("East"); break;
    case WEST: Serial.print("West"); break;
  }
  Serial.println();
}

void printPoint(Point p) {
  Serial.print("(");
  Serial.print((int)p.x);
  Serial.print(",");
  Serial.print((int)p.y);
  Serial.print(",");
  Serial.print((int)p.z);
  Serial.println(")");
}

void printDir(Direction d) {
  switch (d) {
    case NORTH: Serial.println("North"); break;
    case SOUTH: Serial.println("South"); break;
    case EAST: Serial.println("East"); break;
    case WEST: Serial.println("West"); break;
  }
}

// v2(r-w/2) = v1(r+w/2)
// r(v2-v1)=v1w/2+v2w/2
// r=w(v2+v1)/2(v2-v1)
double _lspeedfunc(double t_ms) {
  return 80 * (t_ms > 1500 ? -1 : 1);
  double t = t_ms / 1000;
  return t * t / 9 * 100;
}
double _rspeedfunc(double t_ms) {
  return -80 * (t_ms > 1500 ? -1 : 1);
  double t = t_ms / 1000;
  return sin(t * 2 * PI / 3) * 100;
}
Vector2D trackPos() {
  Vector2D pos(0, 0), deltapos(0, 0);
  encL = 0, encR = 0;
  int prevEncL = 0, prevEncR = 0;
  double dl, dr, dCenter, theta;  // theta-clockwise angle from north
  long long startTime = millis(), timer = startTime, currTime = startTime;
  while (currTime - startTime < 3 * 1000) {
    // Serial.print("Left speed: "); Serial.print(_lspeedfunc(currTime - startTime));
    // Serial.print("; Right speed: "); Serial.print(_rspeedfunc(currTime - startTime));
    lmotors(_lspeedfunc(currTime - startTime));
    rmotors(_rspeedfunc(currTime - startTime));
    // Serial.print(" --- ");
    // Serial.print(currTime-startTime);
    // Serial.print("; ");
    // Serial.println(currTime-timer);
    if (currTime - timer > 50) {
      dl = encToCm(encL - prevEncL);
      dr = encToCm(encR - prevEncR);
      prevEncL = encL;
      prevEncR = encR;
      dCenter = (dl + dr) / 2;
      theta = aToR(getBNO());
      deltapos = Vector2D(dCenter * sin(theta), dCenter * cos(theta));
      Serial.print("DL: ");
      Serial.print(dl);
      Serial.print("; DR: ");
      Serial.print(dr);
      Serial.print("; DCenter: ");
      Serial.print(dCenter);
      Serial.print("; Theta: ");
      Serial.print(theta);
      Serial.print("; Delta pos: ");
      deltapos.print();
      timer = millis();

      double sum = dl + dr;
      bool tankTurning = (abs(sum) < 0.6);

      if (tankTurning) {
        timer = millis();  // still update time
        Serial.println(" FILTERED");
        continue;  // skip this update
      }
      pos = pos + deltapos;
      Serial.println();
    }
    currTime = millis();
  }
  Serial.println("Final pos: ");
  pos.print();
  return pos;
}

#define TURN_WEIGHT 4
void velocityToMovement(Vector2D unitVelocity) {
  unitVelocity = unitVelocity * 100;
  double theta = aToR(getBNO());
  Vector2D forward(sin(theta), cos(theta));
  Vector2D left(-cos(theta), sin(theta));  // Left is 90° CCW from forward

  double baseSpeed = unitVelocity * forward;
  double turnAmount = (unitVelocity * left) * TURN_WEIGHT;
  double lspeed = baseSpeed - turnAmount;
  double rspeed = baseSpeed + turnAmount;
  double maxspeed = max(abs(lspeed), abs(rspeed));
  if (maxspeed > 100) {
    lspeed *= 100 / maxspeed;
    rspeed *= 100 / maxspeed;
  }
  lmotors(lspeed);
  rmotors(rspeed);
}
Vector2D directionToVector(Direction d) {
  switch (d) {
    case NORTH: return Vector2D(0, 1);
    case SOUTH: return Vector2D(0, -1);
    case EAST: return Vector2D(1, 0);
    case WEST: return Vector2D(-1, 0);
  }
  return Vector2D(0, 0);
}

#define FORWARD_WEIGHT 9
#define FIELD_STRENGTH 13
#define WALL_STRENGTH 5
#define OBJECT_STRENGTH 10  //untuned
#define OBJECT_RADIUS 2.5     //untuned
#define DELTA_TIME 20       //millis
#define TANK_TURN_DEADZONE 0.2
void obstacleTest() {
  Vector2D pos(0, 0), deltapos(0, 0);
  Vector2D target(-5, 30);
  Vector2D object(5, 20);
  Vector2D velocity = (target + (pos * -1)).normalized() * FORWARD_WEIGHT;
  Vector2D force(0, 0);
  int prevEncL = 0, prevEncR = 0;
  double dl, dr, dCenter, theta;  // theta-clockwise angle from north
  long long timer = millis(), currTime = timer;
  Vector2D facingVector = directionToVector(NORTH);
  while (pos * facingVector < target * facingVector) {
    currTime = millis();
    if (currTime - timer > DELTA_TIME) {
      dl = encToCm(encL - prevEncL);
      dr = encToCm(encR - prevEncR);
      prevEncL = encL;
      prevEncR = encR;
      dCenter = (dl + dr) / 2;
      theta = aToR(getBNO());
      deltapos = Vector2D(dCenter * sin(theta), dCenter * cos(theta));
      timer = millis();

      if (abs(dl + dr) < TANK_TURN_DEADZONE) {
        timer = millis();  // still update time
        continue;          // skip this update
      }
      pos = pos + deltapos;
      force = target + (pos * -1);
      force = force.normalized() * FIELD_STRENGTH;
      force.x += WALL_STRENGTH / max(0.1, (pos.x + 15));
      force.x += WALL_STRENGTH / max(0.1, (15 - pos.x));
      force.x += OBJECT_STRENGTH / max(0.5, abs(pos.x - object.x) - OBJECT_RADIUS) * (0 < object.x ? -1 : 1);  //(dist.x*dist.x+dist.y*dist.y)));
      velocity = (velocity + force).normalized() * FORWARD_WEIGHT;
      Serial.print("Force: ");
      force.print();
      Serial.print("; Velocity: ");
      velocity.print();
      Serial.print("; Pos: ");
      pos.print();
      Serial.println();
    }
    velocityToMovement(velocity.normalized());
  }
}

ReturnError Robot::moveToTarget(Vector2D target, bool clearObjects) {
  if (clearObjects) _objects.clear();
  Vector2D roundedPos = Vector2D((int)_coords.x - ((int)_coords.x % 30) + ((int)_coords.x % 30 > 15 ? 30 : 0), (int)_coords.y - ((int)_coords.y % 30) + ((int)_coords.y % 30 > 15 ? 30 : 0));
  Vector2D deltapos(0, 0);
  Vector2D velocity = (target - _coords).normalized() * FORWARD_WEIGHT;
  Vector2D force(0, 0);
  int prevEncL = 0, prevEncR = 0;
  encL = 0;
  encR = 0;
  double dl, dr, dCenter, theta;  // theta-clockwise angle from north
  long long timer = millis(), currTime = timer;
  Vector2D facingVector = directionToVector(_facing);
  Serial.print("\nStarting Coords: ");
  _coords.print();
  Serial.print("\nRounded Coords: ");
  roundedPos.print();
  Serial.print("\nFacing: ");
  facingVector.print();
  Serial.print("\nTarget: ");
  target.print();
  if (clearObjects) Serial.println("\nCleared objects!");
  else {
    Serial.println("\nObjects Kept:");
    for (Vector2D obj : _objects) {
      obj.print();
      Serial.println();
    }
  }
  Serial.println();
  while (_coords * facingVector < target * facingVector) {
    currTime = millis();
    if (currTime - timer > DELTA_TIME) {
      dl = encToCm(encL - prevEncL);
      dr = encToCm(encR - prevEncR);
      prevEncL = encL;
      prevEncR = encR;
      dCenter = (dl + dr) / 2;
      theta = aToR(getBNO());
      deltapos = Vector2D(dCenter * sin(theta), dCenter * cos(theta));
      timer = millis();
      if (abs(dl + dr) < TANK_TURN_DEADZONE) {
        timer = millis();  // still update time
        continue;          // skip this update
      }
      _coords = _coords + deltapos;

      force = (target - _coords).normalized() * FIELD_STRENGTH;
      // force.x += WALL_STRENGTH / max(0.1, (pos.x + 15));
      // force.x += WALL_STRENGTH / max(0.1, (15 - pos.x));
      Vector2D right = directionToVector((Direction)(((int)_facing + 1) % 4));
      force = force + (right * (WALL_STRENGTH / max(0.1, 15 + abs((_coords - roundedPos) * right))));
      force = force + (right * (WALL_STRENGTH / max(0.1, 15 - abs((_coords - roundedPos) * right))));
      Serial.print("Right: "); right.print(); Serial.print("; Left Wall: "); (right * (WALL_STRENGTH /max(0.1,15+abs((_coords-roundedPos)*right)))).print();Serial.print(15+abs((_coords-roundedPos)*right)); Serial.print("; Right Wall: "); (right * (WALL_STRENGTH /max(0.1,15-abs((_coords-roundedPos)*right)))).print(); Serial.print(15-abs((_coords-roundedPos)*right));
      // force.x += OBJECT_STRENGTH / max(0.5,abs(pos.x-object.x)-OBJECT_RADIUS) * (0<object.x?-1:1);//(dist.x*dist.x+dist.y*dist.y)));
      for (Vector2D object : _objects) {
        force = force + (right * (OBJECT_STRENGTH / max(.8, (_coords - object) * right - OBJECT_RADIUS) * ((_coords - object) * right < 0 ? -1 : 1)));
      }
      velocity = (velocity + force).normalized() * FORWARD_WEIGHT;
      Serial.print("Force: ");
      force.print();
      Serial.print("; Velocity: ");
      velocity.print();
      Serial.print("; Pos: ");
      _coords.print();
      Serial.println();
    }
    velocityToMovement(velocity.normalized());
  }
  return GOOD;
}