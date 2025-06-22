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
  enc = 0;
  double prevErr = 0, dist = 0, err = 0;
  double leftDist, rightDist;
  while (encToCm(enc) < cm) {
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
  enc = 0;
  int colorIter = 0;
  bool blueTrigger = false, silverTrigger = false, redTrigger = false;
  // Serial.print("enc: ");
  // Serial.println(enc);
  while (enc < cmToEnc(cm)) {
    int temptof = readTOF(FRONT_TOF);
    // Serial.print("Cm traveled: ");
    // Serial.print(encToCm(enc));
    // Serial.print(", Goal: ");
    // Serial.print(cm);
    // Serial.print(", Front TOF: ");
    // Serial.println(temptof);

    if (encToCm(enc) > (cm * .7) && temptof < 8) break;
    // Serial.print("goal: ");
    // Serial.println(cmToEnc(cm));
    // Serial.print("enc: ");
    // Serial.println(enc);

    if (digitalRead(LEFT_LIMIT_SWITCH_PIN) == LOW || digitalRead(RIGHT_LIMIT_SWITCH_PIN) == LOW) {
      backwardCm(FORWARD_MOVE_SPEED, encToCm(enc) + 1);
      return BLACKTILE;
    }
    if (interrupted) {
      interruptFunc();
    }
    if (restartPi == 0) {
      restartPi--;
      Serial1.print("r");
    }
#ifdef RAMP_ON
    // Serial.print("Ramp Tilt: "); Serial.println(abs(getTilt()));
    if (abs(getTilt()) > RAMP_TILT_THRESH) {

      if (getTilt() > 0) {
        Serial.println("Going up ramp...");
        incline = true;
      } else {
        Serial.println("Going down ramp...");
        incline = false;
      }
      // stop_motors(); delay(500);// while(digitalRead(20)==HIGH);
      enc = 0;
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
          if(tofError>15||tofError<-15) tofError = 0;
          lmotors(baseSpeed + tofError * (incline ? 5 : 1));
          rmotors(baseSpeed - tofError * (incline ? 5 : 1));
          distForward += encToCm(enc - prevEnc) * cos((aToR(angle = getTilt())));
          Serial.print("Enc: ");
          Serial.print(enc);
          Serial.print(" Prev: ");
          Serial.print(prevEnc);
          Serial.print(" Angle: ");
          Serial.print(angle);
          Serial.print(" Cos: ");
          Serial.println(cos(aToR(angle)));
          prevEnc = enc;
        }
      }
      distForward *= .9;  // tested error
      if (!incline) distForward += 10;
      Serial.print("Dist forward: ");
      Serial.println(distForward);
      stop_motors();
      delay(500);
      // while (1) {
      //   switch ((getColor())) {
      //     case WHITE: Serial.println(" WHITE"); break;
      //     case BLACK: Serial.println(" BLACK"); break;
      //     case BLUE: Serial.println(" BLUE"); break;
      //     case RED: Serial.println(" RED"); break;
      //     case SILVER: Serial.println(" SILVER"); break;
      //     case UNK:
      //       Serial.println(" UNKNOWN");
      //       break;
      //       // default: Serial.print(" confusion "); Serial.println(temp);
      //   }
      // }
      

      rampTilesForward = distForward / 30;
      if ((int)distForward % TILE_LENGTH > 15) rampTilesForward++;
      while (abs(getTilt()) > 5) {
        forward(RAMP_MOVE_SPEED * (.75 + 0.04 * (angle)));
      }
      switch (getColor()) {
        case BLUE:
          blueTrigger = true;
          tone(BUZZER,400,100);
          delay(200);
          tone(BUZZER,500,200);
          forwardCm(60, 5);
          return BLUETILE;
          break;
        case RED:
          forwardCm(60, 5);
          return REDTILE;
        // case BLACK:
        //   tone(BUZZER,200,500);
        //   backwardCm(incline?30:40,5);
        //   while ((abs(getTilt())) > RAMP_TILT_THRESH){
        //     baseSpeed = RAMP_MOVE_SPEED * (1 - 0.01 * (angle - 20));  //PID
        //     tofError = (double)(30 - TOF_WIDTH) / 2 - readTOF(LEFT_TOF);
        //     if(tofError>15||tofError<-15) tofError = 0;
        //     lmotors(-baseSpeed - tofError * (incline ? 1 : 5));
        //     rmotors(-baseSpeed + tofError * (incline ? 1 : 5));
        //   }
        //   backwardCm(incline?40:50,5);
        //   return BLACKTILE;
      }
      if (distForward < 20) {
        Serial.println("Stairs?");
        forwardCm(40, incline ? 20 : 10);
        Serial.println("Did adjust");
        stop_motors();
        delay(500);
        rampTilesForward = incline ? 1 : 0;
        return STAIRS;
      }
      forwardCm(60, 5);
      stop_motors();
      delay(500 + blueTrigger?4500:0);
      return RAMP;
    }
#endif
    if (!(colorIter++ % 5)) {
      switch (getColor()) {
        case BLUE:
          if (blueTrigger||encToCm(enc)<20) break;  //encToCm(enc) < 20 ||
          stop_motors();
          delay(200);
          if (getColor() == BLUE) {
            blueTrigger = true;
            tone(BUZZER,400,50);
            delay(50);
            tone(BUZZER,500,50);
            delay(50);
            tone(BUZZER,600,200);
          }
          break;
        case BLACK:
          //if(redTrigger) break;
          stop_motors();
          delay(200);
          if (getColor() == BLACK) {
            tone(BUZZER,200,500);
            backwardCm(FORWARD_MOVE_SPEED, encToCm(enc) + 1);
            restartPi = cmToEnc(5);
            if(blueTrigger) {
              stop_motors();
              delay(5000);
            }
            return BLACKTILE;
          }
          break;
        case SILVER:
          // Serial.println("In silver...");
          if (encToCm(enc) < 20 || silverTrigger) break;
          stop_motors();
          delay(200);
          if (getColor() == SILVER) {
            tone(BUZZER, 400, 500);
            delay(500);
            tone(BUZZER, 500, 500);
            Serial.print("Triggered Silver at angle ");
            Serial.println(getTilt());
            silverTrigger = true;
          }
          break;
        case RED:
          if (status == TRAVERSING) {
            stop_motors();
            delay(200);
            if (getColor() == RED) {
              backwardCm(FORWARD_MOVE_SPEED, encToCm(enc) + 1);
              return REDTILE;
            }
            break;
          }
          else if(!redTrigger && encToCm(enc)>20){
            stop_motors();
            delay(200);
            if(getColor()==RED){
              tone(BUZZER, 600, 500);
              redTrigger = true;
            }
          }
        case WHITE:
        default:
          break;
      }
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
  enc = 0;
  stop_motors();
  delay(10);
  if (blueTrigger) return BLUETILE;
  if (silverTrigger) return SILVERTILE;
  if (redTrigger) return REDTILE;
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
  pos.x = 0;
  pos.y = 0;
  pos.z = 0;
  facing = NORTH;
  status = DANGERZONE;
}

/*
 * Move the robot in the direction dir
 *
 * @param Direction dir to move towards
 * @return Error status of movement (e.g. black tile, ramp)
 */
ReturnError Robot::moveRobot(Direction dir) {
  facing = dir;
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
  // Serial.println(abs(enc));
  // delay(10);
  switch (robotForward(TILE_MOVE_DIST / sin(aToR(sideAlignment())))) {
      // Serial.println(abs(enc));
      // Serial.println(abs(enc));
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
      return BLUETILE;
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
    pos = nextPoint(pos, d);
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
  turning = true;
  deg %= 360;
  if (deg < 0) deg += 360;
  double err = deg - getBNO();
  if (err > 180) err -= 360;
  if (err < -180) err += 360;
  while (err > 1 || err < -1) {
    if (interrupted) {
      stop_motors();
      interruptFunc();
    }
    lmotors((err > 0 ? BASE_TURN_SPEED : -BASE_TURN_SPEED));
    rmotors((err > 0 ? -BASE_TURN_SPEED : BASE_TURN_SPEED));
    err = deg - getBNO();
    if (err > 180) err -= 360;
    if (err < -180) err += 360;
    //Serial.print("Error: "); Serial.println(err);
  }
  //Serial.println("DONE");
  turning = false;
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
  switch (status) {
    case TRAVERSING: Serial.print("Traversing... "); break;
    case DANGERZONE: Serial.print("In Danger Zone... "); break;
    case BACKTRACKING: Serial.print("Backtracking... "); break;
    case FINISH: Serial.print("Finished! "); break;
    case END: Serial.print("Run over... "); break;
  }
  Serial.print("Robot at (");
  Serial.print(pos.x);
  Serial.print(",");
  Serial.print(pos.y);
  Serial.print(",");
  Serial.print(pos.z);
  Serial.print(") facing ");
  switch (facing) {
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