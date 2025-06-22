#include "maze.h"
#include "Wire.h"
#include "fileIO.h"
Robot robot;
Maze maze(&robot);

#define RAMP_ON
#define CAM_ON
#define OBSTACLE_ON
// #define OLD_BOT
#define NEW_BOT

void printBits(int c, int amt) {
  for (int i = amt - 1; i >= 0; i--) {
    Serial.print((c >> i) & 1);
    if (!(i % 4)) Serial.print(" ");
  }
}
void printMaze(Maze m) {
  Serial.println("\nTiles:");
  for (std::pair<Point, Tile> tiles : m.maze) {
    Serial.print("(");
    Serial.print(tiles.first.x);
    Serial.print(",");
    Serial.print(tiles.first.y);
    Serial.print(",");
    Serial.print(tiles.first.z);
    Serial.print("): ");
    printBits(tiles.second, 16);
    Serial.println();
  }
  Serial.println("Connections:");
  for (std::pair<Point, Point> connection : maze.rampConnections) {
    Serial.print("(");
    Serial.print(connection.first.x);
    Serial.print(",");
    Serial.print(connection.first.y);
    Serial.print(",");
    Serial.print(connection.first.z);
    Serial.print(") -> ");
    Serial.print("(");
    Serial.print(connection.second.x);
    Serial.print(",");
    Serial.print(connection.second.y);
    Serial.print(",");
    Serial.print(connection.second.z);
    Serial.println(")");
  }
  Serial.println();
}

void setup() {

  //limit switch init
  pinMode(RIGHT_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LEFT_LIMIT_SWITCH_PIN, INPUT_PULLUP);


  Serial.begin(9600);
  Serial1.begin(9600);
  delay(500);
  //while(!Serial);
  Serial.println("Serial Connected");
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  //file io init
  LittleFS.begin();
  Serial.println(" file system ready");
  while (digitalRead(20) == HIGH) {
    if (digitalRead(21) != HIGH) {
      // clearFile();
      break;
    }
  }
  // printMaze(maze);

  //pi turn on
  commBegin();  //serial init
  Serial.println(" comm ready");
  Serial1.print("b");

  //enc init
  pinMode(ENC_PIN, INPUT);
  pinMode(ENC_PIN_INTER, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_INTER), enc_update, RISING);

  //i2c init
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();

  Wire1.setSCL(SCL_PIN1);
  Wire1.setSDA(SDA_PIN1);
  Wire1.begin();

  Serial.println(" i2c ready");

  //color sensor init
  colorBegin();
  Serial.println(" color ready");




  //imu init
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  //tof init
  tofInit();
  Serial.println(" tof ready");


  Serial.println(" led ready");




  //servo init
  servo.attach(SERVO_PIN);
  Serial.println(" servo ready");

  //stepper init
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  Serial.println(" stepper ready");

  //led init
  ledInit();

  pinMode(BUZZER, OUTPUT);
  tone(BUZZER, 400, 200);

  // downloadMaze(&maze);
  // printMaze(maze);
  // robot.print();
  // maze.updateTile();
  enc = 0;
  Serial.println("Ready to start");
}
enum NewDirection {
  FORWARD = 1,
  LEFT = 2,
  RIGHT = 3
};
enum NewStatus {
  FINDING = 0,
  INROOM = 1,
  GOBACK = 2,
  DONE = 3,
  NOCODE = 4
};
std::stack<NewDirection> pathToRoom;
NewStatus robotStatus = FINDING;
Direction robotFacing = NORTH;
Direction enterFacing = NORTH;
void printPath(){
  std::stack<NewDirection> temp = pathToRoom;
  while(!temp.empty()){
    if(temp.top()==FORWARD) Serial.println("Forward,");
    if(temp.top()==LEFT) Serial.println("Left,");
    if(temp.top()==RIGHT) Serial.println("Right,");
    temp.pop();
  }
}
int iter = 0;
void loop() {

  // std::vector<Direction> directions = {NORTH, NORTH, EAST, EAST, SOUTH, SOUTH, WEST, WEST};
  // robot.moveDirections(directions);
  // Serial.println(getBNO());
  // Serial.println(getColor());
  // printTOFs();
  // forward(50);
  //printColorSensorData();
    //   while(1){
    //   switch((getColor())){
    //     case WHITE: Serial.println(" WHITE"); break;
    //     case BLACK: Serial.println(" BLACK"); break;
    //     case BLUE: Serial.println(" BLUE"); break;
    //     case RED: Serial.println(" RED"); break;
    //     case SILVER: Serial.println(" SILVER"); break;
    //     case UNK: Serial.println(" UNKNOWN"); break;
    //     // default: Serial.print(" confusion "); Serial.println(temp);
    //   }
    // }
Serial.print("\nRobot facing: "); Serial.println(robot.facing);
printPath();

  switch (robotStatus) {
    case FINDING:
      {
        if (readTOF(FRONT_TOF) > MIN_DIST) {
          Serial.println("forward...");
          switch (robot.moveRobot(robot.facing)) {
            case REDTILE:  //right
              Serial.println("Saw red");
              pathToRoom.push(FORWARD);
              if (readTOF(RIGHT_TOF)>15) {
                Serial.println("Going Right");
                robot.facing = (Direction)(((int)robot.facing + 1) % 4);
                robot.turn_to(directionAngle(robot.facing));
                pathToRoom.push(RIGHT);
              } else {
                Serial.println("Going Left");
                robot.facing = (Direction)(((int)robot.facing + 3) % 4);
                robot.turn_to(directionAngle(robot.facing));
                pathToRoom.push(LEFT);
              }
              break;
            case BLUETILE:  //left
              Serial.println("Saw blue");
              pathToRoom.push(FORWARD);
              if (readTOF(LEFT_TOF)>15) {
                Serial.println("Going Left");
                robot.facing = (Direction)(((int)robot.facing + 3) % 4);
                robot.turn_to(directionAngle(robot.facing));
                pathToRoom.push(LEFT);
              } else {
                Serial.println("Going Right");
                robot.facing = (Direction)(((int)robot.facing + 1) % 4);
                robot.turn_to(directionAngle(robot.facing));
                pathToRoom.push(RIGHT);
              }
              break;
            case SILVERTILE:
              pathToRoom.push(FORWARD);
              robotStatus = INROOM;
              enterFacing = robot.facing;
              Serial.println("\n\nAT ROOM\n");
              break;
            case STAIRS:
            case RAMP:
              {
                pathToRoom.push(FORWARD);
                if (getColor() == RED) {
                  tone(BUZZER, 600, 500);
                  if (readTOF(RIGHT_TOF)>15) {
                    Serial.println("Going Right");
                    robot.facing = (Direction)(((int)robot.facing + 1) % 4);
                    robot.turn_to(directionAngle(robot.facing));
                    pathToRoom.push(RIGHT);
                  } else {
                    Serial.println("Going Left");
                    robot.facing = (Direction)(((int)robot.facing + 3) % 4);
                    robot.turn_to(directionAngle(robot.facing));
                    pathToRoom.push(LEFT);
                  }
                } else if (getColor() == BLUE) {
                  tone(BUZZER, 400, 50);
                  delay(50);
                  tone(BUZZER, 500, 50);
                  delay(50);
                  tone(BUZZER, 600, 200);
                  if (readTOF(LEFT_TOF)>15) {
                    Serial.println("Going Left");
                    robot.facing = (Direction)(((int)robot.facing + 3) % 4);
                    robot.turn_to(directionAngle(robot.facing));
                    pathToRoom.push(LEFT);
                  } else {
                    Serial.println("Going Right");
                    robot.facing = (Direction)(((int)robot.facing + 1) % 4);
                    robot.turn_to(directionAngle(robot.facing));
                    pathToRoom.push(RIGHT);
                  }
                }
                break;
              }
            case GOOD:
            default:
              pathToRoom.push(FORWARD);
              break;
          }

        } else if (readTOF(LEFT_TOF) > 15) {
          Serial.println("Going Left");
          robot.facing = (Direction)(((int)robot.facing + 3) % 4);
          robot.turn_to(directionAngle(robot.facing));
          pathToRoom.push(LEFT);
        } else if (readTOF(RIGHT_TOF) > 15) {
          Serial.println("Going Right");
          robot.facing = (Direction)(((int)robot.facing + 1) % 4);
          robot.turn_to(directionAngle(robot.facing));
          pathToRoom.push(RIGHT);
        }
        break;
      }
    case INROOM:
      {
        if (readTOF(FRONT_TOF) > MIN_DIST) {
          switch (robot.moveRobot(robot.facing)) {
            case SILVERTILE:
              robotStatus = GOBACK;
              robot.facing = (Direction)(((int)enterFacing + 2) % 4);
              robot.turn_to(robot.facing);
              Serial.println("\n\nDONE WITH ROOM\n");
              break;
          }

        } else if (readTOF(LEFT_TOF) > 15) {
          robot.facing = (Direction)(((int)robot.facing + 3) % 4);
          robot.turn_to(directionAngle(robot.facing));
        } else if (readTOF(RIGHT_TOF) > 15) {
          robot.facing = (Direction)(((int)robot.facing + 1) % 4);
          robot.turn_to(directionAngle(robot.facing));
        }
        break;
      }
    case GOBACK:
      {
        if (pathToRoom.empty()) {
          robotStatus = DONE;
          Serial.println("\n\nDONE\n");
          break;
        }
        switch (pathToRoom.top()) {
          case FORWARD:
            if (robot.moveRobot(robot.facing) == STAIRS)
              pathToRoom.pop();
            break;
          case LEFT:
            robot.facing = (Direction)(((int)robot.facing + 1) % 4);
            robot.turn_to(directionAngle(robot.facing));
            break;
          case RIGHT:
            robot.facing = (Direction)(((int)robot.facing + 3) % 4);
            robot.turn_to(directionAngle(robot.facing));
            break;
        }
        pathToRoom.pop();
        break;
      }
    case DONE:
      {
        stop_motors();
        blink(5, 1000);
        robotStatus = NOCODE;
        break;
      }
    case NOCODE: break;
  }
    // forward(100);
    // forward(60); delay(1000);
    // int x = getBNO();
    // while(abs(getBNO) < x + 90) {
    //  lmotors(70);
    //  rmotors(0);
    //  delay(150);
    //  stop_motors();
    //  delay(300);
    //  rmotors(-55);
    //  lmotors(0);
    //  delay(150);
    //  stop_motors();
    //  delay(300);
    // }
    // stop_motors();
    // delay(1000);
    // backward(60); delay(1000);
    //   Serial.println("foward...");
    //   forward(30);
    //   // delay(250);


    // drop(true);
    // drop(false);
    // int temp;




#ifdef CAM_ON
  if (interrupted) {
    interruptFunc();
  }
#endif

  // printTOFs();


  // Serial.print("\nAt point ");
  // printPoint(robot.pos);
  // switch (robot.status) {
  //   case TRAVERSING:
  //   case DANGERZONE:
  //     switch (robot.moveDirections(maze.findNextMove())) {
  //       case NOMOVES:
  //         if (robot.status == DANGERZONE) robot.status = BACKTRACKING;
  //         Serial.println("Can't find moves");
  //         break;
  //       case BLACKTILE:
  //         maze.maze[robot.pos] |= BLACKPOINT;
  //         robot.pos = nextPoint(robot.pos, (Direction)((robot.facing + 2) % 4));  // return robot's position
  //         break;
  //       case REDTILE:
  //         maze.maze[robot.pos] |= REDPOINT;
  //         robot.pos = nextPoint(robot.pos, (Direction)((robot.facing + 2) % 4));  // return robot's position
  //         break;
  //       case RAMP:
  //         {
  //           Point flatExit = nextPoint(robot.pos, robot.facing, rampTilesForward);
  //           if (incline) flatExit.z++;
  //           else flatExit.z--;
  //           Point finalRamp = nextPoint(flatExit, (Direction)((robot.facing + 2) % 4));
  //           maze.rampConnections[robot.pos] = flatExit;
  //           maze.rampConnections[finalRamp] = nextPoint(robot.pos, (Direction)((robot.facing + 2) % 4));
  //           maze.AddRamp(finalRamp, (Direction)((robot.facing + 2) % 4));
  //           maze.AddRamp(robot.pos, robot.facing);
  //           maze.AddWall(robot.pos, (Direction)((robot.facing + 1) % 4));
  //           maze.AddWall(robot.pos, (Direction)((robot.facing + 3) % 4));
  //           maze.AddWall(finalRamp, (Direction)((robot.facing + 1) % 4));
  //           maze.AddWall(finalRamp, (Direction)((robot.facing + 3) % 4));
  //           robot.pos = flatExit;
  //           maze.updateTile();
  //           break;
  //         }
  //       case SILVERTILE:
  //         maze.updateTile();
  //         Serial.println("\nUploading...");
  //         printMaze(maze);
  //         uploadMaze(&maze);
  //         break;
  //       case GOOD:
  //         maze.updateTile();
  //         break;
  //     }
  //     break;
  //   case BACKTRACKING:
  //     Serial.println("Backtracking");
  //     stop_motors();
  //     delay(5000);
  //     robot.moveDirections(maze.findOrigin());
  //     robot.status = FINISH;
  //     break;
  //   case FINISH:
  //     stop_motors();
  //     blink(5, 1000);
  //     robot.status = END;
  //   case END: break;
  // }
  // Serial.print("Ended at point ");
  // printPoint(robot.pos);



  // printColorSensorData();

  //blink();
  //testForward();
  // delay(175);
  // stop_motors();
  // delay(100);
  // rmotors(75);
  // delay(175);
  // stop_motors();
  // forward(60);
  // delay(20);
  // stop_motors();
  // delay(300);

  // lmotors(100);
  // rmotors(100);

  //printEncs();
}

// void setup1(){

// }
// void loop1(){

// }
