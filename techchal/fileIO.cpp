#include "fileIO.h"

const char *filename = "/maze.txt";

void uploadMaze(Maze *m) {
  File outfile = LittleFS.open(filename, "w");
  if (!outfile) {                         // if failed - maybe forgot to set the Filesystem in IDE
    // Serial.println("fail open outfile");  // or didn't call LittleFS.begin()
    // while (1)
    //   ;
    return;
  }
  Serial.println("Writing to file...");


  char buf[6];  // print robot
  // x,y,z,facing,status
  buf[0] = (char)(m->robot->pos.x);
  buf[1] = (char)(m->robot->pos.y);
  buf[2] = (char)(m->robot->pos.z);
  buf[3] = NORTH;//(char)(m->robot->facing);
  buf[4] = (char)(m->robot->status);
  outfile.write(buf, 5);

  buf[0] = m->maze.size();  // # of tiles
  outfile.write(buf, 1);
  for (std::pair<Point, Tile> tiles : m->maze) {  // print maze
    // x,y,z,Tile high byte,Tile low byte
    buf[0] = (char)(tiles.first.x);
    buf[1] = (char)(tiles.first.y);
    buf[2] = (char)(tiles.first.z);

    buf[3] = (char)(tiles.second >> 8);
    buf[4] = (char)(tiles.second);
    outfile.write(buf, 5);
  }

  buf[0] = m->rampConnections.size();  // # of connections
  outfile.write(buf, 1);
  for (std::pair<Point, Point> connection : m->rampConnections) {  // print connections
    // first point (x,y,z), second point (x,y,z)
    buf[0] = (char)(connection.first.x);
    buf[1] = (char)(connection.first.y);
    buf[2] = (char)(connection.first.z);

    buf[3] = (char)(connection.second.x);
    buf[4] = (char)(connection.second.y);
    buf[5] = (char)(connection.second.z);
    outfile.write(buf, 6);
  }
  outfile.close();
}
inline int charToSignedInt(char c) {
  return c > 100 ? c - 256 : c;
}
void downloadMaze(Maze *m) {
  File infile = LittleFS.open(filename, "r");
  if (!infile) {  // failed - file not there or forgot to create it
    return;
  }
  Serial.println("Reading from file...");
  if (!infile.available()) {
    Serial.println("Empty file...");
    return;
  }
  char buf[6];
  infile.readBytes(buf, 5);  //read robot
  m->robot->pos.x = charToSignedInt(buf[0]);
  m->robot->pos.y = charToSignedInt(buf[1]);
  m->robot->pos.z = charToSignedInt(buf[2]);
  m->robot->facing = (Direction)buf[3];
  m->robot->status = (Status)buf[4];

  infile.readBytes(buf, 1);           // # of tiles
  m->maze.clear();
  for (int i = buf[0]; i > 0; i--) {  // maze download
    infile.readBytes(buf, 5);
    Point p;
    p.x = charToSignedInt(buf[0]);
    p.y = charToSignedInt(buf[1]);
    p.z = charToSignedInt(buf[2]);
    m->maze[p] = ((short)buf[3] << 8) | buf[4];
  }

  infile.readBytes(buf, 1);           // # of connections
  m->rampConnections.clear();
  for (int i = buf[0]; i > 0; i--) {  // conections
    infile.readBytes(buf, 6);
    Point p1, p2;
    p1.x = charToSignedInt(buf[0]);
    p1.y = charToSignedInt(buf[1]);
    p1.z = charToSignedInt(buf[2]);
    p2.x = charToSignedInt(buf[3]);
    p2.y = charToSignedInt(buf[4]);
    p2.z = charToSignedInt(buf[5]);
    m->rampConnections[p1] = p2;
  }

  infile.close();
}

void clearFile(){
  File closefile = LittleFS.open(filename, "w");
  if (!closefile) {  // failed - file not there or forgot to create it
    Serial.println("Failed to open file");
    return;
  }
  Serial.println("Clearing file...");
  closefile.close();
}