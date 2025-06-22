#include "maze.h"

/*
 * Return if there is a wall in between the tile at point p and tile in direction d
 *
 * @param Point p and direction d
 * @return Bool of whether there is a wall or not
 */
bool Maze::hasWall(Point p, Direction d) {
  switch (d) {
    case NORTH:
      return (maze[p] & NWALL) | (maze[nextPoint(p, d)] & SWALL);
    case SOUTH:
      return (maze[p] & SWALL) | (maze[nextPoint(p, d)] & NWALL);
    case EAST:
      return (maze[p] & EWALL) | (maze[nextPoint(p, d)] & WWALL);
    case WEST:
      return (maze[p] & WWALL) | (maze[nextPoint(p, d)] & EWALL);
  }
  return 1;
}

/*
 * Return if there is a ramp going in direction d
 *
 * @param Point p and direction d
 * @return Bool of whether there is a ramp or not
 */
#ifdef RAMP_ON
bool Maze::hasRamp(Point p, Direction d) {
  switch (d) {
    case NORTH:
      return maze[p] & NRAMP;
    case SOUTH:
      return maze[p] & SRAMP;
    case EAST:
      return maze[p] & ERAMP;
    case WEST:
      return maze[p] & WRAMP;
  }
  return 0;
}
#else
bool Maze::hasRamp(Point p, Direction d) {
  return 0;
}
#endif

Maze::Maze(Robot* r) {
  robot = r;
}

/*
 * Find the next closest unvisited tile and return the directions to it
 *
 * @param None
 * @return Vector of the cardinal directions needed to get to the next unvisited tile
 */
std::vector<Direction> Maze::findNextMove() {
  std::map<Point, Point, PointCmp> paths;  // Paths back to original position from any point
  std::map<Point, bool, PointCmp> bfsVisited;
  bfsVisited[robot->_pos] = 1;
  std::queue<Point> q;  // BFS queue
  q.push(robot->_pos);
  std::vector<Direction> pathBack(0);  // Returned path
  while (!q.empty()) {
    Point p = q.front();
    q.pop();
    if (!(maze[p] & VISITED)) {  // Found closest unvisited tile - find path back
      Point prev = p;
      Point trace = paths[p];
      std::stack<Direction> s;
      for (;;) {
        if (prev.y > trace.y) s.push(NORTH);
        else if (prev.y < trace.y) s.push(SOUTH);
        else if (prev.x > trace.x) s.push(EAST);
        else if (prev.x < trace.x) s.push(WEST);
        if (samePoint(trace, robot->_pos)) break;
        prev = trace;
        trace = paths[trace];
      }
      while (!s.empty()) {
        pathBack.push_back(s.top());
        s.pop();
      }
      break;
    }
    printPoint(p);
    Serial.print(" connects to -> ");
    Point next = nextPoint(p, robot->_facing);
    if (hasRamp(next, robot->_facing) || hasRamp(next, (Direction)((robot->_facing + 2) % 4)))  //check if ramp exists
      next = rampConnections[next];
    if (!hasWall(p, robot->_facing) && !bfsVisited[next] && !(maze[next] & BLACKPOINT)) {  // Check if the tile infront is valid
      if (!(robot->_status == TRAVERSING && maze[next] & REDPOINT)) {                      // Check if tile is in dangerzone
        q.push(next);
        printPoint(next);
        if (paths.count(next) == 0) paths[next] = p;
        bfsVisited[next] = 1;
      }
    }

    next = nextPoint(p, (Direction)((robot->_facing + 3) % 4));
    if (hasRamp(next, (Direction)((robot->_facing + 3) % 4)) || hasRamp(next, (Direction)((robot->_facing + 1) % 4)))
      next = rampConnections[next];
    if (!hasWall(p, (Direction)((robot->_facing + 3) % 4)) && !bfsVisited[next] && !(maze[next] & BLACKPOINT)) {
      if (!(robot->_status == TRAVERSING && maze[next] & REDPOINT)) {
        q.push(next);
        printPoint(next);
        if (paths.count(next) == 0) paths[next] = p;
        bfsVisited[next] = 1;
      }
    }
    next = nextPoint(p, (Direction)((robot->_facing + 1) % 4));
    if (hasRamp(next, (Direction)((robot->_facing + 1) % 4)) || hasRamp(next, (Direction)((robot->_facing + 3) % 4)))
      next = rampConnections[next];
    if (!hasWall(p, (Direction)((robot->_facing + 1) % 4)) && !bfsVisited[next] && !(maze[next] & BLACKPOINT)) {
      if (!(robot->_status == TRAVERSING && maze[next] & REDPOINT)) {
        q.push(next);
        printPoint(next);
        if (paths.count(next) == 0) paths[next] = p;
        bfsVisited[next] = 1;
      }
    }
    next = nextPoint(p, (Direction)((robot->_facing + 2) % 4));
    if (hasRamp(next, (Direction)((robot->_facing + 2) % 4)) || hasRamp(next, robot->_facing))
      next = rampConnections[next];
    if (!hasWall(p, (Direction)((robot->_facing + 2) % 4)) && !bfsVisited[next] && !(maze[next] & BLACKPOINT)) {
      if (!(robot->_status == TRAVERSING && maze[next] & REDPOINT)) {
        q.push(next);
        printPoint(next);
        if (paths.count(next) == 0) paths[next] = p;
        bfsVisited[next] = 1;
      }
    }
    Serial.println();
  }
  if (pathBack.empty()) {
    if (robot->_status == TRAVERSING) {
      robot->_status = DANGERZONE;  // No more tiles outside danger zone - start traversing danger zone
      return findNextMove();
    } else
      robot->_status = BACKTRACKING;  // No more tiles to traverse in entire maze - start backtracking
  }
  return pathBack;
}

/*
 * Find the origin and return the directions to it
 *
 * @param None
 * @return Vector of the cardinal directions needed to get to the next unvisited tile
 */
std::vector<Direction> Maze::findOrigin() {
  std::map<Point, Point, PointCmp> paths;  // Paths back to original position from any point
  std::map<Point, bool, PointCmp> bfsVisited;
  bfsVisited[robot->_pos] = 1;
  std::queue<Point> q;  // BFS queue
  q.push(robot->_pos);
  std::vector<Direction> pathBack(0);  // Returned path
  while (!q.empty()) {
    Point p = q.front();
    q.pop();
    if (p.x == 0 && p.y == 0) {  // Found origin - find path back
      Point prev = p;
      Point trace = paths[p];
      std::stack<Direction> s;
      for (;;) {
        if (prev.y > trace.y) s.push(NORTH);
        else if (prev.y < trace.y) s.push(SOUTH);
        else if (prev.x > trace.x) s.push(EAST);
        else if (prev.x < trace.x) s.push(WEST);
        if (samePoint(trace, robot->_pos)) break;
        prev = trace;
        trace = paths[trace];
      }
      while (!s.empty()) {
        pathBack.push_back(s.top());
        s.pop();
      }
      break;
    }

    Point next = nextPoint(p, robot->_facing);
    if (hasRamp(next, robot->_facing) || hasRamp(next, (Direction)((robot->_facing + 2) % 4)))  //check if ramp exists
      next = rampConnections[next];
    if (!hasWall(p, robot->_facing) && !bfsVisited[next] && !(maze[next] & BLACKPOINT)) {  // Check if the tile infront is valid
      q.push(next);
      if (paths.count(next) == 0) paths[next] = p;
      bfsVisited[next] = 1;
    }
    next = nextPoint(p, (Direction)((robot->_facing + 1) % 4));
    if (hasRamp(next, (Direction)((robot->_facing + 1) % 4)) || hasRamp(next, (Direction)((robot->_facing + 3) % 4)))
      next = rampConnections[next];
    if (!hasWall(p, (Direction)((robot->_facing + 1) % 4)) && !bfsVisited[next] && !(maze[next] & BLACKPOINT)) {
      q.push(next);
      if (paths.count(next) == 0) paths[next] = p;
      bfsVisited[next] = 1;
    }
    next = nextPoint(p, (Direction)((robot->_facing + 3) % 4));
    if (hasRamp(next, (Direction)((robot->_facing + 3) % 4)) || hasRamp(next, (Direction)((robot->_facing + 1) % 4)))
      next = rampConnections[next];
    if (!hasWall(p, (Direction)((robot->_facing + 3) % 4)) && !bfsVisited[next] && !(maze[next] & BLACKPOINT)) {
      q.push(next);
      if (paths.count(next) == 0) paths[next] = p;
      bfsVisited[next] = 1;
    }
    next = nextPoint(p, (Direction)((robot->_facing + 2) % 4));
    if (hasRamp(next, (Direction)((robot->_facing + 2) % 4)) || hasRamp(next, robot->_facing))
      next = rampConnections[next];
    if (!hasWall(p, (Direction)((robot->_facing + 2) % 4)) && !bfsVisited[next] && !(maze[next] & BLACKPOINT)) {
      q.push(next);
      if (paths.count(next) == 0) paths[next] = p;
      bfsVisited[next] = 1;
    }
  }
  return pathBack;
}

/*
 * Update the walls of the current tile the robot is on
 *
 * @param None
 * @return None
 */
void Maze::updateTile() {
  bool wallN, wallS, wallE, wallW;
  Serial.println("getting vals");
  switch (robot->_facing) {
    case NORTH:
      wallN = readTOF(FRONT_TOF) < MIN_DIST;
      wallS = readTOF(BACK_TOF) < MIN_DIST;
      wallE = readTOF(RIGHT_TOF) < MIN_DIST;
      wallW = readTOF(LEFT_TOF) < MIN_DIST;
      break;
    case SOUTH:
      wallS = readTOF(FRONT_TOF) < MIN_DIST;
      wallN = readTOF(BACK_TOF) < MIN_DIST;
      wallW = readTOF(RIGHT_TOF) < MIN_DIST;
      wallE = readTOF(LEFT_TOF) < MIN_DIST;
      break;
    case EAST:
      wallE = readTOF(FRONT_TOF) < MIN_DIST;
      wallW = readTOF(BACK_TOF) < MIN_DIST;
      wallS = readTOF(RIGHT_TOF) < MIN_DIST;
      wallN = readTOF(LEFT_TOF) < MIN_DIST;
      break;
    case WEST:
      wallW = readTOF(FRONT_TOF) < MIN_DIST;
      wallE = readTOF(BACK_TOF) < MIN_DIST;
      wallN = readTOF(RIGHT_TOF) < MIN_DIST;
      wallS = readTOF(LEFT_TOF) < MIN_DIST;
      break;
  }
  Serial.println("got vals");
  if (wallN) {
    maze[robot->_pos] |= NWALL;
    maze[nextPoint(robot->_pos, NORTH)] |= SWALL;
  }
  if (wallS) {
    maze[robot->_pos] |= SWALL;
    maze[nextPoint(robot->_pos, SOUTH)] |= NWALL;
  }
  if (wallW) {
    maze[robot->_pos] |= WWALL;
    maze[nextPoint(robot->_pos, WEST)] |= EWALL;
  }
  if (wallE) {
    maze[robot->_pos] |= EWALL;
    maze[nextPoint(robot->_pos, EAST)] |= WWALL;
  }
  maze[robot->_pos] |= VISITED;
}

void Maze::AddRamp(Point p, Direction d) {
  switch (d) {
    case NORTH:
      maze[p] |= NRAMP;
      break;
    case SOUTH:
      maze[p] |= SRAMP;
      break;
    case EAST:
      maze[p] |= ERAMP;
      break;
    case WEST:
      maze[p] |= WRAMP;
      break;
  }
}

void Maze::AddWall(Point p, Direction d) {
  Point next = nextPoint(p, d);
  switch (d) {
    case NORTH:
      maze[p] |= NWALL;
      maze[next] |= SWALL;
      break;
    case SOUTH:
      maze[p] |= SWALL;
      maze[next] |= NWALL;
      break;
    case EAST:
      maze[p] |= EWALL;
      maze[next] |= WWALL;
      break;
    case WEST:
      maze[p] |= WWALL;
      maze[next] |= EWALL;
      break;
  }
}

void printBits(int c, int amt) {
  for (int i = amt - 1; i >= 0; i--) {
    Serial.print((c >> i) & 1);
    if (!(i % 4)) Serial.print(" ");
  }
}
void printMaze(Maze* m) {
  Serial.println("\nPRINTING MAZE...");
  Serial.println("Tiles:");
  if (m->maze.empty()) Serial.println("EMPTY");
  for (std::pair<Point, Tile> tiles : m->maze) {
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
  if (m->rampConnections.empty()) Serial.println("EMPTY");
  for (std::pair<Point, Point> connection : m->rampConnections) {
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