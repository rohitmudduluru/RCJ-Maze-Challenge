#pragma once
#include <LittleFS.h>
#include "maze.h"

extern const char* filename;// = "/maze.txt";

void uploadMaze(Maze *m);
void downloadMaze(Maze* m);
void clearFile();