#pragma once

#include <Arduino.h>
#include "Constants.h"
#include "math.h"

// For rendering, assume all far top and far left walls exist as they will be out of bounds 
// and aren't saved for storage optimisation purposes
namespace mtrn3100 {
class Compact2D {
};
class Map {
public:
  Map() {
    // Initialise visited
    for (int i = 0; i < MAP_LENGTH; i++) {
      for (int j = 0; j < MAP_HEIGHT; j++) {
        visited[i][j] = false;
      }
    }

    // Initialise has path to true everywhere (assume there are no walls in the maze)
    for (int i = 0; i < MAP_LENGTH - 1; i++) {
      for (int j = 0; j < MAP_HEIGHT - 1; j++) {
        hasPath[i][j][0] = true;
        hasPath[i][j][1] = true;
      }
    }	
  }
	
	void visitCell(int x, int y) {
		if (!visited[x][y]) {
			numVisited++;
			visited[x][y] = true;
		}
	}

	void setWall(int x, int y, int direction) {
		if(direction == UP) {
			if (y <= 0) return;
			hasPath[x][y-1][1] = false; 
		}
		if(direction == RIGHT) {
			if (x >= MAP_LENGTH - 1) return;
			hasPath[x+1][y][0] = false;
		}
		if(direction == DOWN) {
			if (y >= MAP_HEIGHT - 1) return;
			hasPath[x][y+1][1] = false;
		}
		if (direction == LEFT) {
			if (x <= 0) return;
			hasPath[x-1][y][0] = false;
		}
	}

	bool wallExists(int x, int y, int direction) {
		if(direction == UP) {
			if (y <= 0) return true;
			return !hasPath[x][y-1][1]; 
		}
		if(direction == RIGHT) {
			if (x >= MAP_LENGTH - 1) return true;
			return !hasPath[x+1][y][0];
		}
		if(direction == DOWN) {
			if (y >= MAP_HEIGHT - 1) return true;
			return !hasPath[x][y+1][1];
		}
		if (direction == LEFT) {
			if (x <= 0) return true;
			return !hasPath[x-1][y][0];
		}

    return false;
	}

	bool wallExists(int x1, int y1, int x2, int y2) {
		if (x1 != x2 && y1 != y2) return false; // Invalid input
		int direction = UP;

		if (x1 < x2) {
			direction = RIGHT;
		}
		else if (x1 > x2) {
			direction = LEFT;
		}
		else if (y1 < y2) {
			direction = DOWN;
		}

		return wallExists(x1, y1, direction);
	}

	bool cellVisited(int x, int y) {
    return visited[x][y];
  }

	bool cellVisited(int x, int y, int direction) {
		if(direction == UP) {
			if (y <= 0) return true;
			return visited[x][y-1]; 
		}
		if(direction == RIGHT) {
			if (x >= MAP_LENGTH - 1) return true;
			return visited[x+1][y];
		}
		if(direction == DOWN) {
			if (y >= MAP_HEIGHT - 1) return true;
			return visited[x][y+1];
		}
		if (direction == LEFT) {
			if (x <= 0) return true;
			return visited[x-1][y];
		}

    return visited[x][y];
  }

	int getNumVisited() {
		return numVisited;
	}

private:
  bool visited[MAP_LENGTH][MAP_HEIGHT] = {false};
  bool hasPath[MAP_LENGTH][MAP_HEIGHT][2] = {false}; // adjacency list, right and down from node
  int numVisited = 1;
};
}