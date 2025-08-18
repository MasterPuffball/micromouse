#pragma once

#include <Arduino.h>
#include "Constants.h"
#include "math.h"

// For rendering, assume all far top and far left walls exist as they will be out of bounds 
// and aren't saved for storage optimisation purposes
namespace mtrn3100 {
class CompactMap {
public:
  CompactMap() {
    clear();
  }

  void clear() {
    for (int i = 0; i < NUM_MAP_BYTES; i++) {
      data[i] = 0;
    }
  }

  void set(int x, int y, bool value) {
    int idx = y * MAP_LENGTH + x;
    int byteIdx = idx / 8;
    int bitIdx = idx % 8;     // idx % 8

    if (value) {
      data[byteIdx] |= (1 << bitIdx);
    }
    else {
      data[byteIdx] &= ~(1 << bitIdx);
    }
  }

  bool get(int x, int y) const {
    int idx = y * MAP_LENGTH + x;
    int byteIdx = idx / 8;
    int bitIdx  = idx % 8;
    return (data[byteIdx] >> bitIdx) & 1;
  }

private:
  uint8_t data[NUM_MAP_BYTES];
};

class Map {
public:
  Map() {
    // Initialise has path to true everywhere (assume there are no walls in the maze)
    for (int i = 0; i < MAP_LENGTH; i++) {
      for (int j = 0; j < MAP_HEIGHT; j++) {
        hasPathRight.set(i, j, true);
        hasPathDown.set(i,j, true);
      }
    }	
  }
	
	void visitCell(int x, int y) {
		if (!visited.get(x, y)) {
			numVisited++;
      visited.set(x, y, true);
		}
	}

	void setWall(int x, int y, int direction) {
		if(direction == UP) {
			if (y <= 0) return;
      hasPathDown.set(x, y-1, false);
		}
		if(direction == RIGHT) {
			if (x >= MAP_LENGTH - 1) return;
      hasPathRight.set(x, y, false);
		}
		if(direction == DOWN) {
			if (y >= MAP_HEIGHT - 1) return;
      hasPathDown.set(x, y, false);
		}
		if (direction == LEFT) {
			if (x <= 0) return;
      hasPathRight.set(x-1, y, false);
		}
	}

	bool wallExists(int x, int y, int direction) {
		if(direction == UP) {
			if (y <= 0) return true;
      return !hasPathDown.get(x, y-1);
		}
		if(direction == RIGHT) {
			if (x >= MAP_LENGTH - 1) return true;
      return !hasPathRight.get(x, y);
		}
		if(direction == DOWN) {
			if (y >= MAP_HEIGHT - 1) return true;
      return !hasPathDown.get(x, y);
		}
		if (direction == LEFT) {
			if (x <= 0) return true;
      return !hasPathRight.get(x-1, y);
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
    return visited.get(x, y);
  }

	bool cellVisited(int x, int y, int direction) {
		if(direction == UP) {
			if (y <= 0) return true;
      return visited.get(x, y-1);
		}
		if(direction == RIGHT) {
			if (x >= MAP_LENGTH - 1) return true;
      return visited.get(x+1, y);
		}
		if(direction == DOWN) {
			if (y >= MAP_HEIGHT - 1) return true;
      return visited.get(x, y+1);
		}
		if (direction == LEFT) {
			if (x <= 0) return true;
      return visited.get(x-1, y);
		}

    return visited.get(x, y);
  }

	int getNumVisited() {
		return numVisited;
	}

private:
  CompactMap visited;
  CompactMap hasPathRight;
  CompactMap hasPathDown;
  uint8_t numVisited = 1;
};
}