#pragma once
#include <U8g2lib.h>
#include "Constants.h"
#include "Map.hpp"

namespace mtrn3100 {
class MapRenderer {
public:
  MapRenderer(U8G2 &display, Map &map) : u8g2(display), map(map) {}

  void drawMap() {
    u8g2.clearBuffer();

    u8g2.firstPage();
    do {
      drawGrid();
      drawWalls();
      drawCompletion();
    } while (u8g2.nextPage());
  }

  void drawGrid() {
    for (uint8_t x = 0; x < MAP_LENGTH + 1; x++) {
      for (uint8_t y = 0; y < MAP_HEIGHT + 1; y++) {
        u8g2.drawDisc(getXCoord(x), getYCoord(y), 0);
      }
    }
  }

  int getXCoord(uint8_t node) {
    return CURSOR_START_X + node * CELL_SIZE;
  }

  int getYCoord(uint8_t node) {
    return CURSOR_START_Y + node * CELL_SIZE;
  }

  void drawWalls() const {
    for (uint8_t x = 0; x < MAP_LENGTH; ++x) {
      for (uint8_t y = 0; y < MAP_HEIGHT; ++y) {
        // north
        if (map.wallExists(x, y, UP)) {
          u8g2.drawHLine(getXCoord(x), getYCoord(y), CELL_SIZE);
        }
        // west
        if (map.wallExists(x, y, LEFT)) {
          u8g2.drawVLine(getXCoord(x), getYCoord(y), CELL_SIZE);
        }
        // right
        if (map.wallExists(x, y, RIGHT)) {
          u8g2.drawVLine(getXCoord(x+1), getYCoord(y), CELL_SIZE);
        }
        // south
        if (map.wallExists(x, y, DOWN)) {
          u8g2.drawHLine(getXCoord(x), getYCoord(y+1), CELL_SIZE);
        }
      }
    }
  }

  float getCompletionPercentage() {
    return ((float)map.getNumVisited() / ((MAP_HEIGHT*MAP_LENGTH) - NON_EXISTENT_CELLS)) * 100.0f;
  }

  void drawCompletion() {
    u8g2.setCursor(75, 60);
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.print(getCompletionPercentage(), 2);
    u8g2.print("%");
  }

private:
    U8G2 &u8g2;
    mtrn3100::Map &map;
};
}