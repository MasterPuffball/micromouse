#include "MapRenderer.hpp"

namespace mtrn3100 {
MapRenderer::MapRenderer(U8G2 &display) : u8g2(display) {}

//void MapRenderer::setRobotPosition(uint8_t x, uint8_t y, uint8_t dir) {
//    robotX = x;
//    robotY = y;
//    robotDir = dir;
//}

//void MapRenderer::initBorderWalls() {
//    // top & bottom borders
//    for (uint8_t x = 0; x < CELLS; ++x) {
//        horizontalWalls[0][x]       = true;
//        horizontalWalls[CELLS][x]   = true;
//    }
//    // left & right borders
//    for (uint8_t y = 0; y < CELLS; ++y) {
//        verticalWalls[y][0]         = true;
//        verticalWalls[y][CELLS]     = true;
//    }
//}

void MapRenderer::setRobotPosition(uint8_t cellX, uint8_t cellY, uint8_t dir) {
    if (cellX >= CELLS) cellX = CELLS - 1;
    if (cellY >= CELLS) cellY = CELLS - 1;
    robotX = cellX;
    robotY = cellY;
    robotDir = (dir & 0x03);
}

void MapRenderer::updateWall(bool horizontal, uint8_t index, uint8_t pos, bool isWall) {
    if (horizontal) {
        // index = node-row y (0..NODES-1), pos = cell-x (0..CELLS-1)
        if (index < NODES && pos < CELLS) {
            horizontalWalls[index][pos] = isWall;
        }
    } else {
        // index = node-col x (0..NODES-1), pos = cell-y (0..CELLS-1)
        if (index < NODES && pos < CELLS) {
            verticalWalls[pos][index] = isWall;
        }
    }
}


void MapRenderer::drawDot(uint8_t x, uint8_t y) {
    u8g2.drawDisc(startX + x * CELL_SIZE, startY + y * CELL_SIZE, 0);
}

void MapRenderer::drawHLine(uint8_t x, uint8_t y) {
    u8g2.drawLine(x, y, x + CELL_SIZE, y);
}

void MapRenderer::drawVLine(uint8_t x, uint8_t y) {
    u8g2.drawLine(x, y, x, y + CELL_SIZE);
}

void MapRenderer::drawGrid() {
    for (uint8_t y = 0; y < NODES; y++) {
        for (uint8_t x = 0; x < NODES; x++) {
            drawDot(x, y);
        }
    }
}

void MapRenderer::drawWalls() const {
  for (uint8_t y = 0; y < MAP_HEIGHT; ++y) {
    for (uint8_t x = 0; x < MAP_LENGTH; ++x) {
      // UP (north) edge of (x,y) → H line at node(x,y)
      if (map.wallExists(x, y, UP)) {
        const int x0 = nodeX(x);
        const int y0 = nodeY(y);
        drawSolidHLine(x0, y0);
      }
      // LEFT (west) edge of (x,y) → V line at node(x,y)
      if (map.wallExists(x, y, LEFT)) {
        const int x0 = nodeX(x);
        const int y0 = nodeY(y);
        drawSolidVLine(x0, y0);
      }
      // RIGHT edge for last column (x == MAP_LENGTH-1) → V line at node(x+1,y)
      if (x == MAP_LENGTH - 1 && map.wallExists(x, y, RIGHT)) {
        const int x0 = nodeX(x + 1);
        const int y0 = nodeY(y);
        drawSolidVLine(x0, y0);
      }
      // DOWN edge for last row (y == MAP_HEIGHT-1) → H line at node(x,y+1)
      if (y == MAP_HEIGHT - 1 && map.wallExists(x, y, DOWN)) {
        const int x0 = nodeX(x);
        const int y0 = nodeY(y + 1);
        drawSolidHLine(x0, y0);
      }
    }
  }
}

//void MapRenderer::markVisited(uint8_t x, uint8_t y) {
//    if (x >= CELLS || y >= CELLS) return;
//    if (!visited[y][x]) {
//        visited[y][x] = true;
//        if (visited_cells_count < CELLS * CELLS) ++visited_cells_count;
//    }
//}

//bool MapRenderer::isVisited(uint8_t x, uint8_t y) const {
//    if (x >= CELLS || y >= CELLS) return true;
//   return visited[y][x];
//}

float MapRenderer::getCompletionPercentage() {
    return ((visited_cells_count) / (CELLS * CELLS)) * 100.0f;
}

void MapRenderer::drawCompletion() {
    u8g2.setCursor(75, 60);
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.print(getCompletionPercentage(), 2);
    u8g2.print("%");
}

void MapRenderer::drawMap() {
    u8g2.clearBuffer();
    drawGrid();
    drawWalls();
	drawCompletion();
}

}
