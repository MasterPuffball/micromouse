#include "MapRenderer.hpp"

MapRenderer::MapRenderer(U8G2 &display) : u8g2(display) {
    initBorderWalls();
    markVisited(robotX, robotY);
}


//void MapRenderer::setRobotPosition(uint8_t x, uint8_t y, uint8_t dir) {
//    robotX = x;
//    robotY = y;
//    robotDir = dir;
//}

void MapRenderer::initBorderWalls() {
    // top & bottom borders
    for (uint8_t x = 0; x < CELLS; ++x) {
        horizontalWalls[0][x]       = true;        // north of row 0
        horizontalWalls[CELLS][x]   = true;        // south of last row
    }
    // left & right borders
    for (uint8_t y = 0; y < CELLS; ++y) {
        verticalWalls[y][0]         = true;        // west of col 0
        verticalWalls[y][CELLS]     = true;        // east of last col
    }
}

void MapRenderer::setRobotPosition(uint8_t cellX, uint8_t cellY, uint8_t dir) {
    if (cellX >= CELLS) cellX = CELLS - 1;
    if (cellY >= CELLS) cellY = CELLS - 1;
    robotX = cellX;
    robotY = cellY;
    robotDir = (dir & 0x03);
    markVisited(robotX, robotY);
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
    u8g2.drawDisc(startX + x * dotSpacing, startY + y * dotSpacing, 0);
}

void MapRenderer::drawHLine(uint8_t x, uint8_t y) {
    u8g2.drawLine(x, y, x + dotSpacing, y);
}

void MapRenderer::drawVLine(uint8_t x, uint8_t y) {
    u8g2.drawLine(x, y, x, y + dotSpacing);
}

void MapRenderer::drawGrid() {
    for (uint8_t y = 0; y < NODES; y++) {
        for (uint8_t x = 0; x < NODES; x++) {
            drawDot(x, y);
        }
    }
}

void MapRenderer::drawWalls() const {
    // Horizontal edges: iterate node rows (ny) and cell-x (cx)
    for (uint8_t ny = 0; ny < NODES; ++ny) {
        for (uint8_t cx = 0; cx < CELLS; ++cx) {
            if (horizontalWalls[ny][cx]) {
                const int x0 = nodeX(cx);
                const int y0 = nodeY(ny);
                u8g2.drawHLine(x0, y0, CELLS);
            }
        }
    }
    // Vertical edges: iterate cell-y (cy) and node cols (nx)
    for (uint8_t cy = 0; cy < CELLS; ++cy) {
        for (uint8_t nx = 0; nx < NODES; ++nx) {
            if (verticalWalls[cy][nx]) {
                const int x0 = nodeX(nx);
                const int y0 = nodeY(cy);
                u8g2.drawVLine(x0, y0, CELLS);
            }
        }
    }
}

void MapRenderer::markVisited(uint8_t x, uint8_t y) {
    if (x >= CELLS || y >= CELLS) return;
    if (!visited[y][x]) {
        visited[y][x] = true;
        if (visited_cells_count < CELLS * CELLS) ++visited_cells_count;
    }
}

bool MapRenderer::isVisited(uint8_t x, uint8_t y) const {
    if (x >= CELLS || y >= CELLS) return true;
    return visited[y][x];
}

float MapRenderer::getCompletionPercentage() {
    return ((visited_cells_count) / (CELLS * CELLS)) * 100.0f;
}

void MapRenderer::drawCompletion() {
    u8g2.clearBuffer();
    drawMap();
    u8g2.setCursor(75, 60);
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.print(getCompletionPercentage(), 2);
    u8g2.print("%");
}

void MapRenderer::drawMap() {
    drawGrid();
    //drawWalls();
}