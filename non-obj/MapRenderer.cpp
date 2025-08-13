#include "MapRenderer.hpp"

MapRenderer::MapRenderer(U8G2 &display) : u8g2(display) {}


void MapRenderer::setRobotPosition(uint8_t x, uint8_t y, uint8_t dir) {
    robotX = x;
    robotY = y;
    robotDir = dir;
}

void MapRenderer::drawDot(uint8_t x, uint8_t y) {
    u8g2.drawDisc(startX + x * dotSpacing, startY + y * dotSpacing, 0);
}

void MapRenderer::drawGrid() {
    for (uint8_t y = 0; y < gridSize; y++) {
        for (uint8_t x = 0; x < gridSize; x++) {
            drawDot(x, y);
        }
    }
}

void MapRenderer::drawWalls() {
    // Draw horizontal walls
    for (uint8_t y = 0; y < gridSize; ++y) {
        for (uint8_t x = 0; x < gridSize - 1; ++x) {
            if (horizontalWalls[y][x]) {
                u8g2.drawHLine(startX + x * dotSpacing, startY + y * dotSpacing, dotSpacing);
            }
        }
    }
    // Draw vertical walls
    for (uint8_t y = 0; y < gridSize - 1; ++y) {
        for (uint8_t x = 0; x < gridSize; ++x) {
            if (verticalWalls[y][x]) {
                u8g2.drawVLine(startX + x * dotSpacing, startY + y * dotSpacing, dotSpacing);
            }
        }
    }
}

void MapRenderer::updateWalls(bool wallFront, bool wallLeft, bool wallRight) {
    // Mark visited
    if (!mazeVisited[robotY][robotX]) {
        mazeVisited[robotY][robotX] = true;
        visited_cells_count++;
    }
    // Orientation-based mapping
    // 0=up, 1=right, 2=down, 3=left
    if (robotDir == 0) { // facing up
        if (wallFront) horizontalWalls[robotY][robotX] = true;           // north wall
        if (wallLeft)  verticalWalls[robotY][robotX] = true;             // west wall
        if (wallRight) verticalWalls[robotY][robotX+1] = true;           // east wall
    }
    else if (robotDir == 1) { // facing right
        if (wallFront) verticalWalls[robotY][robotX+1] = true;           // east wall
        if (wallLeft)  horizontalWalls[robotY][robotX] = true;           // north wall
        if (wallRight) horizontalWalls[robotY+1][robotX] = true;         // south wall
    }
    else if (robotDir == 2) { // facing down
        if (wallFront) horizontalWalls[robotY+1][robotX] = true;         // south wall
        if (wallLeft)  verticalWalls[robotY][robotX+1] = true;           // east wall
        if (wallRight) verticalWalls[robotY][robotX] = true;             // west wall
    }
    else if (robotDir == 3) { // facing left
        if (wallFront) verticalWalls[robotY][robotX] = true;             // west wall
        if (wallLeft)  horizontalWalls[robotY+1][robotX] = true;         // south wall
        if (wallRight) horizontalWalls[robotY][robotX] = true;           // north wall
    }
}


float MapRenderer::getCompletionPercentage() {
    return ((visited_cells_count) / (MAZE_SIZE * MAZE_SIZE)) * 100.0f;
}

void MapRenderer::drawCompletion() {
    u8g2.clearBuffer();
    drawMap();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(75, 60);
    u8g2.print(getCompletionPercentage(), 2);
    u8g2.print("%");
}

void MapRenderer::drawMap() {
    drawGrid();
	drawWalls();
}

//bool wallFront = getFrontDist() < 90;
//bool wallLeft  = getLeftDist()  < 90;
//bool wallRight = getRightDist() < 90;

//mapRenderer.updateWalls(wallFront, wallLeft, wallRight);
//mapRenderer.setRobotPosition(currentX, currentY, currentDir);
//mapRenderer.drawCompletion(); // updates display
//u8g2.sendBuffer();