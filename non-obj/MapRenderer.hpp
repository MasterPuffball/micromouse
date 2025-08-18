#pragma once
#include <U8g2lib.h>
#include "Constants.h"
#include "Map.hpp"

class MapRenderer {
public:
    MapRenderer(U8G2 &display, const Map &map) : u8g2(display), map(map) {}

    // --- Constants ---
    static constexpr uint8_t CELLS = 9;           // 9x9 cells
    static constexpr uint8_t NODES = CELLS + 1;   // 10x10 dots
    static constexpr uint8_t CELL_SIZE = 72;

    void drawGrid();
    void drawCompletion();

    //void setRobotPosition(uint8_t x, uint8_t y, uint8_t dir);
    //void updateWalls(bool wallFront, bool wallLeft, bool wallRight);
    void drawMap();

    // Visited for % completion
    float getCompletionPercentage();

private:
    U8G2 &u8g2;
    static constexpr uint8_t dotSpacing = 6;
    static constexpr uint8_t startX = 10;
    static constexpr uint8_t startY = 5;
    uint8_t visited_cells_count = 0;

    //bool horizontalWalls[NODES][CELLS] = {{false}}; // [node-y][cell-x]  : top/bottom of cells
    //bool verticalWalls  [CELLS][NODES] = {{false}}; // [cell-y][node-x]  : left/right of cells

    //bool visited[NODES][NODES] = {{false}};

    //uint8_t robotX = 0, robotY = 0, robotDir = 0;
    // CellWall mazeWalls[NODES][NODES];
    //void updateWall(uint8_t cx, uint8_t cy, char dir, bool isWall);
    //void drawHLine(uint8_t x, uint8_t y);
    //void drawVLine(uint8_t x, uint8_t y);

    void drawDot(uint8_t x, uint8_t y);
    void drawWalls();
};