#pragma once
#include <U8g2lib.h>

class MapRenderer {
public:
    MapRenderer(U8G2 &display);

    // --- Constants (cells vs. nodes) ---
    static constexpr uint8_t CELLS = 9;           // 9x9 cells
    static constexpr uint8_t NODES = CELLS + 1;   // 10x10 dots (grid intersections)

    void drawGrid();
    void drawCompletion();

    void setRobotPosition(uint8_t x, uint8_t y, uint8_t dir);
    void updateWalls(bool wallFront, bool wallLeft, bool wallRight);
    void drawMap();

    // Visited bookkeeping (for % completion)
    void markVisited(uint8_t x, uint8_t y);
    bool isVisited(uint8_t x, uint8_t y) const;
    float getCompletionPercentage();

private:
    U8G2 &u8g2;
    static constexpr uint8_t dotSpacing = 6;
    static constexpr uint8_t startX = 10;
    static constexpr uint8_t startY = 5;

    // Lattice dots + edges
    bool horizontalWalls[NODES][CELLS] = {{false}}; // [node-y][cell-x]  : top/bottom of cells
    bool verticalWalls  [CELLS][NODES] = {{false}}; // [cell-y][node-x]  : left/right of cells

    // Track visited cells count
    bool visited[NODES][NODES] = {{false}};
    uint8_t visited_cells_count = 0;

    uint8_t robotX = 0, robotY = 0, robotDir = 0; // 0=up,1=right,2=down,3=left

    uint8_t mazeWalls[NODES][NODES] = {{0}};

    // CellWall mazeWalls[NODES][NODES];
    void drawHLine(uint8_t x, uint8_t y);
    void drawVLine(uint8_t x, uint8_t y);
    void updateWall(uint8_t cx, uint8_t cy, char dir, bool isWall);

    void initBorderWalls();
    void drawDot(uint8_t x, uint8_t y);
    void drawWalls();
};