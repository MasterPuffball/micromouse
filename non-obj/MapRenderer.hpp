#pragma once
#include <U8g2lib.h>

class MapRenderer {
public:
    MapRenderer(U8G2 &display);
    void drawGrid();
    void drawCompletion();
    float getCompletionPercentage();

    void setRobotPosition(uint8_t x, uint8_t y, uint8_t dir);
    void updateWalls(bool wallFront, bool wallLeft, bool wallRight);
    void drawMap();

private:
    U8G2 &u8g2;
    static constexpr uint8_t dotSpacing = 6;
    static constexpr uint8_t gridSize = 10;
    static constexpr uint8_t startX = 10;
    static constexpr uint8_t startY = 5;
    static constexpr uint8_t MAZE_SIZE = 9;

    // Track visited cells count
    bool visited[gridSize][gridSize] = {{false}};
    uint8_t visited_cells_count = 0;

    uint8_t robotX = 0;
    uint8_t robotY = 0;
    uint8_t robotDir = 0; // 0=up,1=right,2=down,3=left

    uint8_t mazeWalls[gridSize][gridSize] = {{0}};

    // CellWall mazeWalls[gridSize][gridSize];
    void drawHLine(uint8_t x, uint8_t y);
    void drawVLine(uint8_t x, uint8_t y);
    void updateCellWall(uint8_t cx, uint8_t cy, char dir, bool isWall);

    void drawDot(uint8_t x, uint8_t y);
    void drawWalls();
};