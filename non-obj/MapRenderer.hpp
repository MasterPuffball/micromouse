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
    uint8_t visited_cells_count = 0;

    uint8_t robotX = 0;
    uint8_t robotY = 0;
    uint8_t robotDir = 0; // 0=up,1=right,2=down,3=left

    // Wall storage
    bool horizontalWalls[gridSize][gridSize] = {{false}};
    bool verticalWalls[gridSize][gridSize] = {{false}};
    bool horizontalWalls[gridSize + 1][gridSize] = {{false}}; // horizontal walls: between rows
    bool verticalWalls[gridSize][gridSize + 1] = {{false}};   // vertical walls: between columns

    void drawDot(uint8_t x, uint8_t y);
    void drawWalls();
};

