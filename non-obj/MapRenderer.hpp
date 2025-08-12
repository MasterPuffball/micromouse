#pragma once
#include <U8g2lib.h>

class MapRenderer {
public:
    MapRenderer(U8G2 &display);
    void drawGrid();
    void drawCompletion();
    float getCompletionPercentage();

private:
    U8G2 &u8g2;
    static constexpr uint8_t dotSpacing = 6;
    static constexpr uint8_t gridSize = 10;
    static constexpr uint8_t startX = 10;
    static constexpr uint8_t startY = 5;
    static constexpr uint8_t MAZE_SIZE = 9;

    // Track visited cells count
    uint8_t visited_cells_count = 0;

    void drawDot(uint8_t x, uint8_t y);
};

