#include "MapRenderer.hpp"

MapRenderer::MapRenderer(U8G2 &display) : u8g2(display) {}

void MapRenderer::drawDot(uint8_t x, uint8_t y) {
    const uint8_t radius = 0;
    u8g2.drawDisc(startX + x * dotSpacing, startY + y * dotSpacing, radius);
}

void MapRenderer::drawGrid() {
    for (uint8_t y = 0; y < gridSize; y++) {
        for (uint8_t x = 0; x < gridSize; x++) {
            drawDot(x, y);
        }
    }
}

float MapRenderer::getCompletionPercentage() const {
    return (static_cast<float>(visited_cells_count) / (MAZE_SIZE * MAZE_SIZE)) * 100.0f;
}

void MapRenderer::drawCompletion() {
    u8g2.clearBuffer();
    drawGrid();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.setCursor(75, 60);
    u8g2.print(getCompletionPercentage(), 2);
    u8g2.print("%");
}
