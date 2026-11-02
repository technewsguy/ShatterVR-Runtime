#include "ui/font.hpp"

#include <cstdlib>
#include <sstream>

namespace sh::ui {

bool BitmapFont::load(const std::string& fnt, bool numbers, int texture_w, int texture_h) {
    numbers_ = numbers;
    grid_ = numbers ? 4 : 8;
    const int count = grid_ * grid_;
    cell_w_ = static_cast<float>(texture_w) / static_cast<float>(grid_);
    cell_h_ = static_cast<float>(texture_h) / static_cast<float>(grid_);
    widths_.assign(static_cast<size_t>(count), 0.0f);
    std::istringstream in(fnt);
    std::string line;
    for (int i = 0; i < count && std::getline(in, line); ++i) {
        const float w = static_cast<float>(std::strtod(line.c_str(), nullptr));
        widths_[static_cast<size_t>(i)] = w == 0.0f ? static_cast<float>(texture_w) / static_cast<float>(grid_) : w;
    }
    return texture_w > 0 && texture_h > 0;
}

std::vector<Glyph> BitmapFont::layout(const std::string& text, float* width) const {
    std::vector<Glyph> out;
    const int first = numbers_ ? '0' : ' ', count = grid_ * grid_;
    const float cell = 1.0f / static_cast<float>(grid_);
    float pen = 0.0f;
    for (const unsigned char ch : text) {
        const int g = static_cast<int>(ch) - first;
        if (g < 0 || g >= count) continue;
        const float u = static_cast<float>(g % grid_) * cell, v = static_cast<float>(g / grid_) * cell;
        out.push_back({pen, pen + cell_w_, u, v, u + cell, v + cell});
        pen += widths_[static_cast<size_t>(g)];
    }
    if (width) *width = pen;
    return out;
}

}  // namespace sh::ui
