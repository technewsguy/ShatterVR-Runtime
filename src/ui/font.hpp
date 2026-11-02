#pragma once

// The game's bitmap fonts (the original's Font): fonts/<name>.png holds a grid of glyphs (4 x 4 for the
// number fonts, 8 x 8 otherwise) and fonts/<name>.fnt one advance width per glyph (texels of the texture's reported
// size; 0 means the full cell). Number fonts start at '0', the others at ' '; characters outside are skipped.

#include <string>
#include <vector>

namespace sh::ui {

struct Glyph {
    float x0, x1;          // the cell's quad in font texels, pen-relative
    float u0, v0, u1, v1;  // the cell in the texture
};

class BitmapFont {
public:
    // fnt: the .fnt text; texture_w/h: the texture's reported size (twice the decoded size in this APK).
    bool load(const std::string& fnt, bool numbers, int texture_w, int texture_h);
    // Font::setText: the glyph quads of text, each from y 0 to the cell height, and the total advance.
    std::vector<Glyph> layout(const std::string& text, float* width) const;
    float cell_height() const { return cell_h_; }

private:
    bool numbers_ = false;
    int grid_ = 8;
    float cell_w_ = 0, cell_h_ = 0;
    std::vector<float> widths_;
};

}  // namespace sh::ui
