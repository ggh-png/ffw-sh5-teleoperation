#pragma once
#include <vector>
#include <string>
#include <cstdint>

struct RGBAPixmap {
    int width  = 0;
    int height = 0;
    std::vector<uint8_t> rgba; // row-major, 4 bytes per pixel (R G B A)
    bool empty() const { return rgba.empty(); }
};

// Minimal PNG loader using only zlib (no libpng).
// Supports: 8-bit RGB and RGBA, all PNG filter types, non-interlaced.
class PNGLoader {
public:
    static RGBAPixmap load(const std::string& path);
};
