#ifndef HIGH_GRAVITY_HPP
#define HIGH_GRAVITY_HPP

#include <vector>

namespace high_gravity {

  struct Colour {
    float r, g, b, a;
  };

  struct Image {
    unsigned width, height;
    std::vector<Colour> pixels;
    Image(unsigned _width, unsigned _height);
    // X runs from left to right, Y runs from top to bottom
    Colour& operator()(unsigned x, unsigned y);
    const Colour& operator()(unsigned x, unsigned y) const;
  };

  // Implementation

  inline Image::Image(unsigned _width, unsigned _height)
    : width(_width), height(_height), pixels(_width * _height) { }

  inline Colour& Image::operator()(unsigned x, unsigned y) {
    return pixels[y * width + x];
  }
  inline const Colour& Image::operator()(unsigned x, unsigned y) const {
    return pixels[y * width + x];
  }

} // namespace high_gravity

#endif // HIGH_GRAVITY_HPP
