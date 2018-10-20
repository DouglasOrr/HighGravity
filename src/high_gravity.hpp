#ifndef HIGH_GRAVITY_HPP
#define HIGH_GRAVITY_HPP

#include <iosfwd>
#include <optional>
#include <variant>
#include <vector>
#include <nlohmann/json_fwd.hpp>

namespace high_gravity {

  // *** Image API ***

  struct Colour {
    float r, g, b;
    static Colour from_json(const nlohmann::json&);
  };
  std::ostream& operator<<(std::ostream&, const Colour&);

  struct Image {
    unsigned width, height;
    std::vector<Colour> pixels;
    Image(unsigned _width, unsigned _height);
    // X runs from left to right, Y runs from top to bottom
    Colour& operator()(unsigned x, unsigned y);
    const Colour& operator()(unsigned x, unsigned y) const;
  };

  // *** Scene API ***

  namespace geometry {

    struct Vector3 {
      float x, y, z;
      static Vector3 from_json(const nlohmann::json&);
    };
    std::ostream& operator<<(std::ostream&, const Vector3&);

    struct Sphere {
      Vector3 position;
      float radius;
      static Sphere from_json(const nlohmann::json&);
    };

    struct Plane {
      Vector3 position;
      Vector3 normal;
      static Plane from_json(const nlohmann::json&);
    };

    typedef std::variant<Sphere, Plane> Shape;
    Shape shape_from_json(const nlohmann::json&);

  } // namespace geometry

  struct Camera {
    geometry::Vector3 position;
    geometry::Vector3 forward;
    geometry::Vector3 up;
    float fov;
    float aspect;
    static Camera from_json(const nlohmann::json&);
  };

  struct Ambient {
    Colour light;
    Colour background;
    static Ambient from_json(const nlohmann::json&);
  };

  struct Light {
    geometry::Vector3 position;
    Colour colour;
    static Light from_json(const nlohmann::json&);
  };

  struct Material {
    Colour colour;
    float diffuse;
    float specular;
    float specular_power;
    float reflection;
    float translucency;
    float refractive_index;
    static Material from_json(const nlohmann::json&);
  };

  struct Group;
  struct Body;
  typedef std::variant<Group, Body> Object;
  Object object_from_json(const nlohmann::json&);

  struct Group {
    std::vector<Object> children;
    std::optional<geometry::Sphere> bounds;
    static Group from_json(const nlohmann::json&);
  };

  struct Body {
    geometry::Shape shape;
    Material material;
    static Body from_json(const nlohmann::json&);
  };

  struct Scene {
    Camera camera;
    Ambient ambient;
    std::vector<Light> lights;
    Object root;
    static Scene from_json(const nlohmann::json&);
  };

  // *** Core API ***

  Image render(const Scene&, unsigned width);

  // *** Implementation ***

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
