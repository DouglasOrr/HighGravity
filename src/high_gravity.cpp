#include "high_gravity.hpp"
#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <sstream>

using json = nlohmann::json;

namespace high_gravity {

  // *** Utility & IO ***

  std::ostream& operator<<(std::ostream& out, const Colour& colour) {
    return out << "rgba=(" << colour.r
               << ", " << colour.g
               << ", " << colour.b;
  }

  Colour Colour::from_json(const json& j) {
    // r g b
    return {j[0], j[1], j[2]};
  }

  namespace geometry {

    Vector3 Vector3::from_json(const json& j) {
      // x y z
      return {j[0], j[1], j[2]};
    }

    std::ostream& operator<<(std::ostream& out, const Vector3& vector) {
      return out << "xyz=(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
    }

    Sphere Sphere::from_json(const json& j) {
      return {Vector3::from_json(j["position"]), j["radius"]};
    }

    Plane Plane::from_json(const json& j) {
      return {Vector3::from_json(j["position"]),
              Vector3::from_json(j["normal"])};
    }

    Shape shape_from_json(const json& j) {
      auto type = j["type"].get<std::string>();
      if (type == "sphere") {
        return Sphere::from_json(j);
      } else if (type == "plane") {
        return Plane::from_json(j);
      } else {
        std::ostringstream str;
        str << "Error! Unexpected shape type \"" << type << "\"";
        throw std::domain_error(str.str());
      }
    }

  } // namespace geometry

  Camera Camera::from_json(const json& j) {
    return {geometry::Vector3::from_json(j["position"]),
            geometry::Vector3::from_json(j["forward"]),
            geometry::Vector3::from_json(j["up"]),
            j["fov"], j["aspect"]};
  }

  Ambient Ambient::from_json(const json& j) {
    return {Colour::from_json(j["colour"])};
  }

  Material Material::from_json(const json& j) {
    return {Colour::from_json(j["colour"])};
  }

  Group Group::from_json(const json& j) {
    auto jchildren = j["children"];
    std::vector<Object> children;
    children.reserve(jchildren.size());
    std::transform(jchildren.begin(), jchildren.end(), std::back_inserter(children),
                   &object_from_json);
    std::optional<geometry::Sphere> bounds;
    if (j.find("position") != j.end()) {
      bounds = geometry::Sphere::from_json(j);
    }
    return {std::move(children), std::move(bounds)};
  }

  Light Light::from_json(const json& j) {
    return {geometry::Vector3::from_json(j["position"]),
            Colour::from_json(j["colour"])};
  }

  Body Body::from_json(const json& j) {
    return {geometry::shape_from_json(j),
            Material::from_json(j["material"])};
  }

  Object object_from_json(const json& j) {
    auto type = j["type"].get<std::string>();
    if (type == "group") {
      return Group::from_json(j);
    } else if (type == "light") {
      return Light::from_json(j);
    } else {
      return Body::from_json(j);
    }
  }

  Scene Scene::from_json(const json& jroot) {
    return {Camera::from_json(jroot["camera"]),
            Ambient::from_json(jroot["ambient"]),
            object_from_json(jroot["root"])};
  }

#ifdef HIG_TESTING

  using namespace Catch::literals;

  struct ApproxColour : Catch::MatcherBase<Colour> {
  private:
    Colour m_expected;

  public:
    constexpr static float Tolerance = 0.01;

    explicit ApproxColour(Colour expected) : m_expected(expected) { }

    virtual bool match(const Colour& actual) const override {
      return (std::abs(m_expected.r - actual.r) < Tolerance)
        and (std::abs(m_expected.g - actual.g) < Tolerance)
        and (std::abs(m_expected.b - actual.b) < Tolerance);
    }

    virtual std::string describe() const override {
      std::ostringstream str;
      str << "close to " << m_expected;
      return str.str();
    }
  };

  struct ApproxVector3 : Catch::MatcherBase<geometry::Vector3> {
  private:
    geometry::Vector3 m_expected;

  public:
    constexpr static float Tolerance = 0.01;

    explicit ApproxVector3(geometry::Vector3 expected) : m_expected(expected) { }

    virtual bool match(const geometry::Vector3& actual) const override {
      return (std::abs(m_expected.x - actual.x) < Tolerance)
        and (std::abs(m_expected.y - actual.y) < Tolerance)
        and (std::abs(m_expected.z - actual.z) < Tolerance);
    }

    virtual std::string describe() const override {
      std::ostringstream str;
      str << "close to " << m_expected;
      return str.str();
    }
  };

  TEST_CASE("Read scene from JSON", "[parsing]") {
    std::ifstream in("data/example.json");
    const auto scene = Scene::from_json(json::parse(in));

    // This must match data/example.json
    // Camera
    REQUIRE_THAT(scene.camera.position, (ApproxVector3({-10, 0, 0})));
    REQUIRE_THAT(scene.camera.forward, (ApproxVector3({1, 0, 0})));
    REQUIRE_THAT(scene.camera.up, (ApproxVector3({0, 0, 1})));
    REQUIRE(scene.camera.fov == 1.57_a);
    REQUIRE(scene.camera.aspect == 1.5_a);
    // Ambient
    REQUIRE_THAT(scene.ambient.colour, (ApproxColour({0.1, 0.1, 0.1})));
    // Root
    auto& root = std::get<Group>(scene.root);
    REQUIRE(not root.bounds);
    REQUIRE(root.children.size() == 3);
    auto& light = std::get<Light>(root.children[0]);
    REQUIRE_THAT(light.position, (ApproxVector3({10, 10, 20})));
    REQUIRE_THAT(light.colour, (ApproxColour({0.9, 0.9, 0.9})));
    // Plane
    auto& plane_body = std::get<Body>(root.children[1]);
    REQUIRE_THAT(plane_body.material.colour, (ApproxColour({0.8, 0.8, 0.8})));
    auto& plane = std::get<geometry::Plane>(plane_body.shape);
    REQUIRE_THAT(plane.position, (ApproxVector3({0, 0, -5})));
    REQUIRE_THAT(plane.normal, (ApproxVector3({0, 0, 1})));
    // Group
    auto& group = std::get<Group>(root.children[2]);
    REQUIRE(group.bounds->radius == 10_a);
    REQUIRE_THAT(group.bounds->position, (ApproxVector3({0, 0, 0})));
    REQUIRE(group.children.size() == 1);
    auto& sphere_body = std::get<Body>(group.children[0]);
    REQUIRE_THAT(sphere_body.material.colour, (ApproxColour({0.2, 0.3, 1.0})));
    auto& sphere = std::get<geometry::Sphere>(sphere_body.shape);
    REQUIRE_THAT(sphere.position, (ApproxVector3({0, 0, 0})));
    REQUIRE(sphere.radius == 5_a);
  }

#endif // HIG_TESTING

  namespace {

    float dot(const geometry::Vector3& a, const geometry::Vector3& b) {
      return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    geometry::Vector3 cross(const geometry::Vector3& a, const geometry::Vector3& b) {
      return {a.y * b.z - a.z * b.y,
              a.z * b.x - a.x * b.z,
              a.x * b.y - b.x * a.y};
    }

    float length(const geometry::Vector3& v) {
      return std::sqrt(dot(v, v));
    }

    geometry::Vector3 operator*(float a, const geometry::Vector3& b) {
      return {a * b.x, a * b.y, a * b.z};
    }

    geometry::Vector3 operator+(const geometry::Vector3& a, const geometry::Vector3& b) {
      return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    geometry::Vector3 operator-(const geometry::Vector3& a, const geometry::Vector3& b) {
      return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    geometry::Vector3 orthogonal_component(const geometry::Vector3& a, const geometry::Vector3& b) {
      float d = dot(a, b);
      return {a.x - d * b.x, a.y - d * b.y, a.z - d * b.z};
    }

    geometry::Vector3 normalize(const geometry::Vector3& v) {
      return (1 / length(v)) * v;
    }

    struct Ray {
      unsigned ttl;
      geometry::Vector3 start;
      geometry::Vector3 direction;
    };
    std::ostream& operator<<(std::ostream& out, const Ray& ray) {
      return out << "Ray{ttl:" << ray.ttl
                 << ", start:" << ray.start
                 << ", direction:" << ray.direction << "}";
    }

    struct Collision {
      float distance;
      const Body* body;
    };
    bool operator<(const Collision& a, const Collision& b) {
      return a.distance < b.distance;
    }

    struct CollisionDetection {
      Ray ray;
      // Objects
      std::optional<Collision> operator()(const Object& obj) const {
        return std::visit(*this, obj);
      }
      std::optional<Collision> operator()(const Group& group) const {
        // Bounds test on the group, then recursively test children
        if ((not group.bounds) or (*this)(*group.bounds)) {
          // Find the nearest child element collision
          std::optional<Collision> nearest;
          for (auto& child : group.children) {
            auto collision = (*this)(child);
            if (collision and ((not nearest) or (*collision < *nearest))) {
              nearest = collision;
            }
          }
          return nearest;
        }
        return std::nullopt;
      }
      std::optional<Collision> operator()(const Body& body) const {
        auto distance = (*this)(body.shape);
        if (distance) {
          return Collision{*distance, &body};
        }
        return std::nullopt;
      }
      std::optional<Collision> operator()(const Light&) const {
        // There are no collisions with lights
        return std::nullopt;
      }
      // Shapes
      std::optional<float> operator()(const geometry::Shape& shape) const {
        return std::visit(*this, shape);
      }
      std::optional<float> operator()(const geometry::Sphere& sphere) const {
        auto relative = sphere.position - ray.start;
        auto d_closest = dot(relative, ray.direction);
        auto discriminant = d_closest * d_closest + sphere.radius * sphere.radius - dot(relative, relative);
        if (discriminant < 0) {
          // The ray does not intersect the sphere anywhere
          return std::nullopt;
        }
        auto sqrt_discriminant = std::sqrt(discriminant);
        if (0 < d_closest - sqrt_discriminant) {
          // We're outside the sphere, so return the smaller distance (hitting the outside)
          return d_closest - sqrt_discriminant;
        }
        if (0 < d_closest + sqrt_discriminant) {
          // We're inside the sphere, so return the larger distance (hitting the inside)
          return d_closest + sqrt_discriminant;
        }
        // The ray intersects the sphere only in the past
        return std::nullopt;
      }
      std::optional<float> operator()(const geometry::Plane& plane) const {
        auto height = dot(plane.normal, plane.position - ray.start);
        auto v_component = dot(plane.normal, ray.direction);//length(cross(plane.normal, ray.direction));
        if (v_component == 0) {
          // Ray is parallel to the plane - no intersection
          return std::nullopt;
        }
        auto distance = height / v_component;
        // Exclude collisions that happen in the past
        return 0 < distance ? std::optional<float>(distance) : std::nullopt;
      }
    };

    Colour blend(const Colour& src, const Colour& other, float weight) {
      return {src.r * weight * other.r,
              src.g * weight * other.g,
              src.b * weight * other.b};
    }

    Colour trace_ray(const Scene& scene, const Ray& ray) {
      std::cerr << "Tracing " << ray << std::endl;
      if (ray.ttl == 0) {
        return {0, 0, 0}; // expired ray fallback
      }
      auto collision = CollisionDetection{ray}(scene.root);
      if (collision) {
        Colour ambient = blend(collision->body->material.colour, scene.ambient.colour, 1.0f);
        return ambient;
      }
      return {0, 0, 0}; // "miss" background fallback
    }

  } // namespace (anon)

  // *** Core ***

  Image render(const Scene& scene, unsigned width) {
    static const auto StartingTtl = 2u;
    auto height = static_cast<unsigned>(width / scene.camera.aspect);

    auto camera_z = normalize(scene.camera.forward);
    auto up = normalize(orthogonal_component(scene.camera.up, camera_z));
    auto w = 2 * std::tan(scene.camera.fov / 2);
    auto camera_x = w * cross(camera_z, up);
    auto camera_y = (w / scene.camera.aspect) * up;

    Image image(width, height);
    for (auto y = 0u; y < height; ++y) {
      for (auto x = 0u; x < width; ++x) {
        auto dy = 0.5f - static_cast<float>(y) / (height - 1);
        auto dx = static_cast<float>(x) / (width - 1) - 0.5f;
        image(x, y) = trace_ray(scene, Ray{StartingTtl,
                                           scene.camera.position,
                                           normalize(camera_z + dy * camera_y + dx * camera_x)});
      }
    }
    return image;
  }

} // namespace high_gravity
