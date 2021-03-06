#include "high_gravity.hpp"
#include <catch2/catch.hpp>
#include <nlohmann/json.hpp>
#include <parallel/algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

using json = nlohmann::json;

namespace high_gravity {

  // *** Utility & IO ***

  namespace {
    template<class T, class UnaryOperation>
    std::vector<T> from_json_to_vector(const json& j, UnaryOperation op) {
      std::vector<T> items;
      items.reserve(j.size());
      std::transform(j.begin(), j.end(), std::back_inserter(items), op);
      return items;
    }

  } // namespace (anonymous)

  std::ostream& operator<<(std::ostream& out, const Colour& colour) {
    return out << "rgb=(" << colour.r
               << ", " << colour.g
               << ", " << colour.b << ")";
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
    return {Colour::from_json(j["light"]),
            Colour::from_json(j["background"])};
  }

  Material Material::from_json(const json& j) {
    return {Colour::from_json(j["colour"]),
            j.value("diffuse", 0.f),
            j.value("specular", 0.f),
            j.value("specular_power", 1.f),
            j.value("reflection", 0.f),
            j.value("translucency", 0.f),
            j.value("refractive_index", 1.f)};
  }

  Group Group::from_json(const json& j) {
    std::optional<geometry::Sphere> bounds;
    if (j.find("position") != j.end()) {
      bounds = geometry::Sphere::from_json(j);
    }
    return {from_json_to_vector<Object>(j["children"], &object_from_json), std::move(bounds)};
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
    } else {
      return Body::from_json(j);
    }
  }

  Scene Scene::from_json(const json& j) {
    return {Camera::from_json(j["camera"]),
            Ambient::from_json(j["ambient"]),
            from_json_to_vector<Light>(j["lights"], &Light::from_json),
            object_from_json(j["root"])};
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
    REQUIRE_THAT(scene.ambient.light, (ApproxColour({0.02, 0.02, 0.02})));
    REQUIRE_THAT(scene.ambient.background, (ApproxColour({0.2, 0.2, 0.18})));
    // Lights
    REQUIRE(scene.lights.size() == 1);
    REQUIRE_THAT(scene.lights[0].position, (ApproxVector3({-10, 10, 10})));
    REQUIRE_THAT(scene.lights[0].colour, (ApproxColour({0.9, 0.9, 0.9})));
    // Root
    auto& root = std::get<Group>(scene.root);
    REQUIRE(not root.bounds);
    REQUIRE(root.children.size() == 3);
    // Plane
    {
      auto& plane_body = std::get<Body>(root.children[0]);
      REQUIRE_THAT(plane_body.material.colour, (ApproxColour({0.9, 0.9, 0.9})));
      REQUIRE(plane_body.material.diffuse == 0.4_a);
      REQUIRE(plane_body.material.specular == 0.3_a);
      REQUIRE(plane_body.material.specular_power == 10_a);
      REQUIRE(plane_body.material.reflection == 1_a);
      REQUIRE(plane_body.material.translucency == 0); // default
      REQUIRE(plane_body.material.refractive_index == 1); // default
      auto& plane = std::get<geometry::Plane>(plane_body.shape);
      REQUIRE_THAT(plane.position, (ApproxVector3({0, 0, -3})));
      REQUIRE_THAT(plane.normal, (ApproxVector3({0, 0, 1})));
    }
    // Sphere
    {
      auto& sphere = std::get<Body>(root.children[1]);
      REQUIRE(sphere.material.translucency == 0.8_a);
      REQUIRE(sphere.material.refractive_index == 1.9_a);
    }
    // Group
    {
      auto& group = std::get<Group>(root.children[2]);
      REQUIRE(group.bounds->radius == 6_a);
      REQUIRE_THAT(group.bounds->position, (ApproxVector3({0, 0, 0})));
      REQUIRE(group.children.size() == 2);
      {
        auto& sphere_body = std::get<Body>(group.children[0]);
        REQUIRE_THAT(sphere_body.material.colour, (ApproxColour({0.2, 0.3, 1.0})));
        auto& sphere = std::get<geometry::Sphere>(sphere_body.shape);
        REQUIRE_THAT(sphere.position, (ApproxVector3({0, 3, 0})));
        REQUIRE(sphere.radius == 3_a);
      }
    }
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
      float refractive_index;
    };
    std::ostream& operator<<(std::ostream& out, const Ray& ray) {
      return out << "Ray{ttl:" << ray.ttl
                 << ", start:" << ray.start
                 << ", direction:" << ray.direction
                 << ", refractive_index: " << ray.refractive_index << "}";
    }

    struct Collision {
      float distance;
      bool internal;
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
        auto collision = (*this)(body.shape);
        if (collision) {
          collision->body = &body;
          return collision;
        }
        return std::nullopt;
      }
      std::optional<Collision> operator()(const Light&) const {
        // There are no collisions with lights
        return std::nullopt;
      }
      // Shapes
      std::optional<Collision> operator()(const geometry::Shape& shape) const {
        return std::visit(*this, shape);
      }
      std::optional<Collision> operator()(const geometry::Sphere& sphere) const {
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
          return Collision{d_closest - sqrt_discriminant, false, nullptr};
        }
        if (0 < d_closest + sqrt_discriminant) {
          // We're inside the sphere, so return the larger distance (hitting the inside)
          return Collision{d_closest + sqrt_discriminant, true, nullptr};
        }
        // The ray intersects the sphere only in the past
        return std::nullopt;
      }
      std::optional<Collision> operator()(const geometry::Plane& plane) const {
        auto height = dot(plane.normal, plane.position - ray.start);
        auto v_component = dot(plane.normal, ray.direction);
        if (v_component == 0) {
          // Ray is parallel to the plane - no intersection
          return std::nullopt;
        }
        auto distance = height / v_component;
        // Exclude collisions that happen in the past
        return 0 < distance ? std::optional<Collision>({distance, height > 0, nullptr}) : std::nullopt;
      }
    };

    struct GetNormal {
      geometry::Vector3 position;
      geometry::Vector3 operator()(const Body& body) {
        return std::visit(*this, body.shape);
      }
      geometry::Vector3 operator()(const geometry::Sphere& sphere) {
        return normalize(position - sphere.position);
      }
      geometry::Vector3 operator()(const geometry::Plane& plane) {
        return plane.normal;
      }
    };

    Colour blend(const Colour& src, const Colour& other, float weight) {
      return {weight * src.r * other.r,
              weight * src.g * other.g,
              weight * src.b * other.b};
    }
    Colour operator+(const Colour& left, const Colour& right) {
      return {left.r + right.r, left.g + right.g, left.b + right.b};
    }

    Colour trace_ray(const Scene& scene, const Ray& ray) {
      static const auto Delta = 0.1f;
      if (ray.ttl == 0) {
        return {0, 0, 0}; // expired ray fallback
      }
      auto collision = CollisionDetection{ray}(scene.root);
      if (collision) {
        auto& material = collision->body->material;
        auto position = ray.start + collision->distance * ray.direction;
        auto normal = (collision->internal ? -1 : 1) * GetNormal{position}(*(collision->body));
        auto position_outside = position + Delta * normal;
        auto reflection = ray.direction - (2 * dot(ray.direction, normal)) * normal;

        // Ambient
        auto colour = blend(material.colour, scene.ambient.light, 1.0f);
        // Lighting
        for (auto& light : scene.lights) {
          auto light_direction = normalize(light.position - position_outside);
          auto shadow = CollisionDetection{Ray{0, position_outside, light_direction, ray.refractive_index}}(scene.root);
          if (not shadow) {
            auto diffuse = material.diffuse * std::max(0.0f, dot(light_direction, normal));
            auto specular = material.specular * pow(std::max(0.0f, dot(light_direction, reflection)),
                                                    material.specular_power);
            colour = colour + blend(material.colour, light.colour, diffuse + specular);
          }
        }
        // Reflection
        if (0 < material.reflection) {
          auto child = trace_ray(scene, Ray{ray.ttl - 1, position_outside, reflection, ray.refractive_index});
          colour = colour + blend(material.colour, child, material.reflection);
        }
        // Refraction
        if (0 < material.translucency) {
          // TODO - ri_out when inside another object?
          auto ri_out = collision->internal ? 1.f : material.refractive_index;
          auto parallel = (ray.refractive_index / ri_out) * (ray.direction - dot(ray.direction, normal) * normal);
          auto cos2_theta = 1 - dot(parallel, parallel);
          if (0 < cos2_theta) {
            auto direction = parallel - sqrt(cos2_theta) * normal;
            auto child = trace_ray(scene, Ray{ray.ttl - 1, position + Delta * direction, direction, ri_out});
            colour = colour + blend(material.colour, child, material.translucency);
          }
        }
        return colour;
      }
      return scene.ambient.background;
    }

  } // namespace (anon)

  // *** Core ***

  Image render(const Scene& scene, unsigned width) {
    static const auto StartingTtl = 4u;
    auto height = static_cast<unsigned>(width / scene.camera.aspect);
    Image image(width, height);

    auto camera_z = normalize(scene.camera.forward);
    auto up = normalize(orthogonal_component(scene.camera.up, camera_z));
    auto w = 2 * std::tan(scene.camera.fov / 2);
    auto camera_x = w * cross(camera_z, up);
    auto camera_y = (w / scene.camera.aspect) * up;

    std::vector<std::tuple<unsigned, unsigned> > indices;
    for (auto y = 0u; y < height; ++y) {
      for (auto x = 0u; x < width; ++x) {
        indices.push_back(std::make_tuple(x, y));
      }
    }
    std::__parallel::transform(indices.begin(), indices.end(), image.pixels.begin(),
                               [&](const std::tuple<unsigned, unsigned>& xy) {
                                 auto dx = static_cast<float>(std::get<0>(xy)) / (width - 1) - 0.5f;
                                 auto dy = 0.5f - static_cast<float>(std::get<1>(xy)) / (height - 1);
                                 return trace_ray(scene, Ray{StartingTtl,
                                                             scene.camera.position,
                                                             normalize(camera_z + dy * camera_y + dx * camera_x),
                                                             1.f});
                   });
    return image;
  }

} // namespace high_gravity
