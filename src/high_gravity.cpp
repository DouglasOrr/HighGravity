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
               << ", " << colour.b
               << ", " << colour.a << ")";
  }

  Colour Colour::from_json(const json& j) {
    // r g b a
    return {j[0], j[1], j[2], j[3]};
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
    constexpr static float Tolerance = 0.01;
    Colour m_expected;

    explicit ApproxColour(Colour expected) : m_expected(expected) { }

    virtual bool match(const Colour& actual) const override {
      return (std::abs(m_expected.r - actual.r) < Tolerance)
        && (std::abs(m_expected.g - actual.g) < Tolerance)
        && (std::abs(m_expected.b - actual.b) < Tolerance)
        && (std::abs(m_expected.a - actual.a) < Tolerance);
    }

    virtual std::string describe() const override {
      std::ostringstream str;
      str << "close to " << m_expected;
      return str.str();
    }
  };

  struct ApproxVector3 : Catch::MatcherBase<geometry::Vector3> {
    constexpr static float Tolerance = 0.01;
    geometry::Vector3 m_expected;

    explicit ApproxVector3(geometry::Vector3 expected) : m_expected(expected) { }

    virtual bool match(const geometry::Vector3& actual) const override {
      return (std::abs(m_expected.x - actual.x) < Tolerance)
        && (std::abs(m_expected.y - actual.y) < Tolerance)
        && (std::abs(m_expected.z - actual.z) < Tolerance);
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
    REQUIRE_THAT(scene.ambient.colour, (ApproxColour({0.1, 0.1, 0.1, 1.0})));
    // Root
    auto& root = std::get<Group>(scene.root);
    REQUIRE(not root.bounds);
    REQUIRE(root.children.size() == 3);
    auto& light = std::get<Light>(root.children[0]);
    REQUIRE_THAT(light.position, (ApproxVector3({10, 10, 20})));
    REQUIRE_THAT(light.colour, (ApproxColour({0.9, 0.9, 0.9, 1.0})));
    // Plane
    auto& plane_body = std::get<Body>(root.children[1]);
    REQUIRE_THAT(plane_body.material.colour, (ApproxColour({0.8, 0.8, 0.8, 1.0})));
    auto& plane = std::get<geometry::Plane>(plane_body.shape);
    REQUIRE_THAT(plane.position, (ApproxVector3({0, 0, -5})));
    REQUIRE_THAT(plane.normal, (ApproxVector3({0, 0, 1})));
    // Group
    auto& group = std::get<Group>(root.children[2]);
    REQUIRE(group.bounds.value().radius == 10_a);
    REQUIRE_THAT(group.bounds.value().position, (ApproxVector3({0, 0, 0})));
    REQUIRE(group.children.size() == 1);
    auto& sphere_body = std::get<Body>(group.children[0]);
    REQUIRE_THAT(sphere_body.material.colour, (ApproxColour({0.2, 0.3, 1.0, 1.0})));
    auto& sphere = std::get<geometry::Sphere>(sphere_body.shape);
    REQUIRE_THAT(sphere.position, (ApproxVector3({0, 0, 0})));
    REQUIRE(sphere.radius == 5_a);
  }

#endif // HIG_TESTING

  // *** Core ***

  Image render(const Scene& scene, unsigned width) {
    unsigned height = static_cast<unsigned>(width / scene.camera.aspect);
    Image image(width, height);
    // TODO: replace fake rendering
    for (auto y = 0u; y < height; ++y) {
      for (auto x = 0u; x < width; ++x) {
        float g = static_cast<float>(y) / (height - 1);
        float b = static_cast<float>(x) / (width - 1);
        image(x, y) = {0, g, b, 1};
      }
    }
    return image;
  }

} // namespace high_gravity
