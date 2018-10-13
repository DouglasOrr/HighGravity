#include <iostream>
#include <fstream>
#include <cxxopts/cxxopts.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

  void read_scene_json(const std::string& path) {
    std::ifstream in(path);
    auto root = json::parse(in);
    std::cout << root << std::endl;
  }

  struct Colour {
    float r, g, b, a;
  };

  struct Image {
    unsigned width, height;
    std::vector<Colour> pixels;
    Image(unsigned _width, unsigned _height) : width(_width), height(_height), pixels(_width * _height) { }
  };

  void write_image_bmp(const Image&, const std::string& path) {
    std::ofstream out(path, std::ios_base::binary);
  }

} // namespace (anon)

int main(int argc, char** argv) {
  cxxopts::Options options("render", "Render a scene to an image, using the ray tracer");
  options.add_options()
    ("i,input", "Scene to render, specified from a JSON file", cxxopts::value<std::string>())
    ("o,output", "Image file to write", cxxopts::value<std::string>())
    ("h,help", "Print help");
  options.parse_positional({"input", "output"});

  try {
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
      std::cerr << options.help() << std::endl;

    } else if (result.count("input") and result.count("output")) {
      std::cerr << "Input: " << result["input"].as<std::string>() << std::endl;
      std::cerr << "Output: " << result["output"].as<std::string>() << std::endl;
      read_scene_json(result["input"].as<std::string>());

      std::cerr << "Doing" << std::endl;
      Image image(128, 64);
      for (auto y = 0u; y < image.height; ++y) {
        for (auto x = 0u; x < image.width; ++x) {
          float g = static_cast<float>(y) / (image.height - 1);
          float b = static_cast<float>(x) / (image.height - 1);
          image.pixels[image.width * y + x] = {0, g, b, 1};
        }
      }
      write_image_bmp(image, result["output"].as<std::string>());
      std::cerr << "All done" << std::endl;

    } else {
      std::cerr << "Error! You must provide input & output file arguments\n\n"
                << options.help() << std::endl;
    }

  } catch (const cxxopts::OptionException& error) {
    std::cerr << "Error! " << error.what()
              << "\n\n" << options.help() << std::endl;
  }
}
