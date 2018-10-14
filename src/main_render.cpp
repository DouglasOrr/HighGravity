#include <algorithm>
#include <fstream>
#include <iostream>
#include <cxxopts/cxxopts.hpp>
#include <nlohmann/json.hpp>

#include "high_gravity.hpp"

using json = nlohmann::json;

namespace {

  char pixel_float_to_char(float value) {
    return 255 * std::max(std::min(value, 1.0f), 0.0f);
  }

  void write_image_bmp(std::ofstream& out, const high_gravity::Image& image) {
    // WARNING: this only works on little-endian architectures

    // 1. Build the header
    static const unsigned BitmapInfoHeaderSize = 40;
    static const unsigned HeaderSize = 14 + BitmapInfoHeaderSize;
    static const unsigned PixelsPerMetre = 11811; // 300 ppi
    const unsigned pixel_data_size = image.height * image.width * 4;

    union {
      char data[HeaderSize];
      struct __attribute__ ((packed)) {
        // Common header
        /* 00 */ char magic[2];
        /* 02 */std::uint32_t file_size;
        /* 06 */ std::uint16_t reserved0;
        /* 08 */ std::uint16_t reserved1;
        /* 0A */ std::uint32_t data_offset;
        // DIB header: BITMAPINFOHEADER
        /* 0E */ std::uint32_t dib_header_size;
        /* 12 */ std::uint32_t width;
        /* 16 */ std::uint32_t height;
        /* 1A */ std::uint16_t colour_planes;
        /* 1C */ std::uint16_t bits_per_pixel;
        /* 1E */ std::uint32_t compression_method;
        /* 22 */ std::uint32_t image_size;
        /* 26 */ std::uint32_t horizontal_resolution;
        /* 2A */ std::uint32_t vertical_resolution;
        /* 2E */ std::uint32_t num_colours;
        /* 32 */ std::uint32_t num_colours_used;
        /* 36 */
      };
    } header;
    // Common header
    header.magic[0] = 'B';
    header.magic[1] = 'M';
    header.file_size = pixel_data_size + HeaderSize;
    header.reserved0 = 0x00;
    header.reserved1 = 0x00;
    header.data_offset = HeaderSize;

    // DIB header: BITMAPINFOHEADER
    header.dib_header_size = BitmapInfoHeaderSize;
    header.width = image.width;
    header.height = image.height;
    header.colour_planes = 1;
    header.bits_per_pixel = 32;
    header.compression_method = 0;
    header.image_size = pixel_data_size;
    header.horizontal_resolution = PixelsPerMetre;
    header.vertical_resolution = PixelsPerMetre;
    header.num_colours = 0;
    header.num_colours_used = 0;

    // 2. Write the data
    out.write(header.data, HeaderSize);
    char pixel_data[4];
    for (unsigned y = 0; y < image.height; ++y) {
      for (unsigned x = 0; x < image.width; ++x) {
        // Invert y as bmp indices bottom-to-top
        auto& pixel = image(x, image.height - 1 - y);
        pixel_data[0] = pixel_float_to_char(pixel.b);
        pixel_data[1] = pixel_float_to_char(pixel.g);
        pixel_data[2] = pixel_float_to_char(pixel.r);
        pixel_data[3] = 0xff;
        out.write(pixel_data, 4);
      }
    }
  }

} // namespace (anon)

int main(int argc, char** argv) {
  cxxopts::Options options("render", "Render a scene to an image, using the ray tracer");
  options.add_options()
    ("i,input", "Scene to render, specified from a JSON file", cxxopts::value<std::string>())
    ("o,output", "Image file to write", cxxopts::value<std::string>())
    ("w,width", "Width of the image to render (height will be set according to aspect ratio)",
     cxxopts::value<unsigned>()->default_value("128"))
    ("h,help", "Print help");
  options.parse_positional({"input", "output"});

  try {
    auto args = options.parse(argc, argv);
    if (args.count("help")) {
      std::cerr << options.help() << std::endl;

    } else if (args.count("input") and args.count("output")) {
      auto input = args["input"].as<std::string>();
      auto output = args["output"].as<std::string>();

      std::ifstream in(input);
      auto scene = high_gravity::Scene::from_json(json::parse(in));
      auto image = high_gravity::render(scene, args["width"].as<unsigned>());

      std::ofstream out(output, std::ios_base::binary);
      write_image_bmp(out, image);
      std::cerr << "Rendered " << input << " -> " << output << std::endl;

    } else {
      std::cerr << "Error! You must provide input & output file arguments\n\n"
                << options.help() << std::endl;
    }

  } catch (const cxxopts::OptionException& error) {
    std::cerr << "Error! " << error.what()
              << "\n\n" << options.help() << std::endl;
  }
}
