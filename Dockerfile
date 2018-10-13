FROM ubuntu:18.04

RUN apt-get update \
    && apt-get install -qy \
       clang \
       cmake \
       g++ \
       lldb \
       ninja-build \
       wget \
    && apt-get clean

# Default compiler to clang
RUN update-alternatives --set c++ /usr/bin/clang++ \
    && update-alternatives --set cc /usr/bin/clang

# Install header-only libraries
RUN cd /usr/include \
    && mkdir catch2 nlohmann cxxopts \
    && wget https://raw.githubusercontent.com/catchorg/Catch2/v2.4.1/single_include/catch2/catch.hpp -qO catch2/catch.hpp \
    && wget https://raw.githubusercontent.com/nlohmann/json/v3.3.0/single_include/nlohmann/json.hpp -qO nlohmann/json.hpp \
    && wget https://raw.githubusercontent.com/jarro2783/cxxopts/v2.1.1/include/cxxopts.hpp -qO cxxopts/cxxopts.hpp
