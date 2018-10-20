FROM ubuntu:18.04

RUN apt-get update \
    && apt-get install -qy \
       software-properties-common \
       wget \
    && wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add \
    && add-apt-repository 'deb [arch=amd64] https://apt.llvm.org/bionic/ llvm-toolchain-bionic-7 main'

RUN apt-get update \
    && apt-get install -qy \
       clang-7 \
       cmake \
       g++ \
       lldb \
       ninja-build \
       wget \
    && apt-get clean

# Default compiler to clang
RUN update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++-7 100 \
    && update-alternatives --install /usr/bin/cc cc /usr/bin/clang-7 100

# Install header-only libraries
RUN cd /usr/include \
    && mkdir catch2 nlohmann cxxopts \
    && wget https://raw.githubusercontent.com/catchorg/Catch2/v2.4.1/single_include/catch2/catch.hpp -qO catch2/catch.hpp \
    && wget https://raw.githubusercontent.com/nlohmann/json/v3.3.0/single_include/nlohmann/json.hpp -qO nlohmann/json.hpp \
    && wget https://raw.githubusercontent.com/nlohmann/json/v3.3.0/include/nlohmann/json_fwd.hpp -qO nlohmann/json_fwd.hpp \
    && wget https://raw.githubusercontent.com/jarro2783/cxxopts/v2.1.1/include/cxxopts.hpp -qO cxxopts/cxxopts.hpp
