FROM ubuntu:18.04

RUN apt-get update \
    && apt-get install -qy \
       clang \
       cmake \
       g++ \
       git-core \
       ninja-build \
       libboost-all-dev \
    && apt-get clean

# Default compiler to clang
RUN update-alternatives --set c++ /usr/bin/clang++ \
    && update-alternatives --set cc /usr/bin/clang

# Install Catch2
RUN git clone https://github.com/catchorg/Catch2.git --branch v2.4.1 /tmp/catch \
    && cd /tmp/catch \
    && cmake -Bbuild -H. -DBUILD_TESTING=OFF \
    && cmake --build build/ --target install \
    && rm -r /tmp/catch
