# HighGravity

A raytracer in modern C++, just for fun. No promises.

## Development

You can set up in Docker (or separately, using the [Dockerfile](Dockerfile) as a guide):

    ./run prepare
    ./run check
    ./run render -- --width=1024 data/example.json example.bmp
