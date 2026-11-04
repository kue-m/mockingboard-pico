# Reproducible firmware build: `docker build -o dist .` runs the host tests and writes the UF2 to ./dist/.
# Pins pico-sdk + picotool to 2.2.0. For a bit-for-bit build, pin the base image by digest first.

FROM debian:bookworm-slim AS build

ENV DEBIAN_FRONTEND=noninteractive
RUN rm -f /etc/dpkg/dpkg.cfg.d/docker
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3 \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        libusb-1.0-0-dev \
        pkg-config \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ARG PICO_SDK_REF=2.2.0
ARG PICOTOOL_REF=2.2.0
ENV PICO_SDK_PATH=/opt/pico-sdk

# pico-sdk (with the submodules the firmware needs, e.g. TinyUSB)
RUN git clone -b ${PICO_SDK_REF} --depth 1 https://github.com/raspberrypi/pico-sdk ${PICO_SDK_PATH} \
    && git -C ${PICO_SDK_PATH} submodule update --init --depth 1

# picotool: produces the UF2 + stamps binary info. Prebuild it so configure needn't fetch it.
RUN git clone -b ${PICOTOOL_REF} --depth 1 https://github.com/raspberrypi/picotool /tmp/picotool \
    && cmake -S /tmp/picotool -B /tmp/picotool/build -DPICO_SDK_PATH=${PICO_SDK_PATH} \
    && cmake --build /tmp/picotool/build -j"$(nproc)" \
    && cmake --install /tmp/picotool/build \
    && rm -rf /tmp/picotool

WORKDIR /src
COPY . .
# Trust the copied repo so the version step can read git state.
RUN git config --global --add safe.directory /src

RUN make clean && make test && make prod \
    && mkdir -p /artifacts \
    && cp dist/mockingboard_pico.uf2 /artifacts/

# `docker build -o dist .` exports the contents of this final stage to ./dist.
FROM scratch AS export
COPY --from=build /artifacts/ /
