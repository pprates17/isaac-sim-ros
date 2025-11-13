FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install deps
RUN apt-get update && apt-get install -y \
    git wget cmake python3 python3-pip build-essential ninja-build \
    genromfs ccache kconfig-frontends python3-empy python3-jinja2 \
    python3-numpy python3-yaml python3-toml python3-packaging \
    python3-dev libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*

# Clone PX4 (main branch)
WORKDIR /PX4-Autopilot
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git . && \
    git submodule update --init --recursive

# Install Python dependencies for PX4
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir empy toml numpy pandas pyserial kconfiglib pyros-genmsg jsonschema

# Build for SITL
RUN make px4_sitl_default

ENV DEBIAN_FRONTEND=dialog
# The compiled binaries live in build/px4_sitl_default/
