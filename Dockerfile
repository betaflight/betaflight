# Betaflight Build Environment
# This Dockerfile provides a consistent build environment for Betaflight firmware
# Useful for Windows developers who don't have native ARM toolchain support

FROM ubuntu:22.04

# Install build dependencies
RUN apt-get update && apt-get install -y \
    git \
    make \
    curl \
    xz-utils \
    python3 \
    && rm -rf /var/lib/apt/lists/*

# Download and install ARM GNU Toolchain
WORKDIR /tools
RUN curl -L -o arm-toolchain.tar.xz "https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz" \
    && tar -xf arm-toolchain.tar.xz \
    && rm arm-toolchain.tar.xz

# Add toolchain to PATH
ENV PATH="/tools/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin:${PATH}"

# Set working directory for builds
WORKDIR /src

# Default command shows available make targets
CMD ["make", "help"]
