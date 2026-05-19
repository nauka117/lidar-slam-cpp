# Ubuntu: PCL и зависимости стабильно ставятся через apt (на любой Linux с Docker).
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    libopenmpi-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
