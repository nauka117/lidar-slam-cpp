FROM archlinux:latest

RUN pacman -Syu --noconfirm \
    && pacman -S --noconfirm base-devel cmake ninja pcl eigen boost \
    && pacman -Scc --noconfirm

WORKDIR /workspace
