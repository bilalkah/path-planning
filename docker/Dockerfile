FROM ubuntu:22.04



RUN apt-get update && \
    apt-get upgrade -y && \
    apt install -y --no-install-recommends \
    git \
    tmux \
    vim \
    nano \
    cmake \
    build-essential \
    libx11-dev \
    libxext-dev \
    libxi-dev \
    libxrandr-dev \
    libxss-dev \
    libxcursor-dev \
    libgl1-mesa-dev \
    libwayland-dev \
    libwayland-egl-backend-dev \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN git clone --recurse-submodules https://github.com/bilalkah/path-planning.git && \
    cd path-planning && \
    bash build.sh

COPY entrypoint.sh /root
RUN chmod +x entrypoint.sh

ENTRYPOINT [ "/root/entrypoint.sh" ]
CMD ["bash"]