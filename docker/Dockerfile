FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update && \
    apt-get upgrade -y && \
    apt install -y --no-install-recommends \
    git \
    tmux \
    vim \
    nano \
    cmake \
    libsfml-dev \
    build-essential \
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