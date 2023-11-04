FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && \
    apt-get upgrade -y

RUN apt-get update && \
    apt install -y --no-install-recommends \
    git \
    tmux \
    vim \
    nano \
    libsfml-dev \
    build-essential \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install cmake -y

WORKDIR /root


RUN git clone --recurse-submodules https://github.com/bilalkah/path-planning.git && \
    cd path-planning && \
    bash build.sh

COPY entrypoint.sh /root
RUN chmod +x entrypoint.sh

ENTRYPOINT [ "entrypoint.sh" ]
CMD ["bash"]