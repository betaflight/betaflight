FROM ubuntu:wily
LABEL maintainer Alexandr Kuzmitsky <brat002@gmail.com>

VOLUME /home/src/
WORKDIR /home/src/

RUN mkdir -p /home/src && apt-get update && \
    apt-get install -y software-properties-common python-software-properties && \
    apt-get remove -y binutils-arm-none-eabi gcc-arm-none-eabi && \
    add-apt-repository -y ppa:terry.guo/gcc-arm-embedded && \
    apt-get update && \
    apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi make git gcc ruby