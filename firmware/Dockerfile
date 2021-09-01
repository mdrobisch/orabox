FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# Install latest chrome dev package and fonts to support major charsets (Chinese, Japanese, Arabic, Hebrew, Thai and a few others)
# Note: this installs the necessary libs to make the bundled version of Chromium that Puppeteer
# installs, work.
RUN apt-get update \
    && apt-get install -y git gnupg wget flex bison gperf python3 python3-pip python3-setuptools \
                          cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 locales
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
RUN mkdir -p /root/esp
RUN cd /root/esp
WORKDIR /root/esp
RUN git clone --recursive https://github.com/espressif/esp-idf.git
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10
RUN cd /root/esp/esp-idf
WORKDIR /root/esp/esp-idf
RUN git fetch
RUN git checkout v4.1
RUN git submodule update --init --recursive
RUN ./install.sh esp32
RUN cd /root/esp
WORKDIR /root/esp
RUN git clone --recursive https://github.com/espressif/esp-adf.git
RUN cd /root/esp/esp-adf
WORKDIR /root/esp/esp-adf
RUN git fetch
RUN git checkout v2.3
RUN git submodule update --init --recursive
ENV ADF_PATH /root/esp/esp-adf
RUN echo "source /root/esp/esp-idf/export.sh" >> /root/.bashrc