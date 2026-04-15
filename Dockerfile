FROM diyer22/tiny_cv2:4.6.0-py38-ubuntu20.04

ENV LC_ALL=C.UTF-8 LANG=C.UTF-8 DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai

RUN apt update && \
    apt install -y unzip wget make g++ iputils-ping net-tools traceroute && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir -U pip uv

# V2.1.2_221208 == libMvCameraControl.so.3.2.2.1
RUN wget https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_221208.zip  \
    && unzip MVS_STD_GML_V2.1.2_221208.zip \
    && dpkg -i MVS-2.1.2_x86_64_20221208.deb \
    && rm MVS*.deb MVS*.gz MVS*.zip

ENV MVCAM_COMMON_RUNENV=/opt/MVS/lib
ENV LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32

WORKDIR /hik_camera

COPY pyproject.toml uv.lock README.md /hik_camera/
COPY hik_camera /hik_camera/hik_camera
RUN uv sync --frozen --no-dev

CMD ["uv", "run", "python", "-m", "hik_camera.hik_camera"]

# docker build -t diyer22/hik_camera ./ && docker run --net=host -v /tmp:/tmp -it diyer22/hik_camera;
