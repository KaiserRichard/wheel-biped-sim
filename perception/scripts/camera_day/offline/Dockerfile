FROM ubuntu:22.04
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates libopencv-dev libeigen3-dev \
    libboost-serialization-dev libssl-dev libglew-dev libgl1-mesa-dev libepoxy-dev \
    python3-pip python3-numpy python3-opencv zlib1g-dev libjpeg-dev libpng-dev
RUN python3 -m pip install rosbags==0.11.0
RUN git clone --depth 1 --branch v0.9.1 https://github.com/stevenlovegrove/Pangolin.git /work/Pangolin && \
    test "$(git -C /work/Pangolin rev-parse HEAD)" = 7364b59a58045d1c429b146e44f6f088fd1cc518 && \
    git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /work/ORB_SLAM3 && \
    git -C /work/ORB_SLAM3 checkout 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
# Mount task sources and finalized evidence read-only. Build outputs stay in /work.
CMD ["sleep", "infinity"]
