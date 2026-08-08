FROM ros:jazzy-ros-base-noble

ARG HAKONIWA_PDU_ENDPOINT_REF=f2d7bc052f9afb93281477507206cc35bf284a62
ARG HAKONIWA_PDU_RPC_REF=9ee3482defb680d579930e6942af9a50cdce5d6e

ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_BUILD_PARALLEL_LEVEL=2

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        cmake \
        g++ \
        git \
        libboost-dev \
        python3-venv \
        ros-jazzy-action-tutorials-interfaces \
        ros-jazzy-example-interfaces \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/hakoniwa-pdu-ros-venv \
    && /opt/hakoniwa-pdu-ros-venv/bin/python -m pip install --upgrade pip "setuptools<80" wheel \
    && /opt/hakoniwa-pdu-ros-venv/bin/python -m pip install cffi pytest hakoniwa-pdu

RUN mkdir -p /opt/src/hakoniwa-pdu-endpoint \
    && cd /opt/src/hakoniwa-pdu-endpoint \
    && git init \
    && git remote add origin https://github.com/hakoniwalab/hakoniwa-pdu-endpoint.git \
    && git fetch --depth 1 origin "${HAKONIWA_PDU_ENDPOINT_REF}" \
    && git checkout --detach FETCH_HEAD \
    && cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX=/opt/hakoniwa-pdu-endpoint \
        -DHAKO_PDU_ENDPOINT_ENABLE_HAKONIWA_CORE=OFF \
        -DHAKO_PDU_ENDPOINT_BUILD_TESTS=OFF \
        -DHAKO_PDU_ENDPOINT_BUILD_EXAMPLES=OFF \
        -DHAKO_PDU_ENDPOINT_BUILD_TOOLS=OFF \
        -DHAKO_PDU_ENDPOINT_BUILD_BENCHMARKS=OFF \
        -DHAKO_PDU_ENDPOINT_INSTALL=ON \
    && cmake --build build --config Release --parallel 2 \
    && cmake --install build --config Release --prefix /opt/hakoniwa-pdu-endpoint

RUN mkdir -p /opt/src/hakoniwa-pdu-rpc \
    && cd /opt/src/hakoniwa-pdu-rpc \
    && git init \
    && git remote add origin https://github.com/hakoniwalab/hakoniwa-pdu-rpc.git \
    && git fetch --depth 1 origin "${HAKONIWA_PDU_RPC_REF}" \
    && git checkout --detach FETCH_HEAD \
    && git submodule update --init --recursive --depth 1 \
    && /opt/hakoniwa-pdu-ros-venv/bin/python tools/hako.py configure \
        --endpoint-root /opt/hakoniwa-pdu-endpoint \
    && cmake --build build --config Release --parallel 2 \
    && /opt/hakoniwa-pdu-ros-venv/bin/python tools/hako.py install \
        --endpoint-root /opt/hakoniwa-pdu-endpoint \
        --install-dir /opt/hakoniwa-pdu-rpc

WORKDIR /workspace
COPY . /workspace

RUN /opt/hakoniwa-pdu-ros-venv/bin/python -m pip install -e /workspace \
    && /opt/hakoniwa-pdu-ros-venv/bin/python -c "import hakoniwa_pdu_ros; print(hakoniwa_pdu_ros.__file__)"

ENV PATH=/opt/hakoniwa-pdu-ros-venv/bin:${PATH} \
    HAKO_PDU_RPC_LIBRARY=/opt/hakoniwa-pdu-rpc/lib/libhakoniwa_pdu_rpc.so \
    LD_LIBRARY_PATH=/opt/hakoniwa-pdu-rpc/lib:/opt/hakoniwa-pdu-endpoint/lib \
    PYTHONPATH=/opt/src/hakoniwa-pdu-rpc/python:/opt/src/hakoniwa-pdu-rpc/hakoniwa-pdu-registry \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=73 \
    PYTHONUNBUFFERED=1

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec /opt/hakoniwa-pdu-ros-venv/bin/python -m pytest -q -s test/native"]
