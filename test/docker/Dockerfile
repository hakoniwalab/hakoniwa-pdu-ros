FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-venv \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/hakoniwa-pdu-ros-venv \
    && /opt/hakoniwa-pdu-ros-venv/bin/python -m pip install --upgrade pip setuptools wheel \
    && /opt/hakoniwa-pdu-ros-venv/bin/python -m pip install pytest hakoniwa-pdu

WORKDIR /workspace
COPY . /workspace

RUN /opt/hakoniwa-pdu-ros-venv/bin/python -m pip install -e /workspace \
    && /opt/hakoniwa-pdu-ros-venv/bin/python -c "import hakoniwa_pdu_ros; print(hakoniwa_pdu_ros.__file__)"

ENV PATH=/opt/hakoniwa-pdu-ros-venv/bin:${PATH} \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=73 \
    PYTHONUNBUFFERED=1

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec /opt/hakoniwa-pdu-ros-venv/bin/python -m pytest -q -s test/native"]
