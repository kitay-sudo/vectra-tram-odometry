# Образ Vectra: ROS 2 Humble + пакеты оценщика, собранные внутри образа.
#
#   docker compose build                       # или:
#   docker build -f docker/Dockerfile -t vectra/tram:compose .
#
# Базовый образ закреплён по digest (индекс multi-arch ros:humble-ros-base,
# снят `docker inspect ros:humble-ros-base` 25.09.2026, образ от 09.09.2026),
# чтобы сборка у жюри шла на тех же пакетах, что у нас. Обновить:
#   docker pull ros:humble-ros-base && docker inspect ros:humble-ros-base \
#     --format '{{json .RepoDigests}}'
# Интернет нужен только на шаге apt/pip при сборке образа. Сам colcon build
# пакетов (tram_vehicle_msgs, tram_msgs, tram_state_estimator) сети не требует:
# проверено в чистом ros:humble-ros-base с `docker run --network none`.
ARG BASE_IMAGE=ros:humble-ros-base@sha256:1813d3c85d7f96ff7d3012d865204583255740182db5d0065f8f8cd029a83138
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONHASHSEED=0 \
    PYTHONUNBUFFERED=1 \
    RCUTILS_LOGGING_BUFFERED_STREAM=0

# python3-psutil — CPU/RSS ноды в tools/ros_probe.py; rosbridge — мост для
# симулятора (compose: сервис bridge); остальное — оценка и тесты.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy python3-yaml python3-pytest python3-pip python3-psutil \
        python3-matplotlib \
        ros-humble-rosbag2-storage-default-plugins \
        ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# rosbags — чтение bag без ROS для офлайн-оценки (tools/eval.py) и фикстур тестов
RUN pip3 install --no-cache-dir rosbags==0.10.9

WORKDIR /ws
COPY ros2_ws/src /ws/src
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --event-handlers console_direct- \
    && echo "source /ws/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "tram_state_estimator", "tram.launch.py"]
