# Образ Vectra: ROS 2 Humble + пакеты оценщика, собранные внутри образа.
# Все зависимости ставятся на этапе сборки образа; colcon build внутри
# контейнера интернета не требует.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONHASHSEED=0

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy python3-yaml python3-pytest python3-pip python3-psutil \
        python3-matplotlib \
        ros-humble-rosbag2-storage-default-plugins \
        ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# rosbags — чтение bag без ROS для офлайн-оценки (tools/eval.py)
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
