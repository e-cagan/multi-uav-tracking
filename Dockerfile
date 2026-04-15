# Temel ROS 2 Humble imajı
FROM osrf/ros:humble-desktop

# Etkileşimli (interaktif) kurulum diyaloglarını engelle
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1. Adım: Yeni depo (repository) ekleyebilmek için gerekli temel araçları kur
RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# 2. Adım: Gazebo'nun resmi uygulama deposunu Docker'ın içine ekle
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. Adım: Sistem bağımlılıklarını kur (Artık Gazebo paketini bulabilecektir)
RUN apt-get update && apt-get install -y \
    bash \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-ros-gzgarden-bridge \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# MAVROS için GeographicLib datasetlerini kur
RUN wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

# AMAV algı (perception) pipeline'ı için Python bağımlılıklarını kur
# DİKKAT: Versiyonlar çalışan yerel sisteme (pip list) göre sabitlenmiştir.
RUN pip3 install --no-cache-dir \
    numpy==1.26.4 \
    ultralytics==8.3.176 \
    supervision==0.27.0.post2 \
    opencv-python-headless==4.9.0.80

# Çalışma dizinini ayarla
WORKDIR /amav_ws

# Host makinedeki src klasörünü ve yolo modelini konteynere kopyala
COPY src/ ./src/
COPY yolov8n.pt ./

# Workspace'i ROS 2 ortamıyla birlikte derle
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Otomatik environment yükleme scripti (Entrypoint)
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /amav_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

# Konteyner başlarken çalıştırılacak giriş noktası
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]