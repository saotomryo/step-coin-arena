FROM ros:jazzy-ros-base-noble@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-venv python3-pip build-essential git ca-certificates nodejs npm \
    ros-jazzy-rmw-cyclonedds-cpp xvfb x11vnc novnc websockify openbox \
    libgl1-mesa-dri libglx-mesa0 libegl1 libglu1-mesa libglfw3 \
    libxkbcommon-x11-0 libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 \
    libxcb-shape0 libxcb-xinerama0 libxcb-xkb1 libxcb-render-util0 \
    fonts-noto-cjk dbus-x11 x11-utils tini \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH=/opt/venv/bin:$PATH
WORKDIR /app
COPY requirements-genesis143.in requirements-workshop.in ./
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir torch==2.9.1 --index-url https://download.pytorch.org/whl/cpu \
    && pip install --no-cache-dir -r requirements-genesis143.in -r requirements-workshop.in "setuptools==79.0.1"
# Override at build time with a tested CLI release for a reproducible publication.
ARG CODEX_VERSION=0.163.0-alpha.10.2
RUN npm install -g @openai/codex@${CODEX_VERSION} && npm cache clean --force
RUN useradd --create-home --uid 10001 robot && mkdir -p /data /home/robot/.codex \
    && chown -R robot:robot /data /home/robot
# Allowlist source only: local logs, credentials and virtualenvs never enter the image.
COPY --chown=robot:robot xrobocon /app/xrobocon
COPY --chown=robot:robot core /app/core
COPY --chown=robot:robot baselines /app/baselines
COPY --chown=robot:robot training_flat /app/training_flat
COPY --chown=robot:robot configs /app/configs
COPY --chown=robot:robot scripts /app/scripts
COPY --chown=robot:robot tests /app/tests
COPY --chown=robot:robot docker /app/docker
RUN pip install --no-cache-dir tzdata==2026.5 && python -m pip check \
    && mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix \
    && chmod +x /app/docker/entrypoint.sh
ENV XROBOCON_RESOURCE_ROOT=/data XROBOCON_ROS2_RUNTIME=system \
    CODEX_HOME=/home/robot/.codex DISPLAY=:99 QT_QPA_PLATFORM=xcb \
    LIBGL_ALWAYS_SOFTWARE=1 PYTHONUNBUFFERED=1
USER robot
EXPOSE 6080 5900
ENTRYPOINT ["/usr/bin/tini", "--", "/app/docker/entrypoint.sh"]
CMD ["desktop"]
