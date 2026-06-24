# Dev Container Setup

This directory contains configuration files for running a **development container** (dev container) using VS Code's [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

## Prerequisites

- **VS Code** with the **Dev Containers** extension installed (`ms-vscode-remote.remote-containers`).
- A container runtime (Docker or Podman) available on your system.

## Setup

### 1. Copy the files to your workspace root

These files are located under `src/crospi_application_template/.devcontainer/`. The Dev Containers extension expects them in a `.devcontainer` folder at the **root of your ROS 2 workspace**.

From your workspace root, run:

```bash
mkdir -p .devcontainer
cp src/crospi_application_template/.devcontainer/devcontainer.json .devcontainer/devcontainer.json
cp src/crospi_application_template/.devcontainer/Dockerfile .devcontainer/Dockerfile
```

> **Note:** Use `cp` (copy) rather than symlinks. The Docker build context resolves relative to the `.devcontainer` folder, and symlinks can cause issues during the image build.

### 2. Open the workspace in VS Code

Open the workspace root folder in VS Code:

```bash
code <my_workspace>
```

### 3. Reopen in container

- Press `F1` and run **Dev Containers: Reopen in Container**.
- Or click the green button `><` in the bottom-left corner and select **Reopen in Container**.

VS Code will build the Docker image and start the container. This may take a few minutes on the first run.

## Container Details

- **Default base image:** `ghcr.io/robotics-research-group-kul/crospi-betfsm-podman:ros2-lyrical`
- **User:** `developer` (UID 1011, with passwordless `sudo`)
- **Workspace path (inside container):** `/home/developer/devel_ws`
- **Networking:** `--network=host` (the container shares the host's network stack)
- **X11 forwarding:** Configured via mounted sockets for GUI applications.
- **Real-time scheduling:** `rtprio` ulimit set to 98, `memlock` unlimited.

## Important Considerations

### Privileged mode

The container runs with `--privileged` and several security opts (`seccomp:unconfined`, `apparmor:unconfined`). This is required for real-time control and hardware access in robotics applications. Be aware that this lowers container isolation.

### Port conflicts

With `--network=host`, any service you start inside the container binds directly to the host's network interfaces. Make sure ports do not conflict with other services on your host.

### GUI applications

The `DISPLAY`, `WAYLAND_DISPLAY`, `XDG_RUNTIME_DIR`, and `PULSE_SERVER` environment variables are forwarded from the host. `LIBGL_ALWAYS_SOFTWARE=1` is set for software OpenGL rendering. If you need hardware-accelerated graphics, you may need to remove that variable and mount the GPU device.

### ROS 2 workspace

The container's working directory `/home/developer/devel_ws` maps directly to your workspace root on the host via a bind mount. All changes in this workspace persist between your container and your machine. Build your ROS 2 packages inside the container as you normally would:

```bash
cd /home/developer/devel_ws
colcon build --symlink-install
```

### Permission errors when creating files

The container user is `developer` (UID 1000). If you encounter permission issues with files created inside the container, ensure your host user's UID matches or adjust the `USER_UID` argument in the `Dockerfile`.
