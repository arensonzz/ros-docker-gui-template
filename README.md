# ROS Docker GUI Template

This template is a starting point for _ROS1 Noetic_ development inside Docker. It contains full Noetic desktop package,
compilers (gcc) and language servers (clangd, ccls) for CPP, programming tools like _git_ and _grep_, last but not least _tmux_ terminal multiplexer. Also my favorite text editor _Neovim_ comes installed with many plugins and
configurations. 

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
## Contents

- [About](#about)
  - [Installed Software](#installed-software)
- [Requirements](#requirements)
- [Usage](#usage)
- [References](#references)
- [License](#license)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## About

I made this template because latest ROS1 version, Noetic, runs only on Ubuntu 20.04 and I happen to be using Ubuntu
22.04. Instead of building Noetic from source, I prefer running it inside Docker container.

The Dockerfile contains my personal dotfiles for bash, tmux and Neovim. So feel free to fork the repo and tailor it to
your liking.

### Installed Software

- **bat** : cat clone with syntax highlighting
- **ccls** : C/CPP language server
- **clangd** : C/CPP language server
- **clang-format**  : C/CPP file formatter
- **curl** : file downloader
- **fd-find** : find alternative
- **fzf** : fuzzy file finder
- **git** : version control tool
- **grep** : pattern matcher
- **language-pack-en** : ubuntu language packs for English
- **less** : file pager
- **iproute2** : ip tools
- **openssh-server** : ssh server if you want to connect with SSH
- **python3** : Python3 programming language
- **python3-pip** : package manager for Python3 
- **sudo** 
- **tmux** : terminal multiplexer
- **tree** : tree like file viewer
- **wget** : file downloader
- **xauth** : authorization utility for X servers
- **xclip** : system clipboard manager
- **nodejs** : needed for Neovim plugins
- **cmake-language-server** : LSP for cmake
- **autopep8** : formatter for Python
- **flake8** : linter for Python
- **neovim** : text editor

## Requirements

1. This is tested using **Ubuntu 22.04** installed Linux machine. Docker image should run fine in a Windows machine but X
   Window setup differs. You must install an X server application like [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv).
1. This is tested using a machine with **NVIDIA GPU**. Docker container uses host computer's GPU using NVIDIA Container
   Toolkit. For different GPU vendors, some tweaks such as disabling GPU support in `run_docker.sh` might be needed.
1. [Docker engine](https://docs.docker.com/engine/install/) must be installed on the host OS.
1. X Window Authorization:
    1. Install `xauth` and `xhost` if you don't have them.

        ```sh
        sudo apt update && sudo apt install -y xauth xhost
        ```
    1. If you don't have `~/.Xauthority` file, generate with the following command:

        ```sh
        xauth generate $DISPLAY . trusted
        ```

    1. Allow local connections to your X Window server. This step allows your container to access host X Window without
       using `--net=host` Docker run option.

        ```sh
        # Don't forget the trailing colon
        xhost +local:
        ```
1. [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) 
    must be installed on the host OS.
## Usage
1. Add your CPP code to the `package/src` folder.
1. Modify `package/CMakeLists.txt` and `package/package.xml` to suit your project's needs.
1. Modify `Dockerfile` to suit your project's needs. Install needed programs and clone needed libraries.
1. Modify `entrypoint.sh`. Source workspaces, export environment variables and add `bash` aliases here. 
1. Build Docker image.

    `sudo docker build -t "ros-dev" docker`
1. Run Docker container using the bash script `run_docker.sh`.

    `./run_docker.sh <image_name> <container_name>`
1. Docker container starts with a `tmux` session. `tmux` settings are defined in the `~/.tmux.conf`. Default prefix is
    set to `Ctrl` + `Space` for outer `tmux` session, and `Ctrl` + `A` for inner `tmux` session (if using nested
    `tmux`).
1. Attach to the same container from another terminal if you need to.

    `docker exec -it bash <container_name>`
1. Run `catkin_make` to build packages and create `compile_commands.json` inside `build` directory.
1. If you want to edit source codes inside package, run `nvim` from the root workspace directory which is `~/catkin_ws`.
    This way `ccls` LSP can properly access `compile_commands.json` and index your includes.

## References

- [A Guide to Docker and ROS - Robotic Sea Bass](https://roboticseabass.com/2021/04/21/docker-and-ros/) - How to setup
    Docker for ROS development
- [MashMB/nvim-ide](https://github.com/MashMB/nvim-ide) - Neovim as IDE in Docker container
- [nachovizzo/ros_in_docker](https://github.com/nachovizzo/ros_in_docker) - Run all your ROS1 nodes inside a completely
    isolated development environment

## License

ROS Docker GUI Template is free software published under the MIT license. See [LICENSE](LICENSE) for details.
