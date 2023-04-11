FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# Define which Neovim COC extensions should be installed
ARG COC="coc-clangd coc-cmake coc-docker coc-git coc-json coc-pyright coc-snippets coc-vimlsp"

# Install system wide packages (alphabetical order)
RUN apt-get update && apt-get install -y --no-install-recommends \
    bat \
    ccls \
    clangd \
    clang-format \
    curl \
    fd-find \
    fzf \
    git \
    grep \
    language-pack-en \
    less \
    iproute2 \
    openssh-server \
    python3 \
    python3-pip \
    sudo \
    tmux \
    tree \
    wget \
    xauth \
    xclip

# Install latest nodejs (needed for Neovim)
RUN curl -sL install-node.vercel.app/lts | bash -s  -- --yes

# Install pip applications
RUN pip3 install pynvim \
    && pip3 install cmake-language-server \
    && pip3 install autopep8 \
    && pip3 install flake8

# Cooperate NodeJS with Neovim
RUN npm i -g neovim

# Install latest Neovim appimage and extract
RUN mkdir -p /nvim && cd /nvim \
    && curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim.appimage \
    && chmod +x ./nvim.appimage \
    && ./nvim.appimage --appimage-extract \
    && chmod +x /nvim/squashfs-root/usr/bin/nvim \
    && ln -s /nvim/squashfs-root/usr/bin/nvim /usr/local/bin

# Create user and add user to the sudoers group
RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo
# Switch to the created user
USER docker

# Install personal dotfiles (https://github.com/arensonzz/dotfiles)
RUN cd ~ \
    && git init --bare $HOME/.dotfiles \
    && /usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME config --local status.showUntrackedFiles no \
    && /usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME remote add origin https://github.com/arensonzz/dotfiles.git \
    && /usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME fetch origin master \
    && /usr/bin/git --git-dir=$HOME/.dotfiles/ --work-tree=$HOME reset --hard origin/master


# Install user packages
#   tmux
RUN git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm && ~/.tmux/plugins/tpm/bin/install_plugins
#   neovim: install plugin manager, install plugins, install coc extensions
RUN curl -fLo ~/.local/share/nvim/site/autoload/plug.vim --create-dirs https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim \
    && nvim --headless +"PlugInstall --sync" +"qall" \
    && mkdir -p ~/.config/coc/extensions \
    && cd ~/.config/coc/extensions && npm install $COC --omit=dev

# Install nvim-treesitter parsers, sleep is needed for sync operation
# Default parsers are defined in the plugins.vim file as follows:
#   bash c cmake comment cpp dockerfile json lua make python regex vim vimdoc yaml
RUN nvim --headless +"TSInstallSync!" +'sleep 20' +'qall'

### ROS
# Create a Catkin workspace and clone repos
RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p ~/catkin_ws/src \
 && cd ~/catkin_ws/src \
 && catkin_init_workspace
 # && git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git \
 
# Build the Catkin workspace
RUN source /opt/ros/noetic/setup.bash \
 && cd ~/catkin_ws \
 && catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
 
# Set the working folder at startup
WORKDIR /home/docker/catkin_ws

# Copy LSP settings for Neovim
COPY .vim/coc-settings.json /home/docker/.config/nvim/
# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]   
