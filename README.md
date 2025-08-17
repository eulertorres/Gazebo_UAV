Instalação

sudo sh -c 'wget -O - https://packages.osrfoundation.org/gazebo.key | apt-key add -'
gsettings set org.gnome.shell.extensions.dash-to-dock click-action 'minimize'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
sudo apt-get install ros-noetic-rospy
sudo apt install flameshot
sudo apt install wine64
sudo snap install notepad-plus-plus
sudo apt install libignition-gazebo6 libignition-gazebo6-dev
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
sudo apt install -y python3-pip python3-dev python3-setuptools g++ make
sudo pip3 install future
./waf configure --board sitl
./waf copter
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
sudo pip3 install MAVProxy
echo "export PATH=$PATH:$HOME/ardupilot/Tools/autotest" >> ~/.bashrc
echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.bashrc
source ~/.bashrc
sudo ln -s /usr/bin/python3 /usr/bin/python
sudo apt install python3-wxgtk4.0
cd ~/
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
sudo apt remove modemmanager
sudo dpkg --add-architecture i386
sudo mkdir -pm755 /etc/apt/keyrings
sudo wget -O /etc/apt/keyrings/winehq-archive.key https://dl.winehq.org/wine-builds/winehq.key
sudo wget -NP /etc/apt/sources.list.d/ https://dl.winehq.org/wine-builds/ubuntu/dists/$(lsb_release -cs)/winehq-$(lsb_release -cs).sources
sudo apt update
sudo apt install --install-recommends winehq-stable
sudo apt install ca-certificates gnupg
sudo gpg --homedir /tmp --no-default-keyring --keyring /usr/share/keyrings/mono-official-archive-keyring.gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
echo "deb [signed-by=/usr/share/keyrings/mono-official-archive-keyring.gpg] https://download.mono-project.com/repo/ubuntu stable-focal main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update
sudo apt install mono-devel
source /opt/ros/noetic/setup.bash 
sudo bash -c 'source /opt/ros/noetic/setup.bash; rosrun mavros install_geographiclib_datasets.sh'
cd ~/catkin_ws
source devel/setup.bash
sudo add-apt-repository ppa:obsproject/obs-studio
sudo apt update
sudo apt install obs-studio
echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib' >> ~/.bashrc
source ~/.bashrc
cd ~/catkin_ws/src
git clone https://github.com/eulertorres/Euler_Drone_Sim
cd  Euler_Drone_Sim/utilidades
pip install -r requirements.txt
mkdir ~/.mavproxy/joysticks
cp ~/catkin_ws/src/Euler_Drone_Sim/utilidades/frsky_x18.yaml ~/.mavproxy/joysticks
sudo apt install htop
sudo apt update
sudo apt install unrar
