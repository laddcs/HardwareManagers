# HardwareManagers
ROS2 that manage hardware components on LADDCS drones.

## Setup HowTo

# Step 1: Install the libirimager package
```
sudo apt-get install cmake freeglut3-dev libusb-1.0-0-dev 
cd Downloads && wget https://github.com/Optris/irdirectsdk_downloads/releases/download/v8.9.3/libirimager-8.9.3-ubuntu-22.04-arm64.deb
sudo dpkg -i libirimager-8.9.3-ubuntu-22.04-arm64.deb
sudo ir_download_calibration
```
Run 'ir_version' to confirm that the package was installed and is working properly.

# Step 2: Install ROS2 Humble. 
Go to https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html and follow the directions on the site.
You should end up installng `ros-humble-ros-base` and `ros-dev-tools`.

Make sure to edit the end of the `~/.bashrc` with
```
source /opt/ros/humble/setup.bash
```

# Step 3: Configure ethernet ip address for PX4
Follow the steps in https://www.forecr.io/blogs/connectivity/how-to-set-static-ip-for-jetson. 
This will require a monitor / mouse and keyboard. Instead of the example ip address they use, use `192.168.0.2`.
If the pixhawk is plugged in and powered on it should respond to 
```
ping 192.168.0.4
```

# Step 4: Setup WFB-NG
Clone the repo and other dependencies and checkout the correct version. rtl8812au is the driver for the alpha-awus wifi-module.
```
git clone https://github.com/svpcom/wfb-ng.git
cd wfb-ng && git checkout origin/release-24.08

cd ..
git clone https://github.com/svpcom/rtl8812au.git
cd rtl8812au && make -j6
sudo make install
cd ..
```

Build and setup wfb-ng
```
git clone https://github.com/jedisct1/libsodium --branch stable
cd libsodium
./configure
make -j6 && make check
sudo make install
cd ..

sudo apt-get install virtualenv fakeroot debhelper
sudo apt-get install python3-twisted libpcap-dev libsodium-dev python3-pyroute2 python3-future python3-all-dev dh-python python3-serial python3-msgpack

make deb
sudo dpkg -i wfb_ng*.deb
```

Now you need to setup the wfb config files. 
First, plug in the wificard and run `ifconfig`. You should get a block containing something looking like this:
```
wlx00c0cab49ead: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 4052
```
`wlx00c0cab49ead` is the serial number of the wifi card. Do:
```
sudo nano /etc/default/wifibroadcast
```
and replace the line `#WFB_NICS="wlan0"` with `#WFB_NICS="wlx00c0cab49ead"`. Save and exit out of nano.

Now do
```
sudo nano /etc/wifibroadcast.cfg
```
and copy in
```
[common]
wifi_channel = 161     # 161 -- radio channel @5825 MHz, range: 5815–5835 MHz, width 20MHz
                       # 1 -- radio channel @2412 Mhz, 
                       # see https://en.wikipedia.org/wiki/List_of_WLAN_channels for reference
wifi_region = 'BO'     # Your country for CRDA (use BO or GY if you want max tx power)

[drone_mavlink]
# use autopilot connected to /dev/ttyUSB0 at 115200 baud:
# peer = 'serial:ttyUSB0:115200'

# Connect to autopilot via malink-router or mavlink-proxy:
peer = 'listen://0.0.0.0:14550'   # incoming connection
# peer = 'connect://127.0.0.1:14550'  # outgoing connection

[drone_video]
peer = 'listen://0.0.0.0:5602'  # listen for video stream (gstreamer on drone)
```
Save and exit out of nano. 
Finally, copy `drone.key` from your machine onto the Jetson with sftp. Then on the Jetson, copy `drone.key` to `/etc/`. 

Run
```
sudo systemctl daemon-reload
sudo systemctl start wifibroadcast@drone
sudo systemctl status wifibroadcast.@drone
```
If it says the service is up, go ahead and run `sudo systemctl enable wifibroadcast.service` to start the service on boot.
On reboot, you can start wifibroadcast on the ground station and run `wfb-cli gs` and if you see rx mavlink packets then the setup worked. You should be able to start QGroundControl and it will auto connect over the UDP link.
Also you can connect with ssh via `ssh hex@10.5.0.2`

# Step 5: Install uXRCE-DDS Agent
```
cd Downloads
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build

cmake ..
make -j6
sudo make install
sudo ldconfig /usr/local/lib/
```

Confirm that the install worked with
```
MicroXRCEAgent udp4 --port 8888 -v
```
If the Pixhawk is powered on then you should be able to see px4 topics on the ground station with `ros2 topic list`.

# Step 6: Install the hardware managers
```

```