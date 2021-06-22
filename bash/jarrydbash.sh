 #!/bin/bash

sudo adduser $USER dialout
sudo apt-get install python-pip python-dev
pip install dronekit
sudo apt-get install -y python3-serial

sudo usermod -a -G dialout ${USER}
sudo usermod -a -G tty ${USER}


sudo apt-get install qttools5-dev-tools
sudo apt-get install qttools5-dev
pip3 install --user pyqt5  
sudo apt-get install python3-pyqt5  
sudo apt-get install pyqt5-dev-tools
sudo apt-get install qttools5-dev-tools 

