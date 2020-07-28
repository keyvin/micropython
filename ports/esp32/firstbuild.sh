cd ~/

git clone https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout 4c81978a3e2220674a432a588292a4c860eef27b
git submodule --init --recursive
echo "installing python requirements via pip"
sudo pip install -r requirements.txt
./install.sh








echo "Adding user to dialout - will need to re-login to get access to com devices"
sudo adduser $USER dialout
echo "Installing picocom"
sudo apt install picocom

echo "use make board=GENERIC-SPIRAM to build spiram version"
echo "If this script finished with no errors"
echo "import idf tools -\". ~/esp-idf/exports.h \""
echo "make && make deploy"
echo "picocom -b 115200 /dev/ttyUSB0"
