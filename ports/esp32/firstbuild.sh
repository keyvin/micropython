pushd ~/

git clone https://github.com/espressif/esp-idf.git
cd esp-idf
curl https://bootstrap.pypa.io/get-pip.py --output get-pip.py

git checkout 4c81978a3e2220674a432a588292a4c860eef27b
git submodule --init --recursive


echo -e "\e[31minstalling python requirements via pip\e[0m"
sudo python2 get-pip.py
sudo apt install python-pip-whl
sudo apt-get install python3-venv
pip install --upgrade virtualenv==16.7.9
sudo pip install -r requirements.txt
./install.sh

popd

python3 -m venv build-venv
source build-venv/bin/activate
pip3 install --upgrade pip
pip3 install -r ~/esp-idf/requirements.txt

python3 -m venv build-venv
source build-venv/bin/activate


echo -e "\e[31mAdding user to dialout - will need to re-login to get access to com devices\e[0m"
sudo adduser $USER dialout
echo "Installing picocom"
sudo apt install picocom

echo -e "\e[0m"
git submodule update --init
pushd ../../mpy-cross/
make
popd

echo && echo


echo "use make board=GENERIC-SPIRAM to build spiram version"
echo "If this script finished with no errors.."
echo "source ~/esp-idf/exports.h  #imports esp tools"
echo "#source build-venv/bin/activate #import venv" 
echo "make && make deploy"
echo "picocom -b 115200 /dev/ttyUSB0"
echo "make BOARD=GENERIC_SPIRAM USER_C_MODULES=../../modules CFLAGS_EXTRA=-DMODULE_OUTPUT_ENABLED=1 all"

echo -e "\e[0m"
