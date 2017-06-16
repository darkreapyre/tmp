#! /bin/bash
sudo apt-get update
sudo apt-get install -y zip unzip libuv1-dev libssl-dev gcc g++ cmake make gfortran cppad
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.1.zip
unzip Ipopt-3.12.1.zip
rm Ipopt-3.12.1.zip
sudo bash install_ipopt.sh Ipopt-3.12.1
