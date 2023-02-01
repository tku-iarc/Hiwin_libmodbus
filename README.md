## Modbus Library for Hiwin Robot

### Getting start

#### Clone this Repository
```
cd <your_workspace>/src
git clone --recurse-submodules https://github.com/tku-iarc/Hiwin_libmodbus.git
```

#### Install Libmodbus
```
# build and install libmodbus to /usr/local/lib
cd Hiwin_libmodbus/libmodbus
./autogen.sh
./configure
sudo make install
# end

# write the export into bashrc so that every time you open a new terminal, it will automatically export
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
```

#### Build
```
cd <your_workspace>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Run the Test Script
```
cd <your_workspace>
source install/setup.bash
ros2 run hiwin_libmodbus Hiwin_API_test.py
```
