## Modbus Library for Hiwin Robot

### Getting start

#### Clone this Repository
```
cd <your_workspace>/src
git clone --recurse-submodules https://github.com/tku-iarc/Hiwin_libmodbus.git
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
