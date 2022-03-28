# Modbus_Hiwin

Installation

舉例以 Ubuntu 或 Debian 來說，作業系統的套件資料庫已經包含了 libmodbus 的套件，只需要輸入以下指令即可完成安裝：

安裝
`$ sudo apt-get install libmodbus5 libmodbus-dev`

如需安裝最新版(請查閱)  [https://libmodbus.org/releases/](https://libmodbus.org/releases/)
`libmodbus-3.0.5`->更換成新版本

`$ wget http://libmodbus.org/site_media/build/libmodbus-3.1.7.tar.gz`

`$ sudo chmod 755 libmodbus-3.1.7.tar.gz`
 
`$ tar zxvf libmodbus-3.1.7.tar.gz -C .`

`$ cd libmodbus-3.1.7/`

`$ ./configure`

`$ make`

`$ sudo make install`

於此即完成安裝。

編譯程式 `cc -o Hiwin_API Hiwin_API.c -lmodbus -I./`

生成so檔 `cc -fPIC -shared -o Hiwin_API.so Hiwin_API.c -lmodbus -I./`

`gcc -shared -Wl,-soname,Hiwin_API-o Hiwin_API.so -fPIC Hiwin_API.c`

執行 `python3 Hiwin_API_test.py`
