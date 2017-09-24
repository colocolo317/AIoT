
# Agriforward-iot

## Hardware
1. SHT20.
2. mbed enable board. 


## Features

- CRC check (Still have some bugs).
- Get `SHT2X sensor` data from I2C protocol.
- Transmit sensor data to the server through `http protocol`

## Installation
1. Install mbed cli. Please refer to the doc of [mbed-cli](https://github.com/ARMmbed/mbed-cli)
2. Install python 3. Please refer to [python doc](https://www.python.org).
3. Install python 3 module `Flask` to open server by command `pip install Flask`.

## How to use
1. `cd <path you want to put this program>`
2. `mbed new https://github.com/qmo1222/Agriforward-iot`
3. `cd Agriforward-iot`
4. `mbed deploy`
5. Change the `SSID` and `PASSWORD` in `mbed_app.json`
6. Change `main.cpp` define variable `HOST_URL` to your server host url.(Port is 8080.)
7. `python ./server/server.py`  
8. `mbed compile -t GCC_ARM -m <The board name> -f`(Assume your board is already connected to the pc.)

## Debug
1. If you have some bugs, you may use `screen` and set `boudrate=115200` to minitor your serial port connected to your board.


Contribute
----------

- Serge Sozonoff source code: [code](https://developer.mbed.org/users/ssozonoff/code/SHT2x/file/2464fed17980/SHT2x.h)
- mxchip EMW3080 source code: [code](https://developer.mbed.org/teams/mxchip-EMW3080/code/emw3080-wifi-example/)
- sandbox mbed-http source code: [code](https://developer.mbed.org/teams/sandbox/code/mbed-http/)

Support
-------

1. Qmo : `https://github.com/qmo1222`
2. YiChun: `https://github.com/YiChunHung`

