# LoRa SDR project

This project will make use of SDR hardware to receive and decode Lora.

## Building dependencies

* First install pothos: https://github.com/pothosware/pothos/wiki
* Next build the blocks in this repository:

```
got clone https://github.com/myriadrf/LoRa-SDR.git
cd LoRa-SDR
mkdir build
cd build
cmake ../
make -j4
sudo make install
```
