# LoRa SDR project

This project will make use of SDR hardware to receive and decode Lora.

* Blog: https://myriadrf.org/blog/lora-modem-limesdr/

## Repository layout

* LoRa*.cpp - Pothos processing blocks and unit tests
* RN2483.py - python utility for controlling the RN2483
* examples/lora_simulation.pth - modem simulation
* examples/lora_sdr_relay.pth - LimeSDR LoRa relay
* examples/lora_sdr_client.pth - LimeSDR LoRa client

## Building project

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
