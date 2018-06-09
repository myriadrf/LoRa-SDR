# LoRa SDR project

This project will make use of SDR hardware to receive and decode Lora.

* Blog: https://myriadrf.org/blog/lora-modem-limesdr/

## Repository layout

* LoRa*.cpp - Pothos processing blocks and unit tests
* RN2483.py - python utility for controlling the RN2483
* examples/ - saved Pothos topologies with LoRa blocks

## Noise simulation

This example demonstrates the LoRa PHY blocks
using a looback path in the presence of noise.

* examples/lora_simulation.pth - modem simulation

## RN2483 receiver

This example receives and demodulates raw symbols
with logic analyzer plot to view the symbols
and triggered waveform plots to view the input.
Once the example is activated, simply run the RN2483.py
script to generate a single waveform to trigger the plots.

* RN2483.py --freq=863.1e6 --bw=0.5e6 --sf=11 --tx="hello"
* examples/rx_RN2483.pth

## Simple relay

This example includes a simple client and relay app.
The relay receives and decodes messages and
relays them into another frequency and sync word.
The client can post messages to the relay
and view the response in a chat box widget.

* examples/lora_sdr_relay.pth - LimeSDR LoRa relay
* examples/lora_sdr_client.pth - LimeSDR LoRa client

## Building project

* First install pothos: https://github.com/pothosware/pothos/wiki
* Next build the blocks in this repository:

```
git clone https://github.com/myriadrf/LoRa-SDR.git
cd LoRa-SDR
mkdir build
cd build
cmake ../
make -j4
sudo make install
```
