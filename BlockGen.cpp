// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0
#include <Pothos/Framework.hpp>
#include <iostream>
#include <cstring>

/***********************************************************************
* |PothosDoc LoRa Block Gen
*
* Generate test blocks for the LoRa modulator for testing purposes.
*
* |category /LoRa
* |keywords lora
*
* |param elements Specify a list of elements to produce.
* |default [100, 200, 300, 400, 500, 600 ,700, 800]
*
* |param ws[Word Size] The number of error correction bits.
* |option [8] "8"
* |option [16] "16"
* |option [32] "32"
* |default "16"
*
* |factory /lora/block_gen()
* |setter setElements(elements)
* |setter setWordSize(ws)
*
**********************************************************************/
class BlockGen : public Pothos::Block
{
public:
	BlockGen(void) : _ws(1) {
		this->setupOutput(0);
		this->registerCall(this, POTHOS_FCN_TUPLE(BlockGen, setElements));
		this->registerCall(this, POTHOS_FCN_TUPLE(BlockGen, setTrigger));
		this->registerCall(this, POTHOS_FCN_TUPLE(BlockGen, setWordSize));
	}

	void setElements(const std::vector<uint32_t> &elems) {
		_elements = elems;
		_active = true;
	}

	void setTrigger(const int value) {
		_active = true;
	}

	static Block *make(void){
		return new BlockGen();
	}

	void setWordSize(const std::string &ws){
		if (ws == "8") _ws = 0;
		else if (ws == "16") _ws = 1;
		else if (ws == "32") _ws = 2;
		else throw Pothos::InvalidArgumentException("LoRaBlockGen::setWordSize(" + ws + ")", "unknown word size");
	}

	void activate(void){
		_active = true;
	}

	void work(void){
		if (!_active) return;
		_active = false;
		Pothos::Packet outPkt;
		if (0 == _ws) {
			Pothos::BufferChunk msgBuff(typeid(uint8_t), _elements.size());
			uint8_t *p = msgBuff.as<uint8_t *>();
			for (size_t i = 0; i < _elements.size(); i++) {
				p[i] = _elements[i] & 0xff;
			}
			outPkt.payload = msgBuff;
		}else if (1 == _ws){
			Pothos::BufferChunk msgBuff(typeid(uint16_t), _elements.size());
			uint16_t *p = msgBuff.as<uint16_t *>();
			for (size_t i = 0; i < _elements.size(); i++) {
				p[i] = _elements[i] & 0xffff;
			}
			outPkt.payload = msgBuff;
		}else {
			Pothos::BufferChunk msgBuff(typeid(uint32_t), _elements.size());
			uint32_t *p = msgBuff.as<uint32_t *>();
			for (size_t i = 0; i < _elements.size(); i++) {
				p[i] = _elements[i];
			}
			outPkt.payload = msgBuff;
		}
		
		this->output(0)->postMessage(outPkt);
	}

private:
	//configuration
	bool _active;
	size_t _ws;
	std::vector<uint32_t> _elements;
};

static Pothos::BlockRegistry registerBlockGen(
	"/lora/block_gen", &BlockGen::make);
