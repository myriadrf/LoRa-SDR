// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Framework.hpp>
#include <iostream>
#include <cstring>

/***********************************************************************
 * |PothosDoc LoRa Test Gen
 *
 * Generate test messages for the LoRa encoder for testing purposes.
 *
 * |category /LoRa
 * |keywords lora
 *
 * |factory /lora/test_gen()
 **********************************************************************/
class TestGen : public Pothos::Block
{
public:
    TestGen(void)
    {
        this->setupOutput(0);
    }

    static Block *make(void)
    {
        return new TestGen();
    }

    void activate(void)
    {
        _count = 0;
    }

    void work(void)
    {
        auto msgStr = std::to_string(_count++);
        Pothos::BufferChunk msgBuff(typeid(uint8_t), msgStr.size());
        std::memcpy(msgBuff.as<void *>(), msgStr.data(), msgStr.size());
        Pothos::Packet outPkt;
        outPkt.payload = msgBuff;
        this->output(0)->postMessage(outPkt);
    }

private:
    //configuration
    unsigned long long _count;
};

static Pothos::BlockRegistry registerTestGen(
    "/lora/test_gen", &TestGen::make);
