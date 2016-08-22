// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Testing.hpp>
#include <Pothos/Framework.hpp>
#include <Pothos/Proxy.hpp>
#include <Pothos/Remote.hpp>
#include <Poco/JSON/Object.h>
#include <iostream>
#include "LoRaCodes.hpp"

POTHOS_TEST_BLOCK("/lora/tests", test_hamming)
{
    bool error;
    unsigned char decoded;

    //test hamming 74 with bit errors
    for (size_t i = 0; i < 16; i++)
    {
        unsigned char byte = i & 0xff;
        unsigned char encoded = encodeHamming74(byte);

        //check no bit errors
        decoded = decodeHamming74(encoded);
        POTHOS_TEST_EQUAL(byte, decoded);

        for (int bit0 = 0; bit0 < 8; bit0++)
        {
            //check 1 bit error
            unsigned char encoded1err = encoded ^ (1 << bit0);
            decoded = decodeHamming74(encoded1err);
            POTHOS_TEST_EQUAL(byte, decoded);
        }
    }

    //test hamming 84 with bit errors
    for (size_t i = 0; i < 16; i++)
    {
        unsigned char byte = i & 0xff;
        unsigned char encoded = encodeHamming84(byte);

        //check no bit errors
        error = false;
        decoded = decodeHamming84(encoded, error);
        POTHOS_TEST_TRUE(not error);
        POTHOS_TEST_EQUAL(byte, decoded);

        for (int bit0 = 0; bit0 < 8; bit0++)
        {
            //check 1 bit error
            error = false;
            unsigned char encoded1err = encoded ^ (1 << bit0);
            decoded = decodeHamming84(encoded1err, error);
            POTHOS_TEST_TRUE(not error);
            POTHOS_TEST_EQUAL(byte, decoded);

            for (int bit1 = 0; bit1 < 8; bit1++)
            {
                if (bit1 == bit0) continue;

                //check 2 bit errors (cant correct, but can detect
                error = false;
                unsigned char encoded2err = encoded1err ^ (1 << bit1);
                decoded = decodeHamming84(encoded2err, error);
                POTHOS_TEST_TRUE(error);
            }
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_interleaver)
{
    for (size_t PPM = 7; PPM <= 12; PPM++)
    {
        std::cout << "Testing PPM " << PPM << std::endl;
        for (size_t RDD = 0; RDD <= 4; RDD++)
        {
            std::cout << "  with RDD " << RDD << std::endl;
            std::vector<uint8_t> inputCws(PPM);
            const auto mask = (1 << (RDD+4))-1;
            for (auto &x : inputCws) x = std::rand() & mask;

            std::vector<uint16_t> symbols(((RDD+4)*inputCws.size())/PPM);
            diagonalInterleave(inputCws.data(), inputCws.size(), symbols.data(), PPM, RDD);

            std::vector<uint8_t> outputCws(inputCws.size());
            diagonalDeterleave(symbols.data(), symbols.size(), outputCws.data(), PPM, RDD);

            POTHOS_TEST_EQUALV(inputCws, outputCws);
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_encoder_to_decoder)
{
    auto env = Pothos::ProxyEnvironment::make("managed");
    auto registry = env->findProxy("Pothos/BlockRegistry");

    auto feeder = registry.callProxy("/blocks/feeder_source", "uint8");
    auto encoder = registry.callProxy("/lora/lora_encoder");
    auto decoder = registry.callProxy("/lora/lora_decoder");
    auto collector = registry.callProxy("/blocks/collector_sink", "uint8");

    std::vector<std::string> testCodingRates;
    testCodingRates.push_back("4/4");
    testCodingRates.push_back("4/5");
    testCodingRates.push_back("4/6");
    testCodingRates.push_back("4/7");
    testCodingRates.push_back("4/8");

    for (size_t SF = 7; SF <= 12; SF++)
    {
        std::cout << "Testing SF " << SF << std::endl;
        for (const auto &CR : testCodingRates)
        {
            std::cout << "  with CR " << CR << std::endl;
            encoder.callVoid("setSpreadFactor", SF);
            decoder.callVoid("setSpreadFactor", SF);
            encoder.callVoid("setCodingRate", CR);
            decoder.callVoid("setCodingRate", CR);

            //create a test plan
            Poco::JSON::Object::Ptr testPlan(new Poco::JSON::Object());
            testPlan->set("enablePackets", true);
            testPlan->set("minValue", 0);
            testPlan->set("maxValue", 255);
            auto expected = feeder.callProxy("feedTestPlan", testPlan);

            //create tester topology
            {
                Pothos::Topology topology;
                topology.connect(feeder, 0, encoder, 0);
                topology.connect(encoder, 0, decoder, 0);
                topology.connect(decoder, 0, collector, 0);
                topology.commit();
                POTHOS_TEST_TRUE(topology.waitInactive());
                //std::cout << topology.queryJSONStats() << std::endl;
            }

            std::cout << "verifyTestPlan" << std::endl;
            collector.callVoid("verifyTestPlan", expected);
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_loopback)
{
    auto env = Pothos::ProxyEnvironment::make("managed");
    auto registry = env->findProxy("Pothos/BlockRegistry");

    const size_t SF = 10;
    auto feeder = registry.callProxy("/blocks/feeder_source", "uint8");
    auto encoder = registry.callProxy("/lora/lora_encoder");
    auto mod = registry.callProxy("/lora/lora_mod", SF);
    auto adder = registry.callProxy("/comms/arithmetic", "complex_float32", "ADD");
    auto noise = registry.callProxy("/comms/noise_source", "complex_float32");
    auto demod = registry.callProxy("/lora/lora_demod", SF);
    auto decoder = registry.callProxy("/lora/lora_decoder");
    auto collector = registry.callProxy("/blocks/collector_sink", "uint8");

    std::vector<std::string> testCodingRates;
    testCodingRates.push_back("4/7");
    testCodingRates.push_back("4/8");

    for (const auto &CR : testCodingRates)
    {
        std::cout << "Testing with CR " << CR << std::endl;

        encoder.callVoid("setSpreadFactor", SF);
        decoder.callVoid("setSpreadFactor", SF);
        encoder.callVoid("setCodingRate", CR);
        decoder.callVoid("setCodingRate", CR);
        mod.callVoid("setAmplitude", 1.0);
        noise.callVoid("setAmplitude", 4.0);
        noise.callVoid("setWaveform", "NORMAL");
        mod.callVoid("setPadding", 512);
        demod.callVoid("setMTU", 512);

        //create a test plan
        Poco::JSON::Object::Ptr testPlan(new Poco::JSON::Object());
        testPlan->set("enablePackets", true);
        testPlan->set("minValue", 0);
        testPlan->set("maxValue", 255);
        testPlan->set("minBuffers", 5);
        testPlan->set("maxBuffers", 5);
        testPlan->set("minBufferSize", 8);
        testPlan->set("maxBufferSize", 128);
        auto expected = feeder.callProxy("feedTestPlan", testPlan);

        //create tester topology
        {
            Pothos::Topology topology;
            topology.connect(feeder, 0, encoder, 0);
            topology.connect(encoder, 0, mod, 0);
            topology.connect(mod, 0, adder, 0);
            topology.connect(noise, 0, adder, 1);
            topology.connect(adder, 0, demod, 0);
            topology.connect(demod, 0, decoder, 0);
            topology.connect(decoder, 0, collector, 0);
            topology.commit();
            POTHOS_TEST_TRUE(topology.waitInactive());
            //std::cout << topology.queryJSONStats() << std::endl;
        }

        std::cout << "decoder dropped " << decoder.call<unsigned long long>("getDropped") << std::endl;
        std::cout << "verifyTestPlan" << std::endl;
        collector.callVoid("verifyTestPlan", expected);
    }
}
