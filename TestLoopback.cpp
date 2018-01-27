// Copyright (c) 2016-2018 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Testing.hpp>
#include <Pothos/Framework.hpp>
#include <Pothos/Proxy.hpp>
#include <Pothos/Remote.hpp>
#include <iostream>
#include "LoRaCodes.hpp"
#include <json.hpp>

using json = nlohmann::json;

POTHOS_TEST_BLOCK("/lora/tests", test_encoder_to_decoder)
{
    auto env = Pothos::ProxyEnvironment::make("managed");
    auto registry = env->findProxy("Pothos/BlockRegistry");

    auto feeder = registry.call("/blocks/feeder_source", "uint8");
    auto encoder = registry.call("/lora/lora_encoder");
    auto decoder = registry.call("/lora/lora_decoder");
    auto collector = registry.call("/blocks/collector_sink", "uint8");

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
            encoder.call("setSpreadFactor", SF);
            decoder.call("setSpreadFactor", SF);
            encoder.call("setCodingRate", CR);
            decoder.call("setCodingRate", CR);

            //create a test plan
            json testPlan;
            testPlan["enablePackets"] = true;
            testPlan["minValue"] = 0;
            testPlan["maxValue"] = 255;
            auto expected = feeder.call("feedTestPlan", testPlan.dump());

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
            collector.call("verifyTestPlan", expected);
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_loopback)
{
    auto env = Pothos::ProxyEnvironment::make("managed");
    auto registry = env->findProxy("Pothos/BlockRegistry");

    const size_t SF = 10;
    auto feeder = registry.call("/blocks/feeder_source", "uint8");
    auto encoder = registry.call("/lora/lora_encoder");
    auto mod = registry.call("/lora/lora_mod", SF);
    auto adder = registry.call("/comms/arithmetic", "complex_float32", "ADD");
    auto noise = registry.call("/comms/noise_source", "complex_float32");
    auto demod = registry.call("/lora/lora_demod", SF);
    auto decoder = registry.call("/lora/lora_decoder");
    auto collector = registry.call("/blocks/collector_sink", "uint8");

    std::vector<std::string> testCodingRates;
    //these first few dont have error correction
    //testCodingRates.push_back("4/4");
    //testCodingRates.push_back("4/5");
    //testCodingRates.push_back("4/6");
    testCodingRates.push_back("4/7");
    testCodingRates.push_back("4/8");

    for (const auto &CR : testCodingRates)
    {
        std::cout << "Testing with CR " << CR << std::endl;

        encoder.call("setSpreadFactor", SF);
        decoder.call("setSpreadFactor", SF);
        encoder.call("setCodingRate", CR);
        decoder.call("setCodingRate", CR);
        mod.call("setAmplitude", 1.0);
        noise.call("setAmplitude", 4.0);
        noise.call("setWaveform", "NORMAL");
        mod.call("setPadding", 512);
        demod.call("setMTU", 512);

        //create a test plan
        json testPlan;
        testPlan["enablePackets"] = true;
        testPlan["minValue"] = 0;
        testPlan["maxValue"] = 255;
        testPlan["minBuffers"] = 5;
        testPlan["maxBuffers"] = 5;
        testPlan["minBufferSize"] = 8;
        testPlan["maxBufferSize"] = 128;
        auto expected = feeder.call("feedTestPlan", testPlan.dump());

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
            POTHOS_TEST_TRUE(topology.waitInactive(0.1, 0));
            //std::cout << topology.queryJSONStats() << std::endl;
        }

        std::cout << "decoder dropped " << decoder.call<unsigned long long>("getDropped") << std::endl;
        std::cout << "verifyTestPlan" << std::endl;
        collector.call("verifyTestPlan", expected);
    }
}
