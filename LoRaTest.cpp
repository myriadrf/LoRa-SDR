// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Testing.hpp>
#include <Pothos/Framework.hpp>
#include <Pothos/Proxy.hpp>
#include <Pothos/Remote.hpp>
#include <Poco/JSON/Object.h>
#include <iostream>

POTHOS_TEST_BLOCK("/lora/tests", test_encoder_to_decoder)
{
    auto env = Pothos::ProxyEnvironment::make("managed");
    auto registry = env->findProxy("Pothos/BlockRegistry");

    auto feeder = registry.callProxy("/blocks/feeder_source", "uint8");
    auto encoder = registry.callProxy("/lora/lora_encoder", 8);
    auto decoder = registry.callProxy("/lora/lora_decoder", 8);
    auto collector = registry.callProxy("/blocks/collector_sink", "uint8");

    //create a test plan
    Poco::JSON::Object::Ptr testPlan(new Poco::JSON::Object());
    testPlan->set("enablePackets", true);
    testPlan->set("minValue", 0);
    testPlan->set("maxValue", 255);
    testPlan->set("minBufferSize", 128);
    testPlan->set("maxBufferSize", 128);
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
