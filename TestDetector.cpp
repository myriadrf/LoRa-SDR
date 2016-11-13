// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Testing.hpp>
#include "LoRaDetector.hpp"
#include "ChirpGenerator.hpp"
#include <iostream>

POTHOS_TEST_BLOCK("/lora/tests", test_detector)
{
    const size_t N = 1 << 10;
    float phaseAccum = 0.0f;
    std::vector<std::complex<float>> downChirp(N);
    genChirp(downChirp.data(), N, 1, N, 0.0f, true, 1.0f, phaseAccum);

    for (size_t sym = 0; sym < N; sym++)
    {
        std::cout << "testing detector on symbol = " << sym << std::endl;
        std::vector<std::complex<float>> chirp(N);
        phaseAccum = M_PI/4; //some phase offset
        genChirp(chirp.data(), N, 1, N, float(2*M_PI*sym)/N, false, 1.0f, phaseAccum);

        LoRaDetector<float> detector(N);
        for (size_t i = 0; i < N; i++) detector.feed(i, downChirp[i]*chirp[i]);
        float power, powerAvg, fIndex;
        const size_t index = detector.detect(power, powerAvg, fIndex);
        std::cout << "  index " << index << std::endl;
        std::cout << "  power " << power << std::endl;
        std::cout << "  powerAvg " << powerAvg << std::endl;
        std::cout << "  snr " << (power-powerAvg) << std::endl;
        std::cout << "  fIndex " << fIndex << std::endl;
        POTHOS_TEST_EQUAL(sym, index);
        POTHOS_TEST_TRUE(power > -10.0);
    }
}
