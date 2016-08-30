// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#pragma once
#include <Pothos/Config.hpp>
#include <complex>
#include <cmath>

/*!
 * Generate a chirp
 * \param [out] samps pointer to the output samples
 * \param N samples per chirp sans the oversampling
 * \param ovs the oversampling size
 * \param NN the number of samples to generate
 * \param f0 the phase offset/transmit symbol
 * \param down true for downchirp, false for up
 * \param ampl the chrip amplitude
 * \param [inout] phaseAccum running phase accumulator value
 * \return the number of samples generated
 */
template <typename Type>
int genChirp(std::complex<Type> *samps, int N, int ovs, int NN, Type f0, bool down, const Type ampl, Type &phaseAccum)
{
    const Type fMin = -M_PI / ovs;
    const Type fMax = M_PI / ovs;
    const Type fStep = (2 * M_PI) / (N * ovs * ovs);
    float f = fMin + f0;
    int i;
    if (down) {
        for (i = 0; i < NN; i++) {
            f += fStep;
            if (f > fMax) f -= (fMax - fMin);
            phaseAccum -= f;
            samps[i] = std::polar(ampl, phaseAccum);
        }
    }
    else {
        for (i = 0; i < NN; i++) {
            f += fStep;
            if (f > fMax) f -= (fMax - fMin);
            phaseAccum += f;
            samps[i] = std::polar(ampl, phaseAccum);
        }
    }
    phaseAccum -= floor(phaseAccum / (2 * M_PI)) * 2 * M_PI;
    return i;
}
