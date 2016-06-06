// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include "kissfft.hh"
#include <complex>
#include <vector>

template <typename Type>
class LoRaDetector
{
public:
    LoRaDetector(const size_t N):
        _fftInput(N),
        _fftOutput(N),
        _fft(N, false)
    {
        return;
    }

    //! feed simply sets an input sample
    void feed(const size_t i, const std::complex<float> &samp)
    {
        _fftInput[i] = samp;
    }

    //! calculates argmax(abs(fft(input)))
    size_t detect(Type &power)
    {
        _fft.transform(_fftInput.data(), _fftOutput.data());
        size_t maxIndex = 0;
        Type maxValue = 0;
        for (size_t i = 0; i < _fftOutput.size(); i++)
        {
            auto bin = _fftOutput[i];
            auto re = bin.real();
            auto im = bin.imag();
            auto mag2 = re*re + im*im;
            if (mag2 > maxValue)
            {
                maxIndex = i;
                maxValue = mag2;
            }
        }
        power = maxValue;
        return maxIndex;
    }

private:
    std::vector<std::complex<Type>> _fftInput;
    std::vector<std::complex<Type>> _fftOutput;
    kissfft<Type> _fft;
};
