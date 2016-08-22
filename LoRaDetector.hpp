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
        _powerScale = 1.0 / (N * N);
        return;
    }

    //! feed simply sets an input sample
    void feed(const size_t i, const std::complex<Type> &samp)
    {
        _fftInput[i] = samp;
    }

    //! calculates argmax(abs(fft(input)))
    size_t detect(Type &power, Type &powerAvg,  Type &fIndex)
    {
        _fft.transform(_fftInput.data(), _fftOutput.data());
        size_t maxIndex = 0;
        Type maxValue = 0;
        size_t N = _fftOutput.size();
        Type total = 0;
        for (size_t i = 0; i < N; i++)
        {
            auto bin = _fftOutput[i];
            auto re = bin.real();
            auto im = bin.imag();
            auto mag2 = re*re + im*im;
            total += mag2;
            if (mag2 > maxValue)
            {
                maxIndex = i;
                maxValue = mag2;
            }
        }
        
        powerAvg = (total - maxValue)/(N-1) * _powerScale;
        power = maxValue * _powerScale;
        
        auto left = _fftOutput[maxIndex > 0?maxIndex-1:N-1];
        auto right = _fftOutput[maxIndex < N-1?maxIndex+1:0];
        
        fIndex = 0.5 * (abs(right) - abs(left)) / (2.0 * sqrt(maxValue) - abs(right) - abs(left));
        
        return maxIndex;
    }
    
    std::vector<std::complex<Type>>* getOutput(){
        return &_fftOutput;
    }

private:
    Type _powerScale;
    std::vector<std::complex<Type>> _fftInput;
    std::vector<std::complex<Type>> _fftOutput;
    kissfft<Type> _fft;
};
