#pragma once

#include "Core/Core.h"

namespace aimpoint {

    template<uint32_t NUM_SAMPLES>
    class SignalLogger {
    public:
        SignalLogger() : m_size(NUM_SAMPLES), loc(0) {
            memset(samples, 0, NUM_SAMPLES*sizeof(float));
        }
        ~SignalLogger() {}

        void AddSample(float sample) {
            samples[loc] = sample;
            loc++;
            if (loc == NUM_SAMPLES)
                loc = 0;

            /*
            // shift everything over 1
            for (int n = 1; n < NUM_SAMPLES; n++) {
                samples[n-1] = samples[n];
            }

            // add new sample to the end
            samples[NUM_SAMPLES-1] = sample;
            */
        }

    //private:
        const uint32_t m_size;
        float samples[NUM_SAMPLES];

        uint32_t loc;
    };
}