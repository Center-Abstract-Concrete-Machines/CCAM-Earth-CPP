#pragma once

template<typename KnobControl, typename CvControl>
struct CvKnob {
    KnobControl* knob;
    CvControl* cvin;
    float noise;

    void Init(KnobControl* knob, CvControl* cvin) {
        this->knob = knob;
        this->cvin = cvin;
    }

    float Value() {
        float value = 0.0f;
        if (knob) {
            value += knob->Value();
        }
        if (cvin) {
            value += cvin->Value();
        }
        value -= noise;
        return daisysp::fclamp(value, 0.0f, 1.0f);
    }
};

inline float randf() {
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

// frequency to 1v/oct helper
inline float ftov(float freq)
{
    // assume 0V = A1
    return log2f(freq/55.0f);
}
