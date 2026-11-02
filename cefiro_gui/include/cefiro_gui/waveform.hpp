#pragma once
#include "cefiro_gui/models.hpp"
#include <algorithm>
#include <cmath>
namespace cefiro::gui {
enum class WaveformType { Constant, Step, Ramp, Pulse, Square, Triangle, Sine, Exponential, Chirp };
struct WaveformConfig {
    WaveformType type{WaveformType::Step};
    double amplitude{10.0}, offset{0.0}, frequency_hz{0.25}, end_frequency_hz{2.0};
    double phase_deg{0.0}, delay_s{0.0}, duration_s{10.0};
    double initial_deg{0.0}, final_deg{10.0}, rise_s{2.0};
    double pulse_s{0.5}, duty_percent{50.0}, tau_s{1.0};
    bool repeat{false};
};
inline double waveformValue(const WaveformConfig& c, double time_s) {
    if (time_s<c.delay_s) {
        if(c.type==WaveformType::Step||c.type==WaveformType::Ramp||c.type==WaveformType::Exponential) return c.initial_deg;
        return c.offset;
    }
    double t=std::max(0.0,time_s-c.delay_s);
    if(c.repeat && c.duration_s>0.0) t=std::fmod(t,c.duration_s);
    const double f=std::max(1.e-9,c.frequency_hz), phase=c.phase_deg*deg2rad;
    switch(c.type) {
    case WaveformType::Constant: return c.offset+c.amplitude;
    case WaveformType::Step: return c.final_deg;
    case WaveformType::Ramp: return c.initial_deg+std::clamp(t/std::max(1.e-9,c.rise_s),0.0,1.0)*(c.final_deg-c.initial_deg);
    case WaveformType::Pulse: return c.offset+(t<c.pulse_s?c.amplitude:0.0);
    case WaveformType::Square: return c.offset+(std::fmod(t*f,1.0)<std::clamp(c.duty_percent/100.,0.,1.)?c.amplitude:-c.amplitude);
    case WaveformType::Triangle: return c.offset+c.amplitude*(1.0-4.0*std::abs(std::fmod(t*f,1.0)-0.5));
    case WaveformType::Sine: return c.offset+c.amplitude*std::sin(2.0*pi*f*t+phase);
    case WaveformType::Exponential: return c.final_deg+(c.initial_deg-c.final_deg)*std::exp(-t/std::max(1.e-9,c.tau_s));
    case WaveformType::Chirp: {
        const double rate=(c.end_frequency_hz-c.frequency_hz)/std::max(1.e-9,c.duration_s);
        return c.offset+c.amplitude*std::sin(2*pi*(f*t+0.5*rate*t*t)+phase);
    }
    }
    return 0.0;
}
}
