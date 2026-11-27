#pragma once
#include "cefiro_gui/models.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <vector>

namespace cefiro::gui {
struct IdentificationFields {
    bool fmu_dshot_valid{}, fmu_armed{}, host_dshot_valid{}, battery_valid{};
    std::uint32_t fmu_sample_ms{};
    std::uint8_t fmu_state{};
    std::array<std::uint16_t,4> fmu_dshot{}, host_dshot{};
    double host_dshot_age_s{}, battery_voltage_v{};
    // Strict orientation validity supplied by the co-sampled message.
    bool orientation_valid{};
    Quaternion orientation{};
    void apply(Telemetry& s) const {
        s.fmu_dshot_valid=fmu_dshot_valid; s.fmu_armed=fmu_armed;
        s.fmu_sample_ms=fmu_sample_ms; s.fmu_state=fmu_state; s.fmu_dshot=fmu_dshot;
        // Keep a causal host fallback captured at IMU receipt if the joined
        // message has no valid host observation. Never invent an FMU value.
        if(host_dshot_valid && validCodes(host_dshot) && std::isfinite(host_dshot_age_s) && host_dshot_age_s>=0. && host_dshot_age_s<=0.30){
            s.host_dshot_valid=true; s.host_dshot=host_dshot;
            s.host_dshot_age_s=host_dshot_age_s; s.host_origin=HostOrigin::Identification;
        }
        if(battery_valid){s.battery_valid=true;s.battery_voltage_v=battery_voltage_v;}
        s.orientation_valid=orientation_valid; s.orientation=orientation;
        if(orientation_valid)s.attitude_deg=orientation.eulerDeg();
    }
};
// Call under RosBridge mutex. The live IMU queue never waits for this join.
class RecordingJoin {
public:
    using Clock=std::chrono::steady_clock;
    using Time=Clock::time_point;
    using Key=std::int64_t;
    static constexpr auto wait=std::chrono::milliseconds(120);
    static constexpr std::size_t capacity=512;
    void clear(){samples_.clear();extras_.clear();}
    void pushImu(Key key, const Telemetry& sample, Time now) {
        if(samples_.size()>=capacity)samples_.pop_front();
        samples_.push_back({key,sample,now,false});
        if(key==0)return; // No unique timestamp; never claim synchronization.
        auto it=std::find_if(extras_.begin(),extras_.end(),[key](const Extra& e){return e.key==key;});
        if(it!=extras_.end()){
            if(now-it->received<=wait){it->fields.apply(samples_.back().sample);samples_.back().matched=true;}
            extras_.erase(it);
        }
    }
    void pushId(Key key, const IdentificationFields& fields, Time now) {
        if(key==0)return;
        auto it=std::find_if(samples_.begin(),samples_.end(),[key](const Pending& s){return s.key==key;});
        if(it!=samples_.end()){
            if(!it->matched && now-it->received<=wait){fields.apply(it->sample);it->matched=true;}
            return;
        }
        if(extras_.size()>=capacity)extras_.pop_front();
        extras_.push_back({key,fields,now});
    }
    std::vector<Telemetry> drain(Time now,bool force=false){
        std::vector<Telemetry> out;
        while(!samples_.empty()){
            const auto& front=samples_.front();
            if(!force && !front.matched && now-front.received<wait)break;
            out.push_back(front.sample);samples_.pop_front();
        }
        while(!extras_.empty()&&now-extras_.front().received>wait)extras_.pop_front();
        return out;
    }
private:
    struct Pending{Key key;Telemetry sample;Time received;bool matched;};
    struct Extra{Key key;IdentificationFields fields;Time received;};
    std::deque<Pending> samples_;
    std::deque<Extra> extras_;
};
}
