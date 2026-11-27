#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
namespace cefiro::gui {
using DshotCodes=std::array<std::uint16_t,4>;
using MotorClock=std::chrono::steady_clock;
inline bool validCodes(const DshotCodes& codes){
    for(auto c:codes)if(c!=0 && (c<48 || c>2047))return false;
    return true;
}
struct NominalMotorMap {
    int min_code{},max_code{};
    double full_rpm{},deadband_rpm{25.0};
    bool valid() const {return min_code>=48 && max_code<=2047 && max_code>min_code &&
        std::isfinite(full_rpm)&&std::isfinite(deadband_rpm)&&deadband_rpm>=0 && full_rpm>deadband_rpm;}
    std::optional<std::array<double,4>> inverse(const DshotCodes& codes) const {
        if(!valid() || !validCodes(codes))return std::nullopt;
        std::array<double,4> result{};
        for(std::size_t i=0;i<4;++i){
            if(codes[i]==0)continue;
            if(codes[i]<min_code || codes[i]>max_code)return std::nullopt;
            result[i]=deadband_rpm+(full_rpm-deadband_rpm)*double(codes[i]-min_code)/double(max_code-min_code);
        }
        return result;
    }
};
enum class HostOrigin {None,Topic,Status,Identification};
inline const char* originName(HostOrigin o){switch(o){case HostOrigin::Topic:return "COMMAND_TOPIC";case HostOrigin::Status:return "STATUS_JSON";case HostOrigin::Identification:return "IDENTIFICATION";default:return "";}}
struct HostDshotSample {
    DshotCodes codes{};
    HostOrigin origin{HostOrigin::None};
    MotorClock::time_point received{};
    double initial_age{};
    double age(MotorClock::time_point now) const {return initial_age+std::chrono::duration<double>(now-received).count();}
    bool validAt(MotorClock::time_point now,double limit=0.30) const {
        return origin!=HostOrigin::None && now>=received && std::isfinite(initial_age)&&initial_age>=0 && age(now)<=limit && validCodes(codes);
    }
};
inline std::optional<HostDshotSample> chooseHost(const HostDshotSample& topic,const HostDshotSample& status,MotorClock::time_point now){
    if(topic.validAt(now)&&(!status.validAt(now)||topic.age(now)<=status.age(now)))return topic;
    if(status.validAt(now))return status;
    return std::nullopt;
}
}
