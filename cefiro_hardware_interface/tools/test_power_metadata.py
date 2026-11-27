#!/usr/bin/env python3
"""Compile actual identification power attachment block with local test doubles."""
from pathlib import Path
import tempfile,subprocess
root=Path(__file__).resolve().parents[1]
s=(root/'src/hardware_node.cpp').read_text()
block=s[s.index('    id.battery_valid=last_battery_'):s.index('    pub_identification_->publish(id);')]
code=r'''
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <iostream>
using Clock=std::chrono::steady_clock;
struct Id{bool battery_valid{},power_tick_valid{},current_valid{},esc_sense_valid{};double battery_voltage_v{},power_age_s{},esc_current_a{},esc_current_sense_v{};std::uint32_t power_sample_ms{};std::uint16_t adc_voltage_raw{},adc_current_raw{};};
struct Harness{
 std::optional<Clock::time_point> last_battery_=Clock::now(),last_power_=Clock::now();
 bool battery_voltage_valid_=true,id_bat_adc_fresh_=true,id_current_valid_=false,id_pwr_valid_=true;
 double id_battery_voltage_=12.6,id_battery_current_=2.0,id_sense_v_=.2;
 std::uint32_t id_battery_tick_=1000,id_pwr_tick_=1000;
 std::uint16_t id_adc_v_=51066,id_adc_i_=1234;
 Id attach(std::optional<std::uint32_t> sample_tick){Id id;
'''+block+r'''
 return id;}
};
int main(){Harness h;auto a=h.attach(1040);assert(a.battery_valid&&a.power_tick_valid&&a.esc_sense_valid&&!a.current_valid&&a.power_age_s==.04&&a.adc_voltage_raw==51066);
 h.id_current_valid_=true;a=h.attach(1040);assert(a.current_valid&&a.esc_current_a==2.);
 h.id_pwr_tick_=1001;assert(!h.attach(1040).esc_sense_valid);assert(!h.attach(999).power_tick_valid);assert(!h.attach(1351).power_tick_valid);assert(!h.attach({}).power_tick_valid);
 h.last_battery_.reset();assert(!h.attach(1040).power_tick_valid&&!h.attach(1040).battery_valid);
 h.last_battery_=Clock::now();h.id_bat_adc_fresh_=false;assert(!h.attach(1040).power_tick_valid);
 h.id_bat_adc_fresh_=true;h.id_battery_tick_=4294967280U;assert(h.attach(24).power_age_s==.04);
 std::cout<<"PASS: exact power tick pairing, stale/future rejection, uncalibrated current, rollover\n";}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td)/'test.cpp';p.write_text(code);out=Path(td)/'test'
 subprocess.run(['c++','-std=c++20','-Wall','-Wextra','-Werror',str(p),'-o',str(out)],check=True);subprocess.run([str(out)],check=True)
