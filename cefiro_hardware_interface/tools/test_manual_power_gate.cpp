#include "cefiro_hardware_interface/manual_power_gate.hpp"
#include <cassert>
#include <iostream>
using cefiro::hardware::ManualPowerGate;
int main() {
  ManualPowerGate gate;
  assert(!gate.reason(false,false).empty());
  gate.battery_valid=gate.battery_adc_fresh=gate.power_adc_fresh=true;
  assert(gate.reason(true,true).empty());
  assert(!gate.reason(false,true).empty());
  assert(!gate.reason(true,false).empty());
  gate.battery_bench=true;
  assert(gate.reason(true,true)=="BENCH_ZERO_ONLY");
  gate.battery_bench=false; gate.power_bench=true;
  assert(gate.reason(true,true)=="BENCH_ZERO_ONLY");
  gate.power_bench=false; gate.bench_denial=true;
  assert(gate.reason(true,true)=="BENCH_ZERO_ONLY");
  assert(gate.reason(false,false)=="BENCH_ZERO_ONLY");
  gate={};
  gate.battery_adc_fresh=gate.power_adc_fresh=true;
  assert(gate.reason(true,true)=="Battery voltage invalid");
  gate.battery_valid=true; gate.power_adc_fresh=false;
  assert(gate.reason(true,true)=="Battery ADC not fresh");
  gate.power_adc_fresh=true; gate.battery_adc_fresh=false;
  assert(gate.reason(true,true)=="Battery ADC not fresh");
  gate.battery_adc_fresh=true;
  assert(gate.reason(true,true).empty());
  gate.battery_valid=false;
  gate.external_manual=true;
  assert(gate.reason(true,true)=="External power profile inhibited by STM32");
  gate.external_ready=true;
  assert(gate.reason(true,true).empty());
  assert(!gate.reason(false,true).empty());
  assert(!gate.reason(true,false).empty());
  gate.power_adc_fresh=false;
  assert(gate.reason(true,true)=="Battery ADC not fresh");
  gate.power_adc_fresh=true;
  gate.power_bench=true;
  assert(gate.reason(true,true)=="BENCH_ZERO_ONLY");
  gate.power_bench=false; gate.bench_denial=true;
  assert(gate.reason(true,true)=="BENCH_ZERO_ONLY");
  gate.bench_denial=false; gate.external_manual=false;
  assert(gate.reason(true,true)=="Battery voltage invalid");
  std::cout << "Manual power gate (battery, bench, external): PASS\n";
}
