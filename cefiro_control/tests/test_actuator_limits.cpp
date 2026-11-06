#include "cefiro_actuators/actuator_model.hpp"
#include <iostream>
#include <array>
using namespace cefiro::actuators;
int main(){
 MotorParameters params;
 params.kv_rpm_per_volt=750.;params.resistance_ohm=.30;params.inductance_h=.0001;
 params.rotor_prop_inertia_kg_m2=.00004;params.current_limit_a=35.;
 params.no_load_current_a=.65;params.viscous_friction_nm_per_rad_s=.000004;
 params.max_rpm=20000.;params.propeller_diameter_m=.254;
 params.thrust_coefficient_ct=.10;params.power_torque_coefficient_cq=.015;
 params.air_density_kg_m3=1.225;params.esc_time_constant_s=.025;
 params.minimum_spinning_duty=.035;params.active_braking=false;
 QuadMotorModel plant(params,12.63,.03);
 for(double rpm : {4400.,4700.,5100.,5400.,5600.,6000.,7500.}){
  plant.reset();std::array<double,4> setpoint{rpm,rpm,rpm,rpm};
  auto command=plant.estimateDshotForRpm(setpoint);
  for(int k=0;k<12000;++k){if(k%40==0)command=plant.estimateDshotForRpm(setpoint);plant.step(.0005,command,true);}
  std::cout << "target="<<rpm<<" dshot="<<command[0]<<" rpm_meas="<<plant.motors()[0].state().rpm<<" bus="<<plant.busVoltage()<<"\n";
 }
}
