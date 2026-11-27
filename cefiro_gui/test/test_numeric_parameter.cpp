#include "cefiro_gui/numeric_parameter.hpp"
#include "cefiro_gui/motor_observation.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
    using namespace cefiro::gui;
    assert(parseNonnegativeParameter("25") == 25.0);
    assert(parseNonnegativeParameter("25.0") == 25.0);
    assert(parseNonnegativeParameter(" 25.125\t") == 25.125);
    assert(parseNonnegativeParameter("2.5e1") == 25.0);
    assert(parseNonnegativeParameter("0") == 0.0);
    for (auto bad : {"25,0", "25 rpm", "nan", "inf", "-1", "", " ", "25x", "1e999"})
        assert(!parseNonnegativeParameter(bad));
    NominalMotorMap map{48,300,12000,std::numeric_limits<double>::quiet_NaN()};
    assert(!map.inverse({0,48,174,300}));
    map.deadband_rpm = *parseNonnegativeParameter("25.0");
    const auto rpm = map.inverse({0,48,174,300});
    assert(rpm && (*rpm)[0]==0 && (*rpm)[1]==25 && (*rpm)[2]==6012.5 && (*rpm)[3]==12000);
    std::cout << "PASS: locale-independent numeric parsing and invalid-map gating\n";
}
