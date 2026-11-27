#include "cefiro_gui/ui_scale.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace cefiro::gui;
int main(){
    const auto near=[](double a,double b){return std::abs(a-b)<1.e-9;};
    assert(near(automaticUiScale(610,480),1.0));
    assert(near(automaticUiScale(1280,740),1.0));
    assert(near(automaticUiScale(1530,910),1.2));
    assert(near(automaticUiScale(1920,1080),1.5));
    assert(near(automaticUiScale(2560,1440),1.9));
    assert(near(automaticUiScale(3840,2160),2.0));
    assert(near(automaticUiScale(3840,600),1.0));
    assert(near(effectiveUiScale(3840,2160,120),1.2));
    assert(near(effectiveUiScale(610,480,200),2.0));
    assert(near(effectiveUiScale(610,480,400),2.0));
    assert(scaledPixels(12,1.5)==18);
    assert(scaledPixels(24,2.)==48);
    assert(layoutWidth(1920,1.5)==1280);
    for(int width=600;width<4000;width+=31){
        const double scale=automaticUiScale(width,1000);
        assert(scale>=1.0&&scale<=2.0);
    }
    std::cout<<"PASS: logical window scale, compact/large/wide screens, manual override, bounded sizing\n";
}
