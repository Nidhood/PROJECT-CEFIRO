#include "cefiro_gui/autotune.hpp"
#include <cassert>
#include <iostream>
#include <filesystem>
using namespace cefiro::gui::autotune;
int main(){
    Model m;
    auto g=lqr(m);
    assert(g.size()==7);
    auto k=g[3];
    assert(std::abs(k.kp-1.70815068)<1e-5 && std::abs(k.kd-.32479975)<1e-5);
    assert(stable(m.a*std::cos(m.offset),m.b,1,m.j,k,0));
    assert(!stable(m.a*std::cos(m.offset),m.b,1,m.j,k,32));
    assert(std::none_of(g.begin(),g.end(),[](auto v){return v.robust;}));
    auto results=optimize(m);
    assert(results.size()==434);
    for(auto& c:results){assert(c.cases==12 && std::isfinite(c.cost));assert(c.pass<=c.cases);}
    assert(std::none_of(results.begin(),results.end(),[](auto c){return c.gain.robust && c.pass==c.cases;}));
    // Exact linear integral regression, independent held-out rows.
    std::vector<Window> w;
    std::array<double,4> expected{24.,.7,26.,25.};
    for(int i=0;i<100;++i){Window a;for(int j=0;j<4;++j)a.x[j]=std::sin((i+1)*(j+1)*.31);a.y=std::inner_product(a.x.begin(),a.x.end(),expected.begin(),0.);a.test=i>=70;w.push_back(a);}
    auto b=regress(w);assert(b);for(int j=0;j<4;++j)assert(std::abs((*b)[j]-expected[j])<1e-8);
    // Real CSV schema. GUI estimates are never treated as firmware AHRS.
    auto path=std::filesystem::temp_directory_path()/"cefiro_autotune_schema_test.csv";
    {std::ofstream f(path);f<<"ros_time_s,roll_deg,pitch_deg,p_deg_s,plant,quaternion_valid,host_dshot_valid,host_dshot_age_s,dshot_host_m1,dshot_host_m2,dshot_host_m3,dshot_host_m4,gui_fusion_valid,gui_roll_deg,gui_pitch_deg\n";for(int i=0;i<120;++i)f<<i*.04<<",0,0,1,REAL,0,1,0.02,150,150,0,0,1,-173,-7\n";}
    auto run=read(path.string());std::filesystem::remove(path);
    assert(run.diagnostic && !run.fmu && run.rows[2].valid && std::abs(run.rows[2].angle+173*rad)<1e-12);
    bool rejected=false;try{(void)fit({run});}catch(const std::runtime_error&){rejected=true;}assert(rejected);
    Run power; Row row;row.voltage_valid=true;row.voltage=12.;row.power_valid=true;row.power=24.;
    row.power_time=1.;power.rows.push_back(row);power.rows.push_back(row);
    row.power_time=1.2;power.rows.push_back(row);
    row.power_valid=false;row.power_time=1.4;power.rows.push_back(row);
    row.power_valid=true;row.power_time=1.6;power.rows.push_back(row);
    auto ps=powerSummary(power);assert(ps.samples==3&&ps.voltageSamples==4&&std::abs(ps.energyJ-4.8)<1e-9&&std::abs(ps.coveredS-.2)<1e-9);
    for(auto& rr:power.rows)rr.power_valid=false;
    ps=powerSummary(power);assert(ps.samples==0&&ps.voltageSamples==4&&ps.minV==12.&&ps.coveredS==0.);
    Run input;input.fmu=true;
    for(int i=0;i<100;++i){Row x;x.t=i*.04;x.angle=-2.;x.rate=.1;x.u=.4;x.valid=true;x.voltage_valid=true;x.voltage=10.;input.rows.push_back(x);}
    auto w0=windows({input},0.,0.,false),wv=windows({input},0.,0.,true);
    assert(!w0.empty()&&w0.size()==wv.size());assert(std::abs(wv[0].x[2]/w0[0].x[2]-100./144.)<1e-9);
    for(auto& rr:input.rows)rr.voltage_valid=false;
    assert(windows({input},0.,0.,true).empty());
    auto local=std::filesystem::temp_directory_path()/"cefiro_autotune_local_test.csv";
    {std::ofstream f(local);f<<"ros_time_s,roll_deg,pitch_deg,p_deg_s,plant,quaternion_valid,fmu_dshot_valid,fmu_sample_ms,dshot_fmu_m1,dshot_fmu_m2,dshot_fmu_m3,dshot_fmu_m4,esc_power_tick_valid,battery_valid,battery_voltage_v,esc_power_age_s,esc_power_sample_ms,esc_current_valid,esc_current_a\n";
     for(int i=0;i<120;++i)f<<i*.05<<",-170,0,1,REAL,1,1,"<<1000+40*i<<",150,150,0,0,1,1,12,0.04,"<<960+40*i<<",0,\n";}
    auto localRun=read(local.string());std::filesystem::remove(local);assert(localRun.fmu&&localRun.rows[10].voltage_valid&&!localRun.rows[10].power_valid&&std::abs(localRun.rows[10].t-.4)<1e-9);
    std::cout<<"PASS: MCU time, voltage hypothesis, no invented current, gap-aware energy\n";
    std::cout<<"PASS: 434 candidates, 12 cases each; known LQR and delay rejection; regression; CSV provenance; rejects unexcited axis.\n";
}
