#include "cefiro_gui/stable_readout.hpp"
#include <QApplication>
#include <QVBoxLayout>
#include <QSlider>
#include <cassert>
using namespace cefiro::gui;
int main(int argc,char** argv){
    QApplication app(argc,argv);
    QWidget window;
    auto* layout=new QVBoxLayout(&window);
    auto* slider=new QSlider(Qt::Horizontal,&window);
    auto* readout=new QLabel(&window);
    auto* master=new QLabel(&window);
    layout->addWidget(slider);
    layout->addWidget(readout,0,Qt::AlignHCenter);
    layout->addWidget(master);
    readout->setAlignment(Qt::AlignLeft|Qt::AlignTop);
    const QStringList templates{"Req 88888 RPM","DShot 8888","Eq 88888 RPM"};
    window.resize(420,260);
    window.show();
    for(int size:{11,16,22}){
        app.setStyleSheet(QString("QLabel {font-size:%1px;}").arg(size));
        reserveReadout(readout,3,true,templates);
        reserveReadout(master,3);
        setReadoutText(readout,"Req 0 RPM\nDShot --\nEq -- RPM");
        setReadoutText(master,"Req 0 RPM\nAwaiting bridge\n");
        app.processEvents();
        const QRect r=readout->geometry(),m=master->geometry(),s=slider->geometry();
        for(int code:{0,48,99,100,999,1000,2047,0}){
            setReadoutText(readout,QString("Req %1 RPM\nDShot %2\nEq %1 RPM").arg(code*5).arg(code));
            setReadoutText(master,QString("Req %1 RPM\nDShot cap 2047\nINHIBITED").arg(code*5),"Full reason");
            app.processEvents();
            assert(readout->geometry()==r && master->geometry()==m && slider->geometry()==s);
            assert(!readout->wordWrap() && !master->wordWrap());
        }
    }
}
