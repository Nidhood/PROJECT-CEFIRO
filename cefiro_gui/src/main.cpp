#include "cefiro_gui/main_window.hpp"
#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <csignal>
namespace { volatile std::sig_atomic_t shutdown_signal=0; void signal_handler(int signal){shutdown_signal=signal;} }
int main(int argc,char** argv){
    QApplication app(argc,argv);app.setApplicationName("CEFIRO HMI");app.setOrganizationName("CEFIRO");
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
    cefiro::gui::MainWindow window;window.show();
    QTimer check;check.setInterval(50);
    QObject::connect(&check,&QTimer::timeout,&app,[&]{if(shutdown_signal)app.quit();});check.start();
    QObject::connect(&app,&QCoreApplication::aboutToQuit,&app,[&]{check.stop();window.shutdown();if(rclcpp::ok())rclcpp::shutdown();});
    return app.exec();
}
