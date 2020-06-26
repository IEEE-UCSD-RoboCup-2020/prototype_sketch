#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/array.hpp>



#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"


using namespace std;
using namespace boost;
using namespace boost::asio;
using byte = unsigned char;



void delay(int delay_ms) {
    this_thread::sleep_for(boost::chrono::milliseconds(delay_ms));
}


class GrSim_Console{

typedef ip::udp udp;
typedef boost::shared_ptr<ip::udp::socket> socket_ptr; // smart pointer(no need to mannually deallocate


private:
    io_service *service;
    udp::endpoint *endpoint;
    socket_ptr socket;


public:
    GrSim_Console(io_service& ios, udp::endpoint& ep) {
        this->service = &ios;
        this->endpoint = &ep;
        socket = socket_ptr(new udp::socket(ios));
        socket->open(udp::v4());
    }

    ~GrSim_Console() {}

    void send_command(bool is_team_yellow, int id, 
                    float upper_left_wheel_speed, float lower_left_wheel_speed,
                    float lower_right_wheel_speed, float upper_right_wheel_speed, 
                    float kick_speed_x, float kick_speed_y, bool spinner) {
        grSim_Packet packet;

        
        packet.mutable_commands()->set_isteamyellow(is_team_yellow);
        packet.mutable_commands()->set_timestamp(0.0);

        

        grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
        command->set_id(id);
        command->set_wheelsspeed(true);
        command->set_wheel1(upper_left_wheel_speed); // upper_left
        command->set_wheel2(lower_left_wheel_speed); // lower_left
        command->set_wheel3(lower_right_wheel_speed); // lower_right
        command->set_wheel4(upper_right_wheel_speed); // upper_right

        command->set_kickspeedx(kick_speed_x);
        command->set_kickspeedz(kick_speed_y);
        command->set_spinner(spinner);


        char to_send[256];
        packet.SerializeToArray(to_send, packet.ByteSizeLong());
        socket->send_to(asio::buffer(to_send), *endpoint);
        
    }
};




int main() {

    io_service service;
    ip::udp::endpoint ep(ip::address::from_string("127.0.0.1"), 20011);
    GrSim_Console console(service, ep);


    while(1) {
        console.send_command(false, 2, 1000, -1000, -1000, 1000, 0, 0, false);
        delay(10);
    }


    return 0;
}

