#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/array.hpp>



#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_refbox_log.pb.h"
#include "messages_robocup_ssl_robot_status.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"


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
    io_service *ios;
    udp::endpoint *ep;
    socket_ptr socket;

public:
    GrSim_Console(io_service& io_srvs, udp::endpoint& endpoint) {
        this->ios = &io_srvs;
        this->ep = &endpoint;
        this->socket = socket_ptr(new udp::socket(io_srvs));
        this->socket->open(udp::v4());
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

        try {
            socket->send_to(asio::buffer(to_send), *ep);
        }
        catch (std::exception& e) {
            // To-do : Exception Handling
            std::cout << "[Exception] " << e.what() << std::endl;
        }
    }
};


class Location_Vector {
public: 
    float x; // horizontal from side view
    float y; // vertical from side view
    float z; // orientation
    Location_Vector() : x(0.00), y(0.00), z(0.00) {}
    Location_Vector(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    friend ostream& operator<<(ostream& os, const Location_Vector& lv)
    {
        os << "<" << lv.x << ", " 
                  << lv.y << ", " 
                  << lv.z << ">";
        return os;
    }
};

const int NUM_ROBOTS = 6;
Location_Vector blue_loc_vecs[NUM_ROBOTS];
Location_Vector yellow_loc_vecs[NUM_ROBOTS]; 


class GrSim_Vision {
    static const unsigned int BUF_SIZE = 100000;
    static constexpr const char* LOCAL_HOST = "127.0.0.1";
    static const int BLUE = 0;
    static const int YELLOW = 1;
    typedef ip::udp udp;
    typedef boost::shared_ptr<ip::udp::socket> socket_ptr; // smart pointer(no need to mannually deallocate
    typedef boost::shared_ptr<boost::array<char, BUF_SIZE>> buffer_array_ptr;
private:
    io_service *ios;
    udp::endpoint *ep;
    socket_ptr socket;
    buffer_array_ptr receive_buffer; 
    udp::endpoint *local_listen_ep;
    boost::mutex mu;
    
public:
    GrSim_Vision(io_service& io_srvs, udp::endpoint& endpoint) {
        this->ios = &io_srvs;
        this->ep = &endpoint;
        this->receive_buffer = buffer_array_ptr(new boost::array<char, BUF_SIZE>());
        this->socket = socket_ptr(new udp::socket(io_srvs));
        this->local_listen_ep = new udp::endpoint(ip::address::from_string(LOCAL_HOST), endpoint.port());
        socket->open(endpoint.protocol());
        socket->bind(endpoint);
        socket->set_option(udp::socket::reuse_address(true));
        socket->set_option(ip::multicast::join_group(endpoint.address()));
    }
    ~GrSim_Vision() {
        delete local_listen_ep;
    }

    void receive_packet() {
        size_t num_bytes_received;
        std::string packet_string;
        SSL_WrapperPacket packet;
        google::protobuf::RepeatedPtrField<SSL_DetectionRobot> *blue_robots, *yellow_robots;
        try {
            num_bytes_received = socket->receive_from(asio::buffer(*receive_buffer), *ep);
            packet_string = std::string(receive_buffer->begin(), 
                                          receive_buffer->begin() + num_bytes_received);
    
            packet.ParseFromString(packet_string);
            
            publish_robots_vinfo(packet.detection().robots_blue(), BLUE);
            publish_robots_vinfo(packet.detection().robots_yellow(), YELLOW);

        }
        catch (std::exception& e) {
            // To-do : Exception Handling
            std::cout << "[Exception] " << e.what() << std::endl;
        }
    }

    void publish_robots_vinfo(const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots,
                             const int team_color) {
        // To-do : ROS pub/sub
        for(auto& bot : robots) {
            mu.lock();
            if(team_color == BLUE) {
                blue_loc_vecs[bot.robot_id()] = Location_Vector(bot.pixel_x(), bot.pixel_y(), bot.orientation());
                // print_robot_vinfo(bot); // for debugging
            }
            if(team_color == YELLOW) {
                yellow_loc_vecs[bot.robot_id()] = Location_Vector(bot.x(), bot.y(), bot.orientation());
                // print_robot_vinfo(bot); // for debugging
            }
            mu.unlock();
        }
    }




    static void print_robot_vinfo(const SSL_DetectionRobot& robot) {
        // To-do : format string alignment
        std::cout << "ID[" << robot.robot_id() << "] "
                  << "[<x,y>:(" << robot.x() << ", " << robot.y() << ")]"
                  << "orien[" << robot.orientation() << "] "
                  << "confidence[" << robot.confidence() << "]"
                  << std::endl;
    }

   
};



int main() {

    io_service service;
    ip::udp::endpoint grsim_ep(ip::address::from_string("127.0.0.1"), 20011);
    ip::udp::endpoint grsim_ssl_vision_ep(ip::address::from_string("224.5.23.3"), 10020);
    GrSim_Console console(service, grsim_ep);

    boost::thread( [&grsim_ssl_vision_ep] () -> void  {
        cout << "New Thread!" << endl;
        io_service service;
        GrSim_Vision sim_vision(service, grsim_ssl_vision_ep);
        while(1) {
            sim_vision.receive_packet();
        }
    });

    while(1) {
        console.send_command(false, 2, 20, 20, 20, 20, 0, 0, false);
        delay(10);
        cout << blue_loc_vecs[2] << " " << yellow_loc_vecs[2]<< endl;

    }


    return 0;
}

