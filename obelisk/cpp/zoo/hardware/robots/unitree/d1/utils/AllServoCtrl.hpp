#pragma once

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <d1/utils/Unit.hpp>
#include <d1/utils/GripperUtils.hpp>

using namespace rapidjson;

struct AllServoCtrl {
    static const int SEQ = 4;
    static const int ADDRESS = 1;
    static const int FUNCODE = 2;
    // mode 0 is the small smoothing of 10Hz data
    // mode 1 is the large smoothing of trajectory use
    // Even for 10 Hz data, however, mode 1 creates smoother trajectories.
    static const int MODE = 1; 

    std::vector<double> qd; // desired joint positions (radians)
    double gripper_pos; // desired gripper1 position (meters)
    std::vector<std::string> joint_names_hardware;
    std::string gripper_name_hardware;
    Unit joint_units_hardware;
    Unit joint_units;
    
    std::string str() {
        /* Format the struct into a string that can be stored in the 
        the data member of the ArmString_ message.
        */
        Document document;
        document.SetObject();

        Value data(kObjectType);
        Document::AllocatorType& allocator = document.GetAllocator();

        data.AddMember("mode", MODE, allocator);

        double joint_pos_hardware;
        double joint_pos;

        // Add joint positions to data with the proper units
        size_t i; 
        for (i = 0; i < qd.size(); i++) {
            joint_pos = qd.at(i);

            if (joint_units_hardware == DEGREES && joint_units == RADIANS) {
                joint_pos_hardware = joint_pos * 180 / M_PI;
            }
            else {
                std::cerr << "Didn't implement unit conversion yet.\n";
                return "";
            }
            
            data.AddMember(
                GenericStringRef(joint_names_hardware.at(i).c_str()), 
                joint_pos_hardware, 
                allocator
            );
        }

        // Add gripper position to data with the proper units
        double gripper_pos_hardware = meters2degrees(gripper_pos);

        data.AddMember(
            GenericStringRef(gripper_name_hardware.c_str()),
            gripper_pos_hardware,
            allocator
        );
        
        document.AddMember("seq", SEQ, allocator);
        document.AddMember("address", ADDRESS, allocator);
        document.AddMember("funcode", FUNCODE, allocator);
        document.AddMember("data", data, allocator);

        StringBuffer buffer;
        Writer<StringBuffer> writer(buffer);
        document.Accept(writer);
        return buffer.GetString();
    }
};