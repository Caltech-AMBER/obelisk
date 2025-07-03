#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <Unit.hpp>

using namespace rapidjson;

struct AllJointCtrl {
    static const int SEQ = 4;
    static const int ADDRESS = 1;
    static const int FUNCODE = 2;
    static const int MODE = 1;

    std::vector<double> q_des;
    std::vector<std::string> joint_names_hardware;
    std::vector<Unit> joint_units_hardware;
    std::vector<Unit> joint_units;
    
    std::string str() {
        /* Format the struct into a string that can be stored in the 
        the data member of the ArmString_ message.
        */
        Document document;
        document.SetObject();

        Value data(kObjectType);
        Document::AllocatorType& allocator = document.GetAllocator();

        data.AddMember("mode", MODE, allocator);

        Unit joint_unit_hardware;
        Unit joint_unit;
        double joint_pos_hardware;
        double joint_pos;

        // Add joint positions to data with the proper units
        for (size_t i = 0; i < q_des.size(); i++) {
            joint_unit_hardware = joint_units_hardware.at(i);
            joint_unit = joint_units.at(i);
            joint_pos = q_des.at(i);

            if (joint_unit_hardware == DEGREES && joint_unit == RADIANS) {
                joint_pos_hardware = joint_pos * 180 / M_PI;
            }
            else if (joint_unit_hardware == MILLIMETERS && joint_unit == METERS) {
                joint_pos_hardware = joint_pos * 1000;
                // This joint6 position is half of what it is on the hardware
                joint_pos_hardware *= 2;
            }
            else {
                std::cerr << "Didn't implement unit conversion yet.\n";
                return "";
            }

            // Round to two decimal places
            joint_pos_hardware = std::round(joint_pos_hardware * 100) / 100;
            data.AddMember(
                GenericStringRef(joint_names_hardware.at(i).c_str()), 
                joint_pos_hardware, 
                allocator
            );
        }

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