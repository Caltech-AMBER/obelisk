#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

using namespace rapidjson;

struct AllJointCtrl {
    static const int SEQ = 4;
    static const int ADDRESS = 1;
    static const int FUNCODE = 2;
    static const int MODE = 1;

    std::vector<double> q_des;
    std::vector<std::string> joint_names;

    std::string str() {
        /* Format the struct into a string that can be stored in the 
        the data member of the ArmString_ message.
        */
        Document document;
        document.SetObject();

        Value data(kObjectType);
        Document::AllocatorType& allocator = document.GetAllocator();

        data.AddMember("mode", MODE, allocator);

        // Add joint positions to data
        double pos_in_rads;
        double pos_in_degrees;

        for (size_t i = 0; i < q_des.size(); i++) {
            pos_in_rads = q_des.at(i);
            pos_in_degrees = pos_in_rads * 180 / M_PI;
            // Round to two decimal places
            pos_in_degrees = std::round(pos_in_degrees * 100) / 100;
            data.AddMember(GenericStringRef(joint_names.at(i).c_str()), pos_in_degrees, allocator);
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