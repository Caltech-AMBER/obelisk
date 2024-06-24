#include "obelisk_node.h"

namespace obelisk {
    class ObeliskController : public ObeliskNode {
       public:
        explicit ObeliskController(const std::string& name)
            : ObeliskNode(name) {
            // Declare all paramters
            this->declare_parameter(
                "callback_group_config_strs", "topic:topc1");
            this->declare_parameter("timer_ctrl_config_str", "topic:topc1");
            this->declare_parameter("pub_ctrl_config_str", "topic:topic1");
            this->declare_parameter("sub_est_config_str", "topic:topic1");
        }

        // Parse config string
        // Associate config str, template, function callback
        // Create all pubs, subs, timers
        // OnConfigure(function_refs);

       protected:
       private:
    }
}  // namespace obelisk
