class MainNode : public rclcpp::Node
{
public:
    MainNode() : rclcpp::Node("node", rclcpp::NodeOptions())
    {
        declare_parameter("param_int", 0);  // Default value of 0
        declare_parameter("param_string", "default_text");   // Default string is given
        
        RCLCPP_INFO(get_logger(), "Integer parameter: %d", get_parameter("param_int").as_int());
        RCLCPP_INFO(get_logger(), "String  parameter: %s", 
                                             get_parameter("param_string").as_string().c_str());
    }
};
