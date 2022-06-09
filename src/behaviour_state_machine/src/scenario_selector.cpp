#include "scenario_selector.h"

KeyboardReader ScenarioSelector::_terminal_reader;

ScenarioSelector::ScenarioSelector() : _nh(""), _private_nh("~")
{
    _private_nh.param<int>("default_mode", _default_mode, -1);

    _pub_scenario_mode = _nh.advertise<std_msgs::Int8>("scenario_mode", 1, true);
}

ScenarioSelector::~ScenarioSelector()
{
}

void ScenarioSelector::MySigintHandler(int sig)
{
    ROS_INFO("close scenario selector!");
    ros::shutdown();
    _terminal_reader.shutdown();
    exit(0);
}

void ScenarioSelector::run()
{
    signal(SIGINT, MySigintHandler);
    ROS_INFO("--------Choose a mode first!-------");
    ROS_INFO("--------0:Stop-------");
    ROS_INFO("--------1:MultiTrajPlanning-------");
    ROS_INFO("--------2:PathTracing-------");
    ROS_INFO("--------3:StaticExec-------");

    while (true)
    {
        //只在第一次生效
        if (_default_mode != -1)
        {
            _send_mode.data = static_cast<int8_t>(_default_mode) + 0x30;
            _default_mode = -1;
        }
        else
        {
            try
            {
                _terminal_reader.readOne(&_send_mode.data);
            }
            catch (const std::runtime_error &)
            {
                std::cerr << "read terminal failed" << std::endl;
                continue;
            }
        }

        switch (_send_mode.data)
        {
        case 0x30:
        {
            ROS_INFO("Change to Scenario: Stop");
            _pub_scenario_mode.publish(_send_mode);
            break;
        }
        case 0x31:
        {
            ROS_INFO("Change to Scenario: MultiTrajPlanning");
            _pub_scenario_mode.publish(_send_mode);
            break;
        }
        case 0x32:
        {
            ROS_INFO("Change to Scenario: PathTracing");
            _pub_scenario_mode.publish(_send_mode);
            break;
        }
        case 0x33:
        {
            ROS_INFO("Change to Scenario: StaticExec");
            _pub_scenario_mode.publish(_send_mode);
            break;
        }
        default:
        {

            break;
        }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scenario_selector");

    ScenarioSelector ss;
    ss.run();

    return -1;
}