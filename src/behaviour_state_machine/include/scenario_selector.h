#ifndef SCENARIO_SELECTOR_H
#define SCENARIO_SELECTOR_H

// std c++
#include <iostream>

// terminal reader
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// ros
#include <ros/ros.h>
#include <std_msgs/Int8.h>

class KeyboardReader
{
public:
    KeyboardReader() : kfd(0)
    {
        initial();
    }

    void initial()
    {
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        // raw.c_cc[VEOL] = 1;
        // raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    }

    void readOne(int8_t *c)
    {
        int rc = read(kfd, c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("read failed");
        }
    }

    void shutdown()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    int kfd;
    struct termios cooked;
};

class ScenarioSelector
{
public:
    ScenarioSelector();
    ~ScenarioSelector();
    void run();

private:
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    ros::Publisher _pub_scenario_mode;

private:
    int _default_mode;

    std_msgs::Int8 _send_mode;

    static KeyboardReader _terminal_reader;

private:
    static void MySigintHandler(int sig);
};

#endif