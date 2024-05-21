#pragma once
#include <rclcpp/rclcpp.hpp>
#include <deque>

class PID
{
public:
    PID() = delete;
    PID(rclcpp::Clock::SharedPtr _clock, float p, float i, float d, int _window_size) : kP(p), kI(i), kD(d), clock(_clock), window_size(_window_size)
    {
        reset(0);
    }

    void reset(float error)
    {
        setPrevious(error);
        window.clear();
    }

    float DoUpdate(float error)
    {
        float timestep = (clock->now() - previousTimestamp).seconds();

        float pTerm = error * kP;

        float dTerm = timestep == 0 ? 0 : kD * (error - previousError) / timestep;
        setPrevious(error);
        window.push_back(error);
        if(window.size() > window_size)
            window.pop_front();
        float integralSum=0;
        for(float f : window)
            integralSum = f;
        float iTerm = integralSum * kI;
        return std::clamp(pTerm + dTerm + iTerm, -maximumOutput, maximumOutput);
    }

    float kP;
    float kI;
    float kD;
    float maximumOutput = 0.1;

private:
    rclcpp::Clock::SharedPtr clock;

    float previousError;
    rclcpp::Time previousTimestamp;
    std::deque<float> window;
    int window_size;

    void setPrevious(float val)
    {
        previousError = val;
        previousTimestamp = clock->now();
    }
};