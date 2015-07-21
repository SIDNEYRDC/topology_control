/**********************************************************************
*   Topology Control Simulator Definition
*   Written by Sidney RDC, 2015.
*   Last Change: 2015 Abr 12 01:20:53
***********************************************************************/

#ifndef SIMULATION_H
#define SIMULATION_H

#include "robot.hpp"

class simulation {
public:
    // Default constructor
    simulation();

    // Initialized constructor
    simulation(std::string input, std::string output);

    // Destructor
    ~simulation();

    void load_input(std::string input);

    void save_output(std::string output);

    void start();

    void iterate();

    void finish();

private:
    unsigned int dc;                // Data capture frequency
    unsigned int iter_num;          // Number of interations
    unsigned int bots_num;          // Number of robots
    unsigned int opt_type;          // Topology control type

    float hl_dt;                    // High level simulation step
    float ll_dt;                    // Low level simulation step
    float vx_max;                   // Maximum linear velocity in x axis
    float vy_max;                   // Maximum linear velocity in y axis
    float va_max;                   // Maximum angular velocity in z axis
    float vx_min;                   // Minimum linear velocity in x axis
    float vy_min;                   // Minimum linear velocity in y axis
    float va_min;                   // Minimum angular velocity in z axis

    bool ref_on;                    // Reference passage between neighbors
    bool com_on;                    // Communication radius constraint
    bool sec_on;                    // Security radius constraint
    bool ros_on;                    // ROS interface

    vector<*robot> bots;            // Robots pointer array

    arma::cube adj_data;            // Adjacency matrix data
    arma::mat pos_data;             // Positions data
    arma::mat vel_data;             // Velocities data
    arma::mat com_data;             // Comunication radius data
    arma::mat sec_data;             // Security radius data

};

#endif

