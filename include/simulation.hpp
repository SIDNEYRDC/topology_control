/******************************************************************************
*   Topology Control Simulator Definition
*   Written by Sidney Carvalho, 2016.
*   Last Change: 2016 Mai 24 18:22:17
******************************************************************************/

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

    // Start the simulation
    void start();

    // Compute each iteraction of the simulation
    void iterate();

    // Finish the simulation
    void finish();

private:
    unsigned int dc_feq;            // Data capture frequency
    unsigned int iter_num;          // Number of interations
    unsigned int bots_num;          // Number of robots
    unsigned int opt_type;          // Topology control type

    float hl_dt;                    // High level simulation step (s)
    float ll_dt;                    // Low level simulation step (s)
    float com_dt;                   // Communication step (s)

    float vx_max;                   // Maximum linear velocity in x axis
    float vy_max;                   // Maximum linear velocity in y axis
    float va_max;                   // Maximum angular velocity in z axis
    float vx_min;                   // Minimum linear velocity in x axis
    float vy_min;                   // Minimum linear velocity in y axis
    float va_min;                   // Minimum angular velocity in z axis

    bool ref_on;                    // Reference passage between neighbors
    bool com_on;                    // Communication radius constraint
    bool sec_on;                    // Security radius constraint
    bool cov_on;                    // Coverage radius constraint
    bool ros_on;                    // ROS interface

    vector<*robot> bots;            // Robots pointer array

    arma::cube adj_data;            // Adjacency matrix data on time
    arma::mat pos_data;             // Positions data on time
    arma::mat vel_data;             // Velocities data on time
    arma::mat com_data;             // Comunication radius data on time
    arma::mat cov_data;             // Coverage radius data on time
    arma::mat sec_data;             // Security radius data on time

    // Load a configuration input file
    void load_input(std::string input);

    // Write the simulation data output file
    void save_output(std::string output);

};

#endif

