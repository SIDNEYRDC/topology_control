/**********************************************************************
*   Tools for access and process file of the type .mat and .conf
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Abr 11 21:45:48
***********************************************************************/

#ifndef FILEUTILS_H
#define FILEUTILS_H

#include <armadillo>
#include "MatFile.h"
#include "utils.hpp"
#include "defines.hpp"

// Datatype to external robot definitions
typedef struct extbot {
    unsigned int id;        // Robot unique identifyer
    unsigned int type;      // Robot type (T_REAL,T_TURTLE,T_STAGE)
    std::string name;       // Name of robot in ROS environment
} extbot;

// Datatype with the topology definitions
typedef struct topology {
    unsigned int dc;                // Data capture frequency
    unsigned int n_iter;            // Number of interations
    unsigned int n_bots;            // Number of robots
    unsigned int opt_type;          // Topology control type

    float dt;                       // Simulation step
    float vx_max;                   // Maximum linear velocity in x axis
    float vy_max;                   // Maximum linear velocity in y axis
    float va_max;                   // Maximum angular velocity in z axis
    float vx_min;                   // Minimum linear velocity in x axis
    float vy_min;                   // Minimum linear velocity in y axis
    float va_min;                   // Minimum angular velocity in z axis

    bool info_scope;                // Scope of the information
    bool ref_pass;                  // Reference passage between neighbors
    bool com_c;                     // Communication radius constraint
    bool sec_c;                     // Security radius constraint
    bool ros;                       // ROS interface

    arma::mat init_x;               // Initial positions array
    arma::mat init_v;               // Initial velocities array

    arma::vec r_com;                // Maxium range of communication
    arma::vec r_sec;                // Minimum range of security

    arma::vec timeout;              // Timeout configurations

    arma::mat x_ref;                // Reference position

    std::vector<extbot> ext_info;   // External robots definitions
} topology;

// Get data from a input filestream
arma::rowvec get_data(const std::string search_key, std::ifstream &file_input);

// From data file stream, get a external configurations
std::vector<extbot> get_external_info(std::ifstream &file_input);

// From data file stream, fill a armadillo array
arma::mat fill_array(const std::string search_key, std::ifstream &file_input);

// Read configuration file and store its data
topology read_topology(std::string config_file);

// Store data in a matlab file
void write_matfile(arma::cube A_data,
                    arma::mat x_data,
                    arma::mat v_data,
                    arma::mat rmax_data,
                    arma::mat rmin_data,
                    arma::mat ref_data,
                    int N_iter);

// Detect and report error in the input file
void check_input_error(topology input);

#endif

