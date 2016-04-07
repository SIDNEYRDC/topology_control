/**********************************************************************
*   Robot class definition for the simulator
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Abr 12 03:59:18
***********************************************************************/

#ifndef ROBOT_H
#define ROBOT_H

#include <tgmath.h>
#include <time.h>
#include <armadillo>
#include "ros_interface.hpp"
//#include "motion_control.hpp"
//#include "k_optimization.hpp"

// Data type for virtual link
typedef struct vlink {
    unsigned int node_j;
    unsigned int node_k;
    float w_jk;
} vlink;

// Data type for critical nodes
typedef struct cnode {
    unsigned int c_node;
    std::vector<vlink> v_links;
} cnode;

// Datatype for message exchange
typedef struct msg {
    unsigned int id;                // Unique identifier for the robot

    bool ref_pass;                  // Reference passage option

    arma::mat x;                    // Contains the positions for the robot and its neighbors
    arma::mat v;                    // The velocities for the robot and its neighbors
    arma::mat A;                    // The adjacency matrix
    arma::vec r_com;                // The communication radius
    arma::vec r_sec;                // The security radius
    arma::rowvec x_r;               // The reference for consensus
    arma::uvec time_out;            // Dropped neighbors array

    std::vector<vlink> v_links;     //The virtual link nodes
} msg;

// Main class for robot instances
class robot {
public:
    unsigned int id;                // Unique identifier
    unsigned int n_bots;            // Number of robots in the network
    unsigned int opt_type;          // Type of topology control
    unsigned int bot_type;          // Type of the robot (T_STAGE,T_REAL,T_TURTLE)
    std::string bot_name;           // Name of the robot in ROS environment
    bool ref_pass;                  // Reference passage option

    float r_com_max;                // Maximum communication radius
    float vx_max;                   // Maximum linear velocity in x axis
    float vy_max;                   // Maximum linear velocity in y axis
    float va_max;                   // Maximum angular velocity in z axis
    float vx_min;                   // Minimum linear velocity in x axis
    float vy_min;                   // Minimum linear velocity in y axis
    float va_min;                   // Minimum angular velocity in z axis

    arma::vec r_com, r_sec;         // Communication and security radius
    arma::mat x, v;                 // Position and velocity matrices
    arma::mat A_l, A_p;             // Logical and physical adjacency matrices
    arma::rowvec x_r;               // The reference

    position pose;                  // Effective position of the robot
    velocity vel;                   // Effective velocity of the robot

    arma::uvec time_out;            // Dropped neighbors

    std::vector<cnode> c_nodes;     // Critical nodes in the neighborhood

    // Constructor
    robot();

    // Destructor
    ~robot();

    // Initialize a instance of a robot
    void init(const int id,
              const float r_com,
              const float r_sec,
              const arma::rowvec x,
              const arma::rowvec v,
              const unsigned int opt_type,
              const unsigned int bot_type,
              const std::string bot_name,
              const bool ref_pass);

    // Receive a message
    void receive(msg);

    // Receive a reference data
    void receive_ref(arma::rowvec x_r);

    // Topology control algorithms
    void topology_control(const unsigned int iter);

    // High level motion control algorithms
    void hl_motion_control(const unsigned int mode, const bool sec_c, const bool com_c, const float dt);

    // Verify if the id robot is critical node and return its index
    bool verify_cnode(unsigned int id, int &index_cnode);
private:
    // Message counter
    arma::umat msg_num;

    // Is true when bi-connectivity is reached
    bool is_biconnected;

    // 1-hop neighborhood of i
    arma::rowvec N_1;

    // Index for message counter
    arma::uvec time_index;

    // Message counter
    void msg_count(int);

    // Time index update
    void msg_update();

    // Find virtual neighbors
    arma::uvec virtual_neighbors(arma::uvec);

    // Low level motion control algorithms
    void ll_motion_control(const float dt);

    // Make the id robot as critical node
    void make_cnode(unsigned int id, vlink v_link);

    // Verify if the virtual link is assigned with the critical node id
    bool verify_vlink(unsigned int id, vlink v_link, int &index_vlink, int &index_cnode);

    // Verify if id is a virtual link part
    bool verify_vlink(unsigned int id, std::vector<vlink> v_links, int &index_vlink);
};

#endif

