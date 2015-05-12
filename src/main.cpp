/**********************************************************************
*	Topology Control Simulator
*	Written by Sidney RDC, 2014.
*	Last Change: 2015 Abr 25 21:07:48
***********************************************************************/

#include <signal.h>
#include "robot.hpp"
#include "file_utils.hpp"
#include "motion_control.hpp"

using namespace std;
using namespace arma;

int main(int argc, char** argv) {
    // Random seeder
    srand(time(NULL));

    // Verify if the parameters are correct
    if(argc < 2) {
        cout << "ERROR: Configuration file not found!" << endl;
        exit(1);
    }

    // ROS communication interface
    ros_interface *ros_com;

    // Time counter variables
    clock_t t0, t1;

    // Read topology configuration from a external file
    const topology topology_config = read_topology(argv[1]);

    // Optimization of links
    const unsigned int opt_type = topology_config.opt_type;

    // Reference pass
    const unsigned int ref_pass = topology_config.ref_pass;

    // Counter for data capture
    unsigned int c = 0;

    // Data capture frequency
    const unsigned int dc = topology_config.dc;

    // Number of interactions (changes of messages)
    const float n_iter = topology_config.n_iter;

    // Simulation step
    const float dt = topology_config.dt;

    // Number of robots in the network
    const unsigned int n = topology_config.n_bots;

    // Communication and security radius constraints
    const bool com_c = topology_config.com_c;
    const bool sec_c = topology_config.sec_c;
    const bool ros = topology_config.ros;

    // Timeout configurations
    const vec timeout = topology_config.timeout;

    // Create robots
    vector<robot> bot(n);

    // Verify if ros option is enabled
    if(ros) ros_com = new ros_interface(argc,argv,"ros_interface");

    // Arrays for data capture
    mat ref_data;
    mat x_data = zeros<mat>(n_iter/dc,2*n);
    mat v_data = zeros<mat>(n_iter/dc,2*n);
    cube A_data = zeros<cube>(n,n,n_iter/dc); // [r,c,s] {r:row c:column s:slice}
    mat rmax_data = zeros<mat>(n_iter/dc,n);
    mat rmin_data = zeros<mat>(n_iter/dc,n);

    // Temporary variables for load topology configurations
    rowvec init_x, init_v, x_ref;
    init_x = init_v = x_ref = zeros<rowvec>(2);
    float r_com, r_sec;
    unsigned int n_ref = 0, bot_type;
    int index_cnode;
    string bot_name;

    for(unsigned int i = 0; i < n; i++) {
        // Load initial positions for the robot i
        if(topology_config.init_x.is_empty() || topology_config.init_x.n_rows < i + 1 || topology_config.init_x(0) == BIG_VAL) {
            init_x(0) = 0;
            init_x(1) = 0;
        } else init_x = topology_config.init_x(i,span());

        // Load initial velocities for the robot i
        if(topology_config.init_v.is_empty() || topology_config.init_v.n_rows < i + 1 || topology_config.init_v(0) == BIG_VAL) {
            init_v(0) = 0;
            init_v(1) = 0;
        } else init_v = topology_config.init_v(i,span());

        // Load communication range for robot i
        if(topology_config.r_com.is_empty() || topology_config.r_com.n_elem < i + 1 || topology_config.r_com(i) == BIG_VAL) r_com = 0;
        else r_com = topology_config.r_com(i);

        // Load security range for robot i
        if(topology_config.r_sec.is_empty() || topology_config.r_sec.n_elem < i + 1 || topology_config.r_sec(i) == BIG_VAL) r_sec = 0;
        else r_sec = topology_config.r_sec(i);

        // Load reference if the robot i receive the reference
        if(!topology_config.x_ref.is_empty() && topology_config.x_ref.n_rows >= i + 1) {
            x_ref = topology_config.x_ref(i,span());

            if(x_ref(0) != BIG_VAL) {
                bot[i].receive_ref(x_ref);

                if(!ref_data.is_empty() && (ref_data(n_ref-1,0) != x_ref(0) || ref_data(n_ref-1,1) != x_ref(1))) {
                    n_ref++;
                    ref_data.resize(n_ref,2);
                    ref_data(n_ref-1,span()) = x_ref;
                } else if(ref_data.is_empty()) {
                    n_ref++;
                    ref_data.resize(n_ref,2);
                    ref_data(n_ref-1,span()) = x_ref;
                }
            }
        }

        // Load external configurations
        if(ros && topology_config.ext_info.size() > i) {
            bot_type = topology_config.ext_info[i].type;
            bot_name = topology_config.ext_info[i].name;

        // Clear type and name
        } else if(!ros || topology_config.ext_info.size() <= i) {
            bot_type = 0;
            bot_name = "";
        }

        // Set the configurations in the robot structure
        bot[i].init(i,r_com,r_sec,init_x,init_v,opt_type,bot_type,bot_name,ref_pass);

        // Add robot to the ROS interface
        if(ros && bot_name != "" && bot_type != 0) {
            position init_pose;
            init_pose.x = init_x(0);
            init_pose.y = init_x(1);

            ros_com->add_node(i,bot_type,bot_name,init_pose);
        }
    }

    // Initial time capture
    t0 = clock();

    // Temporary positions array
    rowvec x_i, x_j;

    // Communication matrix
    mat Ac = ones<mat>(n,n);
    //Ac
    //<< 0 << 1 << 0 << 1 << 0 << 0 << endr
    //<< 0 << 0 << 1 << 0 << 0 << 0 << endr
    //<< 0 << 1 << 0 << 0 << 0 << 0 << endr
    //<< 1 << 0 << 0 << 0 << 0 << 0 << endr
    //<< 1 << 0 << 0 << 1 << 0 << 0 << endr
    //<< 0 << 0 << 1 << 0 << 0 << 0 << endr;

    // The main loop
    for(unsigned int iter = 0; iter < n_iter; iter++) {
        if(ros) {
            // Simulation step (s)
            ros_com->clock(dt);

            // Receive data from ROS interface
            for(unsigned int i = 0; i < n; i++) {
                if(bot[i].bot_type != 0 && bot[i].bot_name != "") {
                    bot[i].pose = ros_com->node_receive(i);

                    bot[i].x(i,0) = bot[i].pose.x;
                    bot[i].x(i,1) = bot[i].pose.y;
                }
            }
        }

        // Communication loop simulator
        for(unsigned int i = 0; i < n - 1; i++) {
            // Verify timeout configurations for robot i
            if(timeout.is_empty() || timeout.n_elem < i + 1 || timeout(i) == BIG_VAL || timeout(i) > iter) {
                // Fill position for i
                x_i = bot[i].x(bot[i].id,span());

                for(unsigned int j = i + 1; j < n; j++) {
                    // Verify timeout configurations for robot j
                    if(timeout.is_empty() || timeout.n_elem < j + 1 || timeout(j) == BIG_VAL || timeout(j) > iter) {
                        // Fill position for j
                        x_j = bot[j].x(bot[j].id,span());

                        // Compute distance between i and j
                        float dist_ij = dist(x_i,x_j);

                        // Verify if the distance between i and j is less who the communication range of j
                        if(dist_ij <= bot[j].r_com(j)) {

                            // Make the message package with content from robot j
                            msg m_j;
                            m_j.id = bot[j].id;
                            m_j.A = bot[j].A_p;
                            m_j.x = bot[j].x;
                            m_j.v = bot[j].v;
                            m_j.r_sec = bot[j].r_sec;
                            m_j.r_com = bot[j].r_com;
                            m_j.x_r = bot[j].x_r;
                            m_j.ref_pass = bot[j].ref_pass;
                            m_j.time_out = bot[j].time_out;

                            if(bot[j].verify_cnode(j,index_cnode)) m_j.v_links = bot[j].c_nodes[index_cnode].v_links;

                            // Robot i receive a message from j
                            if(Ac(i,j)) bot[i].receive(m_j);
                        }

                        // Verify if the distance between i and j is less who the communication range of i
                        if(dist_ij <= bot[i].r_com(i)) {
                            // Make the message package with content from robot i
                            msg m_i;
                            m_i.id = bot[i].id;
                            m_i.A = bot[i].A_p;
                            m_i.x = bot[i].x;
                            m_i.v = bot[i].v;
                            m_i.r_sec = bot[i].r_sec;
                            m_i.r_com = bot[i].r_com;
                            m_i.x_r = bot[i].x_r;
                            m_i.ref_pass = bot[i].ref_pass;
                            m_i.time_out = bot[i].time_out;

                            if(bot[i].verify_cnode(i,index_cnode)) m_i.v_links = bot[i].c_nodes[index_cnode].v_links;

                            // Robot j receive a message from i
                            if(Ac(j,i)) bot[j].receive(m_i);
                        }
                    }
                }
            }
        }

        for(unsigned int i = 0; i < n; i++) {
            // Topology control for robot i
            bot[i].topology_control(iter);

            // Verify timeout configurations for robot j
            if(timeout.is_empty() || timeout.n_elem < i + 1 || timeout(i) == BIG_VAL || timeout(i) > iter) {
                // High level motion control
                bot[i].hl_motion_control(CONNECTIVITY,sec_c,com_c,dt);

            // Reset the robot velocities
            } else if(timeout(i) <= iter) {
                bot[i].v(i,span()) = zeros<rowvec>(2);
                bot[i].vel.x = 0;
                bot[i].vel.yaw = 0;
            }

            // Send null velocities when the simulation is finish
            if(ros && iter == n_iter - 1) {
                ros_com->node_send(i,velocity());

            // Send velocities to ROS interface
            } else if(ros && bot[i].bot_name != "" && bot[i].bot_type != 0) {
                ros_com->node_send(i,bot[i].vel);
            }
        }

        // Capture the data
        if(iter == c * dc) {
            unsigned int index = 0;

            for(unsigned int i = 0; i < 2 * n; i = i + 2) {
                x_data(c,i) = bot[index].x(index,0);
                x_data(c,i+1) = bot[index].x(index,1);

                v_data(c,i) = bot[index].v(index,0);
                v_data(c,i+1) = bot[index].x(index,1);

                index++;
            }

            for(unsigned int i = 0; i < n; i++) {
                if(bot[i].n_bots == n)
                    A_data.slice(c).col(i) = bot[i].A_l.col(i);
                else {
                    mat aux = bot[i].A_l;
                    aux.resize(n,n);
                    A_data.slice(c).col(i) = aux.col(i);
                }
                rmax_data(c,i) = bot[i].r_com(i);
                rmin_data(c,i) = bot[i].r_sec(i);
            }

            c++;
            //cout << "iter::" << iter << endl;
        }
    }

    // Final time capture
    t1 = clock();

    cout << "Simulation time:" << calc_time(t0,t1) << "s" << endl;

    // Write a .mat file
    write_matfile(A_data,x_data,v_data,rmax_data,rmin_data,ref_data,n_iter);

    return 0;
}

