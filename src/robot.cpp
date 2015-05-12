/**********************************************************************
*   Robot class implementation for the simulator
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Abr 27 16:54:09
***********************************************************************/

#include <cstdlib>
#include "robot.hpp"
#include "motion_control.hpp"
#include "k_optimization.hpp"

using namespace std;
using namespace arma;

// Constructor
robot::robot() {
    // Number of robots in the network
    n_bots = 1;

    // Initially the network isn't bi-connecty
    is_biconnected = false;

    // Set robot type as generic (0)
    bot_type = 0;

    // Set robot external name as ""
    bot_name = "";
}

// Destructor
robot::~robot() {
    // NOP
}

// Start the robot with these parameters
void robot::init(const int id,
                 const float r_com,
                 const float r_sec,
                 const arma::rowvec x,
                 const arma::rowvec v,
                 const unsigned int opt_type,
                 const unsigned int bot_type,
                 const std::string bot_name,
                 const bool ref_pass) {

    this->id = id;

    this->opt_type = opt_type;
    this->bot_type = bot_type;
    this->bot_name = bot_name;
    this->ref_pass = ref_pass;

    this->r_com.set_size(id+1);
    this->r_com.fill(0);
    this->r_com(id) = r_com;
    this->r_com_max = r_com;

    this->r_sec.set_size(id+1);
    this->r_sec.fill(0);
    this->r_sec(id) = r_sec;

    this->x.set_size(id+1,2);
    this->x.fill(0);
    this->x(id,span::all) = x;

    pose.x = x(0);
    pose.y = x(1);
    pose.z = 0;
    pose.roll = 0;
    pose.pitch = 0;
    pose.yaw = 0;

    this->v.set_size(id+1,2);
    this->v.fill(0);
    this->v(id,span::all) = v;

    vel.x = v(0);
    vel.y = v(1);
    vel.z = 0;
    vel.roll = 0;
    vel.pitch = 0;
    vel.yaw = 0;

    this->A_p.set_size(id+1,id+1);
    this->A_p.fill(0);
    this->A_l = this->A_p;

    this->time_out.set_size(id+1);
    this->time_out.fill(0);

    this->msg_num.set_size(id+1,HIST_SIZE);
    this->msg_num.fill(0);

    this->time_index.set_size(id+1);
    this->time_index.fill(0);
}

// Receive a message
void robot::receive(msg m) {
    // Define the lenght of A_i and A_j
    if(A_p.n_rows > m.A.n_rows) m.A.resize(A_p.n_rows,A_p.n_rows);
    else if(A_p.n_rows < m.A.n_rows) A_p.resize(m.A.n_rows,m.A.n_rows);

    // Update number of the robots in the network
    n_bots = A_p.n_rows;

    // Resize logical adjacency matrix
    if(A_l.n_rows < n_bots) A_l.resize(n_bots,n_bots);

    // Resize positions and velocities arrays
    if(x.n_rows < n_bots) x.resize(n_bots,2);
    if(v.n_rows < n_bots) v.resize(n_bots,2);

    // Resize ranges vectors
    if(r_sec.n_elem < n_bots) r_sec.resize(n_bots);
    if(r_com.n_elem < n_bots) r_com.resize(n_bots);

    // Compute the message number received from each neighbor
    msg_count(m.id);

    // Neighborhood of i
    uvec N = find(A_p.col(id));

    // Neighborhood of j
    uvec N_j = find(m.A.col(m.id));

    // Positions vector
    rowvec x_j = m.x(m.id,span());
    rowvec x_i = x(id,span());

    // Distance between i and j
    const float dist_ij = dist(x_i,x_j);

    // Find the nearest neighbor k
    float min_dist_ik = BIG_VAL, min_dist_jk = BIG_VAL;
    for(unsigned int k = 0; k < N.n_elem; k++) {
        float dist_ik = dist(x_i,x(N(k),span()));
        float dist_jk = dist(x_j,x(N(k),span()));

        // The minimum distance between i and its neighbors
        if(dist_ik < min_dist_ik) min_dist_ik = dist_ik;

        // The minimum distance between j and i's neighbor k
        if(dist_jk < min_dist_jk) min_dist_jk = dist_jk;
    }

    int index_vlink, index_cnode, temp;

    // Add link rule
    switch(opt_type) {
        case 1:
            if(m.A(id,m.id) > 0 || (dist_ij < min_dist_jk) || (dist_ij < min_dist_ik)) A_p(m.id,id) = dist_ij;
            break;
        case 3:
            // Check if id robot is part of virtual link
            if(verify_vlink(id,m.v_links,index_vlink)) {
                if(verify_cnode(m.id,index_cnode)) {
                    for(unsigned int i = 0; i < c_nodes[index_cnode].v_links.size(); i++) {
                        if(!verify_vlink(c_nodes[index_cnode].v_links[i].node_j,m.v_links,temp) ||
                           !verify_vlink(c_nodes[index_cnode].v_links[i].node_k,m.v_links,temp)) {
                            A_l(c_nodes[index_cnode].v_links[i].node_j,c_nodes[index_cnode].v_links[i].node_k) = 0;
                            A_l(c_nodes[index_cnode].v_links[i].node_k,c_nodes[index_cnode].v_links[i].node_j) = 0;

                            c_nodes[index_cnode].v_links.erase(c_nodes[index_cnode].v_links.begin()+i);
                        }
                    }
                }

                // Make the virtual link
                A_l(m.v_links[index_vlink].node_j,m.v_links[index_vlink].node_k) = A_l(m.v_links[index_vlink].node_k,m.v_links[index_vlink].node_j) = norm_2<rowvec>(x(m.v_links[index_vlink].node_j,span()) - x(m.v_links[index_vlink].node_k,span()),2);

                // Store the critical node
                make_cnode(m.id,m.v_links[index_vlink]);

            // Remove the virtual link with id if j robot is not critical node
            } else if(verify_cnode(m.id,index_cnode)) {
                for(unsigned int i = 0; i < c_nodes[index_cnode].v_links.size(); i++) {
                    A_l(c_nodes[index_cnode].v_links[i].node_j,c_nodes[index_cnode].v_links[i].node_k) = A_l(c_nodes[index_cnode].v_links[i].node_k,c_nodes[index_cnode].v_links[i].node_j) = 0;
                }

                c_nodes.erase(c_nodes.begin()+index_cnode);
            }
        default:
            A_p(m.id,id) = dist_ij;
    }

    // Dropped robots
    uvec dropped = find(m.time_out);

    // Get timeout array from neighbor
    if(!dropped.is_empty()) {
        for(unsigned int j = 0; j < dropped.n_elem; j++) {
            // Detect and remove virtual links with j
            for(unsigned int i = 0; i < c_nodes.size(); i++) {
                if(verify_vlink(dropped(j),c_nodes[i].v_links,index_vlink)) {
                    c_nodes[i].v_links.erase(c_nodes[i].v_links.begin()+index_vlink);

                    A_p(dropped(j),id) = 0;
                    A_l(dropped(j),id) = 0;
                }
            }

            A_p(dropped(j),m.id) = 0;
            A_l(dropped(j),m.id) = 0;
        }
    }

    // Save the j robot position and velocity
    x(m.id,span()) = m.x(m.id,span());
    v(m.id,span()) = m.v(m.id,span());

    // Save the j robot ranges
    r_sec(m.id) = m.r_sec(m.id);
    r_com(m.id) = m.r_com(m.id);

    // Save the j robot adjacency
    A_p(span(),m.id) = m.A(span(),m.id);

    // Get all neighbors of j (1-hop neighbors of i)
    for(unsigned int k = 0; k < N_j.n_elem; k++) {
        // Verify if the robot isn't j or i
        if(N_j(k) != id && N_j(k) != m.id) {
            // Save the 1-hop neighbor's positions
            x(N_j(k),span()) = m.x(N_j(k),span());

            // Save the 1-hop neighbor's velocities
            v(N_j(k),span()) = m.v(N_j(k),span());

            // Save the 1-hop neighbor's ranges
            r_sec(N_j(k)) = m.r_sec(N_j(k));
            r_com(N_j(k)) = m.r_com(N_j(k));

            // Save the 1-hop neighbor's adjacency matrix
            A_p(span(),N_j(k)) = m.A(span(),N_j(k));
        }
    }

    // Reference pass
    if(m.ref_pass && !m.x_r.is_empty()) {
        x_r = m.x_r;
    }
}

// Receive the reference
void robot::receive_ref(rowvec x_r) {
    this->x_r = x_r;
}

// Update for adjacency matrix
void robot::topology_control(const unsigned int iter) {
    // Udate the message history
    msg_update();

    // Find the neighbors of id
    uvec N = find(A_p.col(id));

    // Clear timeout array
    time_out.set_size(0);

    // Delete mute robots
    for(unsigned int j = 0; j < N.n_elem; j++) {
        if(sum(msg_num.row(N(j))) / msg_num.n_cols == msg_num(N(j),time_index(N(j)))) {
            A_p.row(N(j)) = zeros<rowvec>(n_bots);
            A_l.row(N(j)) = zeros<rowvec>(n_bots);
            A_p.col(N(j)) = zeros<vec>(n_bots);
            A_l.col(N(j)) = zeros<vec>(n_bots);
            time_out.resize(N(j)+1);
            time_out(N(j)) = 1;

            cout << "bot:" << id << " timeout:" << N(j) << endl;

            int index_cnode, index_vlink;

            // Detect and remove the virtual links assigned to N(j)
            if(verify_cnode(N(j),index_cnode)) {
                for(unsigned int i = 0; i < c_nodes[index_cnode].v_links.size(); i++) {
                    A_l(c_nodes[index_cnode].v_links[i].node_j,c_nodes[index_cnode].v_links[i].node_k) = A_l(c_nodes[index_cnode].v_links[i].node_k,c_nodes[index_cnode].v_links[i].node_j) = 0;

                    cout << "bot:" << id << " remove:" << c_nodes[index_cnode].v_links[i].node_j << " - " << c_nodes[index_cnode].v_links[i].node_k << endl;
                }

                c_nodes.erase(c_nodes.begin()+index_cnode);
            }

            // Remove id's virtual link composed with N(j)
            while(verify_cnode(id,index_cnode) && verify_vlink(N(j),c_nodes[index_cnode].v_links,index_vlink)) {
                c_nodes[index_cnode].v_links.erase(c_nodes[index_cnode].v_links.begin()+index_vlink);
            }

        }
    }

    // Update neighborhood of id
    N = find(A_p.col(id));

    // Exit if robot is disconnected
    if(N.is_empty()) return;

    /* Select the type of topology control-----------------------------/
     * 1: Minimization with MST
     * 2: Bi-connectivity maintenance with TSP
     * 3: Bi-connectivity maintenace with virtual link
     *----------------------------------------------------------------*/
    if(opt_type == 0) {
        // Logical adjacency matrix equal physical adjacency matrix
        A_l = A_p;
    } else if(opt_type == 1) {
        // Reduce the adjacency matrix to 1-hop neighbors matrix (local info)
        vector<mat> out = min_mat(id,A_p,N1);

        // Minimal spanning tree optimization
        mat A_i = mst_optimization(out[0]);

        // Update logical adjacency matrix
        for(unsigned int i = 0; i < A_i.n_rows - 1; i++) {
            for(unsigned int j = i+1; j < A_i.n_cols; j++) {
                A_l(out[1].at(0,i),out[1].at(0,j)) = A_i(i,j);
                A_l(out[1].at(0,j),out[1].at(0,i)) = A_i(j,i);
            }
        }

        // Maximum distance between i and its logical neighbor j
        float max_dist_ij = -BIG_VAL;

        // Gain to minimal communication range
        const float delta_com = 2 * r_sec(id) * 0.15;

        // Find the greater link of the robot id
        for(unsigned int j = 0; j < n_bots; j++) {
            if(A_l(j,id) > max_dist_ij) {
                max_dist_ij = A_l(j,id);

                // Reduce the communication range to optimal value
                if(max_dist_ij + delta_com < r_com_max && max_dist_ij + delta_com > 2 * r_sec(id))
                    r_com(id) = max_dist_ij + delta_com;
                else if(max_dist_ij + delta_com < r_com_max)
                    r_com(id) = 2 * r_sec(id) + delta_com;
                else
                    r_com(id) = r_com_max;
            }
        }

    } else if(opt_type == 2) {
        // Update logical adjacency matrix with physical adjacency matrix
        A_l = logic_or(A_l,A_p,x);

        // Reduce the adjacency matrix to 1-hop neighbors matrix (local info)
        vector<mat> out = min_mat(id,A_p,N1);

        // Verify if the 1-hop neighborhood are changed
        if(N_1.is_empty() || N_1.n_elem != out[1].n_cols || accu(N_1 - out[1].row(0)) != 0) {
            N_1 = out[1].row(0);
            is_biconnected = false;
        }

        // If the network is not bi-connected, run the tsp algorithm
        if(!is_biconnected) {
            // Number of robot in 1-hop neighborhood
            unsigned int n = out[1].n_cols;

            // Distance matrix to all 1-hop neighbors
            mat D = zeros<mat>(n,n);

            // Calculate distance for all 1-hop neighbors
            for(unsigned int i = 0; i < n - 1; i++) {
                for(unsigned int j = i + 1; j < n; j++) {
                    D(i,j) = D(j,i) = norm_2<rowvec>(x(out[1].at(0,i),span())-x(out[1].at(0,j),span()),2);
                }
            }

            // Minimal bi-connected graph optimization
            mat A_i = tsp_optimization(D);

            // Update logical adjacency matrix
            for(unsigned int i = 0; i < A_i.n_rows - 1; i++) {
                for(unsigned int j = i+1; j < A_i.n_cols; j++) {
                    A_l(out[1].at(0,i),out[1].at(0,j)) = A_i(i,j);
                    A_l(out[1].at(0,j),out[1].at(0,i)) = A_i(j,i);
                }
            }

            // The network is bi-connected
            is_biconnected = true;
        }
    } else if(opt_type == 3) {
        int index_cnode, index_vlink;
        unsigned int vj = BIG_VAL, vk = BIG_VAL;
        float min_dist_jk = BIG_VAL, dist_jk = BIG_VAL;

        // Update logical adjacency matrix with physical adjacency matrix
        A_l = logic_or(A_l,A_p,x);

        // Temporary matrix
        mat temp = A_p;

        // Add virtual links to temporary matrix
        if(verify_cnode(id,index_cnode)) {
            for(unsigned int i = 0; i < c_nodes[index_cnode].v_links.size(); i++) {
                temp(c_nodes[index_cnode].v_links[i].node_j,c_nodes[index_cnode].v_links[i].node_k) = temp(c_nodes[index_cnode].v_links[i].node_k,c_nodes[index_cnode].v_links[i].node_j) = c_nodes[index_cnode].v_links[i].w_jk;
            }
        }

        // Reduce the adjacency matrix to 1-hop neighbors matrix (local info)
        vector<mat> out = min_mat(id,temp,N1);

        // Real id for the minimized adjacency matrix
        uvec real_id = find(out[1].row(0) == id);

        // Detect if the robot is a critical node
        if(iter >= n_bots - 1 && is_critical(out[0],real_id(0))) {
            // Find the nearest neighbors of i that are not neighbors yet
            for(unsigned int j = 0; j < N.n_elem-1; j++) {
                rowvec x_j = x(N(j),span());
                for(unsigned int k = j+1; k < N.n_elem; k++) {
                    dist_jk = dist(x_j,x(N(k),span()));

                    if(!A_p(N(k),N(j)) && !A_p(N(j),N(k)) && dist_jk < min_dist_jk) {
                        min_dist_jk = dist_jk;

                        vj = N(j);
                        vk = N(k);
                    }
                }
            }

            if(vj + vk < BIG_VAL) {
                // Make the virtual link
                vlink v_link;
                v_link.node_j = vj;
                v_link.node_k = vk;
                v_link.w_jk = dist_jk;

                // Make id as critical node
                make_cnode(id,v_link);

                // Make the logical adjacency matrix
                A_l(vj,vk) = A_l(vk,vj) = dist_jk;
            }
        } else if(iter >= n_bots - 1) {
            // Remove physical links from virtual links array (case id critical node)
            if(verify_cnode(id,index_cnode)) {
                // Reduce the adjacency matrix to 1-hop neighbors matrix (local info)
                out = min_mat(id,A_p,N1);

                // Real id for the minimized adjacency matrix
                real_id = find(out[1].row(0) == id);

                if(!is_critical(out[0],real_id(0))) {
                    for(unsigned int i = 0; i < c_nodes[index_cnode].v_links.size(); i++) {
                        A_p(c_nodes[index_cnode].v_links[i].node_j,c_nodes[index_cnode].v_links[i].node_k) = A_p(c_nodes[index_cnode].v_links[i].node_k,c_nodes[index_cnode].v_links[i].node_j) = 0;

                        cout << "bot:" << id << " remove_vlink:" << c_nodes[index_cnode].v_links[i].node_j << " - " << c_nodes[index_cnode].v_links[i].node_k << endl;
                    }

                    c_nodes.erase(c_nodes.begin()+index_cnode);
                } else {
                    for(unsigned int i = 0; i < c_nodes[index_cnode].v_links.size(); i++) {
                        if(A_p(c_nodes[index_cnode].v_links[i].node_j,c_nodes[index_cnode].v_links[i].node_k) > 0 || A_p(c_nodes[index_cnode].v_links[i].node_k,c_nodes[index_cnode].v_links[i].node_j) > 0) {
                            c_nodes[index_cnode].v_links.erase(c_nodes[index_cnode].v_links.begin()+i);

                            cout << "bot:" << id << " remove_vlink:" << c_nodes[index_cnode].v_links[i].node_j << " - " << c_nodes[index_cnode].v_links[i].node_k << endl;
                        } else {
                            // Find the nearest neighbors of i that are not neighbors yet
                            for(unsigned int j = 0; j < N.n_elem-1; j++) {
                                rowvec x_j = x(N(j),span());
                                for(unsigned int k = j+1; k < N.n_elem; k++) {
                                    dist_jk = dist(x_j,x(N(k),span()));

                                    if(!A_p(N(k),N(j)) && !A_p(N(j),N(k)) && dist_jk < min_dist_jk) {
                                        min_dist_jk = dist_jk;

                                        vj = N(j);
                                        vk = N(k);
                                    }
                                }
                            }

                            // Verify if exist a better virtual link
                            if(dist_jk < norm_2<rowvec>(x(c_nodes[index_cnode].v_links[i].node_j,span()) - x(c_nodes[index_cnode].v_links[i].node_k,span()),2)) {
                                // Remove older virtual link from adjacency matrix
                                A_l(c_nodes[index_cnode].v_links[i].node_j,c_nodes[index_cnode].v_links[i].node_k) = A_l(c_nodes[index_cnode].v_links[i].node_k,c_nodes[index_cnode].v_links[i].node_j) = 0;

                                // Remove older virtual link from v_links array
                                c_nodes[index_cnode].v_links.erase(c_nodes[index_cnode].v_links.begin()+i);

                                // Make new virtual link
                                vlink v_link;
                                v_link.node_j = vj;
                                v_link.node_k = vk;
                                v_link.w_jk = dist_jk;

                                // Add new virtual link
                                c_nodes[index_cnode].v_links.push_back(v_link);

                                // Make the logical adjacency matrix
                                A_l(vj,vk) = A_l(vk,vj) = dist_jk;
                            }
                        }
                    }
                }
            }

            // Remove physical links from virtual links array (case id is in virtual link)
            for(unsigned int i = 0; i < c_nodes.size(); i++) {
                if(verify_vlink(id,c_nodes[i].v_links,index_vlink) && (A_p(c_nodes[i].v_links[index_vlink].node_j,c_nodes[i].v_links[index_vlink].node_k) > 0 || A_p(c_nodes[i].v_links[index_vlink].node_k,c_nodes[i].v_links[index_vlink].node_j) > 0)) {
                    c_nodes[i].v_links.erase(c_nodes[i].v_links.begin()+index_vlink);

                    cout << "bot:" << id << " remove_vlink:" << c_nodes[i].v_links[index_vlink].node_j << " - " << c_nodes[i].v_links[index_vlink].node_k << endl;
                }
            }
        }
    }
}

// High level motion control algorithms
void robot::hl_motion_control(const unsigned int mode, const bool sec_c, const bool com_c, const float dt) {
    switch(mode) {
        // Motion control to topology control
        case CONNECTIVITY: {
            // Virtual neighbors
            uvec N_l = virtual_neighbors(find(A_l.col(id)));

            // Control function
            v(id,span()) = connectivity_control(id,A_l,N_l,x,x_r,v,dt,r_sec,r_com,sec_c,com_c,opt_type);

            // Low level control
            ll_motion_control(dt);
        } break;

        // Motion control to rendezvous problem
        case RENDEZVOUS: {
            // Control function
            v(id,span()) = rendezvous(id,A_l,x,x_r,v,dt,r_sec,r_com,sec_c,com_c);

            // Low level control
            ll_motion_control(dt);
        }
    }
}

// Low level motion control algorithms
void robot::ll_motion_control(const float dt) {
    // Omnidirectional control
    if(bot_type == 0) {
        // Effective velocities are equal to high-level velocities
        vel.x = v(id,0);
        vel.y = v(id,1);

        // New positions for x,y
        x(id,span()) = x(id,span()) + v(id,span()) * dt;

    // Differential control
    } else {
        // Control signals and estimated angle
        float fx = 0, fa = 0, ar;

        // Control gains
        const float fa_gain = 0.1;
        const float fx_gain = 1;

        // Calculate the estimated position
        const rowvec x1 = x(id,span()) + v(id,span()) * dt;

        // Determine the distance between actual point and estimated point
        const float dx = x1(0) - x(id,0);     // Adjacent
        const float dy = x1(1) - x(id,1);     // Opositte

        // Actual angle
        const float a0 = pose.yaw;

        // Determine the estimated angle to X inertial axis
        fabs(dy) > MAX_ERR || fabs(dx) > MAX_ERR ? ar = atan2(dy,dx) : ar = a0;

        // Difference between actual angle and the estimated angle
        float da = ar - a0;

        // Determine the arc direction according with the quatrand of angles
        switch(quadrant(a0)) {
            case 1:
                if(quadrant(ar) == 3 && fabs(da) > PI)
                    da = (2 * PI - fabs(ar)) - a0;
                break;
            case 2:
                if(quadrant(ar) == 3 || (quadrant(ar) == 4 && fabs(da) > PI))
                    da = (2 * PI - fabs(ar)) - a0;
                break;
            case 3:
                if(quadrant(ar) == 2 || (quadrant(ar) == 1 && fabs(da) > PI))
                    da = ar - (2 * PI - fabs(a0));
                break;
            case 4:
                if(quadrant(ar) == 2 && fabs(da) > PI)
                    da = ar - (2 * PI - fabs(a0));
        }

        const float da_min = 0.01;
        const float da_max = 0.35;
        const float va_max = 1;
        const float dx_min = 0.1 * dt;
        const float vx_max = 1 / dt;
        const float dx_max = vx_max * dt;

        float kx, ka;

        if(fabs(da) >= da_min && fabs(da) <= da_max) {
            ka = da / da_max;
        } else if(fabs(da) > da_max && da > 0) {
            ka = 1;
        } else if(fabs(da) > da_max && da < 0) {
            ka = -1;
        } else if(fabs(da) < da_min) {
            ka = 0;
        }

        if(fabs(dx) >= dx_min && fabs(dx) <= dx_max) {
            kx = sqrt(pow(dx,2) + pow(dy,2));
        } else if(fabs(dx) < dx_min) {
            kx = 0;
        } else if(fabs(dx) > dx_max) {
            kx = 1;
        }

        // Calculate the angular action control
        //fabs(da) < MIN_ERR ? fa = 0 : fa = (da / dt) * fa_gain;

        cout << "ka:" << ka << " kx:" << kx << endl;

        fa = ka * va_max;
        fx = kx * vx_max * (1 - fabs(ka));

        // Calculate the linear action control
        //fabs(da) > 1 ? fx = 0 : fx = (fabs(dx) / dt) * fx_gain;

        // Set the effective velocities
        vel.x = fx;
        vel.yaw = fa;

        // Show control information
        //cout << "bot:" << id << " a0:" << (a0*180/PI) << " ar:" << (ar*180/PI)
            //<< " da:" << (da*180/PI) << " fa:" << fa << endl;
        //cout << " x0:" << x(id,0) << " xr:" << x1(0)
            //<< " dx:" << dx << " fx:" << fx << endl;
        //cout << " y0:" << x(id,1) << " yr:" << x1(1)
            //<< " dy:" << dy << endl;
    }
}

// Message counter
void robot::msg_count(int id_j) {
    if(msg_num.n_rows < n_bots) msg_num.resize(n_bots,HIST_SIZE);
    if(time_index.n_elem < n_bots) time_index.resize(n_bots);

    msg_num(id_j,time_index(id_j)) += 1;
}

// Message history update
void robot::msg_update() {
    for(unsigned int j = 0; j < n_bots; j++) {
        time_index(j) += 1;
        if(time_index(j) < msg_num.n_cols && time_index(j) != 0) {
            msg_num(j,time_index(j)) = msg_num(j,time_index(j)-1);
        } else if(time_index(j) == msg_num.n_cols) {
            time_index(j) = 0;
            msg_num(j,time_index(j)) = msg_num(j,msg_num.n_cols-1);
        }
    }
}

// Find virtual neighbors
uvec robot::virtual_neighbors(uvec N_p) {
    uvec N_l;
    rowvec x_j, x_i = x(id,span());

    for(unsigned int j = 0; j < N_p.n_elem; j++) {
        x_j = x(N_p(j),span());
        if(dist(x_i,x_j) > r_com(id)) {
            N_l.resize(N_l.n_elem+1);
            N_l(N_l.n_elem-1) = N_p(j);
        }
    }

    return N_l;
}

// Make the id robot as critical node
void robot::make_cnode(unsigned int id, vlink v_link) {
    int index_vlink, index_cnode;

    if(!verify_vlink(id,v_link,index_vlink,index_cnode)) {
        if(!verify_cnode(id,index_cnode)) {
            cnode temp_cnode;
            temp_cnode.c_node = id;
            temp_cnode.v_links.push_back(v_link);
            c_nodes.push_back(temp_cnode);
        } else {
            c_nodes[index_cnode].v_links.push_back(v_link);
        }
    }
}

// Verify if the robot id is critical node and return its index
bool robot::verify_cnode(unsigned int id, int &index_cnode) {
    index_cnode = -1;

    for(unsigned int i = 0; i < c_nodes.size(); i++) {
        if(c_nodes[i].c_node == id) {
            index_cnode = i;
            return true;
        }
    }
    return false;
}

// Verify if the virtual link is assigned to id
bool robot::verify_vlink(unsigned int id, vlink v_link, int &index_vlink, int &index_cnode) {

    if(verify_cnode(id,index_cnode)) {
        for(unsigned int i = 0; i < c_nodes[index_cnode].v_links.size(); i++) {
            if((c_nodes[index_cnode].v_links[i].node_j == v_link.node_j
             || c_nodes[index_cnode].v_links[i].node_j == v_link.node_k)
             && (c_nodes[index_cnode].v_links[i].node_k == v_link.node_k
             || c_nodes[index_cnode].v_links[i].node_k == v_link.node_j)) {
                index_vlink = i;
                return true;
             }

        }
    }

    return false;
}

// Verify if id is a virtual link part
bool robot::verify_vlink(unsigned int id, std::vector<vlink> v_links, int &index_vlink) {
    for(unsigned int j = 0; j < v_links.size(); j++) {
        if(v_links[j].node_j == id || v_links[j].node_k == id) {
            index_vlink = j;
            return true;
        }
    }

    return false;
}

