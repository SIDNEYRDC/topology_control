/**********************************************************************
*   Tools for access and process file of the type .mat and .conf
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Mai 27 17:57:15
***********************************************************************/

#include "file_utils.hpp"

using namespace arma;
using namespace std;

// Get data from a input filestream
rowvec get_data(const string search_key, ifstream &file_input) {
    unsigned int pose0, pose1;
    rowvec out = zeros<rowvec>(1);
    string line = "";

    // Find the search key
    while(!contains(line,search_key) && !file_input.eof()) {
        getline(file_input, line);
    }

    // If search key was fouded
    if(contains(line,search_key,pose0,pose1)) {
        // Get all number on the line
        vector<float> num = get_float(line.substr(pose1));

        // Set size of armadillo array
        out.resize(num.size());

        // Store on armadillo array
        for(unsigned int i = 0; i < num.size(); i++) {
            out(i) = num[i];
        }
    }

    // Reset file_input
    if(file_input.eof()) {
        file_input.clear();
        file_input.seekg(0,ios::beg);
    }

    return out;
}

// From data file stream, get a external configurations
vector<extbot> get_external_info(ifstream &file_input) {
    vector<extbot> out;
    unsigned int n_bot = 0, init_pose, end_pose;
    const string search_key = "[external-robots]";
    string line = "";

    // Find the search key
    while(!contains(line,search_key) && !file_input.eof()) {
        getline(file_input,line);
    }

    // If search key was founded
    if(contains(line,search_key)) {
        stringstream bot_key;

        // Get new line
        getline(file_input, line);

        while(!contains(line,"[") && !file_input.eof()) {
            // Generate the robot search key
            bot_key.str("");
            bot_key << n_bot << ":";

            // Verity if the robot search key is present on line
            if(contains(line,bot_key.str(),init_pose,end_pose) && init_pose == 0) {
                // Set size array
                out.resize(n_bot+1);

                // Store on array
                out[n_bot].id = n_bot;
                out[n_bot].type = get_float(line.substr(end_pose))[0];

                // Store name in array
                if(contains(line,"\'",init_pose,end_pose)) {
                    string temp = line.substr(end_pose);
                    if(contains(temp,"\'",init_pose,end_pose)) {
                        out[n_bot].name = temp.substr(0,init_pose);
                    } else {
                        cout << "INPUT ERROR: 'name' expected!\n";
                        exit(1);
                    }
                }

                // Increment the robot number
                n_bot++;

                // Read new line
                getline(file_input,line);

            // When the robot dont have data
            } else if(line[0] > 47 && line[0] < 58) {
                // Set size of array
                out.resize(n_bot+1);

                // Set default values
                out[n_bot].id = n_bot;
                out[n_bot].type = 0;
                out[n_bot].name = "";

                // Increment the robot number
                n_bot++;
            } else {
                // Read new line
                getline(file_input,line);
            }
        }
    }

    // Reset file_input
    if(file_input.eof()) {
        file_input.clear();
        file_input.seekg(0,ios::beg);

    // Return to previous line
    } else file_input.seekg(-line.size()-1,ios::cur);

    return out;
}

// From data file stream, fill a armadillo array
mat fill_array(const string search_key, ifstream &file_input) {
    mat out;
    unsigned int n_bot = 0, init_pose, end_pose;
    string line = "";

    // Find the search key
    while(!contains(line,search_key) && !file_input.eof()) {
        getline(file_input,line);
    }

    // If search key was founded
    if(contains(line,search_key)) {
        stringstream bot_key;

        // Get new line
        getline(file_input, line);

        while(!contains(line,"[") && !file_input.eof()) {
            // Generate the robot search key
            bot_key.str("");
            bot_key << n_bot << ":";

            // Verity if the robot search key is present on line
            if(contains(line,bot_key.str(),init_pose,end_pose) && init_pose == 0) {
                // Get all number on the line
                vector<float> num = get_float(line.substr(end_pose));

                // Set size of armadillo array
                out.resize(n_bot+1,num.size());

                // Store on armadillo array
                for(unsigned int i = 0; i < num.size(); i++) {
                    out(n_bot,i) = num[i];
                }

                // Increment the robot number
                n_bot++;

                // Read new line
                getline(file_input,line);

            // When the robot dont have data
            } else if(line[0] > 47 && line[0] < 58) {
                // Set size of armadillo
                out.is_empty() ? out.resize(n_bot+1,1) : out.resize(n_bot+1,out.n_cols);

                // Store BIG_VAL
                out(n_bot,0) = BIG_VAL;

                // Increment the robot number
                n_bot++;
            } else {
                // Read new line
                getline(file_input,line);
            }
        }
    }

    // Reset file_input
    if(file_input.eof()) {
        file_input.clear();
        file_input.seekg(0,ios::beg);

    // Return to previous line
    } else file_input.seekg(-line.size()-1,ios::cur);

    return out;
}

// Read configuration file and store its data
topology read_topology(string config_file) {
    ifstream file_input;
    topology topology_data;

    const string sim_config = "[simulation-configuration]";
    const string robot_pose = "[robot-positions]";
    const string robot_vel = "[robot-velocities]";
    const string robot_ranges = "[robot-ranges]";
    const string reference = "[reference]";
    const string timeout = "[timeout]";
    const string external_robots = "[external-robots]";

    file_input.open(config_file.c_str());

    if(file_input.is_open()) {
        // Get the simulation configurations
        topology_data.n_iter = get_data("n_iter:",file_input).at(0);
        topology_data.dc = get_data("dc:",file_input).at(0);
        topology_data.dt = get_data("dt:",file_input).at(0);
        topology_data.vx_max = get_data("vx_max:",file_input).at(0);
        topology_data.vy_max = get_data("vy_max:",file_input).at(0);
        topology_data.va_max = get_data("va_max:",file_input).at(0);
        topology_data.vx_min = get_data("vx_min:",file_input).at(0);
        topology_data.vy_min = get_data("vy_min:",file_input).at(0);
        topology_data.va_min = get_data("va_min:",file_input).at(0);
        topology_data.info_scope = get_data("info_scope:",file_input).at(0);
        topology_data.opt_type = get_data("opt_type:",file_input).at(0);
        topology_data.ref_pass = get_data("ref_pass:",file_input).at(0);
        topology_data.com_c = get_data("com_c:",file_input).at(0);
        topology_data.sec_c = get_data("sec_c:",file_input).at(0);
        topology_data.ros = get_data("ros:",file_input).at(0);

        // Store the initial positions and velocities
        topology_data.init_x = fill_array(robot_pose,file_input);
        topology_data.init_v = fill_array(robot_vel,file_input);

        // Store the number of robots
        topology_data.n_bots = topology_data.init_x.n_rows;

        // Store the ranges of communication and security
        mat out = fill_array(robot_ranges,file_input);
        topology_data.r_com = out.col(0);
        topology_data.r_sec = out.col(1);

        // Store the external configurations
        topology_data.ext_info = get_external_info(file_input);

        // Store the reference
        topology_data.x_ref = fill_array(reference,file_input);

        // Store the timeout
        topology_data.timeout = fill_array(timeout,file_input);
    }

    file_input.close();

    // Verify topology file error
    check_input_error(topology_data);

    return topology_data;
}

// Store data in a matlab file
void write_matfile(cube A_data, mat x_data, mat v_data, mat rmax_data, mat rmin_data, mat ref_data, int N_iter) {
    MatFile matlab_file;

    const unsigned int n = x_data.n_cols / 2;

    // File name with 'sim-n-N.mat' {n::number of agents N::interactions}
    std::stringstream sstm;
    sstm << "../matlab/sim-" << n << "-" << N_iter << ".mat";
    string filename = sstm.str();

    // Open new matlab file
    matlab_file.Open(filename);

    // Data transfer for armadillo arrays
    double x_parse[2*n][x_data.n_rows];
    double v_parse[2*n][v_data.n_rows];
    double A_parse[A_data.n_slices][n][n];
    double rmax_parse[n][rmax_data.n_rows];
    double rmin_parse[n][rmin_data.n_rows];
    double ref_parse[2][ref_data.n_rows];

    // Transfering ranges data
    for(unsigned int t = 0; t < rmax_data.n_rows; t++) {
        for(unsigned int i = 0; i < n; i++) {
            rmax_parse[i][t] = rmax_data(t,i);
            rmin_parse[i][t] = rmin_data(t,i);
        }
    }

    // Reference positions transfer
    for(unsigned int i = 0; i < ref_data.n_rows; i++) {
        ref_parse[0][i] = ref_data(i,0);
        ref_parse[1][i] = ref_data(i,1);
    }

    // Transfering positions and velocities arrays
    for(unsigned int i = 0; i < x_data.n_rows; i++) {
        for(unsigned int j = 0; j < 2 * n; j++) {
            x_parse[j][i] = x_data(i,j);
            v_parse[j][i] = v_data(i,j);
        }
    }

    // Transfering 3d arrays
    for(unsigned int i = 0;i < n; i++) {
        for(unsigned int j = 0; j < n; j++) {
            for(unsigned int k = 0; k < A_data.n_slices; k++) {
                A_parse[k][j][i] = A_data(i,j,k);
            }
        }
    }

    // Data write in .mat file
    matlab_file.WriteMatrix("x_plot",2*n,x_data.n_rows,*x_parse);
    matlab_file.WriteMatrix("v_plot",2*n,v_data.n_rows,*v_parse);
    matlab_file.WriteCube("A_plot",A_data.n_slices,n,n,**A_parse);
    matlab_file.WriteMatrix("ref",2,ref_data.n_rows,*ref_parse);
    matlab_file.WriteMatrix("r_max",n,rmax_data.n_rows,*rmax_parse);
    matlab_file.WriteMatrix("r_min",n,rmin_data.n_rows,*rmin_parse);

    // Close .mat file
    if(matlab_file.IsOpen()) matlab_file.Close();
}

// Detect and report error in the input file
void check_input_error(topology input) {
    unsigned char error_code = 0;

    // Identify the error code
    if(input.n_iter == 0) error_code = 1;
    else if(input.dc >= input.n_iter) error_code = 2;
    else if(input.opt_type > 3) error_code = 3;
    else if(input.ref_pass > 1) error_code = 4;
    else if(input.r_com.is_empty()) error_code = 5;
    else if(input.r_sec.is_empty()) error_code = 6;
    else if(input.init_x.is_empty()) error_code = 7;
    else if(input.dt <= 0) error_code = 8;

    // Select the error report
    switch(error_code) {
        case 1:
            cout << "INPUT ERROR: Number of iteractions must be greater than 0!\n";
            exit(1);
        case 2:
            cout << "INPUT ERROR: Data capture option must be lower than number of iteractions!\n";
            exit(1);
        case 3:
            cout << "INPUT ERROR: Topology control type must be between 0 and 3!\n";
            exit(1);
        case 4:
            cout << "INPUT ERROR: Reference passage option must be between 0 and 1!\n";
            exit(1);
        case 5:
            cout << "INPUT ERROR: Communication radius can not be empty!\n";
            exit(1);
        case 6:
            cout << "INPUT ERROR: Security radius can not be empty!\n";
            exit(1);
        case 7:
            cout << "INPUT ERROR: Positions for robots can not be empty!\n";
            exit(1);
        case 8:
            cout << "INPUT ERROR: Simulation step must be greater than 0!\n";
            exit(1);
    }
}

