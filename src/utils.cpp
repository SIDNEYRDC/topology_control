/**********************************************************************
*   Tools for default C\C++ Programming
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Mar 24 02:47:16
***********************************************************************/

#include "utils.hpp"

using namespace std;
using namespace arma;
using namespace QuadProgPP;

// Return the maximum between two floats
float max_float(float x, float y) {
    if(x > y) return x;
    return y;
}

// Show a matrix of QuadProg type
void show_matrix(const Matrix<double> mat) {
    const unsigned int n = mat.nrows();
    const unsigned int m = mat.ncols();
    for(unsigned int i=0; i<n; i++) {
        for(unsigned int j=0; j<m; j++) {
            if(j < m-1) cout << mat[i][j] << " ";
            else cout << mat[i][j];
        }
        cout << endl;
    }
    cout << endl;
}

// Show a vector of QuadProg type
void show_vector(const Vector<double> vec) {
    const unsigned int n = vec.size();
    for(unsigned int i=0; i<n; i++) {
        if(i < n-1) cout << vec[i] << " ";
        else cout << vec[i];
    }
    cout << endl;
}

// Return the distance between two vectors containing x and y coordinates
float dist(rowvec a, rowvec b) {
    return sqrt(pow(a(0) - b(0),2) + pow(a(1) - b(1),2));
}

// Compute the time between two times
double calc_time(clock_t t0, clock_t t1) {
    return ((double)(t1 - t0) / CLOCKS_PER_SEC);
}

// Fill a std::vector with arma::rowvec elements
void fill_vec(vector<int> &vec, urowvec elems) {
    unsigned int n = elems.n_elem;
    vec.resize(n);

    for(unsigned int i = 0; i < n; i++) {
        vec[i] = elems(i);
    }
}

// Find a value in a vector and return its position
bool find_value(int value, vector<int> vec, unsigned int &pose) {
    for(unsigned int i = 0; i < vec.size(); i++) {
        if(vec[i] == value) {
            pose = i;
            return true;
        }
    }

    return false;
}

// Find a value in a vector
bool find_value(int value, vector<int> vec) {
    for(unsigned int i = 0; i < vec.size(); i++) {
        if(vec[i] == value) return true;
    }

    return false;
}

// Search a character in a string
bool find_char(std::string input, char search, unsigned int& pos) {
    for(unsigned int i = 0; i < input.size(); i++) {
        if(input[i] == search) {
            pos = i;
            return true;
        }
    }

    return false;
}

// Search a character in a string
bool find_char(std::string input, char search) {
    for(unsigned int i = 0; i < input.size(); i++) {
        if(input[i] == search) {
            return true;
        }
    }

    return false;
}

// Verify if a string input contains a substring search and return the start pose and end pose of this
bool contains(std::string input, std::string search, unsigned int &init_pose, unsigned int &end_pose) {
    std::size_t found = input.find(search);

    if(found != std::string::npos) {
        init_pose = found;
        end_pose = init_pose + search.size();
        return true;
    }
    return false;
}

// Verify if a string input contains a substring search
bool contains(std::string input, std::string search) {
    std::size_t found = input.find(search);

    if(found != std::string::npos) return true;
    return false;
}

// Verify if a unsigned int array contain a element
bool contains(unsigned int elem, uvec array) {
    unsigned int n = array.n_elem;

    for(unsigned int i = 0; i < n; i++) {
        if(array(i) == elem) return true;
    }

    return false;
}

// Verify if a float array contain a element
bool contains(float elem, vec array) {
    unsigned int n = array.n_elem;

    for(unsigned int i = 0; i < n; i++) {
        if(array(i) == elem) return true;
    }

    return false;
}

// Extract all float numbers on a string
vector<float> get_float(std::string input) {
    vector<float> out;
    unsigned int n = 1;

    // Alias of size_t
    std::string::size_type sz;

    // While string dont end
    while(input.size() > 0) {
        out.resize(n);
        out[n-1] = std::stof(input,&sz);
        input = input.substr(sz);
        if(contains(input,"\'")) break;
        n++;
    }

    return out;
}

// Determine the quadrant of a input angle in rad
short quadrant(float angle) {
    // 1st quadrant
    if(angle >= 0 && angle < PI/2) return 1;

    // 2nd quadrant
    if(angle >= PI/2 && angle < PI) return 2;

    // 3rd quadrant
    if(angle < 0 && angle >= -PI/2) return 3;

    // 4th quadrant
    if(angle < -PI/2 && angle >= -PI) return 4;

    // Default value
    return 0;
}

