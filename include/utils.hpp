/**********************************************************************
*   Tools for default C\C++ Programming
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Abr 05 20:19:08
***********************************************************************/

#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <cmath>
#include <armadillo>
#include <QuadProg++.hh>
#include <time.h>
#include "defines.hpp"

// Triangular matrix type
#define UPPER 0
#define LOWER 1

// Return the maximum between two floats
float max_float(float x, float y);

// Show a matrix of QuadProg type
void show_matrix(const QuadProgPP::Matrix<double> mat);

// Show a vector of QuadProg type
void show_vector(const QuadProgPP::Vector<double> vec);

// Return the distance between two vectors containing x and y coordinates
float dist(arma::rowvec a, arma::rowvec b);

// Compute the time between two times
double calc_time(clock_t t0, clock_t t1);

// Fill a std::vector with arma::rowvec elements
void fill_vec(std::vector<int> &vec, arma::urowvec elems);

// Find a value in a vector and return its position
bool find_value(int value, std::vector<int> vec, unsigned int &pose);

// Find a value in a vector
bool find_value(int value, std::vector<int> vec);

// Search a character in a string and return this position
bool find_char(std::string input, char search, unsigned int &pos);

// Search a character in a string
bool find_char(std::string input, char search);

// Verify if a string input contains a substring search and return the start pose and end pose of this
bool contains(std::string input, std::string search, unsigned int &init_pose, unsigned int &end_pose);

// Verify if a string input contains a substring search
bool contains(std::string input, std::string search);

// Verify if a unsigned int array contain a element
bool contains(unsigned int elem, arma::uvec array);

// Verify if a float array contain a element
bool contains(float elem, arma::vec array);

// Extract all float numbers on a string
std::vector<float> get_float(std::string input);

// Determine the quadrant of a input angle
short quadrant(float angle);

/////////////////////////////////////////////////////////////////////////////
// TEMPLATE FUNCTIONS
/////////////////////////////////////////////////////////////////////////////

// Verify if a std::vector contains a element of type T
template <typename T> bool contains(const T elem, std::vector<T> array) {
    for(unsigned int i = 0; i < array.size(); i++) {
        if(array[i] == elem) return true;
    }

    return false;
}

// Verify if a std::vector contains a element of type T (overload to return the position)
template <typename T> bool contains(const T elem, unsigned int &pose, std::vector<T> array) {
    for(unsigned int i = 0; i < array.size(); i++) {
        if(array[i] == elem) {
            pose = i;
            return true;
        }
    }

    return false;
}

// Return the maximum between two elements of type T
template <typename T> T max(const T e1, const T e2) {
    if(e2 > e1) return e2;
    return e1;
}

// Return the mininum between two elements of type T
template <typename T> T min(const T e1, const T e2) {
    if(e2 < e1) return e2;
    return e1;
}

// Allocate 2D arrays
template <class T> T **create2D(int rows, int cols) {
    T **array2D = new T *[rows];

    for(unsigned int i = 0; i < rows; i++) {
        array2D[i] = new T [cols];
    }

    return array2D;
}

// Generate a square diagonal matrix to quadprog type
template <typename T> QuadProgPP::Matrix<T> trimat(const T elem, const unsigned int ord, const unsigned int mode) {
    QuadProgPP::Matrix<T> out(ord,ord);
    unsigned int i = 0, j = 0;

    out = 0;

    switch(mode) {
        case UPPER:
            for(i = 0; i < ord; i++) {
                for(j = i; j < ord; j++) {
                    out[i][j] = elem;
                }
            }
            break;
        case LOWER:
            for(i = 0; i < ord; i++) {
                for(j = i; j < ord; j++) {
                    out[j][i] = elem;
                }
            }
            break;
        default:
            std::cout << "ERROR: choose mode between UPPER or LOWER!" << std::endl;
            break;
    }

    return out;
}

// Generate a square identity matrix to quadprog type
template <typename T> QuadProgPP::Matrix<T> ident(const unsigned int ord) {
    QuadProgPP::Matrix<T> out(ord,ord);

    out = 0;

    for(unsigned int i = 0; i < ord; i++) {
        out[i][i] = 1;
    }

    return out;
}

// Resize a quadprog matrix type
template <typename T> void resize(QuadProgPP::Matrix<T> &in, const unsigned int n, const unsigned int m) {
    QuadProgPP::Matrix<T> out(n,m);

    out = 0;

    for(unsigned int i = 0; i < in.nrows(); i++) {
        for(unsigned int j = 0; j < in.ncols(); j++) {
            out[i][j] = in[i][j];
        }
    }

    in = out;
}

// Overload of resize function to vector type
template <typename T> void resize(QuadProgPP::Vector<T> &in, const unsigned int n) {
    QuadProgPP::Vector<T> out(n);

    out = 0;

    for(unsigned int i = 0; i < in.size(); i++) {
        out[i] = in[i];
    }

    in = out;
}

// Return the euclidian norm of a vector type T
template <class T> float norm_2(const T in, const unsigned int in_size) {
    float sum = 0;

    for(unsigned int i = 0; i < in_size; i++) {
        sum += pow(in[i],2);
    }

    return sqrt(sum);
}

#endif

