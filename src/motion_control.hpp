/**********************************************************************
*   Motion control algorithms
*   Written by Sidney RDC, 2014.
*   Last Change: 2015 Abr 08 15:52:04
***********************************************************************/

#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include <armadillo>
#include <QuadProg++.hh>
#include "utils.hpp"
#include "defines.hpp"

// High level motion control algorithms
#define POTENTIAL 0
#define CONNECTIVITY 1
#define RENDEZVOUS 2

// Ordoñez algorithm to rendezvous
arma::rowvec rendezvous(unsigned int id, arma::mat A, arma::mat x, arma::rowvec x_r, arma::mat v, float dt, arma::vec r_sec, arma::vec r_com, bool c_sec, bool c_com);

// Control with potential fields
arma::vec potential_fields(int id, arma::mat A, arma::mat x, arma::mat v, float r_max, float r_min);

// Connectivity control for topology control algorithms
arma::rowvec connectivity_control(unsigned int id, arma::mat A_l, arma::uvec N_l, arma::mat x, arma::rowvec x_r, arma::mat v, float dt, arma::vec r_sec, arma::vec r_com, bool c_sec, bool c_com, unsigned int opt_type);

#endif

