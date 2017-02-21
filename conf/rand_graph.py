#!/usr/bin/python
# -*- coding: utf-8 -*-

###############################################################################
# Random Geometric Graph Generator
# Written by SIDNEY RDC using NetworkX library.
# Last Change: 2017 Feb 21 17:39:10
###############################################################################

import argparse
import numpy as np
from scipy import spatial
import networkx as nx
import matplotlib.pyplot as plt
import math, random, sys

# generate a random geometric graph in a determined area
def random_geometric_graph(n, radius, size=1, ne=None, dim=2, pos=None):
    G=nx.Graph()
    G.name="Random Geometric Graph"
    G.add_nodes_from(range(n))
    if pos is None:
        # random positions
        for n in G:
            G.node[n]['pos']=[random.random() * size for i in range(0,dim)]
    else:
        nx.set_node_attributes(G,'pos',pos)
    # connect nodes within "radius" of each other
    # n^2 algorithm, could use a k-d tree implementation
    nodes = G.nodes(data=True)
    e=0
    while nodes:
        u,du = nodes.pop()
        pu = du['pos']
        for v,dv in nodes:
            pv = dv['pos']
            d = sum(((a-b)**2 for a,b in zip(pu,pv)))
            if d <= radius**2:
                G.add_edge(u,v)
                e = e + 1

            if e == ne:
                break

        if e == ne:
            break
    return G

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='rand_graph.py - Random Geometric Graph Generator')
    parser.add_argument('-n', '--nodes', dest='nodes', type=int, required=True, help='Number of nodes on the graph')
    parser.add_argument('-r', '--radius', dest='radius', type=float, required=True, help='Radius of communication')
    parser.add_argument('-s', '--size', dest='size', type=float, required=False, help='Size area of random generate')
    parser.add_argument('-e', '--edges', dest='edges', type=int, required=False, help='Number of edges on the graph')
    parser.add_argument('-f', '--factor', dest='scale', type=int, required=False, help='Scale factor of size')
    args = parser.parse_args()

    # Verify the scale factor
    if args.scale > 1:
        scale = args.scale
    else:
        scale = 1

    # Verify the size
    if args.size > 1:
        size = args.size
    else:
        size = 1

    # Verify the number of edges constraint
    if ((args.edges < args.nodes - 1) or (args.edges > (args.nodes * (args.nodes - 1) / 2))) and args.edges != None:
        print 'ERROR: The e must be: n-1 <= e <= n*(n-1)/2'
        exit(1)

    # Generate a connected graph
    print 'Generating a connected graph...'
    while True:
        G = random_geometric_graph(args.nodes,args.radius,size,args.edges)
        pos = nx.get_node_attributes(G,'pos')
        if nx.is_connected(G) == True:
            print 'is connected: %s' % (nx.is_connected(G))
            break

    # Configuration output file
    f = open('topology.conf','w')

    # Initial headers
    f.write('# Topology configuration file\n')

    f.write('\n###############################################################################')
    f.write('\n[simulation-setup]\n')
    f.write('# Number of iterations\n')
    f.write('n_iter: 100\n')

    f.write('\n# Number of robots\n')
    f.write('n_bot: %d\n' % args.nodes)

    f.write('\n# Number of references (targets)\n')
    f.write('n_ref: 0\n')

    f.write('\n# Data capture frequency (iterations)\n')
    f.write('dc: 1\n')

    f.write('\n# Enable ROS interface')
    f.write('\n# 0=off 1=on\n')
    f.write('ros: 0\n')

    f.write('\n# Information scope')
    f.write('\n# 0=local 1=global\n')
    f.write('info_scope: 0\n')

    f.write('\n###############################################################################')
    f.write('\n[control-setup]\n')
    f.write('# Integration step (s)\n')
    f.write('dt: 0.1\n')

    f.write('\n# Prediction horizon\n')
    f.write('ph: 5\n')

    f.write('\n# Weights to motion control\n')
    f.write('# gamma: γ₁ γ₂ γ₃ γ₄ γ₅ γ₆ γ₇\n')
    f.write('gamma: 1 1 1 1 1 0.1 0\n')

    f.write('\n# Lower bound to control signal\n')
    f.write('# u_min: xmin ymin θmin\n')
    f.write('u_min: -1 -1 -1\n')

    f.write('\n# Upper bound to control signal\n')
    f.write('# u_max: xmax ymax θmax\n')
    f.write('u_max: 1 1 1\n')

    f.write('\n# Enable communication radius constraint\n')
    f.write('# 0=off 1=on\n')
    f.write('c_com: 0\n')

    f.write('\n# Enable security radius constraint\n')
    f.write('# 0=off 1=on\n')
    f.write('c_sec: 0\n')

    f.write('\n###############################################################################')
    f.write('\n[network-setup]\n')
    f.write('# Gaussian white noise standard deviation (σ)\n')
    f.write('sigma: 0.25\n')

    f.write('\n# RSSI maximum attenuation factor (μ) (dBm)\n')
    f.write('# min=-90 max=0\n')
    f.write('mu: 0\n')

    f.write('\n# Path loss exponent (Φ)\n')
    f.write('# free space=2\n')
    f.write('# urban area=2.7 to 3.5\n')
    f.write('# suburban area=3 to 5\n')
    f.write('# indoor (line-of-sight)=1.6 to 1.8\n')
    f.write('phi: 2\n')

    f.write('\n# Use RSSI readings\n')
    f.write('# 0=off 1=on\n')
    f.write('rssi: 1\n')

    f.write('\n# RSSI noise\n')
    f.write('# 0=off 1=on\n')
    f.write('rssi_noise: 1\n')

    f.write('\n# RSSI threshold (dBm)\n')
    f.write('# min=-90 max=0\n')
    f.write('rssi_lim: -90\n')

    f.write('\n# Topology type\n')
    f.write('# 0=fixed 1=dynamic\n')
    f.write('top_type: 1\n')

    f.write('\n# Omnet++ interface\n')
    f.write('# 0=off 1=on\n')
    f.write('omnet: 0\n')

    # Headers for position
    f.write('\n###############################################################################')
    f.write('\n[robot-positions]\n')
    f.write('# bot: x y θ\n')

    # Write positions on archive file
    for n in pos:
        f.write('%d: %f %f 0\n' % (n+1,pos[n][0]*scale,pos[n][1]*scale))

    # Headers for velocity
    f.write('\n###############################################################################')
    f.write('\n[robot-velocities]\n')
    f.write('# bot: vx vy vθ\n')

    # Headers for radius
    f.write('\n###############################################################################')
    f.write('\n[robot-ranges]\n')
    f.write('# bot: rsec rcov rcom\n')

    # Write radius on archive file
    for n in xrange(0, args.nodes):
        f.write('%d: 0.0 %.1f %.1f\n' % (n+1,(args.radius*scale*0.9)/2,args.radius*scale))

    # Headers to external robots
    f.write('\n###############################################################################')
    f.write('\n[external-robots]\n')
    f.write('# bot: type \'name\'\n')
    f.write('# 1=real 2=stage 3=turtle\n')

    # Headers to reference
    f.write('\n###############################################################################')
    f.write('\n[reference]\n')
    f.write('# ref: x y θ\n')

    # Headers to timeout configuration
    f.write('\n###############################################################################')
    f.write('\n[timeout]\n')
    f.write('# bot: timeout (iter)\n')

    # Close file
    f.close()

    # Draw graph
    nx.draw(G,pos)

    # Draw labels
    nx.draw_networkx_labels(G,pos,font_size=10,font_family='sans-serif')

    # Draw nodes
    nx.draw_networkx_nodes(G,pos,node_color='#E0E0E0')

    # Show and save figure
    plt.savefig("graph.pdf")
    plt.show()

