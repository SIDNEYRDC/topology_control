###############################################################################
# Random Geometric Graph Generator
# Written by SIDNEY RDC using NetworkX library.
# Last Change: 2015 Mar 24 17:34:31
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

    f.write('\n[simulation-configuration]\n')
    f.write('# Number of iterations\n')
    f.write('n_iter: 100\n')

    f.write('\n# Data capture frequency\n')
    f.write('dc: 1\n')

    f.write('\n# Information scope')
    f.write('\n# 0=local 1=global\n')
    f.write('info_scope: 0\n')

    f.write('\n# Topology control type')
    f.write('\n# 0=off 1=mst 2=tsp 3=vlink\n')
    f.write('opt_type: 0\n')

    f.write('\n# Enable reference pass')
    f.write('\n# 0=off 1=on\n')
    f.write('ref_pass: 0\n')

    f.write('\n# Enable communication radius constraint')
    f.write('\n# 0=off 1=on\n')
    f.write('com_c: 0\n')

    f.write('\n# Enable security radius constraint')
    f.write('\n# 0=off 1=on\n')
    f.write('sec_c: 0\n')

    f.write('\n# Enable ROS interface')
    f.write('\n# 0=off 1=on\n')
    f.write('ros: 0\n')

    # Headers for position
    f.write('\n[robot-positions]\n')
    f.write('# bot: x y\n')

    # Write positions on archive file
    for n in pos:
        f.write('%d: %f %f\n' % (n,pos[n][0]*scale,pos[n][1]*scale))

    # Headers for velocity
    f.write('\n[robot-velocities]\n')
    f.write('# bot: vx vy\n')

    # Headers for radius
    f.write('\n[robot-ranges]\n')
    f.write('# bot: R r\n')

    # Write radius on archive file
    for n in xrange(0, args.nodes):
        f.write('%d: %.1f 0.0\n' % (n,args.radius*scale))

    # Headers to external robots
    f.write('\n[external-robots]\n')
    f.write('# bot: type \'name\'\n')
    f.write('# 1=real 2=stage 3=turtle\n')

    # Headers to reference
    f.write('\n[reference]\n')
    f.write('# bot: x y\n')

    # Headers to timeout configuration
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

