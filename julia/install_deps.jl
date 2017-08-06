# Install julia dependencies and extra tools
Pkg.add("MAT")                           # matlab files support
Pkg.add("Gurobi"); Pkg.build("Gurobi")   # optimization solver
Pkg.add("HDF5"); Pkg.build("HDF5")       # hdf5 matlab binary file
Pkg.add("Convex")                        # modeling of convex problems
Pkg.add("JuMP")                          # modeling of general optimization problems
Pkg.update()                             # update all installed packages

