# Install julia dependencies and extra tools
Pkg.add("MAT")          # matlab files support
Pkg.build("Gurobi")     # optimization solver
Pkg.add("Convex")       # modeling of convex problems
Pkg.add("JuMP")         # modeling of general optimization problems

