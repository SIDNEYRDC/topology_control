#==============================================================================
 = Optimization Module
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2016 Jun 04 16:07:51
 = Info: This file contains the optimization module that is used to solve some
 = problems using mathematic programming and optimization algorithms.
 =============================================================================#

module OPT

using JuMP      # AMPL interface to julia
using Gurobi    # to use Gurobi as optimization solver

# public functions
export tsp

#
#=
 = TSP function
 = Info: Traveling Salesman Problem Algorithm solved by Miler et al. (1960)
 = using integer linear programming. The original formulation can be found
 = in: doi.org/10.1145/321043.321046
 = Use: H = tsp(W), where W is a weighted matrix and H is the TSP solution
 =
 = Formulation:
 =
 = min     ∑_i ∑_j w_ij ∙ e_ij
 = s.t.    ∑_i e_ij = 1    i ≠ j   j = 1,...,n
 =         ∑_j e_ij = 1    j ≠ i   i = 1,...,n
 =         u_i - u_j + n ∙ e_ij ≤ n - 1    2 ≤ i ≠ j ≤ n
 =         e_ij > 0    j = 1,...,n
 =#

function tsp(W)
    # gets the input matrix size
    const n = size(W, 1)

    # check if the input matrix is square
    if n != size(W, 2)
        println("ERROR: tsp function operates only with square matrices!")
        return W
    end

    # define optimization model
    m = JuMP.Model(solver = GurobiSolver(OutputFlag = 0))

    # define edges matrix
    @variable(m, e[1 : n, 1 : n] >= 0, Int)

    # define extra variables
    @variable(m, u[1 : n] >= 0, Int)

    # define objective function
    @objective(m, Min, sum(e.*W))

    # define flow constraints (∑_j e_ij = 1   j ≠ i   i = 1,...,n)
    @constraint(m, (e.*(ones(n, n) - eye(n)))*ones(n) .== ones(n))

    # define flow constraints (∑_i e_ij = 1   i ≠ j   j = 1,...,n)
    @constraint(m, (e.*(ones(n, n) - eye(n)))'*ones(n) .== ones(n))

    # define sub-tour constraints (u_i - u_j + n ∙ e_ij ≤ n - 1   2 ≤ i ≠ j ≤ n)
    @constraint(m, xyconstr[i = 2 : n, j = 2 : n, i != j], u[i] - u[j] + n*e[i, j] <= n - 1)

    # ensure that there is no value in the main diagonal
    @constraint(m, xyconstr[i = 1 : n], e[i, i] == 0)

    # solve the problem
    status = JuMP.solve(m)

    # check if the optimal solution was reached
    if status == :Optimal
        return convert(Array{UInt8, 2}, JuMP.getvalue(e) + JuMP.getvalue(e)')
    else
        return zeros(UInt8, n, n)
    end
end

end

