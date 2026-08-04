# Solver-agnostic JuMP nonlinear operator for closed-tour distance.
# A solver wrapper recognises `MOI.ScalarNonlinearFunction` with head
# `:sum_distances` and lowers it into its own modelling primitives.

"""
    sum_distances(dist_matrix, nodes)

Build a JuMP nonlinear expression representing the closed-tour cost over
`nodes` using `dist_matrix` as the edge weights. Has no scalar fallback
method — solvers that consume `:sum_distances` provide the lowering.
"""
function sum_distances end

"""
    op_sum_distances

`JuMP.NonlinearOperator` wrapping [`sum_distances`](@ref). When called
with at least one JuMP-tainted argument it constructs a
`GenericNonlinearExpr` with head `:sum_distances`.
"""
const op_sum_distances = JuMP.NonlinearOperator(sum_distances, :sum_distances)
