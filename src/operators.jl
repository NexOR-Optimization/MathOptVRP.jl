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

# Below are type piracy that can be removed once
# https://github.com/jump-dev/JuMP.jl/pull/3451 is merged. They let
# JuMP accept arrays as leaves of `GenericNonlinearExpr` (needed for
# `op_sum_distances(dist_matrix, nodes)`), broadcast `moi_function` over
# arrays, and pick a variable-ref type for arrays of JuMP scalars (so
# `NonlinearOperator` dispatches to expression-building rather than to
# calling the stub scalar function).

JuMP._is_real(::Array{<:Real}) = true

JuMP._is_real(::Array{<:JuMP.AbstractJuMPScalar}) = true

JuMP.moi_function(x::Array) = JuMP.moi_function.(x)

function JuMP.variable_ref_type(
    ::Type{JuMP.GenericNonlinearExpr},
    ::AbstractArray{T},
) where {T<:JuMP.AbstractJuMPScalar}
    return JuMP.variable_ref_type(T)
end
