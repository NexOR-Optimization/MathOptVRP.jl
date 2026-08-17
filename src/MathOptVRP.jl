"""
    MathOptVRP

Solver-agnostic MOI sets and JuMP helpers for vehicle-routing problems:

  * sets — `Permutation`, `Partition`, `PartitionPD`, `TimeWindows`,
    `CapacitatedTimeWindows`, `Capacity`
  * operator — `sum_distances` / `op_sum_distances` for closed-tour cost
    objectives

A solver wrapper (e.g. `Hexaly.Optimizer`) implements
`MOI.add_constrained_variables(model, ::Partition)` and
`MOI.add_constraint(model, f, ::TimeWindows)` to give these sets their
operational meaning; this package only defines the shapes, dimensions,
JuMP `build_variable` overloads and the `:sum_distances` nonlinear
operator.
"""
module MathOptVRP

import MathOptInterface as MOI
import JuMP

include("sets.jl")
include("bridges.jl")
include("operators.jl")
include("Tests.jl")

# Implemented in the `MathOptVRPORToolsExt` package extension (loaded when
# `ORTools` is in scope). Returns a fresh `MOI.AbstractOptimizer` backed by
# OR-Tools' CP-SAT routes constraint. Pass `MathOptVRP.ortools_optimizer`
# as the JuMP optimizer factory.
function ortools_optimizer end

end # module
