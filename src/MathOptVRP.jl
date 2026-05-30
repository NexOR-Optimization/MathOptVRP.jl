"""
    MathOptVRP

Solver-agnostic MOI sets and JuMP helpers for vehicle-routing problems:

  * sets — `List`, `Partition`, `PartitionPD`, `TimeWindows`,
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
include("operators.jl")

# Solver-independent test entry points. The implementations live in the
# `MathOptVRPTestExt` package extension and are loaded automatically when
# `using Test` is in scope. The signature of every `test_<variant>` is
# `test_<variant>(optimizer_factory; read_routes, kwargs...)` where
# `read_routes(model, nodes) -> Vector{Vector{Int}}` recovers each truck's
# 0-indexed visited-customer sequence from the solved model and the
# matrix of decision variables (this part is necessarily solver-specific).
function test_tsp end
function test_vrp end
function test_vrppd end
function test_vrptw end
function test_cvrp end
function test_cvrptw end
function runtests end

end # module
