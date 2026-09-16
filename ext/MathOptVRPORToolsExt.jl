module MathOptVRPORToolsExt

# Minimal MathOptVRP backend on top of OR-Tools' CP-SAT solver. We accept
# one `MathOptVRP.PartitionPD` set of variables (the most general route
# constructor; `MathOptVRP.Partition` and `MathOptVRP.Permutation` reach us
# bridged into it, see `MathOptVRP.Bridges`) and a
# `MOI.ScalarNonlinearFunction` objective built from
# `MathOptVRP.op_sum_distances` (one leaf per truck, optionally wrapped in
# `:+` nodes), then lower the problem to a CP-SAT `CpModelProto` with a
# `RoutesConstraintProto` and hand it to `ORTools.SolveCpModelWithParameters`,
# the CP-SAT C API. Pickup/delivery pairs (same vehicle + precedence) are not
# natively expressible in `RoutesConstraintProto`, so they are lowered to
# extra per-arc "rank"/"route id" propagation constraints, see
# `_build_cp_model`.

# ORTools has a `CPSATOptimizer` but that we could extend but it still seems to be WIP,
# e.g., no `optimize!` function and https://github.com/google/or-tools/pull/5219
# so we just roll our own for now.

# The OR-Tools binaries are not a dependency of this extension: `ORTools`
# picks them up from whichever of `ORTools_jll` or `ORToolsBinaries` the
# user imports.

# VRPTW is accepted as an alternative to the `:sum_distances` objective:
# a linear `sum(t)` objective over free `t[i]` variables plus one
# `MathOptVRP.TimeWindows` constraint per truck column. Time-window
# propagation has no native CP-SAT primitive (unlike `RoutesConstraintProto`
# itself), so it is lowered the same way pickup/delivery precedence is: one
# extra integer variable per customer (arrival time, and — only for the arc
# back to the depot — a "completion" value), tied together by linear
# constraints reified on each arc literal. See `_build_cp_model_time_windows`.

import MathOptInterface as MOI
import MathOptVRP
import ORTools

const Sat = ORTools.Sat
const PB = ORTools.PB

# Per-truck data parsed out of one `MathOptVRP.TimeWindows` constraint; see
# `MOI.add_constraint(::Optimizer, ::Union{MOI.VectorOfVariables,MOI.VectorAffineFunction}, ::MathOptVRP.TimeWindows)`.
# Per-truck data parsed out of one `MathOptVRP.TimeWindows` constraint;
# see `MOI.add_constraint(::Optimizer, ::MOI.VectorAffineFunction, ::MathOptVRP.TimeWindows)`.
struct _TimeWindowsEntry
    t_var::MOI.VariableIndex
    first_node::Int
    last_node::Int
    set::MathOptVRP.TimeWindows
end

mutable struct Optimizer <: MOI.AbstractOptimizer
    next_variable::Int
    next_constraint::Int
    variable_to_position::Dict{MOI.VariableIndex,Tuple{Int,Int}}
    partition::Union{Nothing,MathOptVRP.PartitionPD}
    objective_sense::MOI.OptimizationSense
    # `ScalarNonlinearFunction` is a `:sum_distances` objective (vrp)
    # `ScalarAffineFunction{Float64}` is a `sum(t)` objective
    # paired with `time_windows_by_column` (vrptw).
    objective_function::Union{
        Nothing,
        MOI.ScalarNonlinearFunction,
        MOI.ScalarAffineFunction{Float64},
    }
    # One `MathOptVRP.TimeWindows` constraint per truck column, keyed by
    # column; populated by `add_constraint`, consumed by `optimize!`.
    time_windows_by_column::Dict{Int,_TimeWindowsEntry}
    silent::Bool
    time_limit::Union{Nothing,Float64}
    solved::Bool
    routes::Vector{Vector{Int}}
    # Per-variable primal value populated after solve: `nodes[i, j]` holds
    # the `i`-th customer visited by truck `j`, or `0` if the route has
    # ended. Lets callers reconstruct routes via `JuMP.value` instead of
    # reaching into this struct.
    variable_values::Dict{MOI.VariableIndex,Int}
    objective_value::Int
    termination_status::MOI.TerminationStatusCode
    primal_status::MOI.ResultStatusCode
    raw_status::String

    function Optimizer()
        return new(
            0,
            0,
            Dict{MOI.VariableIndex,Tuple{Int,Int}}(),
            nothing,
            MOI.FEASIBILITY_SENSE,
            nothing,
            Dict{Int,_TimeWindowsEntry}(),
            false,
            nothing,
            false,
            Vector{Int}[],
            Dict{MOI.VariableIndex,Int}(),
            0,
            MOI.OPTIMIZE_NOT_CALLED,
            MOI.NO_SOLUTION,
            "",
        )
    end
end

MathOptVRP.ortools_optimizer() = Optimizer()

MOI.get(::Optimizer, ::MOI.SolverName) = "ORTools (CP-SAT)"

function MOI.is_empty(m::Optimizer)
    return m.partition === nothing &&
           m.objective_function === nothing &&
           m.objective_sense == MOI.FEASIBILITY_SENSE &&
           isempty(m.time_windows_by_column) &&
           !m.solved
end

function MOI.empty!(m::Optimizer)
    m.next_variable = 0
    m.next_constraint = 0
    empty!(m.variable_to_position)
    m.partition = nothing
    m.objective_sense = MOI.FEASIBILITY_SENSE
    m.objective_function = nothing
    empty!(m.time_windows_by_column)
    m.solved = false
    empty!(m.routes)
    empty!(m.variable_values)
    m.objective_value = 0
    m.termination_status = MOI.OPTIMIZE_NOT_CALLED
    m.primal_status = MOI.NO_SOLUTION
    m.raw_status = ""
    return
end

# Parameters

MOI.supports(::Optimizer, ::MOI.Silent) = true
MOI.get(m::Optimizer, ::MOI.Silent) = m.silent
function MOI.set(m::Optimizer, ::MOI.Silent, silent::Bool)
    m.silent = silent
    return
end

MOI.supports(::Optimizer, ::MOI.TimeLimitSec) = true
MOI.get(m::Optimizer, ::MOI.TimeLimitSec) = m.time_limit
MOI.set(m::Optimizer, ::MOI.TimeLimitSec, ::Nothing) = (m.time_limit = nothing; return)
function MOI.set(m::Optimizer, ::MOI.TimeLimitSec, v::Real)
    m.time_limit = Float64(v)
    return
end

# Objective

MOI.supports(::Optimizer, ::MOI.ObjectiveSense) = true
MOI.get(m::Optimizer, ::MOI.ObjectiveSense) = m.objective_sense
function MOI.set(m::Optimizer, ::MOI.ObjectiveSense, s::MOI.OptimizationSense)
    m.objective_sense = s
    return
end

function MOI.supports(::Optimizer, ::MOI.ObjectiveFunction{MOI.ScalarNonlinearFunction})
    return true
end

function MOI.supports(
    ::Optimizer,
    ::MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}},
)
    return true
end

function MOI.get(m::Optimizer, ::MOI.ObjectiveFunctionType)
    m.objective_function isa MOI.ScalarAffineFunction{Float64} &&
        return MOI.ScalarAffineFunction{Float64}
    return MOI.ScalarNonlinearFunction
end

function MOI.set(
    m::Optimizer,
    ::MOI.ObjectiveFunction{MOI.ScalarNonlinearFunction},
    f::MOI.ScalarNonlinearFunction,
)
    m.objective_function = f
    return
end

function MOI.set(
    m::Optimizer,
    ::MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}},
    f::MOI.ScalarAffineFunction{Float64},
)
    m.objective_function = f
    return
end

function MOI.get(m::Optimizer, ::MOI.ListOfModelAttributesSet)
    attrs = Any[MOI.ObjectiveSense()]
    if m.objective_function !== nothing
        push!(attrs, MOI.ObjectiveFunction{typeof(m.objective_function)}())
    end
    return attrs
end

# Variables

function MOI.supports_add_constrained_variables(::Optimizer, ::Type{MathOptVRP.PartitionPD})
    return true
end

function MOI.add_constrained_variables(m::Optimizer, set::MathOptVRP.PartitionPD)
    m.partition === nothing ||
        error("ORTools: only one MathOptVRP.PartitionPD set is supported per model")
    n_rows = set.num_services + 2 * set.num_pickup_deliveries
    n_cols = set.num_trucks
    n = n_rows * n_cols
    vars = Vector{MOI.VariableIndex}(undef, n)
    for k = 1:n
        m.next_variable += 1
        v = MOI.VariableIndex(m.next_variable)
        vars[k] = v
        row = ((k - 1) % n_rows) + 1
        col = ((k - 1) ÷ n_rows) + 1
        m.variable_to_position[v] = (row, col)
    end
    m.partition = set
    m.next_constraint += 1
    ci =
        MOI.ConstraintIndex{MOI.VectorOfVariables,MathOptVRP.PartitionPD}(m.next_constraint)
    return vars, ci
end

# `vrptw` declares free `t[i]` variables (one per truck, not part of the
# `Partition`) purely so `sum(t)` can serve as the objective; nonnegativity
# is already implied by membership in the `TimeWindows`
# set (`t[i]` is that set's `route_end` entry). `variable_to_position`
# intentionally has no entry for these, which is how the `TimeWindows`
# `MOI.add_constraint` method below tells a `t` variable apart from a
# `Partition` node variable.

function MOI.add_variable(m::Optimizer)
    m.next_variable += 1
    return MOI.VariableIndex(m.next_variable)
end

function MOI.supports_constraint(
    ::Optimizer,
    ::Type{MOI.VariableIndex},
    ::Type{MOI.GreaterThan{Float64}},
)
    return true
end

function MOI.add_constraint(m::Optimizer, ::MOI.VariableIndex, ::MOI.GreaterThan{Float64})
    m.next_constraint += 1
    return MOI.ConstraintIndex{MOI.VariableIndex,MOI.GreaterThan{Float64}}(
        m.next_constraint,
    )
end

# Incremental interface

MOI.supports_incremental_interface(::Optimizer) = true
function MOI.copy_to(dest::Optimizer, src::MOI.ModelLike)
    return MOI.Utilities.default_copy_to(dest, src)
end

# ── TimeWindows constraint parsing ────────────────────────────────────
# `MathOptVRP.TimeWindows{WITHOUT_START_TIME}(travel, earliest, latest,
# service, num_items)` is applied, one per truck, to
# `[route_end; first_node; route...; last_node]` where `route_end` is a
# free variable (see `MOI.add_constrained_variable` above), `route` is one
# column of `Partition` variables (`num_items` of them), and `first_node`
# / `last_node` are one-based indices into `travel`/`earliest`/`latest`/
# `service` for two (possibly distinct) logical copies of the depot.
# `earliest`/`latest` are indexed by customer location, matching how
# `_jobs_and_shipments` sets `Job.id`.

function MOI.supports_constraint(
    ::Optimizer,
    ::Type{<:Union{MOI.VectorOfVariables,MOI.VectorAffineFunction{Float64}}},
    ::Type{<:MathOptVRP.TimeWindows{MathOptVRP.WITHOUT_START_TIME}},
)
    return true
end

function MOI.add_constraint(
    m::Optimizer,
    f::Union{MOI.VectorOfVariables,MOI.VectorAffineFunction{Float64}},
    s::MathOptVRP.TimeWindows{MathOptVRP.WITHOUT_START_TIME},
)
    items = _normalize_items(f)
    length(items) == MOI.dimension(s) || error(
        "ORTools: TimeWindows constraint expects $(MOI.dimension(s)) entries, got ",
        "$(length(items))",
    )
    items[1] isa MOI.VariableIndex || error(
        "ORTools: TimeWindows entry 1 (`route_end`) must be a variable; got ",
        "$(typeof(items[1]))",
    )
    items[2] isa Real ||
        error("ORTools: TimeWindows entry 2 (first_node) must be a `Real`")
    items[end] isa Real ||
        error("ORTools: TimeWindows last entry (last_node) must be a `Real`")
    first_node = round(Int, items[2])
    last_node = round(Int, items[end])
    column = nothing
    for k = 3:(length(items)-1)
        it = items[k]
        it isa MOI.VariableIndex ||
            error("ORTools: TimeWindows node entries must be variables; got $(typeof(it))")
        pos = get(m.variable_to_position, it, nothing)
        pos === nothing &&
            error("ORTools: variable $(it) is not part of a registered Partition")
        if column === nothing
            column = pos[2]
        elseif column != pos[2]
            error("ORTools: TimeWindows mixes variables from columns $(column) and $(pos[2])")
        end
    end
    column === nothing &&
        error("ORTools: TimeWindows constraint has no interior node variables")
    haskey(m.time_windows_by_column, column) &&
        error("ORTools: only one TimeWindows constraint per truck column is supported")

    t_var = items[1]::MOI.VariableIndex
    m.time_windows_by_column[column] = _TimeWindowsEntry(t_var, first_node, last_node, s)
    m.next_constraint += 1
    return MOI.ConstraintIndex{typeof(f),typeof(s)}(m.next_constraint)
end

# `sum(t)` lowers to a `MOI.ScalarAffineFunction` with a unit-coefficient
# term per truck's `t` variable and a zero constant.
function _time_vars(f::MOI.ScalarAffineFunction{Float64})
    iszero(f.constant) || error("ORTools: VRPTW objective must have a zero constant")
    vars = MOI.VariableIndex[]
    for term in f.terms
        isone(term.coefficient) || error(
            "ORTools: VRPTW objective must be a plain `sum(t)` over unit-coefficient terms",
        )
        push!(vars, term.variable)
    end
    return vars
end

# ── Objective parsing ─────────────────────────────────────────────────
# JuMP produces `sum(op_sum_distances(M, [depot; col; depot]) for i = 1:T)`
# as a `ScalarNonlinearFunction`. The root is either a single
# `:sum_distances` leaf (T == 1) or a tree of `:+` nodes whose leaves are
# all `:sum_distances`. Each leaf's args[1] is the distance matrix and
# args[2] is `[depot, var, var, ..., var, depot]`.

function _collect_sum_distances_leaves!(
    leaves::Vector{MOI.ScalarNonlinearFunction},
    f::MOI.ScalarNonlinearFunction,
)
    if f.head == :+
        for a in f.args
            a isa MOI.ScalarNonlinearFunction ||
                error("ORTools: unsupported `:+` arg of type $(typeof(a))")
            _collect_sum_distances_leaves!(leaves, a)
        end
    elseif f.head == :sum_distances
        push!(leaves, f)
    else
        error("ORTools: unsupported ScalarNonlinearFunction head `$(f.head)`")
    end
    return
end

function _normalize_items(raw)
    if raw isa MOI.VectorOfVariables
        return Any[vi for vi in raw.variables]
    elseif raw isa MOI.VectorAffineFunction
        n = length(raw.constants)
        T = eltype(raw.constants)
        per_row = [MOI.ScalarAffineTerm{T}[] for _ = 1:n]
        for vt in raw.terms
            push!(per_row[vt.output_index], vt.scalar_term)
        end
        return Any[
            _simplify_item(MOI.ScalarAffineFunction(per_row[i], raw.constants[i])) for
            i = 1:n
        ]
    elseif raw isa AbstractVector
        return Any[_simplify_item(el) for el in raw]
    end
    return error("ORTools: `:sum_distances` arg 2 has unexpected type $(typeof(raw))")
end

_simplify_item(x) = x
function _simplify_item(f::MOI.ScalarAffineFunction)
    if isempty(f.terms)
        return f.constant
    end
    if length(f.terms) == 1 && iszero(f.constant) && isone(f.terms[1].coefficient)
        return f.terms[1].variable
    end
    return f
end

function _parse_leaf(m::Optimizer, leaf::MOI.ScalarNonlinearFunction)
    length(leaf.args) == 2 ||
        error("ORTools: `:sum_distances` expects 2 args, got $(length(leaf.args))")
    matrix = leaf.args[1]
    matrix isa AbstractMatrix{<:Real} || error(
        "ORTools: `:sum_distances` arg 1 must be a real matrix; got $(typeof(matrix))",
    )
    items = _normalize_items(leaf.args[2])
    length(items) >= 3 ||
        error("ORTools: `:sum_distances` vector must be `[depot; col; depot]`")
    items[1] isa Real || error("ORTools: depot_start must be a `Real`")
    items[end] isa Real || error("ORTools: depot_end must be a `Real`")
    depot_start = round(Int, items[1])
    depot_end = round(Int, items[end])
    depot_start == depot_end || error("ORTools: depot_start != depot_end is not supported")
    column = nothing
    for k = 2:(length(items)-1)
        it = items[k]
        it isa MOI.VariableIndex ||
            error("ORTools: interior items must be variables; got $(typeof(it))")
        pos = get(m.variable_to_position, it, nothing)
        pos === nothing &&
            error("ORTools: variable $(it) is not part of a registered Partition")
        if column === nothing
            column = pos[2]
        elseif column != pos[2]
            error("ORTools: `:sum_distances` mixes columns")
        end
    end
    column === nothing && error("ORTools: `:sum_distances` has no interior variables")
    return matrix, depot_start, column::Int
end

# ── CP-SAT model construction ───────────────────────────────────────
# Internal node numbering: 0 = depot, 1..n_loc-1 = customers (in original
# external order, skipping the depot). External ids are the 1-based
# `MathOptVRP` ones, that is, the indices of the distance matrix. Arc
# variable indexing flattens `(i, j)` with `i != j` over
# `n_loc * (n_loc - 1)` 0-indexed slots.

_arc_var_index(i::Int, j::Int, n_loc::Int) = i * (n_loc - 1) + (j > i ? j - 1 : j)

# `RoutesConstraintProto` only encodes connectivity (one multi-circuit
# through the depot); it has no notion of pickup/delivery pairing. So a
# pair `(p, d)` (external ids) is lowered to two auxiliary "dimensions",
# propagated one arc at a time via enforced linear constraints:
#   - `route[i]`: the internal id of the first customer on `i`'s route,
#     used as a route identifier so `route[p] == route[d]` means "same
#     vehicle".
#   - `rank[i]`: `i`'s 1-based position along its route, so
#     `rank[p] < rank[d]` means "visited before".
# Every non-depot node has exactly one incoming arc (the `routes`
# constraint enforces this), so these are well-defined for every node.
function _add_pickup_delivery_constraints!(
    constraints::Vector{Sat.ConstraintProto},
    variables::Vector{Sat.IntegerVariableProto},
    n_loc::Int,
    n_arcs::Int,
    pd_pairs::Vector{Tuple{Int,Int}},
    ext_to_int::Dict{Int,Int},
)
    isempty(pd_pairs) && return
    n_customers = n_loc - 1
    rank_var(i::Int) = Int32(n_arcs + (i - 1))
    route_var(i::Int) = Int32(n_arcs + n_customers + (i - 1))
    for i = 1:n_customers
        push!(variables, Sat.IntegerVariableProto("rank_$i", Int64[1, n_customers]))
    end
    for i = 1:n_customers
        push!(variables, Sat.IntegerVariableProto("route_$i", Int64[1, n_customers]))
    end
    for i = 0:(n_loc-1), j = 1:(n_loc-1)
        i == j && continue
        lit = Int32(_arc_var_index(i, j, n_loc))
        if i == 0
            # rank needs to start at one
            push!(
                constraints,
                Sat.ConstraintProto(
                    "rank_start_$j",
                    Int32[lit],
                    PB.OneOf(
                        :linear,
                        Sat.LinearConstraintProto(
                            Int32[rank_var(j)],
                            Int64[1],
                            Int64[1, 1],
                        ),
                    ),
                ),
            )
            # route id == internal first customer id
            push!(
                constraints,
                Sat.ConstraintProto(
                    "route_start_$j",
                    Int32[lit],
                    PB.OneOf(
                        :linear,
                        Sat.LinearConstraintProto(
                            Int32[route_var(j)],
                            Int64[1],
                            Int64[j, j],
                        ),
                    ),
                ),
            )
        else
            # precedence constraint
            push!(
                constraints,
                Sat.ConstraintProto(
                    "rank_step_$(i)_$(j)",
                    Int32[lit],
                    PB.OneOf(
                        :linear,
                        Sat.LinearConstraintProto(
                            Int32[rank_var(j), rank_var(i)],
                            Int64[1, -1],
                            Int64[1, 1],
                        ),
                    ),
                ),
            )
            # identical route constraint
            push!(
                constraints,
                Sat.ConstraintProto(
                    "route_step_$(i)_$(j)",
                    Int32[lit],
                    PB.OneOf(
                        :linear,
                        Sat.LinearConstraintProto(
                            Int32[route_var(j), route_var(i)],
                            Int64[1, -1],
                            Int64[0, 0],
                        ),
                    ),
                ),
            )
        end
    end
    for (p_ext, d_ext) in pd_pairs
        p, d = ext_to_int[p_ext], ext_to_int[d_ext]
        push!(
            constraints,
            Sat.ConstraintProto(
                "same_route_$(p)_$(d)",
                Int32[],
                PB.OneOf(
                    :linear,
                    Sat.LinearConstraintProto(
                        Int32[route_var(p), route_var(d)],
                        Int64[1, -1],
                        Int64[0, 0],
                    ),
                ),
            ),
        )
        push!(
            constraints,
            Sat.ConstraintProto(
                "precedence_$(p)_$(d)",
                Int32[],
                PB.OneOf(
                    :linear,
                    Sat.LinearConstraintProto(
                        Int32[rank_var(d), rank_var(p)],
                        Int64[1, -1],
                        Int64[1, n_customers],
                    ),
                ),
            ),
        )
    end
    return
end

# Shared skeleton for every CP-SAT model built here: the `x_k` arc
# variables, the single `RoutesConstraintProto` (one multi-circuit through
# depot node `0`), and the "at most `n_trucks` routes" limit. Callers add
# whatever extra variables/constraints/objective their variant needs on top.
function _routes_skeleton(n_loc::Int, n_trucks::Int)
    n_arcs = n_loc * (n_loc - 1)
    variables = Sat.IntegerVariableProto[
        Sat.IntegerVariableProto("x_$k", Int64[0, 1]) for k = 0:(n_arcs-1)
    ]

    tails = Int32[]
    heads = Int32[]
    literals = Int32[]
    for i = 0:(n_loc-1), j = 0:(n_loc-1)
        i == j && continue
        push!(tails, Int32(i))
        push!(heads, Int32(j))
        push!(literals, Int32(_arc_var_index(i, j, n_loc)))
    end
    routes = Sat.RoutesConstraintProto(
        tails,
        heads,
        literals,
        Int32[],
        Int64(0),
        Sat.var"RoutesConstraintProto.NodeExpressions"[],
    )

    constraints = Sat.ConstraintProto[]
    push!(constraints, Sat.ConstraintProto("routes", Int32[], PB.OneOf(:routes, routes)))

    # Limit truck count: sum of outgoing arcs from depot ≤ n_trucks.
    out_vars = Int32[]
    out_coeffs = Int64[]
    for j = 1:(n_loc-1)
        push!(out_vars, Int32(_arc_var_index(0, j, n_loc)))
        push!(out_coeffs, Int64(1))
    end
    vehicle_lim = Sat.LinearConstraintProto(out_vars, out_coeffs, Int64[0, n_trucks])
    push!(
        constraints,
        Sat.ConstraintProto("vehicle_limit", Int32[], PB.OneOf(:linear, vehicle_lim)),
    )
    return variables, constraints, n_arcs
end

function _build_cp_model(
    M::AbstractMatrix{<:Real},
    ext_depot::Int,
    n_trucks::Int,
    pd_pairs::Vector{Tuple{Int,Int}} = Tuple{Int,Int}[],
)
    n_loc = size(M, 1)
    n_loc == size(M, 2) || error("ORTools: distance matrix must be square; got $(size(M))")
    int_to_ext = Int[ext_depot]
    for ext = 1:n_loc
        ext == ext_depot && continue
        push!(int_to_ext, ext)
    end
    @assert length(int_to_ext) == n_loc
    ext_to_int = Dict(ext => i - 1 for (i, ext) in enumerate(int_to_ext))

    variables, constraints, n_arcs = _routes_skeleton(n_loc, n_trucks)

    _add_pickup_delivery_constraints!(
        constraints,
        variables,
        n_loc,
        n_arcs,
        pd_pairs,
        ext_to_int,
    )

    obj_vars = Int32[]
    obj_coeffs = Int64[]
    for i = 0:(n_loc-1), j = 0:(n_loc-1)
        i == j && continue
        cost = round(Int64, M[int_to_ext[i+1], int_to_ext[j+1]])
        cost == 0 && continue
        push!(obj_vars, Int32(_arc_var_index(i, j, n_loc)))
        push!(obj_coeffs, cost)
    end
    objective = Sat.CpObjectiveProto(
        obj_vars,
        obj_coeffs,
        0.0,
        1.0,
        Int64[],
        false,
        Int64(0),
        Int64(0),
        Int64(0),
    )

    model = Sat.CpModelProto(
        "vrp",
        variables,
        constraints,
        objective,
        nothing,
        Sat.DecisionStrategyProto[],
        nothing,
        Int32[],
        nothing,
    )
    return model, int_to_ext
end

# ── VRPTW: TimeWindows lowering ──────────────────────────────────────
# `RoutesConstraintProto` has no native time-window primitive, so it is
# lowered the same way pickup/delivery precedence is (see
# `_add_pickup_delivery_constraints!`): one extra integer variable per
# customer for its service-start time, propagated arc by arc via enforced
# linear constraints. A second per-customer variable, `contrib`, captures
# the completion time of the route if that customer happens to be the
# last one before returning to the depot (`0` otherwise, since nothing
# forces it above its `0` lower bound unless the corresponding
# arc-into-depot literal is set); minimizing `sum(contrib)` is therefore
# exactly minimizing the total completion time summed over all routes.
# Internal numbering here is simpler than `_build_cp_model`'s: `0` = depot,
# `1..n_customers` = customer ids, which already match `MathOptVRP`'s own
# (external) `Partition` numbering one for one, since `TimeWindows` node
# values are themselves one-based indices into `earliest`/`latest`/`service`.
function _build_cp_model_time_windows(
    travel::AbstractMatrix{<:Real},
    earliest::AbstractVector{<:Real},
    latest::AbstractVector{<:Real},
    service::AbstractVector{<:Real},
    first_node::Int,
    last_node::Int,
    n_customers::Int,
    n_trucks::Int,
)
    n_loc = n_customers + 1
    variables, constraints, n_arcs = _routes_skeleton(n_loc, n_trucks)

    time_var(c::Int) = Int32(n_arcs + (c - 1))
    contrib_var(c::Int) = Int32(n_arcs + n_customers + (c - 1))
    for c = 1:n_customers
        push!(
            variables,
            Sat.IntegerVariableProto(
                "time_$c",
                Int64[round(Int, earliest[c]), round(Int, latest[c])],
            ),
        )
    end
    # A generous, finite upper sentinel for `contrib`: no single completion
    # time can exceed the largest window plus one hop's worth of travel and
    # service, and `RoutesConstraintProto`'s propagation needs a bounded
    # domain (unlike a literal `typemax(Int64)`, which risks overflow in the
    # solver's own arithmetic).
    sentinel =
        round(Int, maximum(latest)) + round(Int, maximum(travel)) +
        round(Int, maximum(service)) + 1

    dep_bound = round(Int, earliest[first_node]) + round(Int, service[first_node])
    for j = 1:n_customers
        lit = Int32(_arc_var_index(0, j, n_loc))
        bound = dep_bound + round(Int, travel[first_node, j])
        push!(
            constraints,
            Sat.ConstraintProto(
                "tw_depot_out_$j",
                Int32[lit],
                PB.OneOf(
                    :linear,
                    Sat.LinearConstraintProto(
                        Int32[time_var(j)],
                        Int64[1],
                        Int64[bound, sentinel],
                    ),
                ),
            ),
        )
    end
    for i = 1:n_customers, j = 1:n_customers
        i == j && continue
        lit = Int32(_arc_var_index(i, j, n_loc))
        bound = round(Int, service[i]) + round(Int, travel[i, j])
        push!(
            constraints,
            Sat.ConstraintProto(
                "tw_step_$(i)_$(j)",
                Int32[lit],
                PB.OneOf(
                    :linear,
                    Sat.LinearConstraintProto(
                        Int32[time_var(j), time_var(i)],
                        Int64[1, -1],
                        Int64[bound, sentinel],
                    ),
                ),
            ),
        )
    end
    end_service = round(Int, service[last_node])
    end_floor = round(Int, earliest[last_node]) + end_service
    for i = 1:n_customers
        push!(
            variables,
            Sat.IntegerVariableProto("contrib_$i", Int64[0, sentinel]),
        )
        lit = Int32(_arc_var_index(i, 0, n_loc))
        # `contrib_i >= arrival_at_depot + service[last_node]`, the
        # `arrival`-based half of `start(last_node) = max(earliest, arrival)`.
        # The `earliest`-based half is the separate "floor" constraint below.
        bound = round(Int, service[i]) + round(Int, travel[i, last_node]) + end_service
        push!(
            constraints,
            Sat.ConstraintProto(
                "tw_depot_in_$i",
                Int32[lit],
                PB.OneOf(
                    :linear,
                    Sat.LinearConstraintProto(
                        Int32[contrib_var(i), time_var(i)],
                        Int64[1, -1],
                        Int64[bound, sentinel],
                    ),
                ),
            ),
        )
        push!(
            constraints,
            Sat.ConstraintProto(
                "tw_depot_in_floor_$i",
                Int32[lit],
                PB.OneOf(
                    :linear,
                    Sat.LinearConstraintProto(
                        Int32[contrib_var(i)],
                        Int64[1],
                        Int64[end_floor, sentinel],
                    ),
                ),
            ),
        )
    end

    objective = Sat.CpObjectiveProto(
        Int32[contrib_var(i) for i = 1:n_customers],
        Int64[1 for _ = 1:n_customers],
        0.0,
        1.0,
        Int64[],
        false,
        Int64(0),
        Int64(0),
        Int64(0),
    )

    model = Sat.CpModelProto(
        "vrptw",
        variables,
        constraints,
        objective,
        nothing,
        Sat.DecisionStrategyProto[],
        nothing,
        Int32[],
        nothing,
    )
    int_to_ext = Int[0; collect(1:n_customers)]
    return model, int_to_ext
end

# Independently recomputes one route's total completion time in Julia
# (rather than trusting the CP-SAT `time`/`contrib` values back off the
# solution), mirroring how `_route_cost` recomputes `:sum_distances`
# against the external matrix. Matches the recurrence documented on
# `MathOptVRP.TimeWindows`: each node's service starts at
# `max(earliest, arrival)`, and travel/service both apply to the fixed
# `first_node`/`last_node` depot occurrences too.
function _route_completion_time(
    route::Vector{Int},
    travel::AbstractMatrix{<:Real},
    earliest::AbstractVector{<:Real},
    service::AbstractVector{<:Real},
    first_node::Int,
    last_node::Int,
)
    isempty(route) && return 0
    depart = round(Int, earliest[first_node]) + round(Int, service[first_node])
    prev = first_node
    for c in route
        arrival = depart + round(Int, travel[prev, c])
        start = max(round(Int, earliest[c]), arrival)
        depart = start + round(Int, service[c])
        prev = c
    end
    arrival = depart + round(Int, travel[prev, last_node])
    start = max(round(Int, earliest[last_node]), arrival)
    return start + round(Int, service[last_node])
end

# Parses the `MathOptVRP.TimeWindows` constraints + `sum(t)` objective into
# the ingredients `_build_cp_model_time_windows` needs. Trucks are
# homogeneous (every column's `TimeWindows` data must agree), so — unlike
# `_build_cp_model`'s per-truck `:sum_distances` leaves — there is no need
# to pin a particular decoded route to a particular column: any consistent
# assignment gives the same `sum(t)`.
function _lower_time_windows(m::Optimizer)
    m.objective_function isa MOI.ScalarAffineFunction{Float64} &&
    m.objective_sense == MOI.MIN_SENSE || error(
        "ORTools: TimeWindows constraints require a `MIN_SENSE` linear `sum(t)` objective",
    )
    n_trucks = m.partition.num_trucks
    sort(collect(keys(m.time_windows_by_column))) == collect(1:n_trucks) || error(
        "ORTools: expected one TimeWindows constraint for each of the $(n_trucks) truck ",
        "columns",
    )
    entries = [m.time_windows_by_column[col] for col = 1:n_trucks]

    obj_vars = _time_vars(m.objective_function)
    Set(obj_vars) == Set(e.t_var for e in entries) && length(obj_vars) == n_trucks || error(
        "ORTools: objective must be `sum(t)` over exactly the TimeWindows constraints' ",
        "time variables",
    )

    ref = entries[1].set
    first_node = entries[1].first_node
    last_node = entries[1].last_node
    for e in entries
        e.set.travel == ref.travel ||
            error("ORTools: per-truck TimeWindows travel matrices must be equal")
        e.set.earliest == ref.earliest ||
            error("ORTools: per-truck TimeWindows `earliest` must agree")
        e.set.latest == ref.latest ||
            error("ORTools: per-truck TimeWindows `latest` must agree")
        e.set.service == ref.service ||
            error("ORTools: per-truck TimeWindows `service` must agree")
        e.first_node == first_node ||
            error("ORTools: per-truck TimeWindows first_node must agree")
        e.last_node == last_node ||
            error("ORTools: per-truck TimeWindows last_node must agree")
    end

    n_customers = m.partition.num_services + 2 * m.partition.num_pickup_deliveries
    customer_locs =
        [i for i = 1:length(ref.earliest) if i != first_node && i != last_node]
    customer_locs == collect(1:n_customers) || error(
        "ORTools: TimeWindows first_node/last_node must be the only entries outside ",
        "1:$(n_customers)",
    )
    return entries, ref, first_node, last_node, n_customers
end

function _decode_routes(solution::Vector{Int64}, n_loc::Int, int_to_ext::Vector{Int})
    outgoing = Dict{Int,Vector{Int}}()
    for i = 0:(n_loc-1)
        outgoing[i] = Int[]
    end
    for i = 0:(n_loc-1), j = 0:(n_loc-1)
        i == j && continue
        idx = _arc_var_index(i, j, n_loc)
        if solution[idx+1] != 0
            push!(outgoing[i], j)
        end
    end
    routes = Vector{Int}[]
    for start in outgoing[0]
        route_ext = Int[]
        cur = start
        guard = 0
        while cur != 0
            push!(route_ext, int_to_ext[cur+1])
            isempty(outgoing[cur]) && error("ORTools: broken route from $cur")
            cur = outgoing[cur][1]
            guard += 1
            guard > n_loc + 1 && error("ORTools: route does not terminate at depot")
        end
        push!(routes, route_ext)
    end
    return routes
end

function _encode(proto)
    io = IOBuffer()
    PB.encode(PB.ProtoEncoder(io), proto)
    return take!(io)
end

# `Sat.SatParameters` is proto-generated so it only has a positional
# constructor over its ~300 fields; start from the proto defaults and
# override the two fields we map from MOI attributes. Fields left at their
# default do not make it to the wire.
function _sat_parameters(m::Optimizer)
    defaults = PB.default_values(Sat.SatParameters)
    params = merge(
        defaults,
        (
            max_time_in_seconds = something(m.time_limit, defaults.max_time_in_seconds),
            log_search_progress = !m.silent,
        ),
    )
    return Sat.SatParameters(params...)
end

function _solve(model::Sat.CpModelProto, params::Sat.SatParameters)
    request, parameters = _encode(model), _encode(params)
    response = Ref{Ptr{Cvoid}}()
    response_len = Ref{Cint}(0)
    ORTools.SolveCpModelWithParameters(
        request,
        length(request),
        parameters,
        length(parameters),
        response,
        response_len,
    )
    # The C API `malloc`s the serialized response and transfers ownership.
    bytes = unsafe_wrap(Vector{UInt8}, Ptr{UInt8}(response[]), response_len[])
    decoded = PB.decode(PB.ProtoDecoder(IOBuffer(bytes)), Sat.CpSolverResponse)
    Libc.free(response[])
    return decoded
end

# ── Optimize ────────────────────────────────────────────────────────

# CpSolverStatus enum values from operations_research/sat/cp_model.proto:
# 0 = UNKNOWN, 1 = MODEL_INVALID, 2 = FEASIBLE, 3 = INFEASIBLE, 4 = OPTIMAL.
function _set_status!(m::Optimizer, response)
    status_int = Int(response.status)
    if status_int == 4
        m.termination_status = MOI.OPTIMAL
        m.primal_status = MOI.FEASIBLE_POINT
    elseif status_int == 2
        m.termination_status = MOI.LOCALLY_SOLVED
        m.primal_status = MOI.FEASIBLE_POINT
    elseif status_int == 3
        m.termination_status = MOI.INFEASIBLE
        m.primal_status = MOI.NO_SOLUTION
    else
        m.termination_status = MOI.OTHER_ERROR
        m.primal_status = MOI.NO_SOLUTION
    end
    m.raw_status = "CpSolverStatus = $(status_int)"
    m.solved = true
    return
end

function _optimize_sum_distances!(m::Optimizer)
    leaves = MOI.ScalarNonlinearFunction[]
    _collect_sum_distances_leaves!(leaves, m.objective_function)
    isempty(leaves) && error("ORTools: empty `:sum_distances` objective")

    parsed = [_parse_leaf(m, leaf) for leaf in leaves]
    n_trucks = length(leaves)
    n_trucks == m.partition.num_trucks || error(
        "ORTools: objective has $n_trucks `:sum_distances` terms but PartitionPD has $(m.partition.num_trucks)",
    )
    M_ref = parsed[1][1]
    depot = parsed[1][2]
    for (mat, dep, _) in parsed
        mat == M_ref || error("ORTools: per-truck `:sum_distances` matrices must be equal")
        dep == depot || error("ORTools: per-truck depots must agree")
    end

    ns, npd = m.partition.num_services, m.partition.num_pickup_deliveries
    pd_pairs = Tuple{Int,Int}[(ns + k, ns + npd + k) for k = 1:npd]

    cp_model, int_to_ext = _build_cp_model(M_ref, depot, n_trucks, pd_pairs)
    n_loc = length(int_to_ext)

    response = _solve(cp_model, _sat_parameters(m))
    _set_status!(m, response)

    if m.primal_status == MOI.FEASIBLE_POINT
        routes_ext = _decode_routes(response.solution, n_loc, int_to_ext)
        # Pad to n_trucks (unused vehicles get empty routes).
        while length(routes_ext) < n_trucks
            push!(routes_ext, Int[])
        end
        m.routes = routes_ext
        # Recompute cost from routes against the external matrix `M_ref` to
        # avoid any ambiguity in CP-SAT's `scaling_factor` / `offset` round-trip.
        cost = 0
        for r in routes_ext
            isempty(r) && continue
            cost += M_ref[depot, r[1]] + M_ref[r[end], depot]
            for k = 2:length(r)
                cost += M_ref[r[k-1], r[k]]
            end
        end
        m.objective_value = cost
        # Project routes back onto the `Partition` variables: slot `(row, col)`
        # holds the `row`-th customer visited by truck `col`, and `0` once the
        # route has ended, as `MathOptVRP.Partition` prescribes.
        for (var, (row, col)) in m.variable_to_position
            route = routes_ext[col]
            m.variable_values[var] = row <= length(route) ? route[row] : 0
        end
    end
    return
end

function _optimize_time_windows!(m::Optimizer)
    entries, ref, first_node, last_node, n_customers = _lower_time_windows(m)
    n_trucks = m.partition.num_trucks

    cp_model, int_to_ext = _build_cp_model_time_windows(
        ref.travel,
        ref.earliest,
        ref.latest,
        ref.service,
        first_node,
        last_node,
        n_customers,
        n_trucks,
    )
    n_loc = length(int_to_ext)

    response = _solve(cp_model, _sat_parameters(m))
    _set_status!(m, response)

    if m.primal_status == MOI.FEASIBLE_POINT
        routes_ext = _decode_routes(response.solution, n_loc, int_to_ext)
        while length(routes_ext) < n_trucks
            push!(routes_ext, Int[])
        end
        m.routes = routes_ext
        # Recompute each route's completion time independently in Julia (see
        # `_route_completion_time`) rather than trusting CP-SAT's `time`/
        # `contrib` values back off the solution.
        route_times = [
            _route_completion_time(r, ref.travel, ref.earliest, ref.service, first_node, last_node)
            for r in routes_ext
        ]
        m.objective_value = sum(route_times)
        for (var, (row, col)) in m.variable_to_position
            route = routes_ext[col]
            m.variable_values[var] = row <= length(route) ? route[row] : 0
        end
        # Which decoded route ends up assigned to which column is arbitrary
        # (see `_lower_time_windows`); every column's `t` variable just needs
        # *some* route's completion time so that `sum(t)` matches the total.
        for col = 1:n_trucks
            m.variable_values[entries[col].t_var] = route_times[col]
        end
    end
    return
end

function MOI.optimize!(m::Optimizer)
    m.partition !== nothing ||
        error("ORTools: model has no `MathOptVRP.PartitionPD` variables")
    if !isempty(m.time_windows_by_column)
        return _optimize_time_windows!(m)
    end
    m.objective_function !== nothing && m.objective_sense == MOI.MIN_SENSE ||
        error("ORTools: requires a `MIN_SENSE` `:sum_distances` objective")
    return _optimize_sum_distances!(m)
end

# ── Solution getters ────────────────────────────────────────────────

MOI.get(m::Optimizer, ::MOI.TerminationStatus) = m.termination_status
function MOI.get(m::Optimizer, attr::MOI.PrimalStatus)
    return attr.result_index == 1 ? m.primal_status : MOI.NO_SOLUTION
end
MOI.get(::Optimizer, ::MOI.DualStatus) = MOI.NO_SOLUTION
MOI.get(m::Optimizer, ::MOI.RawStatusString) = m.raw_status
MOI.get(m::Optimizer, ::MOI.ResultCount) = m.primal_status == MOI.NO_SOLUTION ? 0 : 1
MOI.get(::Optimizer, ::MOI.SolveTimeSec) = 0.0

function MOI.get(m::Optimizer, attr::MOI.ObjectiveValue)
    MOI.check_result_index_bounds(m, attr)
    return Float64(m.objective_value)
end

function MOI.get(m::Optimizer, attr::MOI.VariablePrimal, vi::MOI.VariableIndex)
    MOI.check_result_index_bounds(m, attr)
    val = get(m.variable_values, vi, nothing)
    val === nothing && error("ORTools: no primal value for variable $(vi)")
    return Float64(val)
end

end # module MathOptVRPORToolsExt
