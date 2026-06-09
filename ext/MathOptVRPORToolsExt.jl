module MathOptVRPORToolsExt

# Minimal MathOptVRP backend on top of OR-Tools' CP-SAT solver. Scope is
# narrow: just enough to run `MathOptVRP.test_vrp`. We accept one
# `MathOptVRP.Partition` set of variables and a `MOI.ScalarNonlinearFunction`
# objective built from `MathOptVRP.op_sum_distances` (one leaf per truck,
# optionally wrapped in `:+` nodes), then lower the VRP to a CP-SAT
# `CpModelProto` with a `RoutesConstraintProto` and shell out to
# `sat_runner` from `ORTools_jll`.

# ORTools has a `CPSATOptimizer` but that we could extend but it still seems to be WIP,
# e.g., no `optimize!` function and https://github.com/google/or-tools/pull/5219
# so we just roll our own for now.

import MathOptInterface as MOI
import MathOptVRP
import ORTools

const Sat = ORTools.Sat
const PB = ORTools.PB
const ORTools_jll = ORTools.ORTools_jll

mutable struct Optimizer <: MOI.AbstractOptimizer
    next_variable::Int
    next_constraint::Int
    variable_to_position::Dict{MOI.VariableIndex,Tuple{Int,Int}}
    partition::Union{Nothing,MathOptVRP.Partition}
    objective_sense::MOI.OptimizationSense
    objective_function::Union{Nothing,MOI.ScalarNonlinearFunction}
    silent::Bool
    time_limit::Union{Nothing,Float64}
    solved::Bool
    routes::Vector{Vector{Int}}
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
            false,
            nothing,
            false,
            Vector{Int}[],
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
           !m.solved
end

function MOI.empty!(m::Optimizer)
    m.next_variable = 0
    m.next_constraint = 0
    empty!(m.variable_to_position)
    m.partition = nothing
    m.objective_sense = MOI.FEASIBILITY_SENSE
    m.objective_function = nothing
    m.solved = false
    empty!(m.routes)
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

MOI.get(m::Optimizer, ::MOI.ObjectiveFunctionType) = MOI.ScalarNonlinearFunction

function MOI.set(
    m::Optimizer,
    ::MOI.ObjectiveFunction{MOI.ScalarNonlinearFunction},
    f::MOI.ScalarNonlinearFunction,
)
    m.objective_function = f
    return
end

function MOI.get(m::Optimizer, ::MOI.ListOfModelAttributesSet)
    attrs = Any[MOI.ObjectiveSense()]
    if m.objective_function !== nothing
        push!(attrs, MOI.ObjectiveFunction{MOI.ScalarNonlinearFunction}())
    end
    return attrs
end

# Variables

function MOI.supports_add_constrained_variables(::Optimizer, ::Type{MathOptVRP.Partition})
    return true
end

function MOI.add_constrained_variables(m::Optimizer, set::MathOptVRP.Partition)
    m.partition === nothing ||
        error("ORTools: only one MathOptVRP.Partition set is supported per model")
    n_rows, n_cols = set.num_clients, set.num_trucks
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
    ci = MOI.ConstraintIndex{MOI.VectorOfVariables,MathOptVRP.Partition}(m.next_constraint)
    return vars, ci
end

# Incremental interface

MOI.supports_incremental_interface(::Optimizer) = true
function MOI.copy_to(dest::Optimizer, src::MOI.ModelLike)
    return MOI.Utilities.default_copy_to(dest, src)
end

# ── Objective parsing (mirrors the Vroom wrapper) ────────────────────

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
# external order, skipping the depot). Arc variable indexing flattens
# `(i, j)` with `i != j` over `n_loc * (n_loc - 1)` 0-indexed slots.

_arc_var_index(i::Int, j::Int, n_loc::Int) = i * (n_loc - 1) + (j > i ? j - 1 : j)

function _build_cp_model(M::AbstractMatrix{<:Real}, ext_depot::Int, n_trucks::Int)
    n_loc = size(M, 1)
    n_loc == size(M, 2) || error("ORTools: distance matrix must be square; got $(size(M))")
    int_to_ext = Int[ext_depot]
    for ext = 0:(n_loc-1)
        ext == ext_depot && continue
        push!(int_to_ext, ext)
    end
    @assert length(int_to_ext) == n_loc

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

    obj_vars = Int32[]
    obj_coeffs = Int64[]
    for i = 0:(n_loc-1), j = 0:(n_loc-1)
        i == j && continue
        cost = round(Int64, M[int_to_ext[i+1]+1, int_to_ext[j+1]+1])
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

function _serialize(model::Sat.CpModelProto)
    io = IOBuffer()
    PB.encode(PB.ProtoEncoder(io), model)
    return take!(io)
end

function _deserialize_response(bytes::Vector{UInt8})
    return PB.decode(PB.ProtoDecoder(IOBuffer(bytes)), Sat.CpSolverResponse)
end

# ── Optimize ────────────────────────────────────────────────────────

function MOI.optimize!(m::Optimizer)
    m.partition !== nothing ||
        error("ORTools: model has no `MathOptVRP.Partition` variables")
    m.objective_function !== nothing && m.objective_sense == MOI.MIN_SENSE ||
        error("ORTools: requires a `MIN_SENSE` `:sum_distances` objective")

    leaves = MOI.ScalarNonlinearFunction[]
    _collect_sum_distances_leaves!(leaves, m.objective_function)
    isempty(leaves) && error("ORTools: empty `:sum_distances` objective")

    parsed = [_parse_leaf(m, leaf) for leaf in leaves]
    n_trucks = length(leaves)
    n_trucks == m.partition.num_trucks || error(
        "ORTools: objective has $n_trucks `:sum_distances` terms but Partition has $(m.partition.num_trucks)",
    )
    M_ref = parsed[1][1]
    depot = parsed[1][2]
    for (mat, dep, _) in parsed
        mat == M_ref || error("ORTools: per-truck `:sum_distances` matrices must be equal")
        dep == depot || error("ORTools: per-truck depots must agree")
    end

    cp_model, int_to_ext = _build_cp_model(M_ref, depot, n_trucks)
    n_loc = length(int_to_ext)

    response = try
        bytes = _serialize(cp_model)
        out_bytes = mktemp() do input_path, input_io
            write(input_io, bytes)
            flush(input_io)
            mktemp() do output_path, _
                params =
                    m.time_limit === nothing ? String[] :
                    String["--params=max_time_in_seconds:$(m.time_limit)"]
                ORTools_jll.sat_runner() do exe
                    # `sat_runner` exits with a non-zero status that encodes
                    # the `CpSolverStatus` (e.g. ~10 for OPTIMAL), so we
                    # `ignorestatus` and rely on the written response proto.
                    cmd = ignorestatus(
                        Cmd([
                            exe,
                            "--input=$input_path",
                            "--output=$output_path",
                            params...,
                        ]),
                    )
                    run(pipeline(cmd; stdout = devnull, stderr = devnull))
                end
                read(output_path)
            end
        end
        _deserialize_response(out_bytes)
    catch err
        m.solved = true
        m.termination_status = MOI.OTHER_ERROR
        m.primal_status = MOI.NO_SOLUTION
        m.raw_status = sprint(showerror, err)
        return
    end

    # CpSolverStatus enum values from operations_research/sat/cp_model.proto:
    # 0 = UNKNOWN, 1 = MODEL_INVALID, 2 = FEASIBLE, 3 = INFEASIBLE, 4 = OPTIMAL.
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
            cost += M_ref[depot+1, r[1]+1] + M_ref[r[end]+1, depot+1]
            for k = 2:length(r)
                cost += M_ref[r[k-1]+1, r[k]+1]
            end
        end
        m.objective_value = cost
    end
    return
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

function MOI.get(m::Optimizer, attr::MOI.VariablePrimal, ::MOI.VariableIndex)
    MOI.check_result_index_bounds(m, attr)
    return 0.0
end

end # module MathOptVRPORToolsExt
