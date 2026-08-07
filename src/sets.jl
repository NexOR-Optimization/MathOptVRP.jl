# Solver-agnostic MOI vector sets for vehicle-routing models. A solver
# wrapper implements `MOI.add_constrained_variables(model, ::Partition)`
# and `MOI.add_constraint(model, f, ::TimeWindows)` to give these sets
# their operational meaning; here we just declare the shape, the JuMP
# variable-construction helpers, and the obvious `Base.copy` overrides.

"""
    List(dimension::Int)

A vector set whose `dimension` variables form a permutation of
`1:dimension`.
"""
struct List <: MOI.AbstractVectorSet
    dimension::Int
end

MOI.dimension(s::List) = s.dimension

"""
    Partition(num_clients::Int, num_trucks::Int)

A `num_clients × num_trucks` matrix of variables; each truck's column is
a (possibly variable-length) sub-list and the columns together partition
`1:num_clients`.

A column has one slot per client, so a truck visiting fewer than
`num_clients` of them has slots to spare: it holds the clients it visits,
in order, and the remaining slots are `0`. A solution is therefore read
back as the entries of a column before its first `0`, without the solver
having to say anything about the length.
"""
struct Partition <: MOI.AbstractVectorSet
    num_clients::Int
    num_trucks::Int
end

MOI.dimension(s::Partition) = s.num_clients * s.num_trucks

function JuMP.build_variable(
    error_fn::Function,
    variables::Matrix{<:JuMP.AbstractVariable},
    set::Partition,
)
    size(variables) == (set.num_clients, set.num_trucks) || error_fn(
        "MathOptVRP.Partition: expected a `$(set.num_clients) × ",
        "$(set.num_trucks)` variable matrix, got `$(size(variables))`.",
    )
    return JuMP.VariablesConstrainedOnCreation(
        vec(variables),
        set,
        JuMP.ArrayShape(size(variables)),
    )
end

"""
    Base.convert(::Type{Partition}, s::List)

`List(n)` and `Partition(n, 1)` have the same flat dimension (`n`) and, for
a single truck, the same "permutation of `1:n`" semantics, so this
conversion always succeeds.
"""
Base.convert(::Type{Partition}, s::List) = Partition(s.dimension, 1)

"""
    Base.convert(::Type{List}, s::Partition)

Only defined when `s.num_trucks == 1`: a multi-truck `Partition` has no
`List` equivalent.
"""
function Base.convert(::Type{List}, s::Partition)
    if s.num_trucks != 1
        # Julia v1.10's `InexactError` only accepts `(func, T, val)`; the
        # variadic constructor that takes a message is v1.11 or later.
        throw(InexactError(:convert, List, s))
    end
    return List(s.num_clients)
end

"""
    PartitionPD(num_services::Int, num_pickup_deliveries::Int, num_trucks::Int)

A partition variant for pickup/delivery routing. The flat dimension is
`(num_services + 2 * num_pickup_deliveries) * num_trucks`. Node
identities are:

  - services:    `1 .. num_services`
  - pickups:     `num_services + 1 .. num_services + num_pd`
  - deliveries:  `num_services + num_pd + 1 .. n_total`

Pickup `num_services + k` is paired with delivery
`num_services + num_pd + k`. Columns are `0`-padded like
[`Partition`](@ref).
"""
struct PartitionPD <: MOI.AbstractVectorSet
    num_services::Int
    num_pickup_deliveries::Int
    num_trucks::Int
end

_pd_n_total(s::PartitionPD) = s.num_services + 2 * s.num_pickup_deliveries

MOI.dimension(s::PartitionPD) = _pd_n_total(s) * s.num_trucks

function JuMP.build_variable(
    error_fn::Function,
    variables::Matrix{<:JuMP.AbstractVariable},
    set::PartitionPD,
)
    n_total = _pd_n_total(set)
    size(variables) == (n_total, set.num_trucks) || error_fn(
        "MathOptVRP.PartitionPD: expected a `$n_total × $(set.num_trucks)` ",
        "variable matrix, got `$(size(variables))`.",
    )
    return JuMP.VariablesConstrainedOnCreation(
        vec(variables),
        set,
        JuMP.ArrayShape(size(variables)),
    )
end

"""
    TimeWindows(travel, earliest, latest, service)

Vector constraint applied to `[t; depot_start; nodes; depot_end]` for one
truck. `t` is the truck's total-time variable; `depot_start` /
`depot_end` are constant node indices; `nodes` is a column of variables
from a `Partition` / `PartitionPD`. The constraint enforces, for every
visited customer, that the service start lies in `[earliest, latest]`,
and links `t >= total_time` so that `@objective(model, Min, sum(t))`
minimises the makespan (travel + waiting + service + return to depot).
"""
struct TimeWindows{T<:Real} <: MOI.AbstractVectorSet
    travel::Matrix{T}
    earliest::Vector{T}
    latest::Vector{T}
    service::T
end

MOI.dimension(s::TimeWindows) = length(s.earliest) + 3

Base.copy(s::TimeWindows) = s

"""
    Capacity(delta, capacity)

Vector constraint on one truck's column of nodes. The cumulative load
along the route (starting at `0`, with `delta[v]` added at each visited
node `v`) never exceeds `capacity`.
"""
struct Capacity{T<:Real} <: MOI.AbstractVectorSet
    delta::Vector{T}
    capacity::T
end

MOI.dimension(s::Capacity) = length(s.delta)

Base.copy(s::Capacity) = s

"""
    CapacitatedTimeWindows(travel, earliest, latest, fixed_time, slope,
                          delta, capacity)

Combined capacity + time-window constraint applied to
`[t; depot_start; nodes; depot_end]` for one truck. Per-node service
time is the affine function `fixed_time + slope * |delta[v]|`. The two
constraints share a set because the time-window recurrence needs the
per-node service time, which depends on the load handled at each node.
"""
struct CapacitatedTimeWindows{T<:Real} <: MOI.AbstractVectorSet
    travel::Matrix{T}
    earliest::Vector{T}
    latest::Vector{T}
    fixed_time::T
    slope::T
    delta::Vector{T}
    capacity::T
end

MOI.dimension(s::CapacitatedTimeWindows) = length(s.earliest) + 3

Base.copy(s::CapacitatedTimeWindows) = s
