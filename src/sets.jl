# Solver-agnostic MOI vector sets for vehicle-routing models. A solver
# wrapper implements `MOI.add_constrained_variables(model, ::Partition)`
# and `MOI.add_constraint(model, f, ::TimeWindows)` to give these sets
# their operational meaning; here we just declare the shape, the JuMP
# variable-construction helpers, and the obvious `Base.copy` overrides.

"""
    Permutation(dimension::Int)

A vector set whose `dimension` variables form a permutation of
`1:dimension`.
"""
struct Permutation <: MOI.AbstractVectorSet
    dimension::Int
end

MOI.dimension(s::Permutation) = s.dimension

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
    Base.convert(::Type{Partition}, s::Permutation)

`Permutation(n)` and `Partition(n, 1)` have the same flat dimension (`n`) and, for
a single truck, the same "permutation of `1:n`" semantics, so this
conversion always succeeds.
"""
Base.convert(::Type{Partition}, s::Permutation) = Partition(s.dimension, 1)

"""
    Base.convert(::Type{Permutation}, s::Partition)

Only defined when `s.num_trucks == 1`: a multi-truck `Partition` has no
`Permutation` equivalent.
"""
function Base.convert(::Type{Permutation}, s::Partition)
    if s.num_trucks != 1
        # Julia v1.10's `InexactError` only accepts `(func, T, val)`; the
        # variadic constructor that takes a message is v1.11 or later.
        throw(InexactError(:convert, Permutation, s))
    end
    return Permutation(s.num_clients)
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

"""
    RouteCompatibility(allowed)

Vector constraint on one route's column of nodes. Visit `i` may occur on the
route only when `allowed[i]` is `true`. This is typically applied to every
trip of a vehicle using the same vehicle-specific compatibility vector.
"""
struct RouteCompatibility <: MOI.AbstractVectorSet
    allowed::Vector{Bool}
end

RouteCompatibility(allowed::AbstractVector{Bool}) =
    RouteCompatibility(collect(allowed))

MOI.dimension(s::RouteCompatibility) = length(s.allowed)

Base.copy(s::RouteCompatibility) = s

"""
    RouteOrder(before, after)

Vector constraint on one route's column of nodes. Every visited node selected
by `before` must precede every visited node selected by `after`. Nodes selected
by neither vector are unconstrained relative to both classes. The two vectors
must have equal length and be disjoint.
"""
struct RouteOrder <: MOI.AbstractVectorSet
    before::Vector{Bool}
    after::Vector{Bool}
    function RouteOrder(
        before::AbstractVector{Bool},
        after::AbstractVector{Bool},
    )
        length(before) == length(after) || throw(DimensionMismatch(
            "RouteOrder vectors must have equal length; got " *
            "$(length(before)) and $(length(after)).",
        ))
        any(before .& after) && throw(ArgumentError(
            "RouteOrder classes must be disjoint.",
        ))
        return new(collect(before), collect(after))
    end
end

MOI.dimension(s::RouteOrder) = length(s.before)

Base.copy(s::RouteOrder) = s

"""
    RouteExtremities(members)

Vector constraint on one route's column of nodes. Each visited node selected
by `members` must either precede every visited unselected node or follow every
visited unselected node. Selected nodes may occur at both ends of the route.
"""
struct RouteExtremities <: MOI.AbstractVectorSet
    members::Vector{Bool}
end

RouteExtremities(members::AbstractVector{Bool}) =
    RouteExtremities(collect(members))

MOI.dimension(s::RouteExtremities) = length(s.members)

Base.copy(s::RouteExtremities) = s

"""
    RouteSchedule(travel, earliest, latest, service, depot;
                  departure_service=0)

Schedule one route while exposing its timing decisions. Apply the set to
`[route_start; route_end; visit_start; route_nodes]`, where `visit_start` and
`route_nodes` both have one entry per visit. `route_nodes` must be one column
of a [`Partition`](@ref), while the other entries are ordinary variables.

For every visited node, service starts inside its time window and no earlier
than completion of the preceding departure/service plus travel. `route_end`
is no earlier than the return to `depot`. Unvisited entries of `visit_start`
are fixed to zero. The inequalities intentionally permit waiting, which is
needed by synchronization and break constraints layered on the schedule.

Node indices are `1:length(service)` and `depot` is a 1-based row/column of
the square `travel` matrix, normally `length(service) + 1`.
"""
struct RouteSchedule{T<:Real} <: MOI.AbstractVectorSet
    travel::Matrix{T}
    earliest::Vector{T}
    latest::Vector{T}
    service::Vector{T}
    depot::Int
    departure_service::T
    function RouteSchedule(
        travel::Matrix{T},
        earliest::Vector{T},
        latest::Vector{T},
        service::Vector{T},
        depot::Integer;
        departure_service::Real = zero(T),
    ) where {T<:Real}
        n = length(service)
        length(earliest) == n == length(latest) || throw(DimensionMismatch(
            "RouteSchedule earliest, latest, and service vectors must have equal length.",
        ))
        size(travel, 1) == size(travel, 2) || throw(DimensionMismatch(
            "RouteSchedule travel matrix must be square; got $(size(travel)).",
        ))
        1 <= depot <= size(travel, 1) || throw(ArgumentError(
            "RouteSchedule depot index $depot is outside the travel matrix.",
        ))
        size(travel, 1) >= n + 1 || throw(DimensionMismatch(
            "RouteSchedule travel matrix needs at least $(n + 1) nodes; got $(size(travel, 1)).",
        ))
        return new{T}(
            travel, earliest, latest, service, Int(depot),
            convert(T, departure_service),
        )
    end
end

MOI.dimension(s::RouteSchedule) = 2 + 2length(s.service)

Base.copy(s::RouteSchedule) = s

"""
    IsEmpty(num_items)

Definition constraint applied to `[is_empty; route]`, where `route` contains
`num_items` proxy variables from one variable-length route. It defines the
binary variable `is_empty` to be one exactly when the route is empty.
"""
struct IsEmpty <: MOI.AbstractVectorSet
    num_items::Int
end

MOI.dimension(s::IsEmpty) = s.num_items + 1

"""
    SumGetIndex(values)

Definition constraint applied to `[total; route]`. It defines `total` as
`sum(values[i] for i in route)`. Solvers with native sequence expressions may
substitute `total` rather than creating a decision variable.
"""
struct SumGetIndex{T<:Real} <: MOI.AbstractVectorSet
    values::Vector{T}
end

SumGetIndex(values::AbstractVector{T}) where {T<:Real} =
    SumGetIndex{T}(collect(values))

MOI.dimension(s::SumGetIndex) = length(s.values) + 1

Base.copy(s::SumGetIndex) = s
