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

@enum StartTimeInclusion WITHOUT_START_TIME WITH_START_TIME

"""
    TimeWindows{W}(travel, earliest, latest, service[, num_items])

Schedule the logical node sequence `[first_node; route; last_node]`, where
`route` is a variable-length sequence with `num_items` proxy entries and all
node values are one-based indices into `travel`, `earliest`, `latest`, and
`service`. Every occurrence, including the fixed first and last nodes, obeys
its time window and participates in travel and service propagation.

For `W == WITHOUT_START_TIME`, apply the set to
`[route_end; first_node; route; last_node]`. The route belongs to the set when
there exists a feasible schedule and `route_end` is no earlier than completion
of the last occurrence.

For `W == WITH_START_TIME`, apply it to
`[start_time; route_end; first_node; route; last_node]`, where `start_time` has
one entry for every node index. It is zero for an absent node and otherwise is
the service start of that node's first occurrence. Thus, if the first and last
nodes are equal, its exposed start time is the departure occurrence;
`route_end` still describes completion of the final occurrence.

The two variants have the same scheduling semantics after projecting out
`start_time`. The variant without exposed times lets solvers use compact
derived expressions. For example, Hexaly.jl uses a recursive array expression
without creating time decisions. The exposed variant may require actual time
variables because other constraints can reference and delay the schedule.
"""
struct TimeWindows{W,T<:Real} <: MOI.AbstractVectorSet
    travel::Matrix{T}
    earliest::Vector{T}
    latest::Vector{T}
    service::Vector{T}
    num_items::Int
    function TimeWindows{W}(
        travel::Matrix{T},
        earliest::Vector{T},
        latest::Vector{T},
        service::Vector{T},
        num_items::Integer = size(travel, 1) - 1,
    ) where {W,T<:Real}
        n = size(travel, 1)
        size(travel, 2) == n || throw(DimensionMismatch(
            "TimeWindows travel matrix must be square; got $(size(travel)).",
        ))
        length(earliest) == n == length(latest) == length(service) ||
            throw(DimensionMismatch(
                "TimeWindows node-data vectors must have one entry per travel-matrix node.",
            ))
        num_items >= 0 || throw(ArgumentError("num_items must be nonnegative."))
        return new{W,T}(travel, earliest, latest, service, Int(num_items))
    end
end

function MOI.dimension(s::TimeWindows{WITHOUT_START_TIME})
    return s.num_items + 3
end
function MOI.dimension(s::TimeWindows{WITH_START_TIME})
    return length(s.service) + s.num_items + 3
end

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
