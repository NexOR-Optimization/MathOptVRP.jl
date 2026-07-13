# `ListToPartitionBridge` lets a solver backend that only implements
# `MOI.add_constrained_variables(model, ::Partition)` transparently also
# support `@variable(model, x[1:n] in MathOptVRP.List(n))`.
#
# A closed tour over `n` nodes has no distinguished depot, any rotation of
# the cycle is an equivalent tour. This bridge exploits that: it pins the
# first `List` variable to the constant `0` (so every bridged tour is
# reported as "starting" at node `0`), and represents the remaining `n - 1`
# nodes as one column of a `Partition(n - 1, 1)` (see the `Base.convert`
# methods in `sets.jl`, which this bridge reuses directly).

"""
    ListToPartitionBridge{T} <: MOI.Bridges.Variable.AbstractBridge

Bridges `MOI.VectorOfVariables`-in-`List(n)` to
`MOI.VectorOfVariables`-in-`Partition(n - 1, 1)`, for backends that
support `Partition` but not `List`.
"""
struct ListToPartitionBridge{T} <: MOI.Bridges.Variable.AbstractBridge
    variables::Vector{MOI.VariableIndex}
    constraint::MOI.ConstraintIndex{MOI.VectorOfVariables,Partition}
end

function MOI.Bridges.Variable.bridge_constrained_variable(
    ::Type{ListToPartitionBridge{T}},
    model::MOI.ModelLike,
    set::List,
) where {T}
    variables, constraint = MOI.add_constrained_variables(model, convert(Partition, set))
    return ListToPartitionBridge{T}(variables, constraint)
end

function MOI.Bridges.Variable.supports_constrained_variable(
    ::Type{<:ListToPartitionBridge},
    ::Type{List},
)
    return true
end

function MOI.Bridges.added_constrained_variable_types(::Type{<:ListToPartitionBridge})
    return Tuple{Type}[(Partition,)]
end

MOI.Bridges.added_constraint_types(::Type{<:ListToPartitionBridge}) = Tuple{Type,Type}[]

# Attributes, bridge acting as a (partial) model: only the `n - 1` real
# `Partition` variables exist in the underlying model; index `1` (the
# pinned depot) has no underlying variable, mirroring `ZerosBridge`.
MOI.get(bridge::ListToPartitionBridge, ::MOI.NumberOfVariables)::Int64 =
    length(bridge.variables)

MOI.get(bridge::ListToPartitionBridge, ::MOI.ListOfVariableIndices) =
    copy(bridge.variables)

function MOI.delete(model::MOI.ModelLike, bridge::ListToPartitionBridge)
    MOI.delete(model, bridge.variables)
    return
end

function MOI.get(::MOI.ModelLike, ::MOI.ConstraintSet, bridge::ListToPartitionBridge)
    return convert(List, Partition(length(bridge.variables), 1))
end

function MOI.Bridges.bridged_function(
    bridge::ListToPartitionBridge{T},
    i::MOI.Bridges.IndexInVector,
) where {T}
    i.value == 1 && return zero(MOI.ScalarAffineFunction{T})
    return convert(MOI.ScalarAffineFunction{T}, bridge.variables[i.value-1])
end

function MOI.Bridges.Variable.unbridged_map(
    ::ListToPartitionBridge,
    ::MOI.VariableIndex,
    ::MOI.Bridges.IndexInVector,
)
    # Not recoverable (the pinned depot slot has no underlying variable),
    # matching `ZerosBridge`'s convention.
    return nothing
end