# `SetConversionBridge{T,S2,S1}` lets a solver backend that only
# implements `MOI.add_constrained_variables(model, ::S2)` transparently
# also support `@variable(model, x[1:n] in S1(...))`, for any pair of
# vector sets connected by a linear map (`S2` is the set actually added to
# the underlying model, `S1` is the set exposed to the user). A new `S1 => S2` pair
# only has to supply `map_set` / `inverse_map_set` (how the sets relate — generic here, via
# `Base.convert`) and `map_function` / `inverse_map_function` (how the
# *variables* relate — necessarily specific to the pair, since `S1` and
# `S2` need not even share a dimension).
#

"""
    SetConversionBridge{T,S2,S1} <: MOI.Bridges.Variable.SetMapBridge{T,S2,S1}

Bridges `MOI.VectorOfVariables`-in-`S1` (what the user asks for) to
`MOI.VectorOfVariables`-in-`S2` (what gets added to the underlying model).
"""
struct SetConversionBridge{T,S2,S1} <: MOI.Bridges.Variable.SetMapBridge{T,S2,S1}
    variables::Vector{MOI.VariableIndex}
    constraint::MOI.ConstraintIndex{MOI.VectorOfVariables,S2}
end

function MOI.Bridges.map_set(
    ::Type{<:SetConversionBridge{T,S2,S1}},
    set::S2,
) where {T,S2,S1}
    return convert(S1, set)
end

function MOI.Bridges.inverse_map_set(
    ::Type{<:SetConversionBridge{T,S2,S1}},
    set::S1,
) where {T,S2,S1}
    return convert(S2, set)
end

MOI.Bridges.map_function(::Type{<:SetConversionBridge}, func) = func

MOI.Bridges.inverse_map_function(::Type{<:SetConversionBridge}, func) = func

# The map is the identity so it is its own adjoint; without these, setting
# `MOI.ConstraintDualStart` (or getting `MOI.ConstraintDual`) on a bridged
# constraint errors.
MOI.Bridges.adjoint_map_function(::Type{<:SetConversionBridge}, func) = func

function MOI.Bridges.inverse_adjoint_map_function(
    ::Type{<:SetConversionBridge},
    func,
)
    return func
end

"""
    ListToPartitionBridge{T} = SetConversionBridge{T,Partition,List}

Bridges `List(n)` to `Partition(n, 1)`, for backends that support
`Partition` but not `List`.
"""
const ListToPartitionBridge{T} = SetConversionBridge{T,Partition,List}
