module Bridges

import MathOptInterface as MOI

using ..MathOptVRP: Partition, Permutation

include("PermutationToPartitionBridge.jl")

const _ALL_BRIDGE_TYPES = Any[PermutationToPartitionBridge]

"""
    add_all_bridges(model::MOI.ModelLike, ::Type{T} = Float64)

Add all `MathOptVRP` bridges to `model`. The model is typically a
[`MOI.Bridges.LazyBridgeOptimizer`](@ref), allowing a backend that supports
[`Partition`](@ref), but not [`Permutation`](@ref), to accept permutation
variables.
"""
function add_all_bridges(model::MOI.ModelLike, ::Type{T} = Float64) where {T}
    for bridge_type in _ALL_BRIDGE_TYPES
        MOI.Bridges.add_bridge(model, bridge_type{T})
    end
    return
end

end # module Bridges
