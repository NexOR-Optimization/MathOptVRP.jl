# Copyright (c) 2025: Benoît Legat and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# Experimental MathOptFormat serialization. MOF's generic set writer serializes
# set fields after dispatching through head_name. The matching set_to_moi
# methods reconstruct the MathOptVRP sets on read.

const _MOF = MOI.FileFormats.MOF

_MOF.head_name(::Type{Permutation}) = "MathOptVRPPermutation"
_MOF.head_name(::Type{Partition}) = "MathOptVRPPartition"
_MOF.head_name(::Type{PartitionPD}) = "MathOptVRPPartitionPD"
function _MOF.head_name(::Type{<:TimeWindows{WITHOUT_START_TIME}})
    return "MathOptVRPTimeWindowsWithoutStartTime"
end
function _MOF.head_name(::Type{<:TimeWindows{WITH_START_TIME}})
    return "MathOptVRPTimeWindowsWithStartTime"
end
_MOF.head_name(::Type{<:Capacity}) = "MathOptVRPCapacity"
function _MOF.head_name(::Type{<:CapacitatedTimeWindows})
    return "MathOptVRPCapacitatedTimeWindows"
end
function _MOF.head_name(::Type{RouteCompatibility})
    return "MathOptVRPRouteCompatibility"
end
_MOF.head_name(::Type{RouteOrder}) = "MathOptVRPRouteOrder"
_MOF.head_name(::Type{RouteExtremities}) = "MathOptVRPRouteExtremities"
_MOF.head_name(::Type{IsEmpty}) = "MathOptVRPIsEmpty"
_MOF.head_name(::Type{<:SumGetIndex}) = "MathOptVRPSumGetIndex"

function _MOF.set_to_moi(::Val{:MathOptVRPPermutation}, ::Type{T}, object::Dict) where {T}
    return Permutation(Int(object["dimension"]))
end

function _MOF.set_to_moi(::Val{:MathOptVRPPartition}, ::Type{T}, object::Dict) where {T}
    return Partition(Int(object["num_clients"]), Int(object["num_trucks"]))
end

function _MOF.set_to_moi(::Val{:MathOptVRPPartitionPD}, ::Type{T}, object::Dict) where {T}
    return PartitionPD(
        Int(object["num_services"]),
        Int(object["num_pickup_deliveries"]),
        Int(object["num_trucks"]),
    )
end

function _mof_matrix(::Type{T}, columns::Vector) where {T}
    return convert(Matrix{T}, hcat(columns...))
end

function _time_windows_from_mof(::Type{T}, ::Val{W}, object::Dict) where {T,W}
    return TimeWindows{W}(
        _mof_matrix(T, object["travel"]),
        convert(Vector{T}, object["earliest"]),
        convert(Vector{T}, object["latest"]),
        convert(Vector{T}, object["service"]),
        Int(object["num_items"]),
    )
end

function _MOF.set_to_moi(
    ::Val{:MathOptVRPTimeWindowsWithoutStartTime},
    ::Type{T},
    object::Dict,
) where {T}
    return _time_windows_from_mof(T, Val(WITHOUT_START_TIME), object)
end

function _MOF.set_to_moi(
    ::Val{:MathOptVRPTimeWindowsWithStartTime},
    ::Type{T},
    object::Dict,
) where {T}
    return _time_windows_from_mof(T, Val(WITH_START_TIME), object)
end

function _MOF.set_to_moi(::Val{:MathOptVRPCapacity}, ::Type{T}, object::Dict) where {T}
    return Capacity(convert(Vector{T}, object["delta"]), convert(T, object["capacity"]))
end

function _MOF.set_to_moi(
    ::Val{:MathOptVRPCapacitatedTimeWindows},
    ::Type{T},
    object::Dict,
) where {T}
    return CapacitatedTimeWindows(
        _mof_matrix(T, object["travel"]),
        convert(Vector{T}, object["earliest"]),
        convert(Vector{T}, object["latest"]),
        convert(T, object["fixed_time"]),
        convert(T, object["slope"]),
        convert(Vector{T}, object["delta"]),
        convert(T, object["capacity"]),
    )
end

function _MOF.set_to_moi(
    ::Val{:MathOptVRPRouteCompatibility},
    ::Type{T},
    object::Dict,
) where {T}
    return RouteCompatibility(convert(Vector{Bool}, object["allowed"]))
end

function _MOF.set_to_moi(::Val{:MathOptVRPRouteOrder}, ::Type{T}, object::Dict) where {T}
    return RouteOrder(
        convert(Vector{Bool}, object["before"]),
        convert(Vector{Bool}, object["after"]),
    )
end

function _MOF.set_to_moi(
    ::Val{:MathOptVRPRouteExtremities},
    ::Type{T},
    object::Dict,
) where {T}
    return RouteExtremities(convert(Vector{Bool}, object["members"]))
end

function _MOF.set_to_moi(::Val{:MathOptVRPIsEmpty}, ::Type{T}, object::Dict) where {T}
    return IsEmpty(Int(object["num_items"]))
end

function _MOF.set_to_moi(::Val{:MathOptVRPSumGetIndex}, ::Type{T}, object::Dict) where {T}
    return SumGetIndex(convert(Vector{T}, object["values"]))
end
