using Test
import MathOptInterface as MOI

function _test_mof_set_fields(actual, expected)
    @test typeof(actual) == typeof(expected)
    for field in fieldnames(typeof(expected))
        @test getfield(actual, field) == getfield(expected, field)
    end
    return
end

@testset "MOF round trip" begin
    travel = Float64[0 1 2; 3 0 4; 5 6 0]
    sets = MOI.AbstractVectorSet[
        MathOptVRP.Permutation(3),
        MathOptVRP.Partition(3, 2),
        MathOptVRP.PartitionPD(1, 1, 2),
        MathOptVRP.TimeWindows{MathOptVRP.WITHOUT_START_TIME}(
            travel,
            [0.0, 1.0, 2.0],
            [10.0, 11.0, 12.0],
            [0.5, 0.6, 0.7],
            2,
        ),
        MathOptVRP.TimeWindows{MathOptVRP.WITH_START_TIME}(
            travel,
            [0.0, 1.0, 2.0],
            [10.0, 11.0, 12.0],
            [0.5, 0.6, 0.7],
            2,
        ),
        MathOptVRP.Capacity([1.0, -1.0, 2.0], 4.0),
        MathOptVRP.CapacitatedTimeWindows(
            travel,
            [0.0, 1.0, 2.0],
            [10.0, 11.0, 12.0],
            0.5,
            1.5,
            [1.0, -1.0, 0.0],
            4.0,
        ),
        MathOptVRP.RouteCompatibility(Bool[true, false, true]),
        MathOptVRP.RouteOrder(Bool[true, false, false], Bool[false, false, true]),
        MathOptVRP.RouteExtremities(Bool[true, false, true]),
        MathOptVRP.IsEmpty(3),
        MathOptVRP.SumGetIndex([2.0, 3.0, 5.0]),
    ]

    source = MOI.Utilities.UniversalFallback(MOI.Utilities.Model{Float64}())
    next_variable = 0
    for set in sets
        variables = MOI.add_variables(source, MOI.dimension(set))
        for variable in variables
            next_variable += 1
            MOI.set(source, MOI.VariableName(), variable, "x[$next_variable]")
        end
        MOI.add_constraint(source, MOI.VectorOfVariables(variables), set)
    end

    writer = MOI.FileFormats.MOF.Model(use_nlp_block = false)
    MOI.copy_to(writer, source)
    text = sprint(write, writer)
    for set in sets
        @test occursin(MOI.FileFormats.MOF.head_name(typeof(set)), text)
    end

    destination = MOI.FileFormats.MOF.Model(use_nlp_block = false)
    read!(IOBuffer(text), destination)
    for expected in sets
        S = typeof(expected)
        indices =
            MOI.get(destination, MOI.ListOfConstraintIndices{MOI.VectorOfVariables,S}())
        @test length(indices) == 1
        actual = MOI.get(destination, MOI.ConstraintSet(), only(indices))
        _test_mof_set_fields(actual, expected)
    end
end
