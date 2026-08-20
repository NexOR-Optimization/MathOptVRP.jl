using JuMP
using MathOptVRP
using Test
import MathOptInterface as MOI

@testset "MathOptVRP" begin
    @testset "Permutation" begin
        s = MathOptVRP.Permutation(5)
        @test s.dimension == 5
        @test MOI.dimension(s) == 5
    end

    @testset "Partition" begin
        s = MathOptVRP.Partition(4, 3)
        @test s.num_clients == 4
        @test s.num_trucks == 3
        @test MOI.dimension(s) == 12

        # JuMP `@variable(model, x[1:nc, 1:nt] in Partition(nc, nt))` goes
        # through `build_variable(::Function, ::Matrix, ::Partition)`.
        model = Model()
        @variable(model, x[1:4, 1:3] in MathOptVRP.Partition(4, 3))
        @test size(x) == (4, 3)
        @test eltype(x) === VariableRef
        # Wrong shape errors via the error function passed by the macro.
        @test_throws ErrorException @variable(
            model,
            y[1:2, 1:3] in MathOptVRP.Partition(4, 3)
        )
    end

    @testset "Permutation <=> Partition conversion" begin
        @test convert(MathOptVRP.Partition, MathOptVRP.Permutation(5)) ==
              MathOptVRP.Partition(5, 1)
        @test convert(MathOptVRP.Permutation, MathOptVRP.Partition(5, 1)) ==
              MathOptVRP.Permutation(5)
        @test_throws InexactError convert(
            MathOptVRP.Permutation,
            MathOptVRP.Partition(5, 2),
        )
    end

    @testset "PartitionPD" begin
        s = MathOptVRP.PartitionPD(2, 3, 4)  # 2 services + 2*3 pd = 8 rows, 4 trucks
        @test s.num_services == 2
        @test s.num_pickup_deliveries == 3
        @test s.num_trucks == 4
        @test MathOptVRP._pd_n_total(s) == 8
        @test MOI.dimension(s) == 32

        model = Model()
        @variable(model, x[1:8, 1:4] in MathOptVRP.PartitionPD(2, 3, 4))
        @test size(x) == (8, 4)
        @test_throws ErrorException @variable(
            model,
            y[1:5, 1:4] in MathOptVRP.PartitionPD(2, 3, 4)
        )
    end

    @testset "TimeWindows" begin
        travel = [0 1 2; 1 0 3; 2 3 0]
        earliest = [0, 0]
        latest = [10, 10]
        s = MathOptVRP.TimeWindows(travel, earliest, latest, 1)
        @test s.travel === travel
        @test s.earliest === earliest
        @test s.latest === latest
        @test s.service == 1
        # `[t; depot_start; nodes...; depot_end]` → 1 + 1 + n + 1 = n + 3.
        @test MOI.dimension(s) == length(earliest) + 3
        # `Base.copy` returns the same logically-immutable set.
        @test Base.copy(s) === s
    end

    @testset "Capacity" begin
        s = MathOptVRP.Capacity([1, -1, 2, -2], 5)
        @test s.delta == [1, -1, 2, -2]
        @test s.capacity == 5
        @test MOI.dimension(s) == 4
        @test Base.copy(s) === s
    end

    @testset "CapacitatedTimeWindows" begin
        travel = zeros(Int, 4, 4)
        s = MathOptVRP.CapacitatedTimeWindows(
            travel,
            [0, 0, 0],
            [10, 10, 10],
            2,
            1,
            [1, -1, 0],
            5,
        )
        @test s.fixed_time == 2
        @test s.slope == 1
        @test s.capacity == 5
        @test MOI.dimension(s) == 3 + 3
        @test Base.copy(s) === s
    end

    @testset "sum_distances / op_sum_distances" begin
        # The bare stub has no methods.
        @test_throws MethodError MathOptVRP.sum_distances(zeros(Int, 2, 2), [1, 2])

        # `op_sum_distances` is a `NonlinearOperator` over `:sum_distances`.
        @test MathOptVRP.op_sum_distances isa JuMP.NonlinearOperator
        @test MathOptVRP.op_sum_distances.head === :sum_distances

        # When called with a JuMP-tainted argument it builds a
        # `GenericNonlinearExpr` rather than calling the stub `func`.
        model = Model()
        @variable(model, nodes[1:3] in MathOptVRP.Permutation(3))
        dist = [0 1 2; 1 0 3; 2 3 0]
        expr = MathOptVRP.op_sum_distances(dist, nodes)
        @test expr isa JuMP.GenericNonlinearExpr
        @test expr.head === :sum_distances
        @test expr.args[1] === dist
        @test expr.args[2] === nodes
    end

    include("Bridges/PermutationToPartitionBridge.jl")

    include("test_ortools.jl")

    @testset "JuMP overrides (type piracy)" begin
        # Plain numeric arrays: now treated as real, so they pass the
        # `_throw_if_not_real` check when used as leaves of nonlinear expressions.
        @test JuMP._is_real(Int[1, 2, 3])
        @test JuMP._is_real(zeros(Int, 2, 2))
        @test JuMP._is_real([1.0, 2.0])

        # Arrays of JuMP scalars (VariableRef, AffExpr, ...): also real.
        model = Model()
        @variable(model, w[1:3])
        @test JuMP._is_real(w)
        @test JuMP._is_real([w[1] + 1, w[2]])    # Vector{AffExpr}

        # `moi_function(::Array)` broadcasts elementwise. JuMP already has
        # specialised methods for `Vector{VariableRef}` etc., so the pirated
        # method only fires for arrays JuMP has no native method for —
        # constant matrices and `Vector{Any}` (the shape produced by
        # `vcat(::Int, vars, ::Int)`).
        @test JuMP.moi_function([1 2; 3 4]) == [1 2; 3 4]
        mixed = Any[1, w[1], 2]
        moi_mixed = JuMP.moi_function(mixed)
        @test length(moi_mixed) == 3
        @test moi_mixed[1] == 1
        @test moi_mixed[2] isa MOI.VariableIndex
        @test moi_mixed[3] == 2

        # `variable_ref_type` picks the variable-ref type out of an array
        # of JuMP scalars, so `NonlinearOperator` dispatch sees the array
        # as JuMP-tainted.
        @test JuMP.variable_ref_type(JuMP.GenericNonlinearExpr, w) === VariableRef
        @test JuMP.variable_ref_type(
            JuMP.GenericNonlinearExpr,
            [w[1] + 1, w[2]],
        ) === VariableRef
    end
end
