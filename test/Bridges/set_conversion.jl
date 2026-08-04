module TestSetConversion

using Test

import MathOptInterface as MOI
import MathOptVRP

function runtests()
    for name in names(@__MODULE__; all = true)
        if startswith("$(name)", "test_")
            @testset "$(name)" begin
                getfield(@__MODULE__, name)()
            end
        end
    end
    return
end

function test_runtests_ListToPartition()
    MOI.Bridges.runtests(
        MathOptVRP.ListToPartitionBridge,
        model -> MOI.add_constrained_variables(model, MathOptVRP.List(3)),
        model ->
            MOI.add_constrained_variables(model, MathOptVRP.Partition(3, 1)),
    )
    return
end

function _set_objective(model, x)
    MOI.set(model, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    f = sum(Float64(i) * x[i] for i in eachindex(x))
    MOI.set(model, MOI.ObjectiveFunction{typeof(f)}(), f)
    return
end

# The variables are mapped through the bridge with the identity so the
# objective must come out of the bridged model unchanged.
function test_runtests_ListToPartition_objective()
    for n in (1, 4)
        MOI.Bridges.runtests(
            MathOptVRP.ListToPartitionBridge,
            model -> begin
                x, _ = MOI.add_constrained_variables(model, MathOptVRP.List(n))
                _set_objective(model, x)
            end,
            model -> begin
                x, _ = MOI.add_constrained_variables(
                    model,
                    MathOptVRP.Partition(n, 1),
                )
                _set_objective(model, x)
            end,
        )
    end
    return
end

function test_supports_constrained_variable()
    BT = MathOptVRP.ListToPartitionBridge{Float64}
    @test MOI.Bridges.Variable.supports_constrained_variable(
        BT,
        MathOptVRP.List,
    )
    @test !MOI.Bridges.Variable.supports_constrained_variable(
        BT,
        MathOptVRP.Partition,
    )
    @test MOI.Bridges.added_constrained_variable_types(BT) ==
          [(MathOptVRP.Partition,)]
    @test isempty(MOI.Bridges.added_constraint_types(BT))
    return
end

# `map_set` (bridge → user) and `inverse_map_set` (user → bridge) are the
# `Base.convert` methods of `src/sets.jl`.
function test_map_set()
    BT = MathOptVRP.ListToPartitionBridge{Float64}
    @test MOI.Bridges.inverse_map_set(BT, MathOptVRP.List(3)) ==
          MathOptVRP.Partition(3, 1)
    @test MOI.Bridges.map_set(BT, MathOptVRP.Partition(3, 1)) ==
          MathOptVRP.List(3)
    return
end

end # module TestSetConversion

TestSetConversion.runtests()
