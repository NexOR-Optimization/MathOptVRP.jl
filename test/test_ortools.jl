using MathOptVRP
using ORTools  # triggers loading of `MathOptVRPORToolsExt`
import ORTools_jll  # provides the OR-Tools binaries to `ORTools`
using Test

@testset "MathOptVRP.test_vrp (ORTools/CP-SAT)" begin
    MathOptVRP.Tests.test_vrp(MathOptVRP.ortools_optimizer)
end

@testset "MathOptVRP.test_vrppd (ORTools/CP-SAT)" begin
    MathOptVRP.Tests.test_vrppd(
        MathOptVRP.ortools_optimizer;
        read_routes = _read_routes,
    )
end
