using MathOptVRP
using ORTools  # triggers loading of `MathOptVRPORToolsExt`
import ORTools_jll  # provides the OR-Tools binaries to `ORTools`
using Test

@testset "$test" for test in [
    MathOptVRP.Tests.test_tsp,
    MathOptVRP.Tests.test_vrp,
    MathOptVRP.Tests.test_vrppd,
]
    test(MathOptVRP.ortools_optimizer)
end
