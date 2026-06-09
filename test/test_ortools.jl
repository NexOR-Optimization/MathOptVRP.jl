using JuMP
using MathOptVRP
using ORTools  # triggers loading of `MathOptVRPORToolsExt`
using Test

# Reach into the extension's `Optimizer.routes` field directly; the
# extension stored routes per truck during `MOI.optimize!`.
function _ortools_read_routes(model, _nodes)
    return JuMP.unsafe_backend(model).routes
end

@testset "MathOptVRP.test_vrp (ORTools/CP-SAT)" begin
    MathOptVRP.test_vrp(
        MathOptVRP.ortools_optimizer;
        read_routes = _ortools_read_routes,
    )
end
