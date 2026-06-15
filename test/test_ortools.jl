using JuMP
using MathOptVRP
using ORTools  # triggers loading of `MathOptVRPORToolsExt`
using Test

# The OR-Tools extension projects the solved routes back onto the partition
# variables, so each `nodes[i, j]` carries either the `i`-th customer visited
# by truck `j` or the depot's location index (used as a "no customer"
# sentinel). That lets us recover routes via `JuMP.value` without reaching
# into the optimizer's internal state.
function _read_routes(_model, nodes)
    n_clients, n_trucks = size(nodes)
    routes = [Int[] for _ = 1:n_trucks]
    for j = 1:n_trucks, i = 1:n_clients
        val = round(Int, JuMP.value(nodes[i, j]))
        0 <= val < n_clients && push!(routes[j], val)
    end
    return routes
end

@testset "MathOptVRP.test_vrp (ORTools/CP-SAT)" begin
    MathOptVRP.Tests.test_vrp(
        MathOptVRP.ortools_optimizer;
        read_routes = _read_routes,
    )
end
