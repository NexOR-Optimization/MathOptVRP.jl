# Solver-independent test entry points. The implementations live in the
# `MathOptVRPTestExt` package extension and are loaded automatically when
# `using Test` is in scope. The signature of every `test_<variant>` is
# `test_<variant>(optimizer_factory; kwargs...)`: the routes are recovered
# from the value of the `Partition` / `PartitionPD` variables, whose
# columns are `0`-padded once a truck's route ends, so nothing here is
# solver-specific.
#
# The submodule is named `Tests` (plural) so it does not shadow stdlib
# `Test` for callers that `using Test` to trigger the extension.
module Tests
function test_tsp end
function test_vrp end
function test_vrppd end
function test_vrptw end
function test_cvrp end
function test_cvrptw end
function runtests end
end # module Tests
