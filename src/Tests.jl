# Solver-independent test entry points. The implementations live in the
# `MathOptVRPTestExt` package extension and are loaded automatically when
# `using Test` is in scope. The signature of every `test_<variant>` is
# `test_<variant>(optimizer_factory; read_routes, kwargs...)` where
# `read_routes(model, nodes) -> Vector{Vector{Int}}` recovers each truck's
# 0-indexed visited-customer sequence from the solved model and the
# matrix of decision variables (this part is necessarily solver-specific).
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
