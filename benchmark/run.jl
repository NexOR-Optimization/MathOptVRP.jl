# Solve a random delivery instance with Hexaly or MaxiCP:
#
#     julia --project=benchmark benchmark/run.jl SOLVER [NODES] [VEHICLES] [SECONDS] [SEED]
#
# where SOLVER is `hexaly` or `maxicp`.

include(joinpath(@__DIR__, "instance.jl"))
include(joinpath(@__DIR__, "model.jl"))

solver = ARGS[1]
num_nodes = parse(Int, get(ARGS, 2, "32"))
num_vehicles = parse(Int, get(ARGS, 3, "4"))
seconds = parse(Int, get(ARGS, 4, "60"))
seed = parse(Int, get(ARGS, 5, "1"))

if solver == "hexaly"
    import Hexaly
    optimizer = Hexaly.Optimizer
elseif solver == "maxicp"
    import MaxiCP
    optimizer = MaxiCP.Optimizer
else
    error("Unknown solver $solver, use `hexaly` or `maxicp`.")
end

instance = random_instance(num_nodes, num_vehicles)
model = delivery_model(instance, optimizer)
set_time_limit_sec(model, seconds)
set_attribute(model, "seed", seed)
set_silent(model)
optimize!(model)
println("solver: ", solver_name(model))
println("termination: ", termination_status(model))
if primal_status(model) == FEASIBLE_POINT
    println("objective: ", round(Int, objective_value(model)))
    trips = count(<(0.5), value.(model[:is_empty]))
    println("trips: ", trips)
end
