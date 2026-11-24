# Delivery benchmark

A solver-independent delivery model written with JuMP and MathOptVRP sets,
solved with [Hexaly](https://github.com/NexOR-Optimization/Hexaly.jl) or
[MaxiCP](https://github.com/NexOR-Optimization/MaxiCP.jl) on random instances.

`instance.jl` generates an instance: customers with time windows, service
times, volumes and weights, served by vans, which can make several trips in
their day by coming back to the depot to reload, and trucks, which make a
single trip. `model.jl` builds the model:

* `Partition` assigns each customer to one trip;
* `VehicleSchedule` schedules the trips of each vehicle, with the reload
  between trips and a break;
* `SumGetIndex` sums the volume, weight and pickups of each trip, bounded by
  linear constraints;
* `SumDistances` gives the length of each trip, priced in the objective;
* `RouteOrder` visits pickups after deliveries and priority customers first;
* `RouteExtremities` puts bulky items first or last in their trip;
* `RouteCompatibility` restricts bulky items to trucks;
* `IsEmpty` with an indicator constraint makes a vehicle leaving before the
  nominal time of the break take it.

The objective is the fixed cost of the used vehicles, plus their working time
and distance.

## Running

Hexaly needs a license; MaxiCP needs a JDK and, because of JavaCall, a single
Julia thread and `JULIA_COPY_STACKS=1`:

```sh
julia --project=benchmark -e 'import Pkg; Pkg.instantiate(); Pkg.build()'
julia --project=benchmark benchmark/run.jl hexaly 32 4 60 1
JULIA_NUM_THREADS=1 JULIA_COPY_STACKS=1 julia --project=benchmark benchmark/run.jl maxicp 32 4 60 1
```

The arguments are the solver, the number of customers, the number of vehicles,
the time limit in seconds and the seed of the solver.
