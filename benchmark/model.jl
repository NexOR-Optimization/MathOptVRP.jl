using JuMP
import MathOptVRP

# The delivery model of `instance`, see `random_instance`, written only with
# JuMP and MathOptVRP sets, so that any solver supporting these sets can solve
# it. Each column of `routes` is one trip; `trips_of[v]` lists the trips of
# vehicle `v`.
function delivery_model(instance, optimizer)
    n = length(instance.service)
    vehicles = instance.vehicles
    nv = length(vehicles)
    trips_of = let r = 0
        [[(r += 1) for _ in 1:vehicles[v].num_trips] for v in 1:nv]
    end
    nr = sum(length, trips_of)
    depot = instance.depot
    model = Model(optimizer)
    @variable(model, routes[1:n, 1:nr] in MathOptVRP.Partition(n, nr))
    @variable(model, is_empty[1:nr], Bin)
    @variable(model, route_volume[1:nr], Int)
    @variable(model, route_weight[1:nr], Int)
    @variable(model, route_pickups[1:nr], Int)
    @variable(model, route_distance[1:nr], Int)
    @variable(model, trip_start[1:nr], Int)
    @variable(model, trip_end[1:nr], Int)
    @variable(model, visit_start[1:n, 1:nv], Int)
    @variable(model, break_taken[1:nv], Bin)
    @variable(model, break_start[1:nv], Int)
    for r in 1:nr
        route = routes[:, r]
        @constraint(model, [is_empty[r]; route] in MathOptVRP.IsEmpty(n))
        @constraint(
            model,
            [route_volume[r]; route] in MathOptVRP.SumGetIndex(instance.volume),
        )
        @constraint(
            model,
            [route_weight[r]; route] in MathOptVRP.SumGetIndex(instance.weight),
        )
        @constraint(
            model,
            [route_pickups[r]; route] in
            MathOptVRP.SumGetIndex(Int.(instance.pickup)),
        )
        @constraint(
            model,
            [route_distance[r]; route] in
            MathOptVRP.SumDistances(instance.distance, depot, depot, n),
        )
        @constraint(
            model,
            route in MathOptVRP.RouteOrder(.!instance.pickup, instance.pickup),
        )
        @constraint(
            model,
            route in
            MathOptVRP.RouteOrder(instance.priority, .!instance.priority),
        )
        @constraint(model, route in MathOptVRP.RouteExtremities(instance.bulky))
    end
    used = [1 - is_empty[first(trips_of[v])] for v in 1:nv]
    day_start = [trip_start[first(trips_of[v])] for v in 1:nv]
    day_end = [trip_end[last(trips_of[v])] for v in 1:nv]
    for v in 1:nv
        vehicle = vehicles[v]
        rs = trips_of[v]
        @constraint(
            model,
            [
                trip_start[rs]
                trip_end[rs]
                break_taken[v]
                break_start[v]
                visit_start[:, v]
                vec(routes[:, rs])
            ] in MathOptVRP.VehicleSchedule(
                instance.travel,
                depot,
                instance.earliest,
                instance.latest,
                instance.service;
                day = vehicle.day,
                num_trips = vehicle.num_trips,
                first_service = vehicle.first_service,
                reload_fixed = vehicle.reload_fixed,
                reload = instance.reload,
                break_length = vehicle.break_length,
                break_window = vehicle.break_window,
            )
        )
        @constraint(model, day_end[v] - day_start[v] <= vehicle.max_amplitude)
        # A used vehicle leaving no later than the nominal time of the break
        # takes it.
        nominal = vehicle.nominal_break
        @constraint(
            model,
            !break_taken[v] => {
                day_start[v] + (nominal + 1) * is_empty[first(rs)] >=
                nominal + 1
            }
        )
        for (t, r) in enumerate(rs)
            @constraint(
                model,
                routes[:, r] in
                MathOptVRP.RouteCompatibility(instance.compatibility[:, v]),
            )
            @constraint(model, route_volume[r] <= vehicle.capacity)
            @constraint(model, route_weight[r] <= vehicle.max_weight)
            @constraint(model, route_pickups[r] <= vehicle.max_pickups)
            # Trip `t` is used only if trip `t - 1` is.
            if t > 1
                @constraint(model, is_empty[r-1] <= is_empty[r])
            end
        end
    end
    @objective(
        model,
        Min,
        sum(
            vehicles[v].fixed_cost * used[v] +
            vehicles[v].cost_per_second * (day_end[v] - day_start[v]) +
            sum(vehicles[v].cost_per_meter * route_distance[r] for r in trips_of[v]) for
            v in 1:nv
        ),
    )
    return model
end
