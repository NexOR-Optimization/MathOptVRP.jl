module MathOptVRPTestExt

import MathOptInterface as MOI
import MathOptVRP
import JuMP
using Random
using Test

# ── Instance builders ─────────────────────────────────────────────────
# Each builder is deterministic given the keyword args. Node identities
# are 1-based, so a node is exactly its row/column in the distance matrix
# `M`: customers are `1:n_customers` and the depot is the last node,
# `n_customers + 1`. `dist_depot` / `dist_matrix` / `earliest` / `latest`
# / `delta` are indexed by customer.

function _tsp_instance(; seed::Int, n::Int)
    Random.seed!(seed)
    x = rand(n)
    y = rand(n)
    d(i, j) = round(Int, 100hypot(x[i] - x[j], y[i] - y[j]))
    dist = [d(i, j) for i = 1:n, j = 1:n]
    return (; n, dist)
end

function _vrp_instance(; seed::Int, n_customers::Int, n_trucks::Int)
    Random.seed!(seed)
    cx = rand(n_customers + 1)
    cy = rand(n_customers + 1)
    d(i, j) = round(Int, 100hypot(cx[i] - cx[j], cy[i] - cy[j]))
    dist_depot = [d(1, c + 1) for c = 1:n_customers]
    dist_matrix = [d(i + 1, j + 1) for i = 1:n_customers, j = 1:n_customers]
    depot = n_customers + 1
    M = zeros(Int, n_customers + 1, n_customers + 1)
    M[1:n_customers, 1:n_customers] .= dist_matrix
    M[n_customers+1, 1:n_customers] .= dist_depot
    M[1:n_customers, n_customers+1] .= dist_depot
    return (; n_customers, n_trucks, dist_depot, dist_matrix, M, depot)
end

function _vrppd_instance(;
    seed::Int,
    num_services::Int,
    num_pickup_deliveries::Int,
    num_trucks::Int,
)
    Random.seed!(seed)
    n_total = num_services + 2 * num_pickup_deliveries
    cx = rand(n_total + 1)
    cy = rand(n_total + 1)
    d(i, j) = round(Int, 100hypot(cx[i] - cx[j], cy[i] - cy[j]))
    dist_depot = [d(1, c + 1) for c = 1:n_total]
    dist_matrix = [d(i + 1, j + 1) for i = 1:n_total, j = 1:n_total]
    depot = n_total + 1
    M = zeros(Int, n_total + 1, n_total + 1)
    M[1:n_total, 1:n_total] .= dist_matrix
    M[n_total+1, 1:n_total] .= dist_depot
    M[1:n_total, n_total+1] .= dist_depot
    return (;
        num_services,
        num_pickup_deliveries,
        num_trucks,
        n_total,
        dist_depot,
        dist_matrix,
        M,
        depot,
    )
end

function _vrptw_instance(; seed::Int, n_customers::Int, n_trucks::Int)
    Random.seed!(seed)
    cx = rand(n_customers + 1)
    cy = rand(n_customers + 1)
    d(i, j) = round(Int, 100hypot(cx[i] - cx[j], cy[i] - cy[j]))
    dist_depot = [d(1, c + 1) for c = 1:n_customers]
    dist_matrix = [d(i + 1, j + 1) for i = 1:n_customers, j = 1:n_customers]
    service_time = 5
    earliest = [rand(0:30) for _ = 1:n_customers]
    latest = [earliest[c] + 300 for c = 1:n_customers]
    depot = n_customers + 1
    M = zeros(Int, n_customers + 1, n_customers + 1)
    M[1:n_customers, 1:n_customers] .= dist_matrix
    M[n_customers+1, 1:n_customers] .= dist_depot
    M[1:n_customers, n_customers+1] .= dist_depot
    return (;
        n_customers,
        n_trucks,
        dist_depot,
        dist_matrix,
        service_time,
        earliest,
        latest,
        M,
        depot,
    )
end

function _cvrp_instance(;
    seed::Int,
    num_services::Int,
    num_pickup_deliveries::Int,
    num_trucks::Int,
    capacity::Int,
    quantity_range = 2:4,
)
    Random.seed!(seed)
    n_total = num_services + 2 * num_pickup_deliveries
    cx = rand(n_total + 1)
    cy = rand(n_total + 1)
    d(i, j) = round(Int, 100hypot(cx[i] - cx[j], cy[i] - cy[j]))
    dist_depot = [d(1, c + 1) for c = 1:n_total]
    dist_matrix = [d(i + 1, j + 1) for i = 1:n_total, j = 1:n_total]
    quantities = [rand(quantity_range) for _ = 1:num_pickup_deliveries]
    delta = zeros(Int, n_total)
    for k = 1:num_pickup_deliveries
        delta[num_services+k] = quantities[k]
        delta[num_services+num_pickup_deliveries+k] = -quantities[k]
    end
    depot = n_total + 1
    M = zeros(Int, n_total + 1, n_total + 1)
    M[1:n_total, 1:n_total] .= dist_matrix
    M[n_total+1, 1:n_total] .= dist_depot
    M[1:n_total, n_total+1] .= dist_depot
    return (;
        num_services,
        num_pickup_deliveries,
        num_trucks,
        n_total,
        dist_depot,
        dist_matrix,
        capacity,
        quantities,
        delta,
        M,
        depot,
    )
end

function _cvrptw_instance(;
    seed::Int,
    num_services::Int,
    num_pickup_deliveries::Int,
    num_trucks::Int,
    capacity::Int,
    fixed_time::Int,
    slope::Int,
    quantity_range = 2:4,
)
    base = _cvrp_instance(;
        seed,
        num_services,
        num_pickup_deliveries,
        num_trucks,
        capacity,
        quantity_range,
    )
    Random.seed!(seed + 1)
    earliest = [rand(0:30) for _ = 1:(base.n_total)]
    latest = [earliest[c] + 400 for c = 1:(base.n_total)]
    return (; base..., earliest, latest, fixed_time, slope)
end

# ── Solution recomputers ──────────────────────────────────────────────

function _route_cost(routes, dist_depot, dist_matrix)
    total = 0
    for r in routes
        isempty(r) && continue
        total += dist_depot[r[1]] + dist_depot[r[end]]
        for k = 2:length(r)
            total += dist_matrix[r[k-1], r[k]]
        end
    end
    return total
end

function _route_total_time_vrptw(routes, inst)
    total = 0
    for r in routes
        isempty(r) && continue
        t = 0
        for (k, c) in enumerate(r)
            travel = k == 1 ? inst.dist_depot[c] : inst.dist_matrix[r[k-1], c]
            arrival = t + travel
            start = max(inst.earliest[c], arrival)
            t = start + inst.service_time
        end
        total += t + inst.dist_depot[r[end]]
    end
    return total
end

function _route_total_time_cvrptw(routes, inst)
    total = 0
    for r in routes
        isempty(r) && continue
        t = 0
        for (k, v) in enumerate(r)
            travel = k == 1 ? inst.dist_depot[v] : inst.dist_matrix[r[k-1], v]
            arrival = t + travel
            start = max(inst.earliest[v], arrival)
            svc = inst.fixed_time + inst.slope * abs(inst.delta[v])
            t = start + svc
        end
        total += t + inst.dist_depot[r[end]]
    end
    return total
end

# ── Solution checkers (assertions via `@test`) ────────────────────────

function _check_partition(routes, n_customers)
    @test sort(reduce(vcat, routes)) == collect(1:n_customers)
    return
end

function _check_vrppd(routes, inst)
    _check_partition(routes, inst.n_total)
    for k = 1:(inst.num_pickup_deliveries)
        p = inst.num_services + k
        d = inst.num_services + inst.num_pickup_deliveries + k
        truck_p = findfirst(r -> p in r, routes)
        truck_d = findfirst(r -> d in r, routes)
        @test truck_p !== nothing
        @test truck_p == truck_d
        seq = routes[truck_p]
        @test findfirst(==(p), seq) < findfirst(==(d), seq)
    end
    return
end

function _check_time_windows(routes, inst)
    for r in routes
        isempty(r) && continue
        t = 0
        for (k, c) in enumerate(r)
            travel = k == 1 ? inst.dist_depot[c] : inst.dist_matrix[r[k-1], c]
            arrival = t + travel
            start = max(inst.earliest[c], arrival)
            @test start <= inst.latest[c]
            t = start + inst.service_time
        end
    end
    return
end

function _check_capacity(routes, inst)
    for r in routes
        load = 0
        max_load = 0
        for v in r
            load += inst.delta[v]
            max_load = max(max_load, load)
        end
        @test 0 <= max_load <= inst.capacity
        @test load == 0
    end
    return
end

function _check_cvrptw(routes, inst)
    _check_vrppd(routes, inst)
    for r in routes
        isempty(r) && continue
        load = 0
        t = 0
        for (k, v) in enumerate(r)
            travel = k == 1 ? inst.dist_depot[v] : inst.dist_matrix[r[k-1], v]
            arrival = t + travel
            start = max(inst.earliest[v], arrival)
            @test start <= inst.latest[v]
            load += inst.delta[v]
            @test 0 <= load <= inst.capacity
            svc = inst.fixed_time + inst.slope * abs(inst.delta[v])
            t = start + svc
        end
        @test load == 0
    end
    return
end

# ── Test functions ────────────────────────────────────────────────────
# Each takes an optimizer factory.

const _OPTIMAL_STATUSES = (MOI.OPTIMAL, MOI.LOCALLY_SOLVED)

# A `Partition` / `PartitionPD` column holds the clients a truck visits, in
# order, and `0` in every slot after the last one, so a route is the entries
# before the first `0`. Nothing here is solver-specific: a solver whose
# routes are shorter than the column pads them to give every variable of the
# set a value.
function _read_routes(nodes)
    n_slots, n_trucks = size(nodes)
    routes = [Int[] for _ = 1:n_trucks]
    for j = 1:n_trucks
        for i = 1:n_slots
            client = round(Int, JuMP.value(nodes[i, j]))
            # `break`, not `continue`: the padding is trailing, so a `0`
            # anywhere else is a solver getting the set wrong and the
            # partition check below has to catch it.
            iszero(client) && break
            push!(routes[j], client)
        end
    end
    return routes
end

# TSP — only solver-specific bit is reading the permutation, but
# `MathOptVRP.List(n)` pins `count(list) == n`, so every `value(nodes[k])`
# is a real customer id.
function MathOptVRP.Tests.test_tsp(
    optimizer_factory;
    seed::Int = 1234,
    n::Int = 6,
    time_limit::Real = 5,
    kwargs...,
)
    inst = _tsp_instance(; seed, n)
    model = JuMP.Model(optimizer_factory)
    JuMP.set_silent(model)
    JuMP.set_time_limit_sec(model, time_limit)
    JuMP.@variable(model, nodes[1:n] in MathOptVRP.List(n))
    JuMP.@objective(model, Min, MathOptVRP.op_sum_distances(inst.dist, nodes))
    JuMP.optimize!(model)
    @test JuMP.termination_status(model) in _OPTIMAL_STATUSES
    tour = [round(Int, JuMP.value(v)) for v in nodes]
    @test sort(tour) == collect(1:n)
    expected = sum(inst.dist[tour[k], tour[mod1(k + 1, n)]] for k = 1:n)
    @test round(Int, JuMP.objective_value(model)) == expected
    return
end

function MathOptVRP.Tests.test_vrp(
    optimizer_factory;
    seed::Int = 1234,
    n_customers::Int = 6,
    n_trucks::Int = 2,
    time_limit::Real = 5,
    kwargs...,
)
    inst = _vrp_instance(; seed, n_customers, n_trucks)
    model = JuMP.Model(optimizer_factory)
    JuMP.set_silent(model)
    JuMP.set_time_limit_sec(model, time_limit)
    JuMP.@variable(
        model,
        nodes[1:(inst.n_customers), 1:(inst.n_trucks)] in
        MathOptVRP.Partition(inst.n_customers, inst.n_trucks),
    )
    JuMP.@objective(
        model,
        Min,
        sum(
            MathOptVRP.op_sum_distances(inst.M, vcat(inst.depot, nodes[:, i], inst.depot))
            for i = 1:(inst.n_trucks)
        ),
    )
    JuMP.optimize!(model)
    @test JuMP.termination_status(model) in _OPTIMAL_STATUSES
    routes = _read_routes(nodes)
    _check_partition(routes, inst.n_customers)
    @test round(Int, JuMP.objective_value(model)) ==
          _route_cost(routes, inst.dist_depot, inst.dist_matrix)
    return
end

function MathOptVRP.Tests.test_vrppd(
    optimizer_factory;
    seed::Int = 1234,
    num_services::Int = 3,
    num_pickup_deliveries::Int = 2,
    num_trucks::Int = 2,
    time_limit::Real = 5,
    kwargs...,
)
    inst = _vrppd_instance(; seed, num_services, num_pickup_deliveries, num_trucks)
    model = JuMP.Model(optimizer_factory)
    JuMP.set_silent(model)
    JuMP.set_time_limit_sec(model, time_limit)
    JuMP.@variable(
        model,
        nodes[1:(inst.n_total), 1:(inst.num_trucks)] in MathOptVRP.PartitionPD(
            inst.num_services,
            inst.num_pickup_deliveries,
            inst.num_trucks,
        ),
    )
    JuMP.@objective(
        model,
        Min,
        sum(
            MathOptVRP.op_sum_distances(inst.M, vcat(inst.depot, nodes[:, i], inst.depot))
            for i = 1:(inst.num_trucks)
        ),
    )
    JuMP.optimize!(model)
    @test JuMP.termination_status(model) in _OPTIMAL_STATUSES
    routes = _read_routes(nodes)
    _check_vrppd(routes, inst)
    @test round(Int, JuMP.objective_value(model)) ==
          _route_cost(routes, inst.dist_depot, inst.dist_matrix)
    return
end

function MathOptVRP.Tests.test_vrptw(
    optimizer_factory;
    seed::Int = 1234,
    n_customers::Int = 4,
    n_trucks::Int = 2,
    time_limit::Real = 5,
    kwargs...,
)
    inst = _vrptw_instance(; seed, n_customers, n_trucks)
    model = JuMP.Model(optimizer_factory)
    JuMP.set_silent(model)
    JuMP.set_time_limit_sec(model, time_limit)
    JuMP.@variable(model, t[1:(inst.n_trucks)] >= 0)
    JuMP.@variable(
        model,
        nodes[1:(inst.n_customers), 1:(inst.n_trucks)] in
        MathOptVRP.Partition(inst.n_customers, inst.n_trucks),
    )
    for i = 1:(inst.n_trucks)
        JuMP.@constraint(
            model,
            [t[i]; inst.depot; nodes[:, i]; inst.depot] in
            MathOptVRP.TimeWindows(inst.M, inst.earliest, inst.latest, inst.service_time)
        )
    end
    JuMP.@objective(model, Min, sum(t))
    JuMP.optimize!(model)
    @test JuMP.termination_status(model) in _OPTIMAL_STATUSES
    routes = _read_routes(nodes)
    _check_partition(routes, inst.n_customers)
    _check_time_windows(routes, inst)
    @test round(Int, JuMP.objective_value(model)) == _route_total_time_vrptw(routes, inst)
    return
end

function MathOptVRP.Tests.test_cvrp(
    optimizer_factory;
    seed::Int = 1234,
    num_services::Int = 2,
    num_pickup_deliveries::Int = 2,
    num_trucks::Int = 2,
    capacity::Int = 5,
    time_limit::Real = 5,
    kwargs...,
)
    inst = _cvrp_instance(;
        seed,
        num_services,
        num_pickup_deliveries,
        num_trucks,
        capacity,
    )
    model = JuMP.Model(optimizer_factory)
    JuMP.set_silent(model)
    JuMP.set_time_limit_sec(model, time_limit)
    JuMP.@variable(
        model,
        nodes[1:(inst.n_total), 1:(inst.num_trucks)] in MathOptVRP.PartitionPD(
            inst.num_services,
            inst.num_pickup_deliveries,
            inst.num_trucks,
        ),
    )
    for i = 1:(inst.num_trucks)
        JuMP.@constraint(
            model,
            nodes[:, i] in MathOptVRP.Capacity(inst.delta, inst.capacity)
        )
    end
    JuMP.@objective(
        model,
        Min,
        sum(
            MathOptVRP.op_sum_distances(inst.M, vcat(inst.depot, nodes[:, i], inst.depot))
            for i = 1:(inst.num_trucks)
        ),
    )
    JuMP.optimize!(model)
    @test JuMP.termination_status(model) in _OPTIMAL_STATUSES
    routes = _read_routes(nodes)
    _check_vrppd(routes, inst)
    _check_capacity(routes, inst)
    @test round(Int, JuMP.objective_value(model)) ==
          _route_cost(routes, inst.dist_depot, inst.dist_matrix)
    return
end

function MathOptVRP.Tests.test_cvrptw(
    optimizer_factory;
    seed::Int = 1234,
    num_services::Int = 2,
    num_pickup_deliveries::Int = 2,
    num_trucks::Int = 2,
    capacity::Int = 5,
    fixed_time::Int = 2,
    slope::Int = 1,
    time_limit::Real = 5,
    kwargs...,
)
    inst = _cvrptw_instance(;
        seed,
        num_services,
        num_pickup_deliveries,
        num_trucks,
        capacity,
        fixed_time,
        slope,
    )
    model = JuMP.Model(optimizer_factory)
    JuMP.set_silent(model)
    JuMP.set_time_limit_sec(model, time_limit)
    JuMP.@variable(model, t[1:(inst.num_trucks)] >= 0)
    JuMP.@variable(
        model,
        nodes[1:(inst.n_total), 1:(inst.num_trucks)] in MathOptVRP.PartitionPD(
            inst.num_services,
            inst.num_pickup_deliveries,
            inst.num_trucks,
        ),
    )
    for i = 1:(inst.num_trucks)
        JuMP.@constraint(
            model,
            [t[i]; inst.depot; nodes[:, i]; inst.depot] in
            MathOptVRP.CapacitatedTimeWindows(
                inst.M,
                inst.earliest,
                inst.latest,
                inst.fixed_time,
                inst.slope,
                inst.delta,
                inst.capacity,
            )
        )
    end
    JuMP.@objective(model, Min, sum(t))
    JuMP.optimize!(model)
    @test JuMP.termination_status(model) in _OPTIMAL_STATUSES
    routes = _read_routes(nodes)
    _check_cvrptw(routes, inst)
    @test round(Int, JuMP.objective_value(model)) == _route_total_time_cvrptw(routes, inst)
    return
end

# `runtests` orchestrates the per-variant tests under one top-level
# testset.
function MathOptVRP.Tests.runtests(
    optimizer_factory;
    time_limit::Real = 5,
    kwargs...,
)
    @testset "MathOptVRP" begin
        @testset "TSP" begin
            MathOptVRP.Tests.test_tsp(optimizer_factory; time_limit, kwargs...)
        end
        @testset "VRP" begin
            MathOptVRP.Tests.test_vrp(
                optimizer_factory;
                time_limit,
                kwargs...,
            )
        end
        @testset "VRPPD" begin
            MathOptVRP.Tests.test_vrppd(
                optimizer_factory;
                time_limit,
                kwargs...,
            )
        end
        @testset "VRPTW" begin
            MathOptVRP.Tests.test_vrptw(
                optimizer_factory;
                time_limit,
                kwargs...,
            )
        end
        @testset "CVRP" begin
            MathOptVRP.Tests.test_cvrp(
                optimizer_factory;
                time_limit,
                kwargs...,
            )
        end
        @testset "CVRPTW" begin
            MathOptVRP.Tests.test_cvrptw(
                optimizer_factory;
                time_limit,
                kwargs...,
            )
        end
    end
    return
end

end # module
