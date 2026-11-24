import Random

# A random delivery instance with `num_nodes` customers and `num_vehicles`
# vehicles, alternating between vans, which may come back to the depot to
# reload, and trucks, which make a single trip but carry more. Distances are in
# meters, times in seconds from midnight, volumes in liters and weights in
# kilograms. The depot is the last row of `distance` and `travel`.
function random_instance(num_nodes::Integer, num_vehicles::Integer; seed = 1)
    rng = Random.MersenneTwister(seed)
    n = num_nodes
    # Customers in a square of 40 km around the depot.
    points = [[rand(rng, -20_000:20_000, 2) for _ in 1:n]; [[0, 0]]]
    distance = [
        round(Int, 1.3 * hypot((points[i] - points[j])...)) for
        i in 1:(n+1), j in 1:(n+1)
    ]
    # 12 m/s with 2 minutes to park.
    travel = [i == j ? 0 : 120 + div(distance[i, j], 12) for
              i in 1:(n+1), j in 1:(n+1)]
    service = rand(rng, 300:60:1200, n)
    # Half of the customers can be served at any time of the day, the others
    # in a window of 3 hours.
    earliest = zeros(Int, n)
    latest = fill(18 * 3600, n)
    for i in 1:n
        if rand(rng, Bool)
            earliest[i] = rand(rng, 7:15) * 3600
            latest[i] = earliest[i] + 3 * 3600
        end
        latest[i] -= service[i]
    end
    volume = rand(rng, 100:50:1500, n)
    weight = rand(rng, 20:10:400, n)
    # Pickups are loaded last so they are visited after the deliveries of
    # their trip, priority customers before the others, and bulky items must
    # be at the door of the truck, so first or last in their trip.
    pickup = rand(rng, n) .< 0.1
    priority = rand(rng, n) .< 0.1
    bulky = rand(rng, n) .< 0.1
    vehicles = map(1:num_vehicles) do v
        van = isodd(v)
        return (
            van = van,
            capacity = van ? 6_000 : 15_000,
            max_weight = van ? 1_200 : 3_500,
            num_trips = van ? 4 : 1,
            first_service = 900,
            reload_fixed = 600,
            day = (6 * 3600, 18 * 3600),
            max_amplitude = 10 * 3600,
            break_length = 1800,
            break_window = (11 * 3600 + 1800, 13 * 3600 + 1800),
            # A vehicle leaving no later than this time takes the break.
            nominal_break = 12 * 3600,
            fixed_cost = van ? 40_000 : 60_000,
            cost_per_second = van ? 8 : 12,
            cost_per_meter = van ? 1 : 2,
            max_pickups = 2,
        )
    end
    # Bulky items need a truck.
    compatibility = [!bulky[i] || !vehicles[v].van for i in 1:n, v in 1:num_vehicles]
    return (;
        distance,
        travel,
        depot = n + 1,
        earliest,
        latest,
        service,
        volume,
        weight,
        reload = div.(volume, 10),
        pickup,
        priority,
        bulky,
        vehicles,
        compatibility,
    )
end
