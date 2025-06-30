from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import osrm

# https://developers.google.com/optimization/routing/tsp#python_9



test = [
    ["Metro Flon", "46.5202985", "6.6302880"],
    ["A la Bossette", "46.5260001", "6.6367336"],
    ["L'A-T-E-L-I-E-R", "46.5268293", "6.6198081"],
    ["Bamee Bar", "46.5172634", "6.6324666"],
    ["Café Louve", "46.5212683", "6.6323482"]
]

bars = [
    ["A la Bossette", "46.5260001", "6.6367336"],
    ["L'A-T-E-L-I-E-R", "46.5268293", "6.6198081"],
    ["Bamee Bar", "46.5172634", "6.6324666"],
    ["Café Louve", "46.5212683", "6.6323482"],
    ["Bar Gala", "46.5211155", "6.6329693"],
    ["Café La Couronne d'Or", "46.5248548", "6.6335757"],
    ["Cylure Binchroom", "46.5235053", "6.6296545"],
    ["La Datcha", "46.5212521", "6.6259539"],
    ["Le Lapin Vert", "46.5235889", "6.6359008"],
    ["Lausanne Cocktail Club", "46.5200811", "6.6323819"],
    ["Meraki", "46.5243504", "6.6338428"],
    ["La Mise en Bière", "46.5237337", "6.6288049"],
    ["Le Ptit Central", "46.5208960", "6.6330015"],
    ["Qwertz", "46.5183958", "6.6340684"],
    ["STREET CELLAR", "46.5211806", "6.6293312"],
    ["Le Vestibule", "46.5232190", "6.6352332"],
    ["Le XIIIe Siecle", "46.5234148", "6.6352989"],
    ["MilkShake", "46.5206021", "6.6347360"],
    ["Trois 14", "46.5278040", "6.6311819"]
]

def get_string_coords(arr, i):
    return arr[i][2] + "," + arr[i][1]


def get_distance_matrix(arr):
    ## Make n x n array
    data = [[0]* len(arr) for i in range(len(arr))]

    ## Fill n x n
    for i in range(len(arr)):
        for j in range(len(arr)):
            if i == j:
                data[i][j] = 0.
            else:
                ## Generate string coordinates
                a = get_string_coords(arr, i)
                b = get_string_coords(arr, j)

                ## REST GET route from i to j
                dist = int(osrm.get_route_distance(a, b))

                ## Write to matrix
                data[i][j] = dist
    
    return data

def create_data_model(arr):
    data = {}
    data["distance_matrix"] = get_distance_matrix(arr)
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()} miles")
    index = routing.Start(0)
    plan_output = "Route for vehicle 0:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += f" {manager.IndexToNode(index)}\n"
    plan_output += f"Route distance: {route_distance}metres\n"
    print(plan_output)

def main():
    print("starting")
    # Init data
    data = create_data_model(test)

    # Routing index manager
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)


if __name__ == "__main__":
    main()

