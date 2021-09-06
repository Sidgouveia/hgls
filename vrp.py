from math import sqrt, pow, floor, ceil
from time import time
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from functools import partial
from random import sample
import localsolver

def manager_routing(nodes, num_vehicles, depot):
    #set a problem (n and K) and single thread for computational experiments
    manager = pywrapcp.RoutingIndexManager(nodes, num_vehicles, depot)
    model_parameters = pywrapcp.DefaultRoutingModelParameters()
    model_parameters.solver_parameters.max_edge_finder_size = nodes
    routing = pywrapcp.RoutingModel(manager, model_parameters)
    routing.set_nb_threads = 1
    return manager, routing

def distance_evaluator(CVRP):
    #c_ij - compute the cost
    def distance_callback(manager, from_index, to_index):
        return CVRP.edges[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
    return distance_callback

def demand_evaluator(CVRP):
    #q_i - compute the demand
    def demand_callback(manager, from_index):
        return CVRP.demands[manager.IndexToNode(from_index)]
    return demand_callback

def default_parameters():
    #Disable all ORtools local search operators 
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.use_full_propagation = False  
    search_parameters.local_search_operators.use_relocate_neighbors = 2
    search_parameters.local_search_operators.use_cross_exchange = 2
    search_parameters.local_search_operators.use_tsp_opt = 2
    search_parameters.local_search_operators.use_make_active = 2
    search_parameters.local_search_operators.use_make_inactive = 2
    search_parameters.local_search_operators.use_make_chain_inactive = 2
    search_parameters.local_search_operators.use_swap_active = 2
    search_parameters.local_search_operators.use_extended_swap_active = 2
    search_parameters.local_search_operators.use_path_lns = 2
    search_parameters.local_search_operators.use_full_path_lns = 2
    search_parameters.local_search_operators.use_tsp_lns = 2
    search_parameters.local_search_operators.use_inactive_lns = 2
    search_parameters.local_search_operators.use_node_pair_swap_active = 2
    search_parameters.local_search_operators.use_relocate_and_make_active = 2
    search_parameters.local_search_operators.use_relocate_expensive_chain = 2
    search_parameters.local_search_operators.use_light_relocate_pair = 2
    search_parameters.local_search_operators.use_global_cheapest_insertion_path_lns = 2
    search_parameters.local_search_operators.use_local_cheapest_insertion_path_lns = 2
    search_parameters.local_search_operators.use_or_opt = 2
    search_parameters.local_search_operators.use_lin_kernighan = 2
    search_parameters.local_search_operators.use_relocate_pair = 2
    search_parameters.local_search_operators.use_relocate_subtrip = 2
    search_parameters.local_search_operators.use_exchange_pair = 2
    search_parameters.local_search_operators.use_exchange_subtrip = 2
    search_parameters.local_search_operators.use_relocate = 2
    search_parameters.local_search_operators.use_exchange = 2
    search_parameters.local_search_operators.use_cross = 2
    search_parameters.local_search_operators.use_two_opt = 2
    return search_parameters

def get_routes(solution, routing, manager):
    #get a routes for a S solution ex:[[1,2,3],[4,5]]
    routes = []
    for k in range(routing.vehicles()):
        index = routing.Start(k)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        routes.append(route[1:])
    return routes

class cvrp(object):
    #CVRP object
    #read a instance file '.vrp'
    def read(self, instance):
        with open('instances/'+instance+'.vrp', 'r') as f:
            file = iter([element for element in f.read().split()])
        self.instance = instance
        while(1):
            token = next(file)
            if token == 'DIMENSION':
                next(file)
                self.n = int(next(file))
            elif token == 'CAPACITY':
                next(file)
                self.capacity = int(next(file))
            elif token == 'EDGE_WEIGHT_TYPE':
                next(file)
                token = next(file)
            elif token == 'NODE_COORD_SECTION':
                break;
        self.original_locations = []
        for i in range(self.n):
            next(file)
            self.original_locations.append((float(next(file)), float(next(file))))
        token = next(file)
        self.original_demands = []
        for i in range(self.n):
            next(file)
            self.original_demands.append(int(next(file)))
        token = next(file)
        self.depot = int(next(file))-1
        self.n -= 1
        self.k = ceil(sum(self.original_demands)/self.capacity)
        bks = {'X-n101-k25': 27591, 'X-n106-k14': 26362, 'X-n110-k13': 14971, 'X-n115-k10': 12747, 'X-n120-k6': 13332, 'X-n125-k30': 55539, 'X-n129-k18': 28940, 'X-n134-k13': 10916, 'X-n139-k10': 13590, 'X-n143-k7': 15700, 'X-n148-k46': 43448, 'X-n153-k22': 21220, 'X-n157-k13': 16876, 'X-n162-k11': 14138, 'X-n167-k10': 20557, 'X-n172-k51': 45607, 'X-n176-k26': 47812, 'X-n181-k23': 25569, 'X-n186-k15': 24145, 'X-n190-k8': 16980, 'X-n195-k51': 44225, 'X-n200-k36': 58578, 'X-n204-k19': 19565, 'X-n209-k16': 30656, 'X-n214-k11': 10856, 'X-n219-k73': 117595, 'X-n223-k34': 40437, 'X-n228-k23': 25742, 'X-n233-k16': 19230, 'X-n237-k14': 27042, 'X-n242-k48': 82751, 'X-n247-k50': 37274, 'X-n251-k28': 38684, 'X-n256-k16': 18839, 'X-n261-k13': 26558, 'X-n266-k58': 75478, 'X-n270-k35': 35291, 'X-n275-k28': 21245, 'X-n280-k17': 33503, 'X-n284-k15': 20215, 'X-n289-k60': 95151, 'X-n294-k50': 47161, 'X-n298-k31': 34231, 'X-n303-k21': 21736, 'X-n308-k13': 25859, 'X-n313-k71': 94043, 'X-n317-k53': 78355, 'X-n322-k28': 29834, 'X-n327-k20': 27532, 'X-n331-k15': 31102, 'X-n336-k84': 139111, 'X-n344-k43': 42050, 'X-n351-k40': 25896, 'X-n359-k29': 51505, 'X-n367-k17': 22814, 'X-n376-k94': 147713, 'X-n384-k52': 65940, 'X-n393-k38': 38260, 'X-n401-k29': 66163, 'X-n411-k19': 19712, 'X-n420-k130': 107798, 'X-n429-k61': 65449, 'X-n439-k37': 36391, 'X-n449-k29': 55233, 'X-n459-k26': 24139, 'X-n469-k138': 221824, 'X-n480-k70': 89449, 'X-n491-k59': 66487, 'X-n502-k39': 69226, 'X-n513-k21': 24201, 'X-n524-k153': 154593, 'X-n536-k96': 94868, 'X-n548-k50': 86700, 'X-n561-k42': 42717, 'X-n573-k30': 50673, 'X-n586-k159': 190316, 'X-n599-k92': 108451, 'X-n613-k62': 59535, 'X-n627-k43': 62164, 'X-n641-k35': 63694, 'X-n655-k131': 106780, 'X-n670-k130': 146332, 'X-n685-k75': 68205, 'X-n701-k44': 81923, 'X-n716-k35': 43387, 'X-n733-k159': 136190, 'X-n749-k98': 77314, 'X-n766-k71': 114454, 'X-n783-k48': 72394, 'X-n801-k40': 73305, 'X-n819-k171': 158121, 'X-n837-k142': 193737, 'X-n856-k95': 88965, 'X-n876-k59': 99299, 'X-n895-k37': 53860, 'X-n916-k207': 329179, 'X-n936-k151': 132725, 'X-n957-k87': 85465, 'X-n979-k58': 118987, 'X-n1001-k43': 72359}
        self.bks = bks[self.instance]
        self.max_time = int(240*(self.n/100))
        pass
    
    def customer_permutation(self):
        #Permute all customers for deterministic metaheuristics
        permutation = [self.depot] + sample(range(1, self.n+1), self.n)
        self.locations = [self.original_locations[i] for i in permutation]
        self.demands = [self.original_demands[i] for i in permutation]
        self.edges = [[floor(sqrt(pow(i[0] - j[0], 2) + pow(i[1] - j[1], 2)) +.5) for j in self.locations] for i in self.locations]
        pass

    def cost(self, s):
        #Compute the total cost of a solution
        costs = []
        for k, route in enumerate(s['routes']):
            route = [self.depot] + route + [self.depot]
            for i in range(len(route)-1):
                costs.append(self.edges[route[i]][route[i+1]])      
        return sum(costs)
    
    def first_solution(self):
        #Generate a Initial Solution based on section 2.1
        #Giant Tour - Route-First
        manager, routing = manager_routing(nodes=self.n+1, num_vehicles=1, depot=self.depot)
        routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitCallback(partial(distance_evaluator(self), manager)))    
        search_parameters = default_parameters()
        search_parameters.solution_limit = 1
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        routing.CloseModelWithParameters(search_parameters)
        solution = routing.SolveWithParameters(search_parameters)
        if solution:
            self.tour = []
            index = routing.Start(0)
            while not routing.IsEnd(index):
                index = solution.Value(routing.NextVar(index)) 
                self.tour.append(manager.IndexToNode(index))
        if self.depot == self.tour[0]:
            self.tour = self.tour[1:]
        elif self.depot == self.tour[-1]:
            self.tour = self.tour[:-1]
        #Clustering routes - Cluster-Second - First-fit 
        s = {}
        s['routes'] = [[]]
        vehicles = [self.capacity]
        for i, node in enumerate(self.tour):
            no_fit = False
            for j in range(len(vehicles)):
                if self.demands[node] <= vehicles[j]:
                    vehicles[j] = vehicles[j] - self.demands[node]
                    s['routes'][j].append(node)
                    break;
                if j == len(vehicles)-1:
                    no_fit = True
            if no_fit is True:
                vehicles.append((self.capacity)-self.demands[node])
                s['routes'].append([node])
        s['cost'] = self.cost(s)
        s['gap'] = ((s['cost']-self.bks)/self.bks)*100
        s['num_vehicles'] = len(s['routes'])
        s['vehicles_capacities'] = [self.capacity for k in s['routes']]
        self.S = [s]
        pass

    def local_search(self, phase='search'):
        manager, routing = manager_routing(nodes=self.n+1, num_vehicles=self.S[-1]['num_vehicles'], depot=self.depot)        
        routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitCallback(partial(distance_evaluator(self), manager))) 
        routing.AddDimensionWithVehicleCapacity(routing.RegisterUnaryTransitCallback(partial(demand_evaluator(self), manager)), 0,  self.S[-1]['vehicles_capacities'],  True, 'Capacity')    
        search_parameters = default_parameters()
        #Enable local search operators, see section 5.1.1
        search_parameters.local_search_operators.use_relocate = 3
        search_parameters.local_search_operators.use_exchange = 3
        search_parameters.local_search_operators.use_cross = 3            
        search_parameters.local_search_operators.use_two_opt = 3
        #Determine the search stop condition, see section 5.1
        #
        if phase == 'search':
            search_parameters.solution_limit = 30
            search_parameters.guided_local_search_lambda_coefficient = .1
            search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        elif phase == 'initial':
            search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT
        routing.CloseModelWithParameters(search_parameters)
        collector = routing.solver().AllSolutionCollector()
        collector.AddObjective(routing.CostVar())
        routing.AddSearchMonitor(collector)
        for i in range(routing.Size()):
            collector.Add(routing.NextVar(i))
        assignment = routing.SolveFromAssignmentWithParameters(routing.ReadAssignmentFromRoutes(self.S[-1]['routes'], True), search_parameters)
        if assignment:
            for i in range(collector.SolutionCount()-1, collector.SolutionCount()):
                s = {}
                s['routes'] = get_routes(collector.Solution(i), routing, manager)
                s['cost'] = collector.Solution(i).ObjectiveValue()
                s['gap'] = ((s['cost']-self.bks)/self.bks)*100
                s['vehicles_capacities'] = [self.capacity for k in s['routes']]
                s['num_vehicles'] = len(s['vehicles_capacities'])
            if phase == 'initial':
                s['time'] = 1
            self.S.append(s)
            pass

    def get_best_solution(self):
        #get best solution found in relation the search
        best = self.S[0]
        for s in self.S:
            if s['cost'] < best['cost']:
                best = s
        self.s = best
        pass
    
    def guided_local_search(self):
        #This our version of GLS aka HGLS, see Algorithm 2 or section 5
        self.customer_permutation() #Permutations
        self.start_time = time() #Start search time
        self.first_solution() #Generate initial soution
        self.local_search(phase='initial') #First Local Search - Best Improvement
        #Create a LocalSolver generalized set-partitioning Model
        with localsolver.LocalSolver() as solver:
            model = solver.model
            customers_sequences = [model.list(self.n) for k in range(self.S[-1]['num_vehicles'])]
            model.constraint(model.partition(customers_sequences))
            demands_array = model.array(self.demands[1:])
            distance_array = model.array()
            distance_depot = []
            for i in range(self.n):
                distance_array.add_operand(model.array(self.edges[i+1][1:]))
                distance_depot.append(self.edges[i+1][0])
            distance_depot_array = model.array(distance_depot)
            route_distances = [None] * self.S[-1]['num_vehicles']
            vehicles_used = [(model.count(customers_sequences[k]) > 0) for k in range(self.S[-1]['num_vehicles'])]
            num_vehicles_used = model.sum(vehicles_used)
            for k in range(self.S[-1]['num_vehicles']):
                sequence = customers_sequences[k]
                c = model.count(sequence)
                demand_selector = model.lambda_function(lambda i: demands_array[sequence[i]])
                route_quantity = model.sum(model.range(0, c), demand_selector)
                model.constraint(route_quantity <= self.capacity)
                dist_selector = model.lambda_function(lambda i: model.at(distance_array, sequence[i-1], sequence[i]))
                route_distances[k] = model.sum(model.range(1, c), dist_selector) + model.iif(c > 0, distance_depot_array[sequence[0]] + distance_depot_array[sequence[c-1]], 0)
            total_distance = model.sum(route_distances)
            model.minimize(num_vehicles_used)
            model.minimize(total_distance)
            model.close()
            #Main loop
            while time()-self.start_time < self.max_time:
                self.local_search()
                for k in range(self.S[-1]['num_vehicles']):
                    route = customers_sequences[k].get_value()
                    route.clear()
                    for i, n in enumerate(self.S[-1]['routes'][k]):    
                        route.add(n-1)
                self.current_time = time()-self.start_time
                self.mip_search_time = floor(self.max_time*.05)
                if self.max_time - self.current_time > 0:
                    if (self.max_time - self.current_time) < self.mip_search_time:
                        solver.param.time_limit = floor(self.max_time - self.current_time)
                    else:
                        solver.param.time_limit = self.mip_search_time
                    solver.param.set_nb_threads(1)
                    solver.param.set_verbosity(0)
                    solver.solve()
                    solver.stop()
                    s = {}
                    s['routes'] = []
                    for k in range(model.get_nb_decisions()):
                        if vehicles_used[k].value == 1 and customers_sequences[k].value != []:
                            s['routes'].append([])
                            for customer in customers_sequences[k].value:
                                s['routes'][len(s['routes'])-1].append(customer+1)
                    s['cost'] = total_distance.value
                    s['gap'] = ((s['cost']-self.bks)/self.bks)*100
                    s['num_vehicles'] = len(s['routes'])
                    s['vehicles_capacities'] = [self.capacity for k in s['routes']]
                    s['time'] = round(((time()-self.start_time)/self.max_time)*100, 0)
                    self.S.append(s)
            self.current_time = int(time()-self.start_time)
            self.get_best_solution()
            self.s['instance'] = self.instance
            y = [s['gap'] for s in self.S if 'time' in s.keys()]
            self.s['search'] = [min(y[:i]) for i in range(1, len(y)+1)]
            self.s['cpu_time'] = [s['time'] for s in self.S if 'time' in s.keys()]
            self.s['locations'] = self.locations
            self.s['demands'] = self.demands
        pass