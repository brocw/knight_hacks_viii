import concurrent.futures
import random
import numpy as np
import math
from functools import lru_cache

#Constrains
MAX_BATTERY_RANGE = 11500.0 #Max optimial battery range of 11.5km
LAUNCH_SITE_INDEX = 0 #The depot

#ACO Parameters (Ant-Colony-Optimization approach)
ALPHA = 1.0 #Pheremones (tau)
BETA = 5.0 #Distance/Heuristic influence (Eta)
RHO = .1 #Pheremone evaporate rate
Q = 100.0 #Pheremone deposit constant
INITIAL_PHEROMONE = 1e-4

#Simulation variables
NUM_ANTS_PER_MISSION_SLOT = 50 #We will run these parallel to each other to find ht enext single mission
MAX_ITERATIONS = 100 #Num of learning cycles

#Loading in data
def load_data(folder_path=r"C:\Users\andre\OneDrive\Desktop\hackathon\knight_hacks_viii"):
    """Loads all the necessary data files from the specified folder"""
    data = {
        'distance_matrix': np.load(f"{folder_path}/distance_matrix.npy"),
        'predecessors': np.load(f"{folder_path}/predecessors.npy"),
        'asset_indexes': np.load(f"{folder_path}/asset_indexes.npy"),
        'photo_indexes': np.load(f"{folder_path}/photo_indexes.npy")
    }

    N = data['distance_matrix'].shape[0]

    #Process indicies to get the required waypoints to visit
    required_waypoints = set()

    #Function to get indicies in slices [first,last]
    def extract_indicies(slice_array):
        if len(slice_array) == 2:
            return set(range(slice_array[0],slice_array[1]))
        elif len(slice_array) > 0:
            return set(slice_array)
        return set()
    
    asset_indicies = extract_indicies(data['asset_indexes'])
    photo_indicies = extract_indicies(data['photo_indexes'])

    #photo indexes required for waypoints
    all_targets = photo_indicies

    #filter targets to make sure they are in the bounds of the NxN matrix and not the depot
    required_waypoints = {i for i in all_targets if i<N and i!= LAUNCH_SITE_INDEX}
    print(f"Loaded {N} total waypoints. {len(required_waypoints)} required targets (Photo waypoints only)")

    data['required_waypoints'] = required_waypoints
    data['N'] = N
    return data

#Pheremone Agent
class PheremoneManager:
    """Manages the global pheremone matrix and evaporation/deposit rules"""
    def __init__(self, N):
        self.phermones = np.full((N,N), INITIAL_PHEROMONE, dtype=np.float64)
        self.N = N

    @lru_cache(maxsize=None)
    def get_pheromones(self):
        """Allows Ant Agents to read the current pheromone levels"""
        #return a copy
        return self.phermones.copy()

    def evaporates(self):
        """Applies evaporation to all the pheromone trails"""
        self.phermones *= (1-RHO)
    
    def deposIt(self, successful_route):
        """Deposits pheromones on the edges of a successful route"""
        path, distance, _ = successful_route

        #Pheromone deposit amount: Q/Total dist
        pheromone_amount = Q/distance

        #Iterate t hrough path edges
        for i in range(len(path)-1):
            u, v = path[i], path[i+1]
            self.phermones[u,v] += pheromone_amount
            #update in both directions to account for symmetry
            self.phermones[v,u] += pheromone_amount

def ant_agent_run(agent_id, pheromone_manager, data,nodes_to_cover):
    """
    Simulates a single Ant Agent building one single mission Route.
    
    Args: nodes_to_cover: the set of waypoints the agent must consider visiting.

    Returns: A tuple (path, distance, visited_set) if a path is built, or None.
    """

    pheromones = pheromone_manager.get_pheromones()
    dist_matrix = data['distance_matrix']

    current_node = LAUNCH_SITE_INDEX
    path = [current_node]
    distance = 0.0

    #Nodes visited IN the mission
    visited_set = set()
    available_targets = set(nodes_to_cover) #set of nodes we CAN visit

    #Heuristic Information
    heuristic = 1.0 / (dist_matrix + 1e-9)
    np.fill_diagonal(heuristic,0.0) #cant move itself

    while True:
        #determine # of allowed moves
        allowed_moves = []
        for next_node in available_targets:
            #check battery constraint:
            dist_to_next = dist_matrix[current_node,next_node]
            dist_home = dist_matrix[next_node, LAUNCH_SITE_INDEX]

            #distance of current route + next leg + return leg
            total_projected_dist = distance + dist_to_next + dist_home

            if total_projected_dist <= MAX_BATTERY_RANGE:
                allowed_moves.append(next_node)
        
        if not allowed_moves:
            #No moves because of battery constraint
            break

        #Calculate probablitiy of Pij for allowed moves
        probabilities = np.zeros(data['N'])

        for j in allowed_moves:
            tau = pheromones[current_node, j]
            eta = heuristic[current_node, j]
            probabilities[j] = (tau**ALPHA) * (eta*BETA)

        #Normalize probabilities and select the next node
        sum_probs = np.sum(probabilities)
        if sum_probs == 0:
            break

        probabilities /= sum_probs

        #select next node using roulette wheel selection
        next_node = np.random.choice(data['N'],1,p=probabilities)[0]

        #Update state
        distance += dist_matrix [current_node, next_node]
        path.append(next_node)
        visited_set.add(next_node)
        available_targets.remove(next_node)
        current_node = next_node

        #Go back to the depot
        if current_node != LAUNCH_SITE_INDEX:
            return_dist = dist_matrix[current_node,LAUNCH_SITE_INDEX]
            distance += return_dist
            path.append(LAUNCH_SITE_INDEX)

        if distance <=MAX_BATTERY_RANGE:
            return(path,distance,visited_set)
        else:
            return None
    
#Main Simulation Orchestrator:
def solve_aco_vrp():
    """
    Main ACO simulation loop, orchestrating sequential missions
    and parallel agents per mission slot
    """

    data = load_data()
    manager =  PheremoneManager(data['N'])
    best_solution = None #Stores the route_lists and total_distance vars

    for i in range(MAX_ITERATIONS):
        print(f"--- Iteration{i+1}/{MAX_ITERATIONS} ---")

        #Evaporation Phase
        manager.evaporates()

        #Mission dispatch loop
        nodes_to_cover = set(data['required_waypoints'])
        current_mission_routes = []
        current_total_distance = 0.0

        #Run missions until all of the nodes are either discover or we are stuck
        while nodes_to_cover:
            #Reset ant pool for current mission
            mission_candidates = []

            #Parallel Agent Run. launch num_ants_per_mission agents at the same time to find next 
            #next mission from the remaining 'nodes_to_cover'
            with concurrent.futures.ThreadPoolExecutor(max_workers=NUM_ANTS_PER_MISSION_SLOT) as executor:
                futures = [executor.submit(ant_agent_run, i, manager, data, nodes_to_cover)
                           for i in range(NUM_ANTS_PER_MISSION_SLOT)]

                for future in concurrent.futures.as_completed(futures):
                    result = future.result()
                    if result:
                        mission_candidates.append(result)
            
            #If none of the agents find a valid path, we stop
            if not mission_candidates:
                print("No agent could find a valid mission")
                break

            #Mission Selection (greedy/ACO choice)
            #Select the candidate that visits the most new required nodes
            best_mission = max(mission_candidates, key=lambda x: len(x[2] & nodes_to_cover))
            path, distance, visited_set = best_mission

            #Check for failure
            if not visited_set:
                break

            #Commit Mission and update state
            current_mission_routes.append((path, distance))
            current_total_distance+=distance
            nodes_to_cover-=visited_set

            #deposit Pheromone
            #reward the successful mission immediately
            print(f" -> Mission added: Dist={distance:.1f}m, Covered {len(visited_set)} targets. {len(nodes_to_cover)} remaining.")
        
        #Best update
        if not nodes_to_cover: #only evaluate if full coverage
            if best_solution is None or current_total_distance < best_solution[1]:
                best_solution = (current_mission_routes, current_total_distance)
                print(f"NEW BEST: Total Distance {current_total_distance:.2f} m")
    
    #Final Output:
    print("\n" + "="*50)
    print("      FINAL ACO-VRP SOLUTION SUMMARY")
    print("="*50)
    if best_solution:
        print(f"Total Distance (All Missions): {best_solution[1]:.2f} m")
        print(f"Total Missions: {len(best_solution[0])}")

        #details
        for i, (path, dist) in enumerate(best_solution[0]):
            print(f"\n--- Mission {i+1} ---")
            print(f"  Distance: {dist:.2f} m")
            print(f"  Targets Visited: {len(set(path) & data['required_waypoints'])}")

if __name__ == "__main__":
    solve_aco_vrp()