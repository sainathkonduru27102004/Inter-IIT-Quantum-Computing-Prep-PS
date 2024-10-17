# Inter-IIT-Quantum-Computing-Prep-PS

## PS: Traffic Congestion Management During Urban Events
The problem of managing traffic congestion dynamically during major urban events is complex
because such events often cause unexpected and significant disruptions to normal traffic flow. A
hybrid traffic management system, which combines classical algorithms and quantum
algorithms, offers an advanced solution by tackling different parts of the problem with the
best-suited techniques.

### Understanding the Problem:
During major urban events (like concerts, sports games, or festivals), large numbers of vehicles
head toward the same areas, often overwhelming existing traffic systems. Traditional traffic
control methods may not be flexible or fast enough to handle such surges, leading to severe
congestion, delayed emergency responses, and general traffic chaos.
To manage this complexity, we need a dynamic traffic management system that can:
1. Optimize vehicle routing from different parts of the city.
2. Adjust traffic signals in real-time based on current congestion levels.
3. Reroute traffic dynamically as new congestion points develop.
4. Ensure priority paths for emergency vehicles when needed.
   
### Method Used 
The method we propose combines classical optimization techniques with quantum computing to
handle traffic management tasks during major urban events. The classical approach is used
for the initial traffic flow analysis, while the quantum approach is employed for more complex,
dynamic tasks that involve real-time traffic control. By using both, we can optimize the system
far beyond what classical methods alone can achieve. The key quantum algorithm chosen is the
Quantum Approximate Optimization Algorithm (QAOA), supported by Quantum Annealing
and Quantum Simulators to simulate the traffic system and optimize high-complexity
scenarios.

#### Classical Method:
##### Dijkstra's Algorithm for Shortest Path Routing:
Dijkstra's algorithm is a well-known classical method for calculating the
shortest or least congested path between two points in a road network.
It treats the city’s road network like a graph, where each intersection is a
node and each road between intersections is an edge. Each road is assigned a “weight”
that represents its travel time or distance. Dijkstra's algorithm then calculates the
quickest path from a starting point to a destination by comparing all possible routes and
selecting the one with the smallest total weight.
This algorithm provides an efficient way to estimate how traffic will be
distributed across the city. By calculating the shortest paths for vehicles, we can balance
the load across the network and prevent congestion from building up in certain areas.

#### Quantum Method:
To improve traffic management and handle more complex tasks, we introduce several quantum
computing techniques. These allow us to manage real-time, high-complexity situations faster
and more efficiently than classical methods alone.

##### Quantum Approximate Optimization Algorithm (QAOA):
QAOA is a quantum algorithm used for solving optimization problems,
particularly useful for minimizing traffic congestion by optimizing traffic signal timings and
rerouting strategies.Intersections with traffic signals are represented as nodes in a graph.
QAOA finds the best timing for each traffic signal to reduce congestion. It does this by
continuously optimizing the settings based on real-time traffic data.Traffic conditions during urban events can change quickly, and fixed
signal timings aren’t effective in managing sudden surges of vehicles. QAOA
dynamically adjusts traffic signals in real-time, ensuring smoother traffic flow and
reduced congestion at intersections.

##### Quantum Annealing:
Quantum annealing is another quantum technique, ideal for solving
high-dimensional optimization problems, like determining the best routes for vehicles
when roads become congested.Quantum annealing explores a large number of possible solutions to
complex routing problems. It quickly finds the optimal route for vehicles, even in
congested areas, and can prioritize emergency vehicles by providing them with faster,
less congested paths.Quantum annealing allows us to dynamically reroute vehicles in
real-time, especially during moments of high congestion. It also ensures that emergency
vehicles can get through heavy traffic without delays.

##### Quantum Simulation for Traffic Modeling:
Quantum simulations use quantum computers to model and predict the
impact of different traffic configurations, helping us plan for sudden changes in traffic
volume during urban events.Quantum simulators can process and analyze complex scenarios, such
as how a sudden increase in vehicles will affect congestion. By modeling these
situations in advance, we can predict where problems might occur and take action
before they happen.Traffic during urban events can be unpredictable. By using quantum
simulations, we can test various traffic control strategies (like signal timings and
rerouting plans) and see how they will impact the actual traffic flow, helping us choose
the best strategy.

##### Why this Hybrid Approach?
By combining classical and quantum methods, this hybrid system enables us to handle both
regular and complex traffic situations. Classical algorithms (like Dijkstra’s) provide a solid,
efficient foundation for traffic management, while quantum algorithms (QAOA, Quantum
Annealing, and Quantum Simulations) handle the more complicated, dynamic problems that
arise in real-time, ensuring smoother and faster traffic flow during urban events. This hybrid
approach significantly improves the overall efficiency of the system.

### Inputs in the Traffic Management Simulation
#### Road Network Data (unordered_map<int, vector<pair<int, int>>> roadNetwork)
What It Represents: The road network is represented as a graph where intersections (nodes) are connected by roads (edges). Each edge has a weight, which could be distance, travel time, or another factor affecting travel.
Where It Is Used: This input is central to the routing algorithm. It's used to calculate the shortest path between intersections and helps determine the most efficient route based on distance. It also plays a role in rerouting during real-time traffic management and in the quantum-inspired optimization process.

#### Traffic Data (unordered_map<int, int> trafficData)
What It Represents: This data tracks the number of vehicles waiting at each intersection. Each key represents an intersection, and the value represents the current traffic volume at that location.
Where It Is Used: Traffic data is critical in optimizing traffic signals. It adjusts the green light duration for intersections with higher traffic, simulating the Quantum Approximate Optimization Algorithm (QAOA). The data also affects rerouting, where vehicles are diverted away from intersections with heavy traffic.

#### Event Data (Event event = {location, scale})
What It Represents: The event data represents special conditions, like a concert or a football game, which causes traffic surges around specific intersections. The location represents where the event is happening, and the scale indicates how big the event is, which can affect how much additional traffic it generates.
Where It Is Used: The event data is factored into the traffic flow models. When an event occurs, it increases the vehicle count in the affected area, influencing traffic signal timings and rerouting decisions. This data also helps prioritize routes to minimize delays in areas around events.

#### Emergency Vehicle Data (EmergencyVehicle emergency = {location, destination})
What It Represents: This input represents an emergency vehicle's current location and its destination. The system uses this data to prioritize its movement, adjusting signal timings and rerouting traffic to ensure the emergency vehicle reaches its destination as quickly as possible.
Where It Is Used: Emergency vehicle data is applied in rerouting strategies. The system calculates the optimal path for the emergency vehicle, giving it priority at traffic signals and rerouting other vehicles out of its way. This ensures minimal delays for the emergency response.


### Inputs in Classical Traffic Optimization Using Dijkstra's Algorithm
#### Start Node and Total Nodes (int startNode, int totalNodes)
What It Represents: The startNode is the intersection from which the shortest path is calculated, while totalNodes refers to the total number of intersections in the road network.
Where It Is Used: These inputs are used in Dijkstra's algorithm to find the shortest path between intersections. The algorithm calculates the shortest distance from the start node to all other nodes in the network, which is then used for routing vehicles along the most efficient path.

#### Inputs in Traffic Signal Optimization (Simulating QAOA)
Intersection Vehicle Counts (unordered_map<int, int> trafficData)
What It Represents: The number of vehicles at each intersection, used to adjust traffic light timings. More vehicles at an intersection will result in longer green lights to ease congestion.
Where It Is Used: In the traffic signal optimization function, vehicle counts at each intersection determine the length of green and red lights. The higher the count, the longer the green light. This mimics the behavior of the Quantum Approximate Optimization Algorithm (QAOA), optimizing signals based on real-time data.

#### Inputs in Quantum Annealing-Based Rerouting
Start and End Nodes for Rerouting (int start, int end)
What It Represents: The start node is where a vehicle begins, and the end node is its destination. The system calculates the most efficient path based on current congestion levels.
Where It Is Used: These inputs are used in the rerouting algorithm (based on Dijkstra’s algorithm but modified for congestion). The algorithm calculates the path that avoids congested roads, balancing travel distance and congestion levels for efficient rerouting of vehicles.

#### Congestion Levels (unordered_map<int, int> congestionLevels)
What It Represents: Congestion levels are real-time indicators of traffic conditions on each road (edge). Higher values indicate more congestion, meaning longer travel times.
Where It Is Used: Congestion levels are integrated into the rerouting algorithm. Roads with higher congestion are penalized in the pathfinding process, leading to rerouting vehicles to less congested paths. This approach balances travel efficiency and real-time traffic conditions.

## 1. Classical Algorithm - Dijkstra's Algorithm for Initial Traffic Flow Analysis
Purpose:
Finds the shortest path between intersections (nodes) in the road network (graph) for initial route calculation.
// Dijkstra's Algorithm for Shortest Path
void dijkstra(int startNode, int totalNodes, unordered_map<int, vector<pair<int, int>>> roadNetwork) {
    // Priority queue to select the node with the shortest distance
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Vector to store the shortest distance from startNode to every other node
    vector<int> distance(totalNodes, INT_MAX);

    // Starting node has a distance of 0
    distance[startNode] = 0;
    pq.push({0, startNode});

    while (!pq.empty()) {
        int currentNode = pq.top().second;
        int currentDistance = pq.top().first;
        pq.pop();

        // Explore neighbors of the current node
        for (auto neighbor : roadNetwork[currentNode]) {
            int nextNode = neighbor.first;
            int edgeWeight = neighbor.second;

            // Relaxation step: check if we can find a shorter path
            if (distance[nextNode] > currentDistance + edgeWeight) {
                distance[nextNode] = currentDistance + edgeWeight;
                pq.push({distance[nextNode], nextNode});
            }
        }
    }

    // Output the shortest distances to each node from startNode
    for (int i = 0; i < totalNodes; i++) {
        cout << "Shortest distance from " << startNode << " to " << i << " is " << distance[i] << endl;
    }
}
Relevance:
This classical algorithm is used to calculate the initial shortest path for vehicles in the traffic network. It's used to compute the most efficient route between two intersections before considering dynamic congestion and real-time events.

## 2. Quantum-Inspired Algorithm - Simulated QAOA for Traffic Signal Timing Optimization
Purpose:
Optimize the duration of green and red traffic signals at each intersection based on traffic data.
// Simulated QAOA-like Traffic Signal Optimization
void optimizeTrafficSignals(unordered_map<int, int> trafficData) {
    // Each intersection has a default green light duration (in seconds)
    unordered_map<int, int> greenLightDuration;
    unordered_map<int, int> redLightDuration;

    // Loop through each intersection's traffic data
    for (auto intersection : trafficData) {
        int intersectionID = intersection.first;
        int vehicleCount = intersection.second;

        // Quantum-inspired heuristic: Adjust green light based on traffic volume
        if (vehicleCount > 100) {
            greenLightDuration[intersectionID] = 90;  // High traffic, longer green light
        } else if (vehicleCount > 50) {
            greenLightDuration[intersectionID] = 60;  // Moderate traffic
        } else {
            greenLightDuration[intersectionID] = 30;  // Low traffic, shorter green light
        }

        // Red light is the complement of green light (total cycle = 120 seconds)
        redLightDuration[intersectionID] = 120 - greenLightDuration[intersectionID];

        // Output the optimized signal timings
        cout << "Intersection " << intersectionID << " Green Light Duration: " << greenLightDuration[intersectionID] << " seconds" << endl;
        cout << "Intersection " << intersectionID << " Red Light Duration: " << redLightDuration[intersectionID] << " seconds" << endl;
    }
}
Relevance:
The signal optimization mimics the Quantum Approximate Optimization Algorithm (QAOA). It dynamically adjusts traffic light timings based on real-time traffic data to improve traffic flow at congested intersections.


## 3. Quantum Annealing-Based Rerouting
Purpose:
Find an optimized route for vehicles based on real-time traffic congestion, using a quantum-inspired approach.
// Quantum Annealing-inspired Rerouting Algorithm
vector<int> rerouteTraffic(int startNode, int endNode, unordered_map<int, vector<pair<int, int>>> roadNetwork, unordered_map<int, int> congestionLevels) {
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    vector<int> distance(roadNetwork.size(), INT_MAX);
    vector<int> parent(roadNetwork.size(), -1);

    distance[startNode] = 0;
    pq.push({0, startNode});

    while (!pq.empty()) {
        int currentNode = pq.top().second;
        int currentDistance = pq.top().first;
        pq.pop();

        // Explore neighbors with congestion consideration
        for (auto neighbor : roadNetwork[currentNode]) {
            int nextNode = neighbor.first;
            int edgeWeight = neighbor.second;

            // Adjust weight with congestion factor (penalize congested roads)
            int congestionFactor = congestionLevels[nextNode];
            int adjustedWeight = edgeWeight + congestionFactor;

            // Relaxation step considering congestion
            if (distance[nextNode] > currentDistance + adjustedWeight) {
                distance[nextNode] = currentDistance + adjustedWeight;
                pq.push({distance[nextNode], nextNode});
                parent[nextNode] = currentNode;
            }
        }
    }

    // Reconstruct the path from startNode to endNode
    vector<int> path;
    for (int at = endNode; at != -1; at = parent[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    return path;  // Return the optimized path
}
Relevance:
This rerouting algorithm simulates the quantum annealing process, optimizing traffic flow by minimizing congestion. The quantum-inspired approach adjusts travel paths in real time by penalizing congested roads, thus reducing overall travel time.


## 4. Emergency Vehicle Prioritization Using Classical and Quantum Optimization
Purpose:
Ensure that emergency vehicles can reach their destination in the shortest time by prioritizing them at traffic signals and through rerouting.
// Emergency Vehicle Priority Routing Algorithm
vector<int> prioritizeEmergencyVehicle(int startNode, int endNode, unordered_map<int, vector<pair<int, int>>> roadNetwork, unordered_map<int, int> trafficData) {
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    vector<int> distance(roadNetwork.size(), INT_MAX);
    vector<int> parent(roadNetwork.size(), -1);

    distance[startNode] = 0;
    pq.push({0, startNode});

    while (!pq.empty()) {
        int currentNode = pq.top().second;
        int currentDistance = pq.top().first;
        pq.pop();

        // Prioritize emergency vehicle at traffic signals by clearing traffic
        trafficData[currentNode] = 0;  // Clear traffic at the current intersection

        for (auto neighbor : roadNetwork[currentNode]) {
            int nextNode = neighbor.first;
            int edgeWeight = neighbor.second;

            if (distance[nextNode] > currentDistance + edgeWeight) {
                distance[nextNode] = currentDistance + edgeWeight;
                pq.push({distance[nextNode], nextNode});
                parent[nextNode] = currentNode;
            }
        }
    }

    // Reconstruct the emergency vehicle's path from startNode to endNode
    vector<int> path;
    for (int at = endNode; at != -1; at = parent[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    return path;  // Return the prioritized path for the emergency vehicle
}
Relevance:
This algorithm prioritizes emergency vehicles by adjusting traffic signals and rerouting traffic. It ensures the vehicle reaches its destination with minimal delay by clearing traffic at key intersections and providing a direct route
