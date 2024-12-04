import streamlit as st
import networkx as nx
import matplotlib.pyplot as plt
import random
import subprocess
import time

# Step 1: App Title and Description
st.title("Disease Spread Simulation in City Networks")
st.markdown("""
This project uses graph theory to simulate the spread of disease in an urban network. 
You can adjust parameters and visualize the results interactively. 
It also utilizes **Dijkstra's Algorithm** to find the least-risky paths!
""")

# Step 2: Input Parameters
st.sidebar.header("Simulation Parameters")
num_nodes = st.sidebar.slider("Number of neighborhoods (nodes):", min_value=5, max_value=50, value=10)
infection_probability = st.sidebar.slider("Infection Probability (0 to 1):", min_value=0.0, max_value=1.0, value=0.2)
initial_infected = st.sidebar.slider("Initial Infected Nodes:", min_value=1, max_value=num_nodes, value=1)
simulation_steps = st.sidebar.slider("Simulation Steps:", min_value=1, max_value=10, value=5)

# Step 3: Generate a Random Graph
st.header("Step 1: City Network Visualization")
G = nx.erdos_renyi_graph(n=num_nodes, p=0.3, directed=True)  # Directed graph

# Assign random weights to edges (representing infection risk)
for u, v in G.edges:
    G.edges[u, v]['weight'] = round(random.uniform(0.1, 1.0), 2)

# Assign initial infection states
for node in G.nodes:
    G.nodes[node]['infected'] = False
initial_infected_nodes = random.sample(list(G.nodes), initial_infected)
for node in initial_infected_nodes:
    G.nodes[node]['infected'] = True

# Function to visualize the graph
def draw_graph(G, path_edges=None):
    plt.figure(figsize=(8, 6))
    pos = nx.spring_layout(G)
    color_map = ['red' if G.nodes[node]['infected'] else 'blue' for node in G.nodes]
    nx.draw(G, pos, with_labels=True, node_color=color_map, node_size=500, edge_color='gray', arrows=True)
    
    if path_edges:  # Highlight specific edges
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='green', width=2)
    
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    st.pyplot(plt)

st.subheader("Initial Graph with Edge Weights")
draw_graph(G)

# Step 4: Simulate Disease Spread (Python)
st.header("Step 2: Disease Spread Simulation (Python)")
infected_over_time = []

for step in range(simulation_steps):
    new_infected = []
    for node in G.nodes:
        if not G.nodes[node]['infected']:  # If not already infected
            neighbors = list(G.neighbors(node))
            for neighbor in neighbors:
                if G.nodes[neighbor]['infected'] and random.random() < infection_probability:
                    new_infected.append(node)
                    break  # Stop checking other neighbors if infected

    for node in new_infected:
        G.nodes[node]['infected'] = True

    infected_over_time.append(len([n for n in G.nodes if G.nodes[n]['infected']]))
    st.subheader(f"Graph After Step {step + 1}")
    draw_graph(G)

# Step 5: Dijkstra's Algorithm (Python)
st.header("Step 3: Least Risky Path (Dijkstra's Algorithm)")

source_node = st.sidebar.number_input("Source Node (for Dijkstra's):", min_value=0, max_value=num_nodes-1, value=0)
target_node = st.sidebar.number_input("Target Node (for Dijkstra's):", min_value=0, max_value=num_nodes-1, value=num_nodes-1)

def dijkstra_python(G, source, target):
    # Initialize distances and predecessors
    dist = {node: float('inf') for node in G.nodes}
    prev = {node: None for node in G.nodes}
    dist[source] = 0
    pq = [(0, source)]  # Priority queue, initialized with the source node
    
    while pq:
        current_dist, u = heapq.heappop(pq)
        
        if u == target:
            break
        
        for v in G.neighbors(u):
            weight = G[u][v]['weight']
            alt = current_dist + weight
            
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(pq, (alt, v))
    
    # Reconstruct the shortest path
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = prev[node]
    
    path.reverse()
    return path

import heapq

# Run Dijkstra's Algorithm
try:
    shortest_path = dijkstra_python(G, source_node, target_node)
    st.write(f"**Shortest Path from Node {source_node} to Node {target_node}:** {shortest_path}")
    
    # Highlight the edges in the shortest path
    shortest_path_edges = [(shortest_path[i], shortest_path[i+1]) for i in range(len(shortest_path)-1)]
    st.subheader("Graph Highlighting Shortest Path")
    draw_graph(G, path_edges=shortest_path_edges)
except:
    st.write(f"No path exists between Node {source_node} and Node {target_node}.")

# Step 6: Run C++ Program and Compare Performance (Subprocess)
st.header("Step 4: Disease Spread Simulation (C++)")

# Measure time taken for C++ program
start_time_cpp = time.time()

# Run the C++ program with subprocess
result = subprocess.run(
    ['./test'],  # Path to the compiled C++ binary
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

# Capture the output from the C++ program
cpp_output = result.stdout.decode('utf-8')
cpp_error = result.stderr.decode('utf-8')

end_time_cpp = time.time()
cpp_execution_time = end_time_cpp - start_time_cpp

if cpp_error:
    st.error(f"Error running C++ program: {cpp_error}")
else:
    st.text(f"C++ Program Output:\n{cpp_output}")
    st.text(f"Execution time (C++): {cpp_execution_time:.4f} seconds")

# Step 7: Compare Performance (Python Simulation)
st.header("Performance Comparison")

# Measure time taken for Python version
start_time_python = time.time()

# Run the Python simulation code (same as above)
G_copy = G.copy()
infected_over_time_python = []

# Debugging print statement to check if simulation runs
st.write("Running Python simulation...")

for step in range(simulation_steps):
    new_infected = []
    for node in G_copy.nodes:
        if not G_copy.nodes[node]['infected']:  # If not already infected
            neighbors = list(G_copy.neighbors(node))
            for neighbor in neighbors:
                if G_copy.nodes[neighbor]['infected'] and random.random() < infection_probability:
                    new_infected.append(node)
                    break  # Stop checking other neighbors if infected

    for node in new_infected:
        G_copy.nodes[node]['infected'] = True

    infected_over_time_python.append(len([n for n in G_copy.nodes if G_copy.nodes[n]['infected']]))

# After the loop, stop the timer and show the execution time
end_time_python = time.time()
python_execution_time = end_time_python - start_time_python

# Debugging output to check the result of Python simulation
st.write(f"Python simulation finished with {len(infected_over_time_python)} infection steps.")

# Show the execution time for Python simulation
st.text(f"Execution time (Python): {python_execution_time:.4f} seconds")

# Step 8: Conclusion
st.write(f"C++ execution was {'faster' if cpp_execution_time < python_execution_time else 'slower'} than Python.")