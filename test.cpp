#include <iostream>
#include <vector>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <limits>
#include <unordered_set>
#include <algorithm>

using namespace std;

// Struct to represent an edge in the graph
struct Edge {
    int to;
    double weight;
};

// Struct to represent a node's infection status
struct Node {
    bool infected;
    vector<Edge> neighbors;
};

class Graph {
public:
    int num_nodes;
    vector<Node> nodes;

    Graph(int n) : num_nodes(n) {
        nodes.resize(n);
    }

    void add_edge(int from, int to, double weight) {
        nodes[from].neighbors.push_back({to, weight});
    }

    // Function to simulate disease spread
    void spread_disease(double infection_probability, int initial_infected) {
        unordered_set<int> infected_nodes;

        // Randomly select initial infected nodes
        while (infected_nodes.size() < initial_infected) {
            int node = rand() % num_nodes;
            infected_nodes.insert(node);
            nodes[node].infected = true;
        }

        // Spread the disease for a set number of steps
        for (int step = 0; step < 5; ++step) {
            unordered_set<int> new_infected;
            for (int node = 0; node < num_nodes; ++node) {
                if (nodes[node].infected) {
                    for (const Edge& edge : nodes[node].neighbors) {
                        if (!nodes[edge.to].infected && rand() % 100 < infection_probability * 100) {
                            new_infected.insert(edge.to);
                        }
                    }
                }
            }
            for (int ni : new_infected) {
                nodes[ni].infected = true;
            }
        }
    }

    // Function to implement Dijkstra's Algorithm to find the least risky path
    vector<int> dijkstra(int start, int end) {
        vector<double> dist(num_nodes, numeric_limits<double>::infinity());
        vector<int> prev(num_nodes, -1);
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
        
        dist[start] = 0.0;
        pq.push({0.0, start});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (u == end) break;

            for (const Edge& edge : nodes[u].neighbors) {
                int v = edge.to;
                double weight = edge.weight;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        // Reconstruct the shortest path
        vector<int> path;
        for (int v = end; v != -1; v = prev[v]) {
            path.push_back(v);
        }
        reverse(path.begin(), path.end());

        return path;
    }

    // Helper function to print the graph's current status
    void print_graph() {
        for (int i = 0; i < num_nodes; ++i) {
            cout << "Node " << i << " (Infected: " << (nodes[i].infected ? "Yes" : "No") << "): ";
            for (const Edge& edge : nodes[i].neighbors) {
                cout << "(" << edge.to << ", " << edge.weight << ") ";
            }
            cout << endl;
        }
    }
};

int main() {
    srand(time(0));

    int num_nodes = 10;
    double infection_probability = 0.2;
    int initial_infected = 1;

    Graph g(num_nodes);

    // Example of creating a graph with 10 nodes and random edges with weights
    g.add_edge(0, 1, 0.5);
    g.add_edge(0, 2, 0.8);
    g.add_edge(1, 3, 0.6);
    g.add_edge(2, 3, 0.7);
    g.add_edge(3, 4, 0.3);
    g.add_edge(4, 5, 0.9);
    g.add_edge(5, 6, 0.4);
    g.add_edge(6, 7, 0.2);
    g.add_edge(7, 8, 0.1);
    g.add_edge(8, 9, 0.3);

    // Step 1: Spread Disease Simulation
    g.spread_disease(infection_probability, initial_infected);
    
    // Step 2: Print the graph after infection spread
    cout << "\nGraph After Disease Spread:" << endl;
    g.print_graph();

    // Step 3: Run Dijkstra's Algorithm
    int source = 0;
    int target = 9;
    vector<int> shortest_path = g.dijkstra(source, target);

    if (shortest_path.empty()) {
        cout << "No path found from node " << source << " to node " << target << endl;
    } else {
        cout << "\nShortest Path from " << source << " to " << target << ": ";
        for (int node : shortest_path) {
            cout << node << " ";
        }
        cout << endl;
    }

    return 0;
}
