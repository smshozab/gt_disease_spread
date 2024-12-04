# Disease Spread Simulation in City Networks

This Streamlit app simulates the spread of a disease in a city network using graph theory. The app allows users to interactively adjust parameters for disease spread and visualize the spread of infection over time. Additionally, it uses **Dijkstra's Algorithm** to compute the least-risky path in the network.

## Features

- **Disease Spread Simulation**: Visualizes how diseases spread through a network of neighborhoods, modeled as nodes in a graph.
- **Dijkstra's Algorithm**: Computes and visualizes the least-risky path between two nodes based on edge weights (infection risk).
- **Interactive Interface**: Adjust parameters such as the number of nodes, infection probability, and initial infected nodes via Streamlit's sidebar.

## Installation

### Prerequisites

- Python 3.7 or higher
- Virtual environment (recommended but not required)

### Local Setup

1. **Clone the repository** or download the app files to your local machine:
   ```bash
   git clone https://github.com/smshozab/gt_disease_spread.git
   cd gt_disease_spread
2. Create a virtual environment (optional but recommended):
   ```bash
   python3 -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
3. Install dependencies: Ensure you have a requirements.txt file with all necessary dependencies. If it's not present, create a requirements.txt in the project root with the following:
   ```bash
   streamlit
    networkx
    matplotlib
    numpy
4. Install the dependencies by running:
   ```bash
    pip install -r requirements.txt

5. Run the app: Start the Streamlit app with:
   ```bash
    streamlit run gt.py
