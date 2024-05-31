# Airport Connectivity Map

This project uses the graph data structure to find optimal flight paths on a set of given airports.

# Graph Creation

The graph is created by reading a CSV file, with each row (excluding first row) containing airport data. The data must be in a specific order. See `airports.csv` for expected formatting.

Airports are represented by vertices, connections between airports are represented by edges. The graph can be directed or undirected, the edges are weighed by distance or cost. By default, the graph is directed and weighed by distance.

# Graph Capabilities
- Find shortest path between two airports
- Find shortest paths between an airport and all airports in a specified state
- Find shortest path between two airports with a given number of stops
- Find number of direct connections an airport has to other airports. Includes inbound and outbound connections.
- Create a minimum spanning tree using Prim's or Kruskal's algorithm

# Team Members
Oscar Lin, Tyler, Ayden

