# Optimizing Transit Routes

This is a computer science research project/paper on the optimization of transit systems. This was completed during an online research internship under Professor Robert Benkoczi at the University of Lethbridge.


The project comprises of a paper and its LaTex source code, along with a C++ implementation of the proposed algorithm. The C++ program is intended to import a graph of a city's transit system, but the data used in the research project is sensitive and cannot be publicly released. Thus, a sample graph is used (|V| = 10), but the algorithm is still fast enough to work on larger graphs (|V| > 10,000).


This paper proposes methods on how to optimize the bus routes of the City of Lethbridge’s transit system, ”Lethbrige Transit”. Three primary variables are considered in this optimization: distance, turns, and coverage. Given the grid nature of the city, the road system can be projected onto a two dimensional ﬁnte, planar, and undirected graph, with intersection as nodes and roads as edges. The method used to compute optimal routes, or paths on the graph, between and through desired bus stops is a modiﬁed Dijkstra’s Algorithm incorporating Dynamic Programming (DP) and vector calculations.
