/*******************************************************
* 			  		  Kevin Jia 				       *
*                                                      *
*     Reads in a graph and locations to visit, then    *
*      computes optimal paths from a single source.    *
* Optimizes for distance, turns, and locations visited *
*******************************************************/

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <list>
#include <queue>
#include <vector>
#define DEBUG false
#define NODE_MAX 31754
#define PI 3.14159265

double EPS = 1e-6; //used to compare floats
double INF = 1e17; //used in Dijkstra calculations

//arbitrary values, can be changed to suit purposes
double CLOSENESS = 150;
double TURN_COST = 1000;
double VISIT_BENEFIT = 4000;

using namespace std;

//node structure used in graph, built in ajdacency list 
//can also be used for coordinates and desired locations
struct node
{
	int vertex;
	float x, y;
	vector<int> neighbours;
	node() { 
		vertex = 0;
		x=  0.0;
		y = 0.0;
		neighbours = vector<int>(0);
	}
	node(int a, double b, double c) {
		vertex = a;
		x = b;
		y = c;
	}
};

vector<node> graph; //graph of road network
vector<node> busStops; //stores desired location
vector<double> dist; //distance table from single source
vector<double> locations; //# of visited locations on path*VISIT_BENEFIT, used for DP
vector<int> p; //p[u] = index of previous node in shortest path to u

void dijkstra(int source); 
vector<int> pathReconstruct(int last);
double nodeDist(node a, node b);
double turn(int u, int v);
double visit(int u, int v);
bool notin(int u, int v);

int main() {
	FILE *nodegraph = fopen("main.nodegraph", "r");
	//fscanf(nodegraph, "W\n31753\n23992\n");
	int i;
	node current, blank;
	graph.resize(NODE_MAX);
	
	//read in main.nodegraph to graph structure (vector<node> graph)
	while (fscanf(nodegraph, "%d,%f,%f,", &current.vertex, &current.x, &current.y) == 3) {
		current.neighbours.clear();
		int edges, neighbour;
		fscanf(nodegraph, "%d;", &edges);
		
		for (int i = 0; i < edges - 1; i++) {
			fscanf(nodegraph, "%d,", &neighbour);
			current.neighbours.push_back(neighbour);
		}
		fscanf(nodegraph, "%d|", &neighbour);
		current.neighbours.push_back(neighbour);
		graph[current.vertex] = current;
	}
	
	current.vertex = 0;
	current.neighbours.clear();
	
	//read in locations desired to be visited (within proximity of CLOSENESS to nodes on the path)
	FILE *busStop = fopen("busStop.txt", "r");
	while (fscanf(busStop, "%d,%f,%f|", &current.vertex, &current.x, &current.y) == 3) {
		busStops.push_back(current);
	}
	dist.resize(graph.size());
	p.resize(graph.size());
	locations.resize(graph.size());
	
	//read in start and end nodes
	int start, end;
	cout << "Enter start point: ";
	cin >> start;
	cout << "Enter end point: ";
	cin >> end;
	if (graph[start].vertex == 0 || graph[end].vertex == 0) {
		cout << "invalid start/end points" << endl;
		return 1;
	}
	
	dijkstra(start);                              //compute paths using dijkstra's 
	vector<int> path = pathReconstruct(end);      //rebuild path to end node
	
	double pathDist = dist[end];
	cout << "Optimal Path: ";
	for (int i = 0; i < path.size(); i++) {
		cout << graph[path[i]].vertex << " ";     //output path
		if (i > 1) pathDist -= turn(path[i-1], path[i]); //subtract additional turn costs from path distance to find true path distance
	}
	cout << endl;
	
	cout << "Total nodeDist: " << pathDist;       //output distance
}

//Given a source, computes distance table and finds shortest paths to all
//modified to incorporate turns and locations visited
//average case O((|E|+|V|)|L| log|V|) time 
void dijkstra(int source) {
	//initialize arrays to "undefined" markers, except for at the source, which is computed
	for (int i=0; i<graph.size(); i++) {
		dist[i] = INF;
		p[i] = -1;                          //undefined
		locations[i] = -1;                  //undefined
	}
	dist[source] = 0.0;
	locations[source] = visit(source, source);
	priority_queue< pair<double, int> > pq;	
	pq.push(make_pair(0.0, source));        //start from the source
	
	while (!pq.empty()) {
		double d = -pq.top().first; 
		int u = pq.top().second;
		pq.pop();							//pop the current node off the queue, we've processed it
		if (fabs(dist[u] - d) > EPS)		//very important: if we have found a better path, don't waste
			continue;						//time exploring a suboptimal one
		for (int i=0; i<graph[u].neighbours.size(); i++) {
			int v = graph[u].neighbours[i];
			double w = nodeDist(graph[u], graph[v]) + turn(u, v);  //find w(eight) as Euclidian distance from u to v and add the turn cost if necessary
			double c = visit(u, v);         //calculate visit benefit, and deduct from both paths compared
			if (dist[v] == INF || ((dist[u] + w - c) < (dist[v] - visit(v, v)) && notin(u,v))) {	//proceed if path is better than the current one, do not return to node already in the path
				dist[v] = dist[u] + w;	    //store current path                                      otherwise will cause an infinite loop (potentially caused by visit benefit reducing distance
				p[v] = u;	     	        //the path goes u->v
				locations[v] = c;           //store locations visited, necessary for dp
				pq.push(make_pair(-dist[v], v));	//continue with dijkstra's, negate dist[v] for min heap
			}
		}
	}
}

//return Euclidean distance (O(1))
double nodeDist(node a, node b) {
	return (sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)));
}

//Given a node, returns a vector of vertices taken in the path from source to that node (O(|P|), P is path nodes)
vector<int> pathReconstruct(int last) {
	vector<int> path;
	
	while (p[last] != -1) {
		path.push_back(last);
		last = p[last];
	}
	path.push_back(last);
	reverse(path.begin(), path.end());
	return path;
}

//returns turn cost if next node v will cause a turn, return 0 if not
double turn(int u, int v) {
	if (p[u] != -1) {
		node vecPU(0, graph[p[u]].x - graph[u].x, graph[p[u]].y - graph[u].y), vecVU(0, graph[v].x - graph[u].x, graph[v].y - graph[u].y);
		node origin;
		//calculate angle using dot product between previous direction and new direction
		double theta = acos((vecPU.x*vecVU.x + vecPU.y*vecVU.y) / nodeDist(vecPU, origin) / nodeDist(vecVU, origin)); 
		if (theta < PI * 5/6) return TURN_COST;				
	}
	return 0;
}

//returns locations visited in shortest path to u and vertex v
//O(|L|) best case and average case given (|l| << |V|), worst case O(|L||P|), |P| is path length
double visit(int u, int v) {
	if (busStops.size() == 0) return 0;                       //return 0 if there are no locations needing visiting
	if (u == v && locations[u] != -1) return locations[u];	  //return correct value if already computed
	if (locations[u] == -1){								  //if not already computed
		//checks all nodes in path to u and vertex, marks all locations visited, and afterwards counts number visited
		bool stopsVisited[busStops.size()] = {false};
		vector<int> path = pathReconstruct(u);
		for (int i = 0; i < path.size(); i++) {
			for (int j = 0; j < busStops.size(); j++) {
				if (nodeDist(graph[path[i]], busStops[j]) <= CLOSENESS) stopsVisited[j] = true;
			}
		}
		for (int j = 0; j < busStops.size(); j++) {
			if (nodeDist(graph[u], busStops[j]) <= CLOSENESS) stopsVisited[j] = true;
		} 
		int totalVisited = 0;
		for (int j = 0; j < busStops.size(); j++) {
			if (stopsVisited[j] == true) totalVisited++;
		}
		return (totalVisited * VISIT_BENEFIT);
	}
	else {												      //if locations for path to u is already computed, but doesn't include v
		//total is locations visited by path to u, plus unique locations visited by v
		int totalVisited = (int) (locations[u] / VISIT_BENEFIT + EPS);
		vector<int> path = pathReconstruct(u);
		for (int j = 0; j < busStops.size(); j++) {			  //linear scan through locations to see if v visits them
			if (nodeDist(graph[v], busStops[j]) <= CLOSENESS) {
				bool unique = true;
				for (int i = 0; i < path.size(); i++) {       //linear scan through path to see if location is uniquely visited by v
					if (nodeDist(graph[path[i]], busStops[j]) <= CLOSENESS) unique = false;
				}
				if (unique == true) totalVisited++;
			}
		}
		return (totalVisited * VISIT_BENEFIT);
	}
}

//checks if v is in shortest path to u, O(|P|)
bool notin(int u, int v) {
	vector<int> path = pathReconstruct(u);
	for (int i = 0; i < path.size(); i++)
		if (path[i] == v) return false;
	return true;
}
