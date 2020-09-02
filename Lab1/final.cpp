//Gilberto Pineda
//CSE512
//Lab 1
//This program implemments BFS, DFS, UCS, and A8 as path find algorithms
//It takes the input from a file and outputs to a file


#include <vector>
#include <iostream>
#include <fstream>
#include <map>
#include <stack>
#include <queue>
using namespace std;
//Define Pair of character and integer
typedef pair<char, int> Pair;

//Edge structure used to build a graph
struct Edge{
	char source;
	char destination;
	int weight;
};

//Compare struture which allowed for the comparison of vectors
//in order to archieve the priority queue desired which acted
//like a minheap
struct compare{
	bool operator()(vector<Pair> const& a, vector<Pair> const& b){
		int i = a.size() - 1;
		int k = b.size() - 1;
		Pair one = a[i];
		Pair two = b[k];
		return one.second > two.second;
	};
};

//Graph class which build out graph to be traversed
class Graph{
public:
	vector<vector<Pair>> adjList;

	Graph(vector<Edge> const &edges, int N){
		adjList.resize(N);

		for(auto &edge: edges){
			char source = edge.source;
			char destination = edge.destination;
			int weight = edge.weight;

			adjList[source].push_back(make_pair(destination, weight));
		}
	}
};

//This function checked to see if the node had already been traversed when using
//BFS and DFS
bool isNotVisited(char node, vector<char>& path){
	int size = path.size();
	for(int i = 0; i <  size; i++){
		if(path[i] == node){
			return false;
		}
	}
	return true;
}

//Modified from the function about to be able to check if a node had be traversed
//when passing a vector of type pair. Used for A* and UCS
bool isNvisited(char node, vector<Pair>& path){
	int size = path.size();
	for(int i = 0; i < size; i++){
		Pair v = path[i];
		if(v.first == node){
			return false;
		}
	}
	return true;
}


//BFS implenmation using a queue of type vector
//function stores all paths and returns the first path it finds to 
//the goal node
vector<char> BFS(Graph const &graph, char source, char goal, int N){

	queue<vector<char>> q;
	vector<char> path;

	path.push_back(source);
	q.push(path);
	

	while(!q.empty()){
		path = q.front();
		q.pop();
		char last = path[path.size() - 1];

		if(last == goal){
			return path;
		}
		for(auto i = last; i == last; i++)
			for(Pair v : graph.adjList[last])
				if(isNotVisited(v.first, path)){
					vector<char> newPath(path);
					newPath.push_back(v.first);
					q.push(newPath);
				}
	}
}

//UCS implemenation using a pirorirty queue
//Functions saves all path and return the path with the lowest path cost
//to the goal node
vector <Pair>  UCS(Graph const &graph, char source, char goal, int N){

	priority_queue<vector<Pair>, vector<vector<Pair>>, compare> q;
	vector<Pair> paths;

	paths.push_back(make_pair(source, 0));
	q.push(paths);
	
	while(!q.empty()){
		paths = q.top();
		q.pop();
		
		Pair last = paths[paths.size() - 1];
		if(last.first == goal){
			return paths;
		}

		for(auto i = last.first; i == last.first; i++){
			for(Pair v : graph.adjList[last.first]){
				if(isNvisited(v.first, paths)){
					vector<Pair> newPath(paths);
					newPath.push_back(make_pair(v.first, v.second + last.second));
					q.push(newPath);
				}
				else{
				for(int i = 0; i < paths.size(); i++){
					Pair x = paths[i];
					if(x.first == v.first){
						if(v.second + last.second < x.second){
								x.second = v.second + last.second;
						}
					}
				}
				vector<Pair> newPath(paths);
				q.push(newPath);
				}
			}
		}
	}
}

//A* implementation using a priority queue and sunday traffic
//works the same as UCS but takes into account sunday traffic which is 
//pass through a map		
vector<Pair> aStar(Graph const &graph, char source, char goal, int N, map<char, int> h){
	
	priority_queue<vector<Pair>, vector<vector<Pair>>, compare> q;
	vector<Pair> paths;

	
	paths.push_back(make_pair(source, 0));
	//Commeneted out since the hueristic should not add weight to the 
	//neighboring nodes of the source node
	//paths.push_back(make_pair(source, h[source]));
	q.push(paths);

	while(!q.empty()){
		paths = q.top();
		q.pop();
		
		Pair last = paths[paths.size() - 1];
		if(last.first == goal){
			return paths;
		}
		
		for(auto i = last.first; i == last.first; i++){
			for(Pair v : graph.adjList[last.first]){
				if(isNvisited(v.first, paths)){
					vector<Pair>newPath(paths);
					newPath.push_back(make_pair(v.first, v.second + last.second + h[last.first]));
					q.push(newPath);
				}
				else{
				for(int i = 0; i < paths.size(); i++){
					Pair x = paths[i];
					if(x.first == v.first){
						if(v.second + last.second + h[last.first] < x.second){
							x.second = v.second + last.first + h[last.first];
						}
					}
				}
				vector<Pair> newPath(paths);
				q.push(newPath);
				}
			}
		}
	}
}				

//DFS implementaion using a stack of type vector
//saves all path and returnt he first path it finds to the goal node
vector<char> DFS(Graph const &graph, char source, char goal, int N){
	stack <vector<char>> s;
	vector<char> path;

	path.push_back(source);
	s.push(path);
	
	while(!s.empty()){
		path = s.top();
		s.pop();
		char last = path[path.size() - 1];
		
		if(last == goal){
			return path;
		}
	
		for(auto i = last; i == last; i++)
			for(Pair v : graph.adjList[last])
				if(isNotVisited(v.first, path)){
					vector<char> newPath(path);
					newPath.push_back(v.first);
					s.push(newPath);
				}
	}
}

//This print function output to a file and is used for BFS and DFS
void print(vector<char> path, int N, Graph const &graph){
	ofstream outFile;
	outFile.open("output.txt");
	
	int curr_weight = 0;
	for(int  i = 0; i < path.size(); i++){
		outFile << path[i] << " " << curr_weight << endl;
		char it = path[i];
		for(Pair v : graph.adjList[it]){
			if(v.first == path[i+1]){
				curr_weight += v.second;
			}
		}
	}
	outFile.close();
}

//This print function outputs to a file and is used for A* and UCS
void printF(vector<Pair> path, int N, Graph const &graph){
	ofstream outFile;
	outFile.open("output.txt");
	
	int curr_weight = 0;
	for(int i = 0; i < path.size(); i++){
		Pair v = path[i];
		outFile << v.first << " " << curr_weight << endl;
		char it = v.first;
		for(Pair x : graph.adjList[it]){
			Pair p = path[i + 1];
			if(x.first == p.first){
				curr_weight += x.second;
			}
		}
	}
	outFile.close();
}

int main(){

	string pathFinder;
	int cost = 0;
	int h = 0;
	int liveTraffic = 0;
	char nodeA, nodeB, goalNode, sourceNode;

	vector<Edge> edges;
	map<char, int> sundayTraffic;
	ifstream inFile;			//declaring infile
	inFile.open("input.txt");		//opening input.txt
	
	getline(inFile, pathFinder);		//Get algorithm to use
	inFile >> sourceNode; 			//Get starting node
	inFile >> goalNode;			//Get goal node
	inFile >> liveTraffic;			//Get the number of verticies
	
	edges.resize(liveTraffic);		//Resize the vector to allocate enough memory for all vertices		
	for(int i = 0; i < liveTraffic; i++){	//Save All vertices onto vector
		inFile >> nodeA >> nodeB >> cost;
		edges[i] = {nodeA, nodeB, cost};
	}
	
	inFile >> h;				//Number of Vertices of sunday traffic

	for(int i = 0; i < h; i++){
		inFile >> nodeA >> cost;
		sundayTraffic.insert({nodeA, cost});
	}
	
	inFile.close();				//Close input file
	Graph graph(edges, liveTraffic - 1);	//Create graph 
	vector<char> path;			//New vector to save path
	vector<Pair> shortestPath;		//vector of type pair to store path from A* and UCS


	if(pathFinder == "BFS"){
		path = BFS(graph, sourceNode, goalNode, liveTraffic);
		print(path, liveTraffic, graph);
	}
	else if(pathFinder == "DFS"){
		path = DFS(graph, sourceNode, goalNode, liveTraffic);
		print(path, liveTraffic, graph);
	}
	else if(pathFinder == "UCS"){
		shortestPath = UCS(graph, sourceNode, goalNode, liveTraffic);
		printF(shortestPath, liveTraffic, graph);
	}
	else if(pathFinder == "A*"){
		shortestPath = aStar(graph, sourceNode, goalNode, liveTraffic, sundayTraffic);
		printF(shortestPath, liveTraffic, graph);
	}	
	
	return 0;
};
