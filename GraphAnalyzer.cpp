#include "GraphHelper.h"
#include "FeatureGraph.h"
#include "GraphAnalyzer.h"
#include <utility>
#include <queue>
#include <algorithm>
#include <iostream>
#include <limits.h>
using namespace std;

bool sortinrev(const pair<float,int> &a,  
               const pair<float,int> &b) 
{ 
       return (a.first > b.first); 
} 

bool sortByLargest(const int &a,  
               const int &b) 
{ 
       return (a > b); 
} 

void GraphAnalyzer::insert(Node n) {
    G.insert(n);
    // TODO Adjust calculations for ratio of open triangles and topKtriangles
};

void GraphAnalyzer::insert(Edge e) {
    G.insert(e);
    vector<vector<int> > x = G.getAdjMatrix();
    vector<Node> w = G.getNodes();
    if(G.findIndexOfId(e.IdA) < x.size() || G.findIndexOfId(e.IdB) < x.size()){
        
        priority_queue<Triangle, vector<Triangle>, sortTriangle> pq;
        for(int i = 0; i <triHeap.size(); i++){
            int count = 0;
            set<int>::iterator it = triHeap.top().ids.begin();
            while (it != triHeap.top().ids.end()){
	            if(*it == e.IdA || *it == e.IdB){
                    count++;
                }
                it++;
            }
            if(count < 2){
                pq.push(triHeap.top());
            }
            triHeap.pop();
        }
        triHeap = pq;

    }
    else{
        vector<int> row = x[G.findIndexOfId(e.IdA)];
        for(int i = 0; i < row.size() - 1; i++){
            if(row[i]!=0){
                Edge z(e.IdA, w[i].id, x[G.findIndexOfId(e.IdA)][i]);
                Triangle r(z, e);
                triHeap.push(r);
            }
        }
    }
    // TODO Adjust calculations for ratio of open triangles and topKtriangles
};

//inserting triangles into heap
void GraphAnalyzer::insertTriHeap(){

  //get nodes
  vector<Node> nodes = G.getNodes();
  
  //get adjacency matrix
  vector< vector<int> > graph = G.getAdjMatrix();

  //vector to store triangles
  vector<Triangle> triVector;
  //for each edge
  for ( int i = 0; i < graph.size();i++){
    for (int j = 0; j < graph[i].size(); j++){

      //if there is an edge
      if (graph[i][j] != 0){

	//for each vertex
	for( int k = j; k < graph[i].size(); k++){

	  //if (j,k) and (k,i) are edges
	  if (graph[i][j] != 0 && graph[i][k] != 0 && k != j){

	    //create Triangle
	    Edge one (nodes[i].id , nodes[j].id, graph[i][j]);
	    Edge two (nodes[i].id, nodes[k].id, graph[i][k]);

	    Triangle tri(one, two);
	    
	    //insert triangle into unordered vector
	    triVector.push_back(tri);
	    
	  }
	  
	}
    }
      
    }
  }
  //insert into Heap, check for duplicate triangles
  for (int i = 0; i < triVector.size();i++){
    bool flag = true;
    for ( int j = 0; j < triVector.size();j++){
      if ( i != j && triVector[i].ids != triVector[j].ids){
          continue;
      }else if(i!=j){
          flag = false;
      }
    }
    if(flag){
        triHeap.push(triVector[i]);
    }
  }
};


//Approximated from Geeks for Geeks (Giving credit)
int findClosestNode(vector<int> distance, vector<bool> visited, int numOfNodes){
    int min = INT_MAX;
    int minIndex = -1;
    for(int i = 0; i<numOfNodes; i++){
        if(visited[i] == false && distance[i] <= min){
            min = distance[i];
            minIndex = i;
        }
    }
    return minIndex;
}

int GraphAnalyzer::dijkstraAlgorithm(int indexOfNode){
    vector<Node> nodes = G.getNodes();
    //Contains all the distances
    vector<int> distance(nodes.size(), INT_MAX);
    //Contains information on wether we have visted the node or not
    vector<bool> visited(nodes.size(), 0);

    vector<vector<int> > graph = G.getAdjMatrix();
    //src node distance is 0
    distance[indexOfNode] = 0;

    //Update distance values of adjacent vertices
    for(int i = 0; i<nodes.size() - 1; i++){
        //Find the minimum distance vertex from the set. Initialize u to the src
        int u = findClosestNode(distance, visited, nodes.size());

        //Mark the source as visited
        visited[u] = true;
        for(int v = 0; v< nodes.size(); v++){
            //Update the distance[v] if it has not been visited, there is an edge between u and v, AND the weight of v + distance[u]
            //and is less than the current distance[v]
            if(graph[u][v] && !visited[v] && distance[u] != INT_MAX
                && distance[v] > distance[u] + graph[u][v])
                {
                distance[v] = distance[u] + graph[u][v];
            }
        } 
    }
    sort(distance.begin(), distance.end(), sortByLargest);
    return distance.front();
}

//Utilizes Dijkstra's algorithm to find shortest path distances
int GraphAnalyzer::diameter() {
    vector<Node> temp = G.getNodes();
    if(getNumberOpenTriangles(G.getAdjMatrix()) == 0 && getNumberOfClosedTrinagles() == 0){
        return -1;
    }
    int maxDiameter = -1;
    //Loops through all the possible source nodes and applies dijkstra's Algorithm to the source node
    for(unsigned int i = 0; i < temp.size(); i++){
        int longestPathForIndexI = dijkstraAlgorithm(i);
        //If the the current diameter is smalled than the current source nodes longest "shortest" path
        //than the diameter = the current longest "shortest" path
        if(longestPathForIndexI>maxDiameter){
            maxDiameter = longestPathForIndexI;
        }
    }
    return maxDiameter;
};

//helper fucntion for factorial
static unsigned int factorial(unsigned int n){
if (n == 0)
       return 1;
    return n * factorial(n - 1);

};

//returns number of open triangles in graph
int GraphAnalyzer::getNumberOpenTriangles(vector< vector<int> > graph){
  

  int result = 0;


  //count # edges
  for(unsigned int i = 0; i < graph.size(); i++){
        //Specific node, where node is center of triangle
        //edge counter
        int edgeCount = 0;
        for (unsigned int j = 0; j < graph[i].size(); j++){
            if (graph[i][j] != 0){
	            edgeCount++;
            }
        }
        //if number of edges is less than 2
        if(edgeCount < 2){
            result += 0;
        }else{
            //calculate # of edges choose 2
            result+= factorial(edgeCount) / ( (factorial(edgeCount - 2)) * 2 );
        }
  }

  //if number of edges is less than 2

  
  return result;
};

vector<vector<int> > multiplyMatrix(vector<vector<int> > x, vector<vector<int> > y){
    vector<int> rows(x.size(), 0);
    vector<vector<int> > result(x.size(), rows);
    for(int i = 0; i< x.size(); i++){
        for(int j = 0; j<x.size(); j++){
            for(int k = 0; k<x.size(); k++){
                result[i][j] += x[i][k] * y[k][j];
            }
        }
    }
    return result;
}

int getTrace(vector<vector<int> > x){
    int trace = 0;
    for(int i = 0; i < x.size(); i++){
        trace += x[i][i];
    }
    return trace;
}

int GraphAnalyzer::getNumberOfClosedTrinagles(){


  vector<vector<int> > graph = G.getAdjMatrix();
    //Removes the weights currently in the graph and substitutes it for ones if edge
    for(int i = 0; i<graph.size(); i++){
        for(int j = 0; j<graph.size(); j++){
            if(graph[i][j] > 0){
                graph[i][j] = 1;
            }
        }
    }
    //Square the matrix to find all possible paths of length 2
    vector<vector<int> > graphSquared = multiplyMatrix(graph, graph);
    //Cube the matrix to find all possible paths of length 3
    vector<vector<int> > graphCubed = multiplyMatrix(graphSquared, graph);
    //Get trace to find all possible paths that start and end in the same location
    int unfilteredNumOfTriangles = getTrace(graphCubed);
    //Divide by 3 due to combinations of 3 nodes and divide by 2 because nature of undirected graphs
    return unfilteredNumOfTriangles/6;
}

float GraphAnalyzer::openClosedTriangleRatio() {
    int numOfClosedTriangles = getNumberOfClosedTrinagles();
    if(numOfClosedTriangles==0){
        return -1;
    }
    vector<vector<int> > graph = G.getAdjMatrix();
    int numOfOpenTriangles = getNumberOpenTriangles(graph);
    //Overcount number of closed triangles 3 times due to cyclical nature of closed triangles.
    return float(numOfOpenTriangles - (3*numOfClosedTriangles)) / float(numOfClosedTriangles);
};

string GraphAnalyzer::topKOpenTriangles(int k) {
    vector<vector<int>> graph = G.getAdjMatrix();
    int openTri = getNumberOpenTriangles(graph)- (3*getNumberOfClosedTrinagles());
    //No open triangles
    if(openTri == 0){
        return "";
    }
    //Tri heap has been created already
//     if (triHeap.size() != 0){
//     return "";
//   }
  //FIrst initial tri heap
  if (triHeap.size() == 0){
    insertTriHeap();
  }

    string result = "";
    if(openTri < k){
        k = openTri;
    }
    priority_queue<Triangle, vector<Triangle>, sortTriangle> pq = triHeap;
    //given a vector max heap that already has max heap property
    //go through k elements of heap
    for (int i = 0; i < k ; i++){
      string nextTriangle = "";
      vector<int> triID;
      set<int>::iterator it = triHeap.top().ids.begin();
      while (it != triHeap.top().ids.end()){
	    triID.push_back(*it);
        it++;
      }
      nextTriangle = to_string(triID[0]) + "," + to_string(triID[1]) + "," + to_string(triID[2]) + ";";
      result += nextTriangle;
      triHeap.pop();
    }
    result = result.substr(0, result.length() - 1);
    cout<<pq.size()<<endl;
    triHeap = pq;
    return result;
};

//Reminder may need bug fix if k is greater than edges for nodeID
vector<int> GraphAnalyzer::topKNeighbors(int nodeID, int k,  vector<float> w) {
    //TODO
    //Gets index of nodeID
    int indexOfNode = G.findIndexOfId(nodeID);
    vector<vector<int> > temp = G.getAdjMatrix();
    vector<int> row = temp[indexOfNode];
    //Gets nodes that contain the feature vector
    vector<Node> nodes = G.getNodes();
    //Holds the dot product and ID first int is Dot prodcut second int is ID
    vector<pair<float, int> > results;
    for(unsigned int i= 0; i <row.size(); i++){
        if(row[i]!=0){
            vector<float> currentFeature = nodes[i].features;
            float dotProduct = 0;
            for(unsigned int j = 0; j < currentFeature.size(); j++){
                dotProduct += currentFeature[j] * w[j];
            }
            pair<float, int> tempP(dotProduct, nodes[i].id);
            results.push_back(tempP);
        }
    }
    sort(results.begin(), results.end(), sortinrev);
    //Final return
    vector<int> finalReturn(k, 0);
    for(int i = 0; i < results.size(); i++){
        finalReturn[i] = results[i].second;
    }
    while(finalReturn.size() != k){
        finalReturn.push_back(0);
    }
    return finalReturn;
};


int GraphAnalyzer::topNonNeighbor(int nodeID, vector<float> w) {
    int indexOfNode = G.findIndexOfId(nodeID);
    vector<Node> currentGraph = G.getNodes();
    //Will hold Nodes that do not have an edge with nodeID
    vector<Node> nodesNotConnected;
    //Will return row containing edge iformation with nodeID
    vector<int> edges = G.getAdjMatrix()[indexOfNode];
    for(int i = 0; i < edges.size(); i++){
        if(edges[i] == 0 && i!=indexOfNode){
            nodesNotConnected.push_back(currentGraph[i]);
        }
    }
    //If the node has an edge with every node return -1
    if(nodesNotConnected.size() == 0){
        return -1;
    }
    //Loops through nodesNotConnected and grabs the id of the top scoring Node
    int topNonNeighbor = -1;
    float maxWeight = -1;
    for(int i = 0; i <nodesNotConnected.size(); i++){
        float dotProduct = 0;
        for(int j = 0; j < w.size(); j++){
            dotProduct+=nodesNotConnected[i].features[j] * w[j];
        }
        if(dotProduct>maxWeight){
            maxWeight = dotProduct;
            topNonNeighbor = nodesNotConnected[i].id;
        }
    }
    return topNonNeighbor;
};


float GraphAnalyzer::jacardIndexOfTopKNeighborhoods(int nodeAID, int nodeBID, int k, vector<float> w) {
    vector<int> topKNeighborA = topKNeighbors(nodeAID, k, w);
    vector<int> topKNeighborB = topKNeighbors(nodeBID, k, w);
    
    vector<int> unionNeighbors;
    set_union(topKNeighborA.begin(), topKNeighborA.end(), topKNeighborB.begin(), topKNeighborB.end(), back_inserter(unionNeighbors));
    vector<int> intersection;
    set_intersection(topKNeighborA.begin(), topKNeighborA.end(), topKNeighborB.begin(), topKNeighborB.end(), back_inserter(intersection));
    return float(intersection.size()) / float(unionNeighbors.size());
};



