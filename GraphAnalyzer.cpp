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
    // TODO Adjust calculations for ratio of open triangles and topKtriangles
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
};

string GraphAnalyzer::topKOpenTriangles(int k) {
    //TODO
    return "2,3,4";
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
    //TODO
    return 1;
};


float GraphAnalyzer::jacardIndexOfTopKNeighborhoods(int nodeAID, int nodeBiID, int k, vector<float> w) {
    //TODO
    return 0;
};



