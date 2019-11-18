#include "GraphHelper.h"
#include "FeatureGraph.h"
#include "GraphAnalyzer.h"
#include <utility>
#include <queue>
#include <algorithm>
#include <iostream>
using namespace std;

bool sortinrev(const pair<int,int> &a,  
               const pair<int,int> &b) 
{ 
       return (a.first > b.first); 
} 

void GraphAnalyzer::insert(Node n) {
    G.insert(n);
    // TODO Adjust calculations for ratio of open triangles and topKtriangles
};

void GraphAnalyzer::insert(Edge e) {
    G.insert(e);
    // TODO Adjust calculations for ratio of open triangles and topKtriangles
};

int GraphAnalyzer::bfsOnGraph(int indexOfNode){
    vector<Node> nodes = G.getNodes();
    vector<int> distance(nodes.size(), -1);
    queue<int> q;
    q.push(indexOfNode);
    distance[indexOfNode] = 0;
    while(!q.empty()){
        int x = q.front();
        q.pop();
        vector<int> edges = G.getAdjMatrix()[x];
        for(unsigned int i = 0; i <edges.size(); i++){
            if(edges[i]!=0){
                //Current index
                int curretIndex = i;
                if(distance[i]==-1){
                    q.push(curretIndex);
                    distance[i] = distance[x] + 1;
                }
            }
        }
    }
    int maxDistance = 0;
    //Find the Maximum distance from the vector distance
    for(unsigned int i = 0; i < distance.size(); i++){
        if(distance[i] > maxDistance){
            maxDistance = distance[i];
        }
    }
    return maxDistance;
};

int GraphAnalyzer::diameter() {
    vector<Node> temp = G.getNodes();
    int maxDiameter = -1;
    for(unsigned int i = 0; i < temp.size(); i++){
        int longestPathForIndexI = bfsOnGraph(i);
        if(longestPathForIndexI>maxDiameter){
            maxDiameter = longestPathForIndexI;
        }
    }
    return maxDiameter;
};


float GraphAnalyzer::openClosedTriangleRatio() {
    //TODO
    return .5;
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
    vector<pair<int, int> > results;
    for(unsigned int i= 0; i <row.size(); i++){
        if(row[i]!=0){
            vector<float> currentFeature = nodes[i].features;
            float dotProduct = 0;
            for(unsigned int j = 0; j < currentFeature.size(); j++){
                dotProduct += currentFeature[j] * w[j];
            }
            pair<int, int> tempP(dotProduct, nodes[i].id);
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



