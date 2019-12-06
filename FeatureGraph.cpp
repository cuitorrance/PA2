#include <string>
#include <vector>
#include <iostream>
#include "FeatureGraph.h"
#include "GraphHelper.h"


using namespace std;


FeatureGraph::FeatureGraph(int N, int d, vector<Node> nodes, vector<Edge> edges) {
    numNodes = 0;
    skillSize = d;
    //Initalize a vector of nodes
    for(unsigned int i = 0; i<nodes.size(); i++){
        insert(nodes[i]);
    }

    vector<int> x {0};
    adjMatrix.push_back(x);

    //Initialize our adjaceny Matrix
    for(unsigned int i = 0; i<edges.size(); i++){
        insert(edges[i]);
    }
};

void FeatureGraph::insert(Node node){
    numNodes++;
    nodes.push_back(node);
};
    
void FeatureGraph::insert(Edge edge){
    //Initialize newAdjMatrix to hold the edge information
    vector<vector<int> > newAdjMatrix;
    for(int i = 0; i < numNodes; i++){
        vector<int> x;
        for(int j = 0; j< numNodes; j++){
            x.push_back(0);
        }
        newAdjMatrix.push_back(x);
    }
    //Update the New AdjMatrix with the old AdjMatrix information
    for(unsigned int i = 0; i<adjMatrix.size(); i++){
        for(unsigned int j = 0; j<adjMatrix[i].size(); j++){
            newAdjMatrix[i][j] = adjMatrix[i][j];
        }
    }
    adjMatrix = newAdjMatrix;
    int firstIndex = findIndexOfId(edge.IdA);
    int secondIndex = findIndexOfId(edge.IdB);
    adjMatrix[firstIndex][secondIndex] = edge.weight;
    adjMatrix[secondIndex][firstIndex] = edge.weight;
};

int FeatureGraph::findIndexOfId(int id){
    for(unsigned int i = 0; i < nodes.size(); i ++){
        if(nodes[i].id == id){
            return i;
        }
    }
    return -1;
}
void FeatureGraph::printAdjMatrix(){
    for(unsigned int i = 0; i < adjMatrix.size(); i++){
        cout<<"|";
        for(unsigned int j = 0; j < adjMatrix[i].size(); j++){
            cout<<adjMatrix[i][j]<<"|";
        }
        cout<<endl;
    }
}


