#include <string>
#include <vector>
#include "FeatureGraph.h"
#include "GraphHelper.h"


using namespace std;


FeatureGraph::FeatureGraph(int N, int d, vector<Node> nodes, vector<Edge> edges) {
    numNodes = 0;
    skillSize = d;
    for(unsigned int i = 0; i<nodes.size(); i++){
        insert(nodes[i]);
    }
    for(unsigned int i = 0; i<edges.size(); i++){
        insert(edges[i]);
    }
    for(int i = 0; i < numNodes; i++){
        vector<int> x;
        for(int j = 0; j< numNodes; j++){
            x.push_back(0);
        }
        adjMatrix.push_back(x);
    }
};

void FeatureGraph::insert(Node node){
    numNodes++;
    nodes.push_back(node);
};
    
void FeatureGraph::insert(Edge edge){
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


