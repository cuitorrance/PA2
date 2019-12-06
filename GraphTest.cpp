#include "GraphHelper.h"
#include "FeatureGraph.h"
#include "GraphAnalyzer.h"
#include <map>
#include <iostream>
using namespace std;

int main() {
    
    vector<Node> nodes  {
       Node(1, vector<float> { 10, 10}),
       Node(2,vector<float> { 20, 20}),
       Node(3, vector<float> { 30, 30}),
       Node(4, vector<float> { 30, 30}),
       Node(5, vector<float> { 30, 30}),
       Node(6, vector<float> { 30, 30})};
    
    vector<Edge> edges {Edge(1,2, 1), Edge(2, 3, 1), Edge(2,5, 1), Edge(3,5,1), Edge(3,4, 1), Edge(5,6, 1)};
  
    int d = 2;

 

    FeatureGraph graph = FeatureGraph(4, d, nodes, edges);
    GraphAnalyzer analyzer = GraphAnalyzer(graph);


    cout << analyzer.diameter() << "\n";

    cout << analyzer.openClosedTriangleRatio() << "\n";

    cout << analyzer.topKOpenTriangles(6) << "\n";

    
    int newNodeID = 10;
    vector<float> newFeatures {3, 3};
    Node newNode = Node(newNodeID, newFeatures);

    analyzer.insert(Edge(4, 6, 1));

    cout << analyzer.diameter() << "\n";

    cout << analyzer.openClosedTriangleRatio() << "\n";

    cout << analyzer.topKOpenTriangles(6) << "\n";
    
    
    vector<float> weights{.5, .5};
    vector<int> neighbors = analyzer.topKNeighbors(2, 3, weights);
    
    for(auto i = neighbors.begin(); i != neighbors.end(); ++i)
        cout << *i << ",";
    cout << "\n";

    cout << analyzer.topNonNeighbor(2, weights) << "\n";

    cout << analyzer.jacardIndexOfTopKNeighborhoods(1, 2, 2, weights);
    return 0;
}