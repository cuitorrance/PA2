#ifndef HELPER_H
#define HELPER_H

#include <vector>
using namespace std;

struct Node {
    // NOTE: Do not edit node struct
    int id;
    vector<float> features;
    
    Node(int id, vector<float> features): id(id), features(features) {}
    Node(const Node &n2) { id = n2.id; features=n2.features;}
};


struct Edge {
    // NOTE: Do not edit Edge struct
    int IdA, IdB, weight;
    
    Edge(int IdA, int IdB, int weight): IdA(IdA), IdB(IdB), weight(weight) {}
};

class Triangle {
    // TODO make a Triangle class with appropriate fields and methods

public:

    // TODO make appropriate constuctor
    Triangle(Edge a, Edge b) { 
        this->totalWeight = a.weight + b.weight;
<<<<<<< HEAD
    }
    Triangle(Edge a, Edge b, Edge c) { 
        this->totalWeight = a.weight + b.weight + c.weight;
=======

	//input IDs
	this->IDa = a.IdA;
	this->IDb = a.IdB;

	//check in case edges are flipped
	if (this->IDb == b.IdA){
	  this->IDc = b.IdB;
	}else{
	  this->IDc = b.IdA;
	}
    }
    Triangle(Edge a, Edge b, Edge c) { 
        this->totalWeight = a.weight + b.weight + c.weight;

	//input IDs
	this->IDa = a.IdA;
	this->IDb = a.IdB;

	//check in case edges are flipped
	if (this->IDb == b.IdA){
	  this->IDc = b.IdB;
	}else{
	  this->IDc = b.IdA;
	}
	
>>>>>>> m3
    }
    
    // Operator overloading for storage in priority queue
    // returns true iff t2 is greater than t1. 
    //
    // Note: Must be transitive
    //      This means if t1<t2 and t2<t3 than t1< t3
    bool operator < (Triangle const &other) {
        if(this->totalWeight < other.totalWeight){
            return true;
        }
        return false;
    }
<<<<<<< HEAD
private:
    int totalWeight;
=======
  int getIDa(){return IDa;}
  int getIDb(){return IDb;}
  int getIDc(){return IDc;}
  
private:
    int totalWeight;
  int IDa;
  int IDb;
  int IDc;
>>>>>>> m3
};


#endif

    




    
