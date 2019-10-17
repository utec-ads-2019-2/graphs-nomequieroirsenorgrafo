// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H

#include "node.h"
#include "edge.h"

struct Traits {
    typedef string N;
    typedef float E;
};

template<typename Tr>
class Graph {
public:
    typedef typename Tr::N N;
    typedef typename Tr::E E;

    typedef Graph<Tr> self;
    typedef Node<self> node;
    typedef Edge<self> edge;

    typedef map<N, node *> NodeSeq;
    typedef map<N, edge *> EdgeSeq;
    typedef typename NodeSeq::iterator NodeIte;
    typedef typename EdgeSeq::iterator EdgeIte;
private:
    NodeSeq nodes;
    EdgeSeq edges;
    NodeIte ni;
    EdgeIte ei;

    const double densityParameter = 0.5;
    //std::set<node*> nodeList;
    //std::set<edge> edgeList;
    std::map<N, set<N>> adjList;
    std::map<N, set<N>> adjList_Transposed;
//    bool is_directed; bool negativeWeight;

public:
    Graph() {}

    /* FUNCTIONS */
    node* addNode(N tag, double x, double y) {
        auto newNode = new node(tag, x, y ); //Node or vertex

        nodes.insert({ tag, newNode});
        return newNode;
    }

    bool addEdge(N from, N to) {
        auto nodeFrom = findNode( from );
        auto nodeTo = findNode( to );

        if(!nodeFrom)
            nodeFrom = addNode(from, 0, 0);

        if(!nodeTo)
            nodeTo = addNode(to, 0, 0);

        auto weight = getDistance(nodeFrom, nodeTo);

        auto newEdge = new edge(weight);

        newEdge->setNodes( nodeFrom, nodeTo );

        edges.insert( { from + to, newEdge });
    }

    bool deleteNode(N tag) {

    }

    bool deleteEdge(N from, N to) {

    }

    node *findNode(N tag) {
        return nodes[ tag ];
    }

    edge *findEdge(N from, N to);

    self prim(N data);

    self kruskal();

    std::pair<bool, map<N, bool>> getBipartiteAndColors();

    std::pair<int, map<N, int>> getStronglyConnectedComponents();

    /* ACCESSES */
//    bool isDirected () const { return is_directed; }
//    bool haveNegativeWeight () const { return negativeWeight; }
    bool isConnected() { return getStronglyConnectedComponents().first == 1; }

    void setDensityParameter(double density) const { densityParameter = density; }

    int getNumberOfNodes() const { return nodes.size(); }

    int getNumberOfEdges() const { return edges.size(); }

    //set<node> getNodeList() const { return nodeList; }

    //set<edge> getEdgeList() const { return edgeList; }

    E getDistance(node* nodeFrom, node* nodeTo) {
        E distance = 0;
        E x, y;

        x = pow(nodeTo->getX() - nodeFrom->getX(), 2);
        y = pow(nodeTo->getY() - nodeFrom->getY(), 2);

        distance = sqrt( x + y);

        return distance;
    }
    double getDensity();

    bool isSink(N data);

    int getOutDegree(N data);

    int getInDegree(N data);

    int getPosNode(N node);

    N getData(int temp);

    void printGraph();

    ~Graph() {
        this->nodes.clear();
    }
};

typedef Graph<Traits> graph;

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
