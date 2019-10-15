// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H

#include "node.h"
#include "edge.h"

struct Traits {
    typedef char N;
    typedef float E;
};

template<typename Tr>
class Graph {
public:
    typedef typename Tr::N N;
    typedef typename Tr::E E;
private:
    typedef Graph<Tr> self;
    typedef Node<self> node;
    typedef Edge<self> edge;

    typedef vector<node *> NodeSeq;
    typedef list<edge *> EdgeSeq;
    typedef typename NodeSeq::iterator NodeIte;
    typedef typename EdgeSeq::iterator EdgeIte;

    NodeSeq nodes;
    NodeIte ni;
    EdgeIte ei;

    const double densityParameter = 0.5;
    std::set<node> nodeList;
    std::set<edge> edgeList;
    std::map<N, set<N>> adjList;
    std::map<N, set<N>> adjList_Transposed;
//    bool is_directed; bool negativeWeight;

public:
    Graph() {}

    /* FUNCTIONS */
    bool addNode(N tag, double x, double y);

    bool deleteNode(N tag);

    bool addEdge(N from, N to, E weight);

    bool deleteEdge(N from, N to);

    node *findNode(N tag);

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

    int getNumberOfNodes() const { return nodeList.size(); }

    int getNumberOfEdges() const { return edgeList.size(); }

    set<node> getNodeList() const { return nodeList; }

    set<edge> getEdgeList() const { return edgeList; }

    double getDensity();

    bool isSink(N data);

    int getOutDegree(N data);

    int getInDegree(N data);

    int getPosNode(N node);

    N getData(int temp);

    void printGraph();

    ~Graph() {

    }
};

typedef Graph<Traits> graph;


#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
