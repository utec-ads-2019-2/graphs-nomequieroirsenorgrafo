// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H

#include <bits/stdc++.h>
using namespace std;

struct Traits {
    typedef char N;
    typedef float E;
};

template <typename Tr>
class Graph{
private:

    template <typename G>
    class Node {
    private:
        typedef typename G::N N;
        typedef typename G::EdgeSeq EdgeSeq;
        EdgeSeq edges;
        N data;
        double x, y;
    public:
        Node() : x{0}, y{0} {}
        Node (N data, double _x, double _y): data{data}, x{_x}, y{_y} {}
        N getData () const { return this->data; }
        double getX () const { return this->x; }
        double getY () const { return this->y; }
        bool operator < (const Node& other) const { return this->data < other.data; }
        bool operator == (const Node& other) const { return this->data == other.data; }
    };

    template <typename G>
    class Edge {
    private:
        typedef typename G::E E;
        typedef typename G::node node;
        E data;
        node* nodes[2]; // Another option: node* from; node* to;
    public:
        Edge () = default;
        explicit Edge (E data): data{data}, nodes{nullptr} {}
        E getData () const { return this->data; }
        node* getFrom () const { return nodes[0]; }
        node* getTo () const { return nodes[1]; }
        std::pair <node, node> getNodes () const { return {nodes[0], nodes[1]}; }
        bool operator < (const Edge& other) const {
            if (this->data != other.data) return this->data < other.data;
            if (nodes[0] != other.from) return nodes[0] < other.from;
            return nodes[1] < other.nodes[1];
        }
        bool operator == (const Edge& other) const {
            return  nodes[0] == other.nodes[0] and nodes[1] == other.nodes[1];
        }
    };

    typedef Graph<Tr> self;
    typedef Node<self> node;
    typedef Edge<self> edge;
    typedef vector<node*> NodeSeq;
    typedef list<edge*> EdgeSeq;

    typedef typename Tr::N N;
    typedef typename Tr::E E;
    typedef typename NodeSeq::iterator NodeIte;
    typedef typename EdgeSeq::iterator EdgeIte;

    NodeSeq nodes;
    NodeIte ni;
    EdgeIte ei;

    const double densityParameter = 0.5;
    std::set <node> nodeList;
    std::set <edge> edgeList;
    std::map <N, set <N>> adjList;
    std::map <N, set <N>> adjList_Transposed;
//    bool is_directed; bool negativeWeight;

public:
    Graph () {}
    ~Graph () {}

    /* FUNCTIONS */
    bool addNode (N tag, double x, double y);
    bool deleteNode (N tag);
    bool addEdge (N from, N to, E weight);
    bool deleteEdge (N from, N to);
    node* findNode (N tag);
    edge* findEdge (N from, N to);
    self prim (N data);
    self kruskal ();
    std::pair <bool, map <N, bool>> getBipartiteAndColors ();
    std::pair <int, map <N, int>> getStronglyConnectedComponents ();

    /* ACCESSES */
//    bool isDirected () const { return is_directed; }
//    bool haveNegativeWeight () const { return negativeWeight; }
    bool isConnected () { return getStronglyConnectedComponents().first == 1; }
    void setDensityParameter (double density) const { densityParameter = density; }
    int getNumberOfNodes () const { return nodeList.size(); }
    int getNumberOfEdges () const { return edgeList.size(); }
    set <node> getNodeList () const { return nodeList; }
    set <edge> getEdgeList () const { return edgeList; }

    double getDensity ();
    bool isSink (N data);
    int getOutDegree (N data);
    int getInDegree(N data);
    int getPosNode (N node);
    N getData (int temp);

    void printGraph();
};

typedef Graph<Traits> graph;


#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
