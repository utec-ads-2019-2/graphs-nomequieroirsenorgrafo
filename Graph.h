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

    typedef unordered_map<N, node *> NodeSeq;
    typedef unordered_map<N, edge *> EdgeSeq; /*typedef vector<edge*> EdgeSeq;*/
    typedef typename NodeSeq::iterator NodeIte;
    typedef typename EdgeSeq::iterator EdgeIte;
private:
    NodeSeq nodes;
    EdgeSeq edges;
    NodeIte ni;
    EdgeIte ei;

    //std::set<node*> nodeList;
    //std::set<edge> edgeList;
    unordered_map<N, unordered_map<N, E> > adjList; /*map <N, set <N>> adjList;*/
    std::unordered_map<N, unordered_map<N, E> > adjList_Transposed; /*map <N, set <N>> adjList_Trans;*/
public:
    Graph() {}

    node *addNode(N tag, double x, double y)
    {
        auto newNode = new node(tag, x, y); //Node or vertex

        nodes.insert({tag, newNode});

        return newNode;
    }

    bool addEdge(N from, N to) {
        auto nodeFromIte = findNode(from);
        auto nodeToIte = findNode(to);
        node *nodeFrom = nullptr;
        node *nodeTo = nullptr;
        //Search nodes and create if don't exist
        if (nodeFromIte == nodes.end())
            nodeFrom = addNode(from, 0, 0);
        else
            nodeFrom = nodeFromIte->second;

        if (nodeToIte == nodes.end())
            nodeTo = addNode(to, 0, 0);
        else
            nodeTo = nodeToIte->second;

        auto weight = getDistance(nodeFrom, nodeTo);

        auto newEdge = new edge(weight);

        newEdge->setNodes(nodeFrom, nodeTo);

        edges.insert({from + to, newEdge});

        addtoAdjacentList(from, to, weight);

        return true;
    }

    bool deleteNode(N tag) {
        bool result = false;
        auto nodeIte = findNode(tag);

        if (nodeIte != nodes.end()) {
            result = true;
            nodes.erase(nodeIte);
        }

        // Deletes edges with Node as origin and destination
        deleteOutEdges(tag);
        deleteInNodes(tag);

        return result;
    }

    bool addtoAdjacentList(N fromNode, N toNode, E weight) {
        unordered_map<N, E> auxEdges;

        auto iteAdj = adjList.find(fromNode); //Iterator

        if (iteAdj == adjList.end()) {
            auxEdges.insert({{toNode, weight}});
            adjList.insert({fromNode, auxEdges});
        } else {
            iteAdj->second.insert({toNode, weight});
        }

        return true;
    }

    bool deleteEdge(N from, N to) {
        bool result = false;
        auto edgeIte = findEdge(from, to);

        if (edgeIte != edges.end()) {
            result = true;
            edges.erase(edgeIte);
        }

        return result;
    }

    NodeIte findNode(N tag) { return nodes.find(tag); }
    EdgeIte findEdge(N from, N to) { return edges.find(from + to);}

    void deleteOutEdges(N tag) {
        auto iteAdj = adjList.find(tag); //Iterator

        if (iteAdj != adjList.end()) {
            for (auto it = iteAdj->second.begin(); it != iteAdj->second.end(); ++it) {
                deleteEdge(tag, it->first);
            }
            adjList.erase(tag);
        }
    }

    void deleteInNodes(N tag) {
        for (auto it = adjList.begin(); it != adjList.end(); ++it) {
            auto iteAdj2 = it->second.find(tag);

            if (iteAdj2 == it->second.end())
                continue;

            auto nodeFrom = it->first;
            for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
                deleteEdge(nodeFrom, tag);
        }
    }

    // Here!!! Felix
    void recorridoMinimapita(N fromNode) {
        unordered_map<N, E> miniMapita;
        auto iteAdj = adjList.find(fromNode); //busco el nodo
        cout << "\n\ndel nodo: " << fromNode << endl;
        cout << "salen los nodos " << endl;
        if (iteAdj != adjList.end()) //valida que exista
            miniMapita = iteAdj->second;
        // recorro el minimapita que son los nodos salientes
        for (auto it = miniMapita.begin() ; it != miniMapita.end() ; ++it) {
            cout << it->first << " con peso" <<it->second << endl;
        }
    }


    /* STEPS (ERNESTO BOOK):
     * 1. Se crea un grafo M con los mismos vértices del grafo original G y sin arcos.
     * 2. En G se selecciona un vértice de partida Vo que se marca como visitado.
     * 3. Los arcos de Vo, cuyos vértices destino no han sido visitados, se encolan en C.
     * 4. Se desencola de C el arco menor en peso y se copia en M.
     * 5. El vértice destino del arco de menor peso Vd se marca como visitado en G
     * 6. Vo es ahora igual a Vd.
     * 7. Se repiten los pasos del 3 al 6 hasta que el número de vértices marcados como visitados*/

    /* PSEUDO CODE (CORMEN) : Q->min priority queue
     * PRIM(G, w, r)
     *  for each u e G.V
     *      u.key = INFINITY
     *      u.pi = NULL
     *  r.key = 0
     *  Q = G.V
     *  while Q is not empty
     *      u = EXTRACT-MIN(Q)
     *      for each v e G.Adj[u]
     *          if v e Q && w(u, v) < v.key
     *              v.pi = u
     *              v.key = w(u, v)
     * */

    self prim(N startNode)
    {
        unordered_map<N, N> parent;
        unordered_map<N, bool> visitedNodes;
        unordered_map<N, E> weightEdges;

        priority_queue<pair<E, N>, vector<pair<E, N>>, greater<pair<E, N>>> minHeap;

        self MST;

        minHeap.push(make_pair(0, startNode));
        parent[startNode] = startNode;
        while(!minHeap.empty())
        {
            N curr = minHeap.top().second;
            E weig = minHeap.top().first;
            minHeap.pop();

            if(visitedNodes[curr]) { continue; }
            visitedNodes[curr] = true;

            if(MST.nodes.find(curr) == MST.nodes.end()){
//                MST->addNode( , , );
            }
            if(parent[curr] != curr){
//                MST->addEdge( , , );
            }
            auto iteAdj = adjList.find(startNode); //busco el nodo

            /* Itero sobre el minimapita*/
            for (auto it = (iteAdj->second).begin() ; it != (iteAdj->second).end() ; ++it)
            {
                N nd = it->first;
                E w = it->second;
                if(visitedNodes[nd]) { continue; }
                if(weightEdges.find(nd) == weightEdges.end() || weightEdges[nd] < w){
                    parent[nd] = curr;
                    weightEdges[nd] = w;
                    minHeap.push(make_pair(w, nd));
                }
            }
        }
        return MST;
    }

    self kruskal();

    std::pair<bool, map<N, bool>> getBipartiteAndColors();

    std::pair<int, map<N, int>> getStronglyConnectedComponents();

    /* ACCESSES */
//    bool isDirected () const { return is_directed; }
//    bool haveNegativeWeight () const { return negativeWeight; }
    bool isConnected() { return getStronglyConnectedComponents().first == 1; }

    int getNumberOfNodes() const { return nodes.size(); }
    int getNumberOfEdges() const { return edges.size(); }
    //set<node> getNodeList() const { return nodeList; }
    //set<edge> getEdgeList() const { return edgeList; }

    E getDistance(node *nodeFrom, node *nodeTo)
    {
        E distance = 0;
        E x, y;

        x = pow(nodeTo->getX() - nodeFrom->getX(), 2);
        y = pow(nodeTo->getY() - nodeFrom->getY(), 2);

        distance = sqrt(x + y);

        return distance;
    }

    double getDensity()
    {
        double V = getNumberOfNodes();
        double E = getNumberOfEdges();

        return E / (V * (V - 1));
    }

    bool isSink(N data);

    int getOutDegree(N data);

    int getInDegree(N data);

    int getPosNode(N node);

    N getData(int temp);

    void printGraph()
    {
//        cout << "Imprimiendo nodos" << endl;
//        for(auto i: nodeList){
//            cout << i.getTag() << " ";
//        }
//        cout << endl;
//        cout << "Imprimiendo aristas" << endl;
//        for(auto i: edgeList){
//            cout << i.getNodes().first <<" "<< i.getNodes().second<< " "  << i.getWeight() << endl;;
//        }
//        cout << endl;

        unordered_map<N, E> miniMap;
        cout << "Imprimiendo la lista de adyacencia" << endl;
        for (auto it = adjList.begin();it != adjList.end();it++){
            cout << it->first<< " : { ";
            miniMap = it->second;
            for (auto  i =  miniMap.begin(); i != miniMap.end(); ++i){
                cout << "(" << i->first << " : " << i->second << ") ";
            }
            cout << "}" << endl;
        }
        unordered_map<N, E> miniMap2;
        cout << "Imprimiendo la lista de adyacencia transpuesta" << endl;
        for (auto it2 = adjList_Transposed.begin();it2 != adjList_Transposed.end();it2++){
            cout << it2->first<< " : { ";
            miniMap2 = it2->second;
            for (auto  i2 =  miniMap2.begin(); i2 != miniMap2.end(); ++i2){
                cout << "(" << i2->first << " : " << i2->second << ") ";
            }
            cout << "}" << endl;
        }

    }

    ~Graph() {
        this->nodes.clear();
        this->edges.clear();
    }
};

typedef Graph<Traits> graph;

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
