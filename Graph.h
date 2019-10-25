// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H

#include "node.h"
#include "edge.h"
#include <unordered_map>
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
    typedef vector<edge *> EdgeSeq;
    typedef typename NodeSeq::iterator NodeIte;
    typedef typename EdgeSeq::iterator EdgeIte;
private:
    NodeSeq nodes;
    NodeIte ni; //iterator for nodes (maybe for project 2)
    EdgeIte ei; //iterator for edges (maybe for project 2)
    bool directed = false;
    unordered_map<N, multimap<E, N> > indexTable;
public:
    Graph() {}

    node *addVertex(N tag, double x, double y)
    {
        auto newNode = new node(tag, x, y); //Node or vertex
        nodes.insert({tag, newNode});

        return newNode;
    }

    node *addVertex(N tag)
    {
        auto newNode = new node(tag); //Node or vertex
        nodes.insert({tag, newNode});

        return newNode;
    }

    bool addEdge(N from, N to)
    {
//        if (findEdge(from, to)) { return false; }
        auto itNewNodeFrom = nodes.find(from);
        auto itNewNodeTo = nodes.find(to);
        auto thisNodFrom = (itNewNodeFrom->second);
        auto thisNodTo = (itNewNodeTo->second);

        auto newNodeFrom = new node{thisNodFrom->data, thisNodFrom->x, thisNodFrom->y};
        auto newNodeTo = new node{thisNodTo->data, thisNodTo->x, thisNodTo->y};
        auto weight = getDistance(newNodeFrom, newNodeTo);

        auto newEdgeFromTo = new edge(weight,newNodeFrom, newNodeTo);
        auto newEdgeToFrom = new edge(weight, newNodeTo, newNodeFrom);

        auto itNodesFrom = nodes.find(from);
        auto itNodesTo = nodes.find(to);
        /* For directed graph */
        if (directed){
            (itNodesFrom->second)->edges.emplace_back(newEdgeFromTo);
            return true;
        }

        (itNodesFrom->second)->edges.emplace_back(newEdgeFromTo);
        (itNodesTo->second)->edges.emplace_back(newEdgeToFrom);

        addToIndexTable(newNodeFrom->data, newNodeTo->data, weight);
        return true;
    }

    bool addEdge(N from, N to, E weight)
    {
//        if (findEdge(from, to)) { return false; }
        auto itNewNodeFrom = nodes.find(from);
        auto itNewNodeTo = nodes.find(to);
        auto thisNodFrom = (itNewNodeFrom->second);
        auto thisNodTo = (itNewNodeTo->second);

        auto newNodeFrom = new node{thisNodFrom->data, thisNodFrom->x, thisNodFrom->y};
        auto newNodeTo = new node{thisNodTo->data, thisNodTo->x, thisNodTo->y};

        auto newEdgeFromTo = new edge(weight,newNodeFrom, newNodeTo);
        auto newEdgeToFrom = new edge(weight, newNodeTo, newNodeFrom);

        auto itNodesFrom = nodes.find(from);
        auto itNodesTo = nodes.find(to);
        /* For directed graph */
        if (directed){
            (itNodesFrom->second)->edges.emplace_back(newEdgeFromTo);
            return true;
        }

        (itNodesFrom->second)->edges.emplace_back(newEdgeFromTo);
        (itNodesTo->second)->edges.emplace_back(newEdgeToFrom);

        addToIndexTable(newNodeFrom->data, newNodeTo->data, weight);
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

    unordered_map<N, N> bfs(N ini) { //Breath First Search

        unordered_map<N, char> color; //(W)hite, (G)ray or (B)lack
        unordered_map<N, int> distance;
        unordered_map<N, N> predecesor;
        queue<N> cola;

        if (findNode(ini) == nodes.end()) {
            auto msg = "Node " + ini + " doesn't exist";
            throw runtime_error(msg);
        }

        for (auto node = nodes.begin(); node != nodes.end(); ++node) {
            auto name = node->first;
            if (name != ini) {
                color.insert( {name, 'W'} );
                distance.insert( {name, 0} );
                predecesor.insert( {name, ""});
            }
         }

        color[ini] = 'G';
        distance[ini] = 0;

        cola.push(ini);

        while (!cola.empty()) {
            auto nodeAux = cola.front();
            cola.pop();

            auto neighbours = indexTable[nodeAux];

            for (auto it = neighbours.begin(); it != neighbours.end(); ++it) {
                auto name = it->second;
                if (color[name] == 'W') { //white
                    color[name] = 'G'; //Gray

                    distance[name] = distance[nodeAux] + 1;
                    predecesor[name] = nodeAux;
                    cola.push(name);
                }
            }
            color[nodeAux] = 'B'; //Black
        }

        return predecesor;
    }


    bool addToIndexTable(N fromNode, N toNode, E weight) {
        multimap<E, N> auxEdges;

        auto iteAdj = indexTable.find(fromNode); //Iterator

        if (iteAdj == indexTable.end()) {
            auxEdges.insert({{weight, toNode}});
            indexTable.insert({fromNode, auxEdges});
        } else {
            iteAdj->second.insert({weight, toNode});
        }

        return true;
    }

    bool deleteEdge(N from, N to) {
        /*FALTA COMPLETAR*/
        auto deleteEdge = findEdge(from, to);
        if (!deleteEdge) { return false; }

//        nodes.at(deleteEdge);

        return true;
    }

    NodeIte findNode(N tag) { return nodes.find(tag); }

    bool findEdge(N from, N to)
    {
        auto nodeFrom = nodes[from];
        if (!nodeFrom) return false;

        auto nodeTo = nodes[to];
        if (!nodeTo) return false;

        for (auto && edg: nodeFrom->edges)
            if (edg->nodes[1]->data == nodeTo->data)
                return true;

        return false;
    }

    void deleteOutEdges(N tag)
    {
        auto iteIndexTable = indexTable.find(tag); //Iterator

        if (iteIndexTable != indexTable.end()) {
            for (auto it = iteIndexTable->second.begin(); it != iteIndexTable->second.end(); ++it) {
                deleteEdge(tag, it->second);
            }
            indexTable.erase(tag);
        }
    }

    void deleteInNodes(N tag) {
        /*for (auto it = indexTable.begin(); it != indexTable.end(); ++it) {
            auto iteAdj2 = it->second.find(tag);

            if (iteAdj2 == it->second.end())
                continue;

            auto nodeFrom = it->first;
            for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
                deleteEdge(nodeFrom, tag);
        }*/
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
     *      u.data = INFINITY
     *      u.pi = NULL
     *  r.data = 0
     *  Q = G.V
     *  while Q is not empty
     *      u = EXTRACT-MIN(Q)
     *      for each v e G.Adj[u]
     *          if v e Q && w(u, v) < v.data
     *              v.pi = u
     *              v.data = w(u, v)
     * */

    self prim(N startNode) {

    }

    self kruskal();

    std::pair<bool, map<N, bool>> getBipartiteAndColors() {

    }

    std::pair<int, map<N, int>> getStronglyConnectedComponents();

    //bool isConnected() { return getStronglyConnectedComponents().first == 1; }
    bool isConnected() {
        bool result=false;
        auto firstNode = nodes.begin();

        auto bfsResult = bfs( firstNode->first );

        for(auto it = bfsResult.begin() ; it != bfsResult.end() ; ++it ) {
            auto resultFind = nodes.find( it->second );
            if ( resultFind != nodes.end() )
                result = true;
        }
        return result;
    }

    int getNumberOfNodes() const { return nodes.size(); }

    int getNumberOfEdges() const { /*return edges.size();*/ }

    E getDistance(node *nodeFrom, node *nodeTo) {
        E distance = 0;
        E x, y;

        x = pow(nodeTo->x - nodeFrom->x, 2);
        y = pow(nodeTo->y - nodeFrom->y, 2);

        distance = sqrt(x + y);

        return distance;
    }

    double getDensity() {
        double V = getNumberOfNodes();
        double E = getNumberOfEdges();

        return E / (V * (V - 1));
    }

    int getOutDegree(N data);

    int getInDegree(N data);

    int getPosNode(N node);

    void printGraph() {
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

        multimap<E, N> miniMap;
        cout << "Imprimiendo la lista de adyacencia" << endl;
        for (auto it = indexTable.begin(); it != indexTable.end(); it++) {
            cout << it->first << " : { ";
            miniMap = it->second;
            for (auto i = miniMap.begin(); i != miniMap.end(); ++i) {
                cout << "(" << i->second << " : " << i->first << ") ";
            }
            cout << "}" << endl;
        }
        unordered_map<N, E> miniMap2;
    }

    ~Graph() {
        this->nodes.clear();
/*        this->edges.clear();*/
    }
};

typedef Graph<Traits> graph;

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
