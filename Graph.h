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
    typedef unordered_map<N, edge *> EdgeSeq;
    typedef typename NodeSeq::iterator NodeIte;
    typedef typename EdgeSeq::iterator EdgeIte;
private:
    NodeSeq nodes;
    EdgeSeq edges;
    NodeIte ni; //iterator for nodes (maybe for project 2)
    EdgeIte ei; //iterator for edges (maybe for project 2)

    unordered_map<N, multimap<E, N> > indexTable;
public:
    Graph() {}

    node *addNode(N tag, double x, double y) {
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

        newEdge->nodes[0] = nodeFrom;
        newEdge->nodes[1] = nodeTo;

        edges.insert({from + to, newEdge});

        addtoIndexTable(from, to, weight);

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

    /*void solve(N ini) {
        queue<N> cola;
        unordered_map<N, bool> visited; //Nodes visited (BFS)

        for(auto node=nodes.begin() ; node != nodes.end() ; ++node )
            visited.insert({ (*node).data , false});

        cola.push(ini);
        visited[ ini ] = true;

        while(!cola.empty()) {
            auto nodeAux = cola.front();
            cola.pop();

            auto result = indexTable[ nodeAux ];
            auto neighbours = result->second;

            for(auto it=neighbours.begin(); it != neighbours.end() ; ++it) {
                if(!visited[*it]) {
                    cola.push( *it );
                    visited[ *it ] = true;
                }
            }
        }

    }*/

    bool addtoIndexTable(N fromNode, N toNode, E weight) {
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
        bool result = false;
        auto edgeIte = findEdge(from, to);

        if (edgeIte != edges.end()) {
            result = true;
            edges.erase(edgeIte);
        }

        return result;
    }

    NodeIte findNode(N tag) { return nodes.find(tag); }

    EdgeIte findEdge(N from, N to) { return edges.find(from + to); }

    void deleteOutEdges(N tag) {
        auto iteAdj = indexTable.find(tag); //Iterator

        if (iteAdj != indexTable.end()) {
            for (auto it = iteAdj->second.begin(); it != iteAdj->second.end(); ++it) {
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

    // Here!!! Felix
    void recorridoMinimapita(N fromNode) {
        multimap<E, N> miniMapita;
        auto iteAdj = indexTable.find(fromNode); //busco el nodo
        cout << "\n\ndel nodo: " << fromNode << endl;
        cout << "salen los nodos " << endl;
        if (iteAdj != indexTable.end()) //valida que exista
            miniMapita = iteAdj->second;
        // recorro el minimapita que son los nodos salientes
        for (auto it = miniMapita.begin(); it != miniMapita.end(); ++it) {
            cout << it->first << " con peso " << it->second << endl;
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

    self prim(N startNode) {
        unordered_map<N, N> parent;
        unordered_map<N, bool> visitedNodes;
        unordered_map<N, E> weightEdges;

        priority_queue<pair<E, N>, vector<pair<E, N>>, greater<pair<E, N>>> minHeap;

        self MST;

        minHeap.push(make_pair(0, startNode));
        parent[startNode] = startNode;
        while (!minHeap.empty()) {
            N curr = minHeap.top().second;
            E weig = minHeap.top().first;
            minHeap.pop();

            if (visitedNodes[curr]) { continue; }
            visitedNodes[curr] = true;

            if (MST.nodes.find(curr) == MST.nodes.end()) {
//                MST->addNode( , , );
            }
            if (parent[curr] != curr) {
//                MST->addEdge( , , );
            }
            auto iteAdj = indexTable.find(startNode); //busco el nodo

            /* Itero sobre el minimapita*/
            for (auto it = (iteAdj->second).begin(); it != (iteAdj->second).end(); ++it) {
                auto w = it->first;
                auto nd = it->second;
                if (visitedNodes[nd]) { continue; }
                if (weightEdges.find(nd) == weightEdges.end() || weightEdges[nd] < w) {
                    parent[nd] = curr;
                    weightEdges[nd] = w;
                    minHeap.push(make_pair(w, nd));
                }
            }
        }
        return MST;
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

    int getNumberOfEdges() const { return edges.size(); }

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
        this->edges.clear();
    }
};

typedef Graph<Traits> graph;

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
