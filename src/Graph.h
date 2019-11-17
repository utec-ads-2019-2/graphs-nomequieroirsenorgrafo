// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H

#include "components/node.h"
#include "components/edge.h"
#include "components/DisjoinSet.h"

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
    typedef typename NodeSeq::const_iterator NodeIte;
private:
    NodeSeq nodes;
    bool directed;

    NodeIte findVertex(N tag) { return nodes.find(tag); }

public:
    Graph() : directed(false) {}
    Graph(bool directed) : directed(directed) {}

    Graph(const Graph &oldGraph) { //Copy constructor

        if ( this->nodes.size() == 0 ) {
            for( auto nodeIte = oldGraph.firstNode() ;
                 nodeIte != oldGraph.lastNode() ;
                 nodeIte++) {
                this->nodes.insert( *nodeIte );
            }
        }

    }

    bool isDirected(){ return directed; }

    void addVertex(node *oldNode) {
        node *newNode = new node(oldNode);
        nodes[oldNode->data] = newNode;
    }

    node *addVertex(N tag) {
        auto newNode = new node(tag); //Node or vertex
        nodes.insert({tag, newNode});

        return newNode;
    }

    node *addVertex(N tag, double x, double y) {
        auto newNode = new node(tag, x, y); //Node or vertex
        nodes.insert({tag, newNode});

        return newNode;
    }
    bool addEdge(N from, N to, E weight) {
        if ((nodes.find(from) == nodes.end()) || (nodes.find(to) == nodes.end())) { return false; }

        node *nodeFrom = nodes[from];
        node *nodeTo = nodes[to];

        if (this->directed) {
            auto edgeFromTo = new edge(nodeFrom, nodeTo, weight);
            nodeFrom->edges.emplace_back(edgeFromTo);
            return true;
        }

        auto edgeFromTo = new edge(nodeFrom, nodeTo, weight);
        nodeFrom->edges.emplace_back(edgeFromTo);
        auto edgeToFrom = new edge(nodeTo, nodeFrom, weight);
        nodeTo->edges.emplace_back(edgeToFrom);

        return true;
    }

    bool addEdge(N from, N to) {
        if ((nodes.find(from) == nodes.end()) || (nodes.find(to) == nodes.end())) { return false; }

        auto weight = getDistance(nodes[from], nodes[to]);

        node *nodeFrom = nodes[from];
        node *nodeTo = nodes[to];

        if (this->directed) {
            auto newEdge = new edge(nodeFrom, nodeTo, weight);
            nodeFrom->edges.emplace_back(newEdge);
            return true;
        }

        auto edgeFromTo = new edge(nodeFrom, nodeTo, weight);
        nodeFrom->edges.emplace_back(edgeFromTo);
        auto edgeToFrom = new edge(nodeTo, nodeFrom, weight);
        nodeTo->edges.emplace_back(edgeToFrom);

        return true;
    }

    bool addDirectedEdge(N from, N to) {
        if ((nodes.find(from) == nodes.end()) || (nodes.find(to) == nodes.end())) { return false; }

        auto weight = getDistance(nodes[from], nodes[to]);

        node *nodeFrom = nodes[from];
        node *nodeTo = nodes[to];

        auto newEdge = new edge(nodeFrom, nodeTo, weight);
        nodeFrom->edges.emplace_back(newEdge);
        return true;

    }

    bool deleteNode(N tag) {
        bool result = false;
        EdgeSeq *edges;
        auto nodeIte = findVertex(tag);

        if (nodeIte != nodes.end()) {
            result = true;
            nodes.erase(nodeIte);
        }
        // Deletes edges with Node(tag) as destination
        for (auto nodeIt = nodes.begin(); nodeIt != nodes.end(); ++nodeIt) {
            edges = &(*nodeIt).second->edges;
            for (auto edgeIt = edges->begin(); edgeIt != edges->end(); ++edgeIt) {
                auto nodeTo = (*edgeIt)->nodes[1]->data;
                if (nodeTo == tag) {
                    edges->erase(edgeIt);
                    break;
                }

            }
        }

        return result;
    }

    bool deleteEdge(N from, N to) {
        auto delEdge = findEdge(from, to);

        if (!delEdge)
            return false;
        else {
            auto edges = &nodes[from]->edges;
            for (auto edgeIt = edges->begin(); edgeIt != edges->end(); ++edgeIt) {
                auto edge = *edgeIt;
                if (edge->nodes[1]->data == to) {
                    edges->erase(edgeIt);
                    break;
                }
            }
        }

        if (!this->directed) {
            delEdge = findEdge(to, from);
            if (!delEdge)
                return false;
            else {
                auto edges = &nodes[to]->edges;
                for (auto edgeIt = edges->begin(); edgeIt != edges->end(); ++edgeIt) {
                    auto edge = *edgeIt;
                    if (edge->nodes[1]->data == from) {
                        edges->erase(edgeIt);
                        break;
                    }
                }
            }
        }

        return true;
    }

    bool findvertex(N tag) {
        auto result = this->findVertex(tag);

        if (result == this->nodes.end())
            return false;
        else
            return true;
    }

    node *findNode(N tag) {
        if (nodes.find(tag) != nodes.end()) {
            node *resultantNode = new node(tag);
            return resultantNode;
        }
        return nullptr;
    }

    edge *findEdge(N from, N to) {
        auto nodeFrom = nodes[from];
        if (!nodeFrom) return nullptr;

        auto nodeTo = nodes[to];
        if (!nodeTo) return nullptr;

        for (auto &&edg: nodeFrom->edges)
            //if (edg->nodes[1]->data == nodeTo->data)
            if (edg->nodes[1] == nodeTo)
                return edg;

        return nullptr;
    }

	self getReversedGraph() {
		self g{directed};

		for (auto it = nodes.begin(); it != nodes.end(); it++) {
            auto neighbours = it->second->edges;
			g.addVertex(it->second);

            for (auto neighIter = neighbours.begin(); neighIter != neighbours.end(); ++neighIter) {
				g.addDirectedEdge((*neighIter)->nodes[1]->data, (*neighIter)->nodes[0]->data);
            }
		}

		return g;
	}

	void bfsVisitor(N ini, bool visited[]) {
		queue<N> cola;

        cola.push(ini);

        while (!cola.empty()) {
            auto nodeAux = cola.front();
            cola.pop();

            auto neighbours = nodes[nodeAux]->edges;

			int idx = 0;
            for (auto it = neighbours.begin(); it != neighbours.end(); ++it) {
                auto name = (*it)->nodes[1]->data;
				if (!visited[idx]) {
					visited[idx] = true;
					cola.push(name);
				}
				idx++;
            }
        }
	}

    unordered_map<N, N> bfs(N ini) { //Breadth First Search

        unordered_map<N, char> color; //(W)hite, (G)ray or (B)lack
        unordered_map<N, int> distance;
        unordered_map<N, N> predecesor;
        queue<N> cola;

        if (findVertex(ini) == nodes.end()) {
            auto msg = "Node " + ini + " doesn't exist";
            throw runtime_error(msg);
        }

        for (auto node = nodes.begin(); node != nodes.end(); ++node) {
            auto name = node->first;
            if (name != ini) {
                color.insert({name, 'W'});
                distance.insert({name, 0});
                predecesor.insert({name, ""});
            }
        }

        color[ini] = 'G';
        distance[ini] = 0;

        cola.push(ini);

        while (!cola.empty()) {
            auto nodeAux = cola.front();
            cola.pop();

            auto neighbours = nodes[nodeAux]->edges;

            for (auto it = neighbours.begin(); it != neighbours.end(); ++it) {
                auto name = (*it)->nodes[1]->data;
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

    self *prim(N start) {
        if ( nodes.find(start) == nodes.end() ) { throw runtime_error("Selected vertex is not part of the graph"); }
        if ( directed ) { throw runtime_error("Prim only works on undirected graphs"); }

        self *mst = new self;
        unordered_map<N, N> origin;
        unordered_map<N, bool> visited;
        unordered_map<N, E> weight;
        priority_queue<pair<E, N>, std::vector<pair<E, N>>, greater<>> Q;

        Q.push(make_pair(0, start));
        origin[start] = start;

        for (auto iterNod = nodes.begin(); iterNod != nodes.end(); ++iterNod) {
            mst->addVertex(iterNod->first);
        }

        while (!Q.empty())
        {
            N dest = Q.top().second;
            E minWeight = Q.top().first;
            Q.pop();

            if (visited[dest]) { continue; }
            visited[dest] = true;

            if (origin[dest] != dest) { mst->addEdge(origin[dest], dest, minWeight); }

            for (auto edg : nodes.at(dest)->edges)
            {
                auto adjNode = edg->nodes[1]->data;
                auto adjWeight = edg->weight;
                if (visited[adjNode]) { continue; }
                if (weight.find(adjNode) == weight.end() || adjWeight < weight[adjNode]) {
                    origin[adjNode] = dest;
                    weight[adjNode] = adjWeight;
                    Q.push(make_pair(adjWeight, adjNode));
                }
            }
        }

        return mst;
    }

    self *kruskal() {
        DisjointSet<N> disjointSet;
        self *mst = new self;
        multimap<E, pair<N, N> > sorted;

        for (auto iterNod = nodes.begin(); iterNod != nodes.end(); ++iterNod) {
            mst->addVertex(iterNod->first);
        }

        //Get the edged sort ascendent
        for (auto nodeIt = nodes.begin(); nodeIt != nodes.end(); ++nodeIt) {
            auto edges = (*nodeIt).second->edges;
            auto v = nodeIt->first;
            disjointSet.makeSet(v);
            for (auto edgeIt = edges.begin(); edgeIt != edges.end(); ++edgeIt) {
                auto edge = *edgeIt;
                sorted.insert({edge->weight, make_pair(edge->nodes[0]->data, edge->nodes[1]->data)});
            }
        }

        for (auto itSorted = sorted.begin(); itSorted != sorted.end(); ++itSorted) {
            auto w = itSorted->first;
            auto pairNN = itSorted->second;
            auto u = pairNN.first;
            auto v = pairNN.second;
            if (disjointSet.findSet(u) != disjointSet.findSet(v)) {
                mst->addEdge(u, v, w);
                disjointSet.makeUnion(u, v);
            }
        }

        return mst;
    }

    auto floydWarshall(){
        unordered_map< N, unordered_map<N, E> > distances;
        unordered_map< N, unordered_map<N, N> > path;

        // Setear a 0 las diagonales y poner INFINITY a lo demas
        for (auto && itMapVertexU: this->nodes)
        {
            for (auto && itMapVertexV: this->nodes) {
                distances[itMapVertexU.first][itMapVertexV.first] = INFINITY;
            }
            distances[itMapVertexU.first][itMapVertexU.first] = 0;
        }

        // Poner los pesos en cada parte de la matriz 0
        for (auto && itVertex: this->nodes)
        {
            for (auto && itEdge : itVertex.second->edges)
            {
                auto nodeFrom = itVertex.first;
                auto nodeTo = itEdge->nodes[1]->data;
                auto edgeWeight = itEdge->weight;

                distances[nodeFrom][nodeTo] = edgeWeight;
                path[nodeFrom][nodeTo] = nodeTo;
            }
        }

        // Empieza el algoritmo de floyd wharshall
        for(auto && itForEachMatrix : this->nodes)
        {
            for(auto && rowsOfTheMatrix : this->nodes)
            {
                for(auto && columnsOfTheMatrix : this->nodes)
                {
                    auto k = itForEachMatrix.first;
                    auto i = rowsOfTheMatrix.first;
                    auto j = columnsOfTheMatrix.first;
                    // A[i,j] = min(A[i, j], A[i,k] + A[k,j]);
                    distances[i][j] = std::min(distances[i][j], distances[i][k] + distances[k][j]);
                    path[i][j] = k;
                }
            }
        }

        return std::make_pair(distances, path);
    }

    bool isBipartite() {
        set<N> red_colored;
        set<N> blue_colored;
        int i;
        bool isbipartite = true;
        unordered_map<N, bool> visited;
        queue<pair<N, char> > cola;
        N nodeIni;
        char next_color;

        auto nodeIt = nodes.begin();
        if (nodeIt == nodes.end()) {
            auto msg = "Node " + nodeIni + " doesn't exist";
        } else
            nodeIni = nodeIt->first;

        for (auto nodeIt = nodes.begin(); nodeIt != nodes.end(); ++nodeIt)
            visited.insert({nodeIt->second->data, false});

        i = 0;
        cola.push(make_pair(nodeIni, 'R'));

        isbipartite = assignColor(nodeIni, i, isbipartite, &red_colored, &blue_colored);
        ++i;

        while (!cola.empty()) {
            auto nodeAux = cola.front();
            cola.pop();

            auto neighbours = nodes[nodeAux.first]->edges;

            if (visited[nodeAux.first])
                continue;

            visited[nodeAux.first] = true;

            next_color = (nodeAux.second == 'R') ? 'B' : 'R';

            for (auto it = neighbours.begin(); it != neighbours.end(); ++it) {
                auto name = (*it)->nodes[1]->data;

                isbipartite = assignColor(name, next_color, isbipartite, &red_colored, &blue_colored);
                if (!isbipartite)
                    break;

                next_color = (nodeAux.second == 'R') ? 'B' : 'R';

                cola.push(make_pair(name, next_color));
            }
            ++i;
            if (!isbipartite)
                break;
        }

        return isbipartite;
    }

    bool assignColor(N tag, char color, bool isbipartite, set<N> *red, set<N> *blue) {
        bool status = isbipartite;

        switch (color) {
            case 'R': {
                auto result = blue->find(tag);
                if (result != blue->end())
                    status = false;
                else
                    red->insert(tag);
            }
                break;
            case 'B': {
                auto result = red->find(tag);
                if (result != red->end())
                    status = false;
                else
                    blue->insert(tag);
            }
                break;
        }

        return status;
    }

    bool isStronglyConnected() {

		auto numNodes = getNumberOfNodes();
		bool visited[numNodes];
		for (int i = 0; i < numNodes; ++i)
			visited[i] = false; 

		auto firstNode = nodes.begin();
		bfsVisitor(firstNode->first, visited);
		for (int i = 0; i < numNodes; i++) 
			if (visited[i] == false) 
				 return false; 

		// Undirected graphs just need to do a BFS and check that all nodes were visited
		if (!directed)
			return true;

		self gr = getReversedGraph();

		for (int i = 0; i < numNodes; ++i)
			visited[i] = false; 

		auto firstNodeR = gr.nodes.begin();
		gr.bfsVisitor(firstNodeR->first, visited);
		for (int i = 0; i < numNodes; i++) 
			if (visited[i] == false) 
				 return false; 

        return true;
    }

    bool isConnected() {
        bool result = false;
        auto firstNode = nodes.begin();

        auto bfsResult = bfs(firstNode->first);

        for (auto it = bfsResult.begin(); it != bfsResult.end(); ++it) {
            auto resultFind = nodes.find(it->second);
            if (resultFind != nodes.end())
                result = true;
        }
        return result;
    }

    int getNumberOfNodes() const { return nodes.size(); }

    int getNumberOfEdges() const {
        int result = 0;
        for (auto itNods = nodes.begin(); itNods != nodes.end(); ++itNods) {
            node *nod = itNods->second;
            auto sizeOfEdgesForEachNode = nod->sizeEdges();
            result += sizeOfEdgesForEachNode;
        }
        if (!this->directed)
            result = result / 2;

        return result;
    }

    E getEdgesWeightSum() {
        E sum = 0;

        for (auto nodeIt = nodes.begin(); nodeIt != nodes.end(); ++nodeIt) {
            auto node = nodeIt->second;
            auto edges = node->edges;
            for (auto edgeIt = edges.begin(); edgeIt != edges.end(); ++edgeIt) {
                auto edge = *edgeIt;
                sum += edge->weight;
            }
        }

        if (this->directed)
            return sum;
        else
            return sum / 2;
    }

    E getDistance(node *nodeFrom, node *nodeTo) {
        const int EARTH_DIAMETER = 12742;
        auto latFrom = nodeFrom->x; auto latTo = nodeTo->x;
        auto longFrom = nodeFrom->y; auto longTo = nodeTo->y;

        auto distance =  EARTH_DIAMETER*asin(sqrt(pow(sin((latTo - latFrom) * (M_PI / 180) / 2), 2) +
                                         pow(sin((longTo - longFrom) * (M_PI / 180) / 2), 2) *
                                         cos(latFrom * M_PI / 180) * cos(latTo * M_PI / 180)));

        return distance;
    }

    double getDensity() {
        double V = getNumberOfNodes();
        double E = getNumberOfEdges();

        auto density = E / (V * (V - 1));

        if (this->directed)
            return density;
        else
            return 2 * density;
    }

    void printGraph() {
        cout << "Imprimiendo GRAFOOOO" << endl;
        for (auto it = nodes.begin(); it != nodes.end(); it++) {
            cout << it->first << " : { ";
            auto miniMap = it->second;
            for (auto i: miniMap->edges) {
                cout << "(" << (i->nodes[0])->data << " : " << (i->nodes[1])->data << " Peso: " << i->weight << ") ";
            }
            cout << "}" << endl;
        }
        cout << endl;
    }
//  Iterators
    NodeIte firstNode() const {
        return this->nodes.begin();
    }

    NodeIte lastNode() const {
        return this->nodes.end();
    }
    ~Graph() {
        this->nodes.clear();
    }
};

typedef Graph<Traits> graph;

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
