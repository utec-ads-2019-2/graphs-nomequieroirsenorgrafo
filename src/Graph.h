// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_GRAPH_H

#include "../components/node.h"
#include "../components/edge.h"
#include "../components/DisjoinSet.h"

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

private:
    NodeIte findVertex(N tag) { return nodes.find(tag); }

    void doDFS(self* DFSGraphTree, N currentVertex, unordered_map<N, bool>& visited, unordered_map<N, N>& parent)
    {
        if (visited[currentVertex]) { return; }
        visited[currentVertex] = true;
//         cout << currentVertex << endl; // Prints the correct order

        if(DFSGraphTree->nodes.find(currentVertex) == DFSGraphTree->nodes.end()){
            DFSGraphTree->addVertex(this->nodes[currentVertex]);
        }
        if(parent[currentVertex] != currentVertex){
            DFSGraphTree->addEdge(parent[currentVertex], currentVertex);
        }

        for (auto && next : this->nodes[currentVertex]->edges)
        {
            auto nextVertex = next->nodes[1]->data;
            parent[nextVertex] = currentVertex;
            doDFS(DFSGraphTree, nextVertex, visited, parent);
        }
    }

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


    auto dfs(N start){
        unordered_map<N, bool> visited;
        unordered_map<N, N> parent;
        auto DFSGraphTree = self(true);

        doDFS(&DFSGraphTree, start, visited, parent);

        return DFSGraphTree;
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

    auto dijkstra(N start, N end){
        auto parentsWithWeight = dodijkstra(start);
        unordered_map<N, N> parents;
        for (auto && parent : parentsWithWeight){
            parents[parent.first] = parentsWithWeight[parent.first].first;
        }
        auto dijkstraGraph = this->buildGraphFromPath(start, end, parents);
        return dijkstraGraph;
    }

    auto dijkstra(N start){
        auto graphTree = self(true);
        auto parents = dodijkstra(start);
        for (auto iterNod = this->nodes.begin(); iterNod != this->nodes.end(); ++iterNod) {
            graphTree.addVertex(iterNod->first);
        }

        for (auto iterNod = this->nodes.begin(); iterNod != this->nodes.end(); ++iterNod)
        {
            for(auto && e : this->nodes[iterNod->first]->edges)
            {
                auto dest = e->nodes[1]->data;
                if (parents[dest].first != dest /*&& !graphTree.findEdge(parents[dest].first, dest)*/) {
                    graphTree.addEdge(parents[dest].first, dest, parents[dest].second);
                }
            }
        }
        return graphTree;
    }

    // SHOULD BE PRIVATE
    auto dodijkstra(N start){
        unordered_map<N, E> distances;
        unordered_map< N, pair<N, E> > parents;
        priority_queue<pair<E, N>, vector<pair<E,N>>, greater<pair<E, N>>> minPQ;

        parents[start] = make_pair(start, 0);
        minPQ.push(make_pair(0, start));

        while(!minPQ.empty()){
            auto current = minPQ.top().second; minPQ.pop();
            auto currentDistance = distances[current];

            for(auto && edge : this->nodes[current]->edges){
                auto adjacentVertex = edge->nodes[1]->data;
                auto candidateDistance = edge->weight + currentDistance;
                if(distances.find(adjacentVertex) == distances.end()){
                    parents[adjacentVertex] = make_pair(current, edge->weight);
                    distances[adjacentVertex] = candidateDistance;
                    minPQ.push(make_pair(candidateDistance, adjacentVertex));
                    continue;
                }
                // RELAX
                if(distances[adjacentVertex] > candidateDistance){
                    distances[adjacentVertex] = candidateDistance;
                    parents[adjacentVertex] = make_pair(current, edge->weight);
                    minPQ.push(make_pair(candidateDistance, current));
                }
            }
        }
        return parents;
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

    auto bellmanFord(N start)
    {
        if (this->findVertex(start) == this->nodes.end()) { throw runtime_error("Vertex is not part of the graph"); }

        unordered_map<N, E> distances;
        unordered_map<N, N> path;
        EdgeSeq edges;

        // INITIALIZE-SINGLE-SOURCE
        for (auto && u : this->nodes) {
            distances[u.first] = INFINITY;
            path[u.first] = u.first;
            for (auto && uEdges: u.second->edges) {
                edges.push_back(uEdges);
            }
        }
        distances[start] = 0;

        unordered_map<N, pair<N, E>> parents;
        parents[start] = std::make_pair(start, 0);

        // RELAX
        for (int i = 0; i < this->getNumberOfNodes() - 1; ++i)
        {
            for (auto && edge: edges)
            {
                auto vertexFrom = path[edge->nodes[0]->data];
                auto vertexTo = path[edge->nodes[1]->data];
                auto weight = edge->weight;
                if (distances[vertexTo] > distances[vertexFrom] + weight && distances[vertexFrom] != INFINITY) {
                    distances[vertexTo] = distances[vertexFrom] + weight;
                    parents[vertexTo] = std::make_pair(vertexFrom, weight);
                }
            }
        }

        // DETERMINE IF THE GRAPH CONTAINS NEGATIVE WEIGHT CYCLES
        for (auto && edge : edges) {
            auto vertexFrom = path[edge->nodes[0]->data];
            auto vertexTo = path[edge->nodes[1]->data];
            auto weight = edge->weight;
            if (distances[vertexTo] > distances[vertexFrom] + weight && distances[vertexFrom] != INFINITY) {
//                throw runtime_error("Graph contains a cycle with negative weights!");
                return false;
            }
        }
        return true;
//        unordered_map<N, self*> answer;
//        for(auto n : this->nodes)
//            answer[n.first] = buildGraphFromPath(start, n.first, parents);
//        return answer;
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

        if (this->directed) { return sum; }
        else { return sum / 2; }
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

    E getHeuristic(node *nodeFrom, node *nodeTo) {
        auto result1 = pow(nodeTo->x - nodeFrom->x, 2);
        auto result2 = pow(nodeTo->y - nodeFrom->y, 2);

        return sqrt(result1 + result2);
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

    self a_star_sp(N start, N target) { //A* shortest path
        unordered_map<N, E> dist; //distance
        unordered_map<N, E> heuristic; //heuristic to target Node
        unordered_map<N, N> parent;
        priority_queue<pair<E, N>, std::vector<pair<E, N>>, greater<>> pq;
        unordered_map<N, E> qp;
        E total_dist;

        if ( nodes.find(start) == nodes.end() )
            throw runtime_error("Start vertex is not part of the graph");

        if ( nodes.find(target) == nodes.end() )
            throw runtime_error("Target vertex is not part of the graph");

        dist[start] = 0;
        parent[start] = start;

        for(auto node: this->nodes) {
            auto inode = node.first;

            if(inode == start)
                continue;

            dist[inode] = INFINITY;
            heuristic[inode] = this->getHeuristic(this->nodes[inode], this->nodes[target]);
        }

        pq.push(make_pair(0, start));
        qp[start] = 0;

        while(!pq.empty()) {
            auto u_node = pq.top().second;

            pq.pop();
            qp.erase(u_node);

            if (u_node == target )
                break;

            //Neighboors
            auto edges = this->nodes[u_node]->edges;
            for (auto edge: edges) {
                auto w_node = edge->nodes[1]->data;
                auto distance = dist[u_node] + edge->weight;

                if(qp.find(w_node) != qp.end() and dist[w_node] > distance) {
                    dist[w_node] = distance;
                    //total_dist = distance + heuristic[u_node];
                    parent[w_node] = u_node;
                } else if (parent.find(w_node) == parent.end() ) {
                    dist[w_node] = distance;
                    total_dist = distance + heuristic[u_node];
                    parent[w_node] = u_node;
                    pq.push({total_dist, w_node});
                    qp[w_node] = total_dist;
                }
            }
        }

        return buildGraphFromPath(start, target, parent);
    }

    self buildGraphFromPath(N start, N target, unordered_map<N, N> path) {
        self newGraph;
        newGraph = self(true);
        bool reachable=false;

        auto prevNode = target;
        auto tmpNode = this->nodes[prevNode];

        newGraph.addVertex(prevNode, tmpNode->x, tmpNode->y);

        while( path.find(prevNode) != path.end() ) {
            if (prevNode == start) {
                reachable=true;
                break;
            }

            auto temp = path[prevNode];
            tmpNode = this->nodes[temp];

            newGraph.addVertex(temp, tmpNode->x, tmpNode->y);

            auto edge = this->findEdge(temp, prevNode);
            newGraph.addEdge(temp, prevNode, edge->weight );

            prevNode = path[prevNode];
        }

        if(!reachable) {
            auto message = "Can't find a path between " + start + " to " + target;
            throw runtime_error(message);
        }

        return newGraph;
    }



    Graph<Tr> *retracePath(N start, N end, unordered_map<N, pair<N, E>> parents){
        if(parents.find(end) == parents.end()) return nullptr;
        auto path = new Graph<Tr>(directed);
        path->addVertex(nodes[end]);
        do{
            path->addVertex(nodes[parents[end].first]);
            path->addEdge(parents[end].first, end, parents[end].second);
            end = parents[end].first;
        }while(parents[end].first != end);
        return path;
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
