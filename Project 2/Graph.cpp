#include "Graph.h"
#include <fstream>
#include <sstream>
#include <queue>
#include <set>
#include <float.h>
#include <functional>
#include <iostream>

// helper struct v(ertex) for djikstra's algorithm
struct v {
    public:
        double dist;
        string prev;
        bool done;
};

// helper compare function (less) between tuples of int(compared), string
bool tupCmp(tuple<double, string>& lhs, tuple<double, string>& rhs) {
    return get<0>(lhs) < get<0>(rhs);
}

Graph::Graph(const char* const & edgelist_csv_fn) {
    // TODO
    edge_count = 0;

    ifstream graph_csv(edgelist_csv_fn);
    string line;
    while(getline(graph_csv, line)) {
        istringstream ss(line);
        string u, v, w;
        getline(ss, u, ',');
        getline(ss, v, ',');
        getline(ss, w, '\n');

        myGraph[u][v] = stod(w);
        myGraph[v][u] = stod(w);

        edge_count++;
    }
    graph_csv.close();
}

unsigned int Graph::num_nodes() {
    // TODO
    return myGraph.size();
}

vector<string> Graph::nodes() {
    // TODO
    vector<string> node_vec;

    for( auto it = myGraph.begin(); it != myGraph.end(); it++) {
        node_vec.push_back(it->first);
    }

    return node_vec;
}

unsigned int Graph::num_edges() {
    // TODO
    return edge_count;
}

unsigned int Graph::num_neighbors(string const & node_label) {
    // TODO
    return myGraph[node_label].size();
}

double Graph::edge_weight(string const & u_label, string const & v_label) {
    // TODO
    if (myGraph.find(u_label) == myGraph.end()) return -1;
    else if (myGraph[u_label].find(v_label) == myGraph[u_label].end()) return -1;
    else return myGraph[u_label][v_label];
}

vector<string> Graph::neighbors(string const & node_label) {
    // TODO
    vector<string> neighbor_nodes;

    for( auto it = myGraph[node_label].begin(); it != myGraph[node_label].end(); it++) {
        neighbor_nodes.push_back(get<0>(*it));
    }

    return neighbor_nodes;
}

vector<string> Graph::shortest_path_unweighted(string const & start_label, string const & end_label) {
    // TODO
    if(start_label == end_label) {
        vector<string> same_label;
        same_label.push_back(start_label);
        return same_label;
    }

    queue<vector<string>> q;
    set<string> visited;
    vector<string> start_path;
    start_path.push_back(start_label);
    q.push(start_path);

    while(!q.empty()) {
        vector<string> path = q.front();
        q.pop();

        string node = path.back();

        if(node == end_label) return path;
        else if (visited.find(node) == visited.end()) {
            for(auto it = myGraph[node].begin(); it != myGraph[node].end(); it++) {
                vector<string> new_path = path;
                new_path.push_back(get<0>(*it));
                q.push(new_path);

                if(get<0>(*it) == end_label) return new_path;
            }

            visited.insert(node);
        }
    }

    vector<string> empty_vec;
    return empty_vec;
}

vector<tuple<string,string,double>> Graph::shortest_path_weighted(string const & start_label, string const & end_label) {
    // TODO
    if (start_label == end_label) {
        vector<tuple<string, string, double>> same_vec;
        tuple<string, string, double> same_tup = make_tuple(start_label, end_label, -1);
        same_vec.push_back(same_tup);
        return same_vec;
    }
    // priority queue to store tuple<double, string>
    priority_queue<tuple<double, string>, vector<tuple<double, string>>, function<bool(tuple<double, string>&, tuple<double, string>&)>> pq(tupCmp);

    // unordered map to store v(ertices)
    unordered_map<string, v> v_map;

    // populate v_map
    for(auto it = myGraph.begin(); it != myGraph.end(); it++) {
        v the_v {
            .dist = DBL_MAX,
            .prev = "",
            .done = false
        };
        v_map.insert(pair<string, v>(it->first, the_v));
    }

    v_map[start_label].dist = 0;
    pq.push(make_tuple(0, start_label));
    while(!pq.empty()) {
        tuple<double, string> curr = pq.top();
        pq.pop();

        string curr_v = get<1>(curr);
        if(v_map[curr_v].done == false) {
            v_map[curr_v].done = true;

            for(auto it = myGraph[curr_v].begin(); it != myGraph[curr_v].end(); it++) {
                double new_dist = v_map[curr_v].dist + it->second;
                if (new_dist < v_map[it->first].dist) {
                    v_map[it->first].prev = curr_v;
                    v_map[it->first].dist = new_dist;
                    pq.push(make_tuple(new_dist, it->first));
                }
            }
        }
    }

    // dijkstra's algorithm finished -> trace back to get path
    // case 1 - unreachable
    if(v_map[end_label].dist == DBL_MAX) {
        vector<tuple<string,string,double>> empty_vec;

        return empty_vec;
    }
    // case 2 - reached
    else {
        vector<tuple<string,string,double>> return_vec;

        auto it = v_map[end_label];
        string curr = end_label;
        while(it.prev != "") {
            return_vec.insert(return_vec.begin(), make_tuple(it.prev, curr, myGraph[it.prev][curr]));
            curr = it.prev;
            it = v_map[it.prev];
        }

        return return_vec;
    }

}

vector<vector<string>> Graph::connected_components(double const & threshold) {
    // TODO

    // return_vector
    vector<vector<string>> return_vec;

    // unvisited_nodes set
    set<string> unvisited;
    for(auto it = myGraph.begin(); it != myGraph.end(); it++) unvisited.insert(it->first);

    // BFS until all nodes are visited
    while(!unvisited.empty()) {
        queue<string> q;
        vector<string> connected;

        string first_in_unvisited = *unvisited.begin();
        unvisited.erase(unvisited.begin());
        q.push(first_in_unvisited);
        connected.push_back(first_in_unvisited);

        while(!q.empty()) {
            string curr = q.front();
            q.pop();
            for(auto it = myGraph[curr].begin(); it != myGraph[curr].end(); it++) {
                if(unvisited.find(it->first) != unvisited.end()) {
                    if(it->second <= threshold) {
                        q.push(it->first);
                        unvisited.erase(unvisited.find(it->first));
                        connected.push_back(it->first);
                    }
                }
            }
        }

        return_vec.push_back(connected);
    }

    return return_vec;
}

// helper compare function (less) between tuples of int(compared), string
bool tupCmp2(tuple<string, string, double>& lhs, tuple<string, string, double>& rhs) {
    return get<2>(lhs) > get<2>(rhs);
}

// helper find
string find_sentinel(unordered_map<string, string>& node_map, string const& x) {
    string curr = x;
    set<string> prev;
    string sentinel = "";

    while(curr != sentinel) {
        prev.insert(curr);
        string next = node_map[curr];
        if(curr == next) sentinel = curr;
        else {
            for(string p: prev) {
                node_map[p] = node_map[next];
            }
        }
        curr = next;
    }
    return sentinel;
}

// helper dijoint set size
int ds_size(unordered_map<string, string>& node_map, string const& x) {
    int size = 0;
    string curr = x;
    while(curr != node_map[curr]) {
        curr = node_map[curr];
        size++;
    }
    return size;
}

// helper union
void my_union(unordered_map<string, string>& node_map, string const & x, string const & y) {
    if(ds_size(node_map, x) > ds_size(node_map, y)) {
        node_map[find_sentinel(node_map, y)] = find_sentinel(node_map, x);
        string dump = find_sentinel(node_map, y);
    }
    else {
        node_map[find_sentinel(node_map, x)] = find_sentinel(node_map, y);
        string dump = find_sentinel(node_map, x);
    }
}

double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    // TODO
    // same
    if(start_label == end_label) return 0;

    // disjoint set as unordered_map -> sentinel if key = value
    unordered_map<string, string> v_map;
    for(auto it = myGraph.begin(); it != myGraph.end(); it++) {
        v_map.insert(pair<string, string>(it->first, it->first));
    }

    // pq to hold edges (sorted)
    priority_queue<tuple<string, string, double>, vector<tuple<string, string, double>>, function<bool (tuple<string, string, double>&, tuple<string, string, double>&)>> pq(tupCmp2);
    set<string> visited;
    for(auto it = myGraph.begin(); it != myGraph.end(); it++) {
        visited.insert(it->first);
        for(auto it2 = myGraph[it->first].begin(); it2 != myGraph[it->first].end(); it2++) {
            if(visited.find(it2->first) != visited.end()) {
                pq.push(make_tuple(it->first, it2->first, it2->second));
            }
        }
    }

    // go through pq
    while(!pq.empty()) {
        tuple<string, string, double> curr = pq.top();
        pq.pop();

        my_union(v_map, get<0>(curr), get<1>(curr));

        if(find_sentinel(v_map, start_label) == find_sentinel(v_map, end_label)) return get<2>(curr);
    }

    // never connected
    return -1;
}
