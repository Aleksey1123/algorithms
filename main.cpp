#include <iostream>
#include <list>
#include <set>
#include "structs.h"
#include "utils.h"
#include <chrono>
#include "cmath"
#include <float.h>
#include "algorithm"

using namespace std;

class BFS //breadth-first-search
{
public:
    Result find_path(Node start, Node goal, Map grid, int connections = 4)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        Result result;
        int steps = 0;
        start.g = 0;
        std::list<Node> OPEN;
        OPEN.push_back(start);
        std::set<Node> CLOSED;
        pair<int, int> delta;
        CLOSED.insert(start);
        bool pathfound = false;
        while(!OPEN.empty() && !pathfound)
        {
            Node current = OPEN.front();
//           cout << current.i << ", " << current.j << endl;
            OPEN.pop_front();
            steps++;
            auto neighbors = grid.get_neighbors(current, connections);
            for(auto n:neighbors) {
                if (CLOSED.find(n) == CLOSED.end())
                {
                    delta = {n.i - current.i, n.j - current.j};
                    if (abs(delta.first) == 1 && abs(delta.second) == 1) {
                        n.g = current.g + sqrt(2);
                    }
                    else n.g = current.g + 1;
                    n.parent = &(*CLOSED.find(current));
                    OPEN.push_back(n);
                    CLOSED.insert(n);
                    if(n == goal) {
                        result.path = reconstruct_path(n);
                        result.cost = n.g;
                        pathfound = true;
                        break;
                    }
                }
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count()/1e+9;
        return result;
    }
    std::list<Node> reconstruct_path(Node n)
    {
        std::list<Node> path;
        while(n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

class AStar
{
public:
    Result find_path(Node start, Node goal, Map grid, string metrictype="Octile", int connections=8, double hweight=1)
    {
        //TODO - implement the main cycle of AStar algorithm

        auto time_now = std::chrono::high_resolution_clock::now();
        Result result;
        bool flag = false;
        int steps = 0;
        start.g = 0;
        std::list<Node> OPEN;
        Node tmp;
        OPEN.push_back(start);
        std::set<Node> CLOSED;
        pair<int, int> delta;
        CLOSED.insert(start);
        bool pathfound = false;
        while(!OPEN.empty() && !pathfound)
        {
            Node current = OPEN.front();
//            cout << current.i << " " << current.j << endl;
            OPEN.remove(current);
            CLOSED.insert(current);
            steps++;
            auto neighbors = grid.get_neighbors(current, connections);
            for(auto n:neighbors) {
                if (CLOSED.find(n) == CLOSED.end()) {
                    delta = {n.i - current.i, n.j - current.j};
                    if (abs(delta.first) == 1 && abs(delta.second) == 1) {
                        n.g = current.g + sqrt(2);
                    }
                    else n.g = current.g + 1;
                    n.h = count_h_value(n, goal, metrictype);
                    n.f = n.g + (hweight * n.h);
                    n.parent = &(*CLOSED.find(current));

                    for(auto iter = OPEN.begin(); iter!=OPEN.end(); ++iter)
                    {
                        if (compare_coord_of_nodes(*iter, n)) {
                            if (n.g < (*iter).g) {
                                iter->g = n.g;
                                iter->f = n.f;
                                iter->parent = n.parent;
                                flag = true;
                                break;
                            }
                        }
                    }
                    if (flag) {
                        flag = false;
                        continue;
                    }
                    OPEN.push_back(n);
                }
            }
            OPEN.sort(compare_f_value_of_nodes);
            if (current == goal) {
                result.path = reconstruct_path(current);
                result.cost = current.g;
                pathfound = true;
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count()/1e+9;
        return result;
    }

    double count_h_value(Node current, Node goal, std::string metrictype="Octile")
    {
        //TODO - add support of all three metrics
        if (metrictype == "Octile") {
            double di = abs(current.i - goal.i);
            double dj = abs(current.j - goal.j);
            current.h = 1 * (di + dj) + (sqrt(2) - 2 * 1) * min(di, dj);
        }
        else if (metrictype == "Manhattan") {
            current.h = abs(current.i - current.j) + abs(goal.i - goal.j);
        }
        else if (metrictype == "Euclid") {
            current.h = sqrt(pow(current.i - goal.i, 2) + pow(current.j - goal.j, 2));
        }

        return current.h;
    }

    std::list<Node> reconstruct_path(Node n)
    {
        //TODO - reconstruct path using back pointers

        std::list<Node> path;
        while(n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }

    bool compare_coord_of_nodes(Node &n1, Node &n2) {
        return (n1.i == n2.i && n1.j == n2.j);
    }

    bool static compare_f_value_of_nodes(Node &first, Node &second) {
        return (first.f < second.f);
    }
};

int main(int argc, char* argv[]) //argc - argumnet counter, argv - argument values
{
    for(int i=0; i<argc; i++)
        std::cout<<argv[i]<<"\n";
    if(argc<2)
    {
        std::cout << "Name of the input XML file is not specified."<<std::endl;
        return 1;
    }
    Loader loader;
//    loader.load_instance("instance1.xml");
    loader.load_instance(argv[1]);
    Result result;
    if(loader.algorithm == "BFS")
    {
        BFS bfs;
        result = bfs.find_path(loader.start, loader.goal, loader.grid, loader.connections);
    }
    else
    {
        if(loader.algorithm == "Dijkstra")
            loader.hweight = 0;
        AStar astar;
        result = astar.find_path(loader.start, loader.goal, loader.grid, loader.metrictype, loader.connections, loader.hweight);
    }
    loader.grid.print(result.path);
    std::cout<<"Cost: "<<result.cost<<"\nRuntime: "<<result.runtime
             <<"\nSteps: "<<result.steps<<"\nNodes created: "<<result.nodes_created<<std::endl;

    system("pause");
}