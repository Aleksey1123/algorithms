#ifndef LAB1V2_STRUCTS_H
#define LAB1V2_STRUCTS_H
#include <vector>
#include <iostream>
#include <list>
#include "iterator"
#define OBSTACLE 1
#define TRAVERSABLE 0
#define PATH 2

#include "cmath"

using namespace std;

struct Node
{
    int i;
    int j;
    double g;
    double h;
    double f;
    const Node* parent;
    Node(int _i=-1, int _j=-1, double _g=0, double _h=0, Node* _parent=nullptr):i(_i), j(_j), g(_g), h(_h), f(_h+_g), parent(_parent){}
    bool operator ==(const Node& other) const
    {
        return this->i == other.i && this->j == other.j;
    }
    bool operator <(const Node& other) const
    {
        return this->i < other.i || (this->i == other.i && this->j < other.j);
    }
};

struct Result
{
    std::list<Node> path; //путь
    double cost; //стоимость пути
    double runtime; //время работы алгоритма
    int steps; //число итераций работы алгоритма
    int nodes_created; //число созданных вершин
};

struct Map {
    int width;
    int height;
    std::vector<std::vector<int>> grid;

    Map(int _width = 0, int _height = 0) : width(_width), height(_height) {
        grid = std::vector<std::vector<int>>(height, std::vector<int>(width, TRAVERSABLE));
    }

    void add_obstacle(int i, int j) {
        grid[i][j] = OBSTACLE;
    }

    bool cell_on_grid(int i, int j) {
        return (i >= 0 && j >= 0 && i < height && j < width);
    }

    bool cell_is_obstacle(int i, int j) {
        return grid[i][j] == OBSTACLE;
    }

    std::vector<Node> get_neighbors(Node s, int connections = 8) {
        //TODO - implement support of diagonal moves -- DONE
        if (connections == 4) {
            std::vector<std::pair<int, int>> deltas_4 = {{0,  1},
                                                         {1,  0},
                                                         {-1, 0},
                                                         {0,  -1}};
            std::vector<Node> neighbors;
            for (auto d: deltas_4) {
                Node n(s.i + d.first, s.j + d.second);
                if (cell_on_grid(n.i, n.j) && !cell_is_obstacle(n.i, n.j))
                    neighbors.push_back(n);
            }
            return neighbors;
        } else if (connections == 8) {
            std::vector<std::pair<int, int>> deltas_8 = {{0,  1}, {1,  0},
                                                         {-1, 0}, {0,  -1},
                                                         {-1, -1}, {-1, 1},
                                                         {1,  -1}, {1,  1}};

            std::vector<Node> neighbors;
            std::pair<int, int> delta;

            for (auto d: deltas_8) {
                Node n(s.i + d.first, s.j + d.second);
                if (d.first == -1 && d.second == -1 || d.first == -1 && d.second == 1 || d.first == 1 && d.second == -1 || d.first == 1 && d.second == 1) {
                    delta = {n.i - s.i, n.j - s.j};
                    if (cell_on_grid(n.i, n.j) && !cell_is_obstacle(s.i + delta.first, s.j) && !cell_is_obstacle(s.i, s.j + delta.second) &&
                        !cell_is_obstacle(n.i, n.j))
                        neighbors.push_back(n);
                    //это условие при котором мы можем пройти по диагонали даже, если есть одна из диагональных границ
//                    else if ((cell_on_grid(n.i, n.j) && !cell_is_obstacle(s.i + delta.first, s.j) && (cell_is_obstacle(s.i, s.j + delta.second)) &&
//                                (!cell_is_obstacle(n.i, n.j)))
//                               ||
//                               (cell_on_grid(n.i, n.j) && !cell_is_obstacle(s.i, s.j + delta.second) && (cell_is_obstacle(s.i + delta.first, s.j)) &&
//                                cell_is_obstacle(n.i, n.j))) {
//                        neighbors.push_back(n);
//                        n.diagonalMove = true;
//                    }
                }
                else {
                    if (cell_on_grid(n.i, n.j) && !cell_is_obstacle(n.i, n.j))
                        neighbors.push_back(n);
                }
            }
            return neighbors;
        }
    }
    void print(std::list<Node> path = {}) {
        for (auto n: path)
            grid[n.i][n.j] = PATH;

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++)
                if (grid[i][j] == OBSTACLE)
                    std::cout << "# ";
                else if (grid[i][j] == TRAVERSABLE)
                    std::cout << ". ";
                else
                    std::cout << "@ ";
                std::cout << std::endl;
            }
            for (auto n: path)
                grid[n.i][n.j] = TRAVERSABLE;
        }
    };
#endif //LAB1V2_STRUCTS_H