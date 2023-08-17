#ifndef __A_STAR_H_
#define __A_STAR_H_

# include<iostream>
#include <vector>
#include <algorithm>

using namespace std;

struct Point {
    int x;
    int y;

    Point():x(0), y(0){};
    Point(int a, int b): x(a), y(b) {};
};

struct Node{
    Point me;
    Point father;
    float g;
    float h;
    float f;
    Node():me(Point()), father(Point()), g(0), f(0), h(0){};
};

class AStar
{
    public:
        AStar();
        void init();
        void find_path();
        void visualize();
    private:
        void map_print(int flag);
        bool is_add_open_list(int x, int y);
        bool is_exceed_index(int x, int y);
        bool is_in_close_list(int x, int y);
        bool is_in_open_list(int x, int y);
        bool is_obstacle(int x, int y);
        bool is_reach_goal();
        void add_close_list(Node node);
        void find_neighbor(Node node);
        void add_open_list(Node node, int x, int y);
        void remove_open_list(int index);
        int find_min_f();
        void fill_real_path();
    
    private:
        Point start_point;
        Point goal_point;
        vector<Point> obs;
        vector<Node> open_list;
        vector<Node> close_list;
        Node father_node;
        vector<Point> real_path;
    private:
        int row;
        int col;
        int open_list_num;
        int close_list_num;
};

#endif