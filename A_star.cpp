# include "A_star.h"

AStar::AStar()
{
    init();
}

void AStar::init()
{
    cout<<"init"<<endl;
    // 寻路1
    // obs = {Point(1, 3), Point(2, 3), Point(3, 3)};
    // start_point.x = 2;
    // start_point.y = 1;
    // goal_point.x = 2;
    // goal_point.y = 6;
    // row = 10;
    // col = 10;

    // 寻路2
    obs = {Point(4, 2), Point(4, 3), Point(4, 4), Point(4, 5), Point(3, 5), 
                 Point(2, 5), Point(6, 1), Point(6, 2), Point(6, 4), Point(6, 5), 
                 Point(6, 7), Point(6, 8), Point(8, 5), Point(9, 5), Point(10, 5), 
                 Point(8, 4), Point(9, 4), Point(10, 4)};
    start_point.x = 2;
    start_point.y = 2;
    goal_point.x = 11;
    goal_point.y = 7;
    row = 12;
    col = 12;

    open_list_num = 0;
    close_list_num = 0;
    open_list = {};
    close_list = {};
    // 加入最初的open_list
    Node start_node;
    start_node.me = start_point;
    start_node.h = abs(goal_point.x-start_point.x)+abs(goal_point.y-start_point.y);
    start_node.f = start_node.g+start_node.h;
    add_close_list(start_node);
    find_neighbor(start_node);
}

void AStar::find_path()
{
    cout<<"finding path"<<endl;
    bool reach_goal;
    while(1)
    {
        int index = find_min_f();
        if(index==-1) break;
        father_node = open_list[index];
        add_close_list(father_node);
        remove_open_list(index);
        find_neighbor(father_node);
        if(is_reach_goal())
        {
            reach_goal = true;
            break;
        }
        if(open_list_num==0)
        {
            reach_goal = false;
            break;
        }
    }
    if(reach_goal)
    {
        cout<<"path has be found"<<endl;
        fill_real_path();
    }
    else cout<<"there is no path to be found"<<endl;
}

void AStar::fill_real_path()
{
    int last_x = goal_point.x;
    int last_y = goal_point.y;
    int start_x = start_point.x;
    int start_y = start_point.y;
    int a = -1;
    int b = -1;
    while(1)
    {
        for(int i=0; i<close_list_num; i++)
        {
            Point p;
            a = close_list[i].me.x;
            b = close_list[i].me.y;
            if(a==last_x && b==last_y)
            {
                p.x = a;
                p.y = b;
                real_path.push_back(p);
                last_x = close_list[i].father.x;
                last_y = close_list[i].father.y;
                // cout<<"("<<a<<","<<b<<")"<<endl;
                if(a==start_x && b==start_y) break;
            }
        }
        if(a==start_x && b==start_y) break;
    }
    reverse(real_path.begin(), real_path.end());
}

void AStar::add_close_list(Node node)
{
    close_list.push_back(node);
    close_list_num++;
}

void AStar::remove_open_list(int index)
{
    auto it = open_list.begin() + index;
    open_list.erase(it);
    open_list_num--;
}

int AStar::find_min_f()
{
    int index = -1;
    int min_f = 9999;
    for(int i =0; i<open_list_num; i++)
    {
        if(min_f > open_list[i].f)
        {
            min_f = open_list[i].f;
            index = i;
        }
    }
    return index;
}

void AStar::find_neighbor(Node node)
{
    for(int i=-1; i<2; i++)
    {
        for(int j=-1; j<2; j++)
        {
            if(i==0 &&  j==0) continue;
            int x = node.me.x + i;
            int y = node.me.y + j;
            if(!is_add_open_list(x, y)) continue;
            add_open_list(node, x, y);
            if(is_reach_goal())
            {
                int goal_index = 0;
                for(int k=0; k<open_list_num; k++)
                {
                    int x = open_list[k].me.x;
                    int y = open_list[k].me.y;
                    if(x == goal_point.x && y == goal_point.y)
                    {
                        goal_index = k;
                        break;
                    }
                }
                close_list.push_back(open_list[goal_index]);
                close_list_num++;
            }
        }
    }
}

void AStar::add_open_list(Node node, int x, int y)
{
    float g = 0;
    int dis = abs(node.me.x-x) + abs(node.me.y-y);
    if(dis == 1) g = 1+node.g;
    else g = 1.4+node.g;
    float h = abs(goal_point.x-x)+abs(goal_point.y-y);
    float f = g+h;
    if(is_in_open_list(x, y))
    {
        // 要更新的点
        int cur_index = 0;
        for(int i=0; i<open_list_num; i++)
        {
            if(x==open_list[i].me.x &&y==open_list[i].me.y)
            {
                cur_index = i;
                break;
            }
        }
        // 判断是否更新父节点
        if(f<open_list[cur_index].f)
        {
            open_list[cur_index].g = g;
            open_list[cur_index].f = f;
            open_list[cur_index].father = node.me;
        }
    }
    else  //直接加入open_list
    {
        Node new_node;
        new_node.me.x = x;
        new_node.me.y = y;
        new_node.father.x = node.me.x;
        new_node.father.y = node.me.y;
        new_node.g = g;
        new_node.h = h;
        new_node.f = f;
        open_list.push_back(new_node);
        open_list_num++;
    }
}

bool AStar::is_in_open_list(int x, int y)
{
    int n = open_list.size();
    for(int i = 0; i<n; i++)
    {
        int a = open_list[i].me.x;
        int b = open_list[i].me.y;
        if(x==a && y==b) return true;
    }
    return false;
}

void AStar::visualize()
{
    cout<<"visualize"<<endl;
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<col; j++)
        {
            int flag = -1;
            // 把路径点写前面是为了提高起点和终点的优先级
            int m = real_path.size();
            for(int k = 0; k<m; k++)
            {
                if(i==real_path[k].x && j==real_path[k].y)
                {
                    flag = 4;
                    break;
                }
            }
            if(i == start_point.x && j == start_point.y) flag=1;
            if(i == goal_point.x && j== goal_point.y) flag=2;
            int n = obs.size();
            for(int k=0; k<n; k++)
            {
                if(i==obs[k].x && j==obs[k].y)
                {
                    flag=3;
                    break;
                }
            }
            map_print(flag);
        }
        cout<<endl;
    }
}

void AStar::map_print(int flag)
{
    switch(flag)
    {
        case 1:  // 起点
            cout<<"@ ";
            break;
        case 2: // 终点
            cout<<"$ ";
            break;
        case 3: // 障碍物
            cout<<"0 ";
            break;
        case 4: //路径点
            cout<<"* ";
            break;
        default:
            cout<<"1 ";
            break;
    }
}

bool AStar::is_add_open_list(int x, int y)
{
    bool a = is_exceed_index(x, y);
    bool b = is_in_close_list(x, y);
    bool c = is_obstacle(x, y);
    bool result = (!a)&&(!b)&&(!c);
    return result;
}

bool AStar::is_exceed_index(int x, int y)
{
    if(x<0 || x>col || y<0 || y>row) return true;
    else return false;
}

bool AStar::is_in_close_list(int x, int y)
{
    for(int i = 0; i<close_list_num; i++)
    {
        int checked_x = close_list[i].me.x;
        int checked_y = close_list[i].me.y ;
        if(x == checked_x && y == checked_y) return true;
    }
    return false;
}

bool AStar::is_obstacle(int x, int y)
{
    int n = obs.size();
    for(int i = 0; i<n; i++)
    {
        int checked_x = obs[i].x;
        int checked_y = obs[i].y ;
        if(x == checked_x && y == checked_y) return true;
    }
    return false;
}

bool AStar::is_reach_goal()
{
    for(int i = 0; i<open_list_num; i++)
    {
        int x = open_list[i].me.x;
        int y = open_list[i].me.y;
        if(x == goal_point.x && y == goal_point.y) return true;
    }
    return false;
}
