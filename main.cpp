# include "A_star.h"

int main(int argc, char **argv)
{
    AStar as = AStar();
    as.find_path();
    as.visualize();
}