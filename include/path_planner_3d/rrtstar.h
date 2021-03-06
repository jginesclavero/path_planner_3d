#ifndef RRTSTAR_H
#define RRTSTAR_H

//#include "obstacles.h"
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <math.h>
#include "geometry_msgs/Pose.h"
#include <time.h>
#define START_ORIENT 0.0

using namespace std;
using namespace geometry_msgs;

struct Node {
    vector<Node *> children;
    Node *parent;
    Point position;
    float orientation;
    double cost;
    //vector<Node *> path;
};

class RRTSTAR
{
public:
    RRTSTAR();
    void initialize();
    Node* getRandomNode();
    Node* nearest(Point point);
    void near(Point point, float radius, vector<Node *>& out_nodes);
    double distance(Point p, Point q);
    double Cost(Node *q);
    double PathCost(Node *qFrom, Node *qTo);
    Pose newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    bool reached();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void deleteNodes(Node *root);
    //Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Point startPos, endPos;
    int max_iter;
    float step_size,end_dist_threshold,rrtstar_neighbor_factor;
    int world_length, world_width, world_height;
    bool ground_robot;
};

#endif // RRTSTAR_H
