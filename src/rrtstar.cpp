#include "path_planner_3d/rrtstar.h"

RRTSTAR::RRTSTAR()
{
    step_size = 0.1;
    max_iter = 3000;
}

/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTSTAR::initialize()
{
    srand(time(NULL));
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->orientation = START_ORIENT;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRTSTAR::getRandomNode()
{
    bool ground_robot = true;
    Node* ret;
    Point point;
    point.x=((double) rand() / (RAND_MAX)) * world_length;
    point.y=((double) rand() / (RAND_MAX)) * world_width;
    if(ground_robot)
      point.z=0.1;
    else
      point.z=((double) rand() / (RAND_MAX)) * world_height;


    float orient = ((double) rand() / (RAND_MAX)) * 2 * 3.142;
    if (point.x >= 0.0 && point.x <= world_length && point.y >= 0.0 && point.y <= world_width && point.z <= world_height && point.z >= 0.0 && orient > 0.0 && orient < 2*3.142) {
        ret = new Node;
        ret->position = point;
        ret->orientation = orient;
        return ret;
    }
    return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTSTAR::distance(Point p, Point q)
{
    Point v;
    v.x = p.x - q.x;
    v.y = p.y - q.y;
    v.z = p.z - q.z;
    return sqrt(powf(v.x, 2) + powf(v.y, 2) + powf(v.z, 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRTSTAR::nearest(Point point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

/**
 * @brief Get neighborhood nodes of a given configuration/position.
 * @param point
 * @param radius
 * @param out_nodes
 * @return
 */
void RRTSTAR::near(Point point, float radius, vector<Node *>& out_nodes)
{
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Pose RRTSTAR::newConfig(Node *q, Node *qNearest)
{
    Point to = q->position;
    Point from = qNearest->position;
    Point intermediate;
    intermediate.x = to.x - from.x;
    intermediate.y = to.y - from.y;
    intermediate.z = to.z - from.z;
    float norm = sqrt(powf(intermediate.x, 2) + powf(intermediate.y, 2) + powf(intermediate.z, 2));
    intermediate.x = intermediate.x / norm;
    intermediate.y = intermediate.y / norm;
    intermediate.z = intermediate.z / norm;


    Pose pos;
    pos.position.x = from.x + step_size * intermediate.x;
    pos.position.y = from.y + step_size * intermediate.y;
    pos.position.z = from.z + step_size * intermediate.z;
    //std::cerr << pos.position.x << '\n';
    //std::cerr << pos.position.y << '\n';
    //std::cerr << pos.position.z << '\n';
    pos.orientation.w = 1.0;
    return pos;
}

/**
 * @brief Return trajectory cost.
 * @param q
 * @return
 */
double RRTSTAR::Cost(Node *q)
{
    return q->cost;
}

/**
 * @brief Compute path cost.
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTSTAR::PathCost(Node *qFrom, Node *qTo)
{
    return distance(qTo->position, qFrom->position);
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTSTAR::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + PathCost(qNearest, qNew);
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    //std::cerr << "x: " << qNew->position.x << " y: " <<qNew->position.y << '\n';

    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRTSTAR::reached()
{
    if (distance(lastNode->position, endPos) < end_dist_threshold)
        return true;
    return false;
}

void RRTSTAR::setStepSize(int step)
{
    step_size = step;
}

void RRTSTAR::setMaxIterations(int iter)
{
    max_iter = iter;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
/*void RRTSTAR::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}*/

/*int main(int argc, char **argv)
{

   return 0;

 }*/
