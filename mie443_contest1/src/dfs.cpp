#include <iostream> 
#include <random>
using namespace std; 

struct Node {
    Node *front = NULL; 
    Node *left = NULL; 
    Node *back = NULL; 
    Node *right = NULL; 
    Node *source = NULL; 
    float pos_x, pos_y; 
}; 

bool random_hit(){
    float result = rand() % 10 / 10.0; 
    if (result <= 0.5) {
        return true; 
    }
    else {
        return false; 
    }
}


int main(){
    float step_size = 5.0; 

    vector<Node> nodes;  //std vector, don't need to define a fixed length, can append later, similar to list in python
    nodes.push_back(Node()); //append a new node to the end of the vector
    nodes[0].pos_x = 0.; 
    nodes[0].pos_y = 0.; 
    nodes[0].source = &nodes[0]; 

    Node *curr_node = &nodes[0];

    // &address.pos_x == address->pox_x == var.pos_x 

    for (int i=0; i < 10; i++){
        cout << "--------------------" << endl; 
        cout << "(" << curr_node->pos_x << "," << curr_node->pos_y << ")" << endl; 
        cout << "Current node: " << curr_node << endl; 
        cout << "Source node: " << curr_node->source << endl; 
        if (!curr_node->front) {
            if (random_hit()) {
                cout << "Hit Front" << endl; 
                nodes.push_back(Node()); // put a empty node -> there is wall in front of it
                curr_node->front = &nodes[-1]; // the new front node will be filled by the emoty node
            }
            else {
                cout << "Going Front" << endl; 
                nodes.push_back(Node()); 
                nodes[-1].source = curr_node; 
                nodes[-1].pos_x = curr_node->pos_x; 
                nodes[-1].pos_y = curr_node->pos_y + step_size; 
                curr_node->front = &nodes[-1]; 
                curr_node = &nodes[-1]; 
            }
        }
        else if (!curr_node->left) {
            if (random_hit()) {
                cout << "Hit Left" << endl; 
                nodes.push_back(Node()); 
                curr_node->left = &nodes[-1]; 
            }
            else {
                cout << "Going Left" << endl; 
                nodes.push_back(Node()); 
                nodes[-1].source = curr_node; 
                nodes[-1].pos_x = curr_node->pos_x - step_size; 
                nodes[-1].pos_y = curr_node->pos_y; 
                curr_node->left = &nodes[-1]; 
                curr_node = &nodes[-1]; 
            }
        }
        else if (!curr_node->right) {
            if (random_hit()) {
                cout << "Hit Right" << endl; 
                nodes.push_back(Node()); 
                curr_node->right = &nodes[-1]; 
            }
            else {
                cout << "Going Right" << endl; 
                nodes.push_back(Node()); 
                nodes[-1].source = curr_node; 
                nodes[-1].pos_x = curr_node->pos_x + step_size; 
                nodes[-1].pos_y = curr_node->pos_y; 
                curr_node->right = &nodes[-1]; 
                curr_node = &nodes[-1]; 
            }
        }
        else {
            cout << "Going Back" << endl; 
            curr_node = curr_node->source; 
        }
    }

    return 0; 
}