#include<stdio.h>
#include<stdlib.h>

unsigned int nodesExpanded;  //number of expanded nodes
unsigned int nodesGenerated; //number of generated nodes
unsigned int solutionLength; //number of moves in solution

#define BLANK_CHARACTER '0'

typedef enum Move {
    UP, DOWN, LEFT, RIGHT, //values for moving up, down, left, right, respectively
    NOT_APPLICABLE         //value assigned for initial and goal input states
} Move;

typedef struct State {
    Move action;           //action that resulted to `this` board state
    char board[3][3];      //resulting board configuration after applying action
} State;

typedef struct Node Node;
typedef struct NodeList NodeList;

typedef struct ListNode {
    Node *currNode;
    struct ListNode *prevNode; //the node before `this` instance
    struct ListNode *nextNode; //the next node in the linked list
} ListNode;

struct NodeList {
    unsigned int nodeCount;    //the number of nodes in the list
    ListNode *head;            //pointer to the first node in the list
    ListNode *tail;            //pointer to the last node in the list
};

typedef struct SolutionPath {
    Move action;
    struct SolutionPath *next;
} SolutionPath;

typedef struct Node Node;
struct Node {
    unsigned int depth; //depth of the node from the root. For A* search,
                        //this will also represent the node's path cost
    unsigned int hCost; //heuristic cost of the node
    State *state;       //state designated to a node
    Node *parent;       //parent node
    NodeList *children; //list of child nodes
};

//=============================================================================================================================================================

State* createState(State *state, Move move) {
    State *newState = malloc(sizeof(State));

    char i, j;        //used for traversing the 3x3 arrays
    char row, col;    //coordinates of the blank character

    for(i = 0; i < 3; ++i) {
        for(j = 0; j < 3; ++j) {
            if(state->board[i][j] == BLANK_CHARACTER) {
                row = i;
                col = j;
            }

            newState->board[i][j] = state->board[i][j];
        }
    }

    if(move == UP && row - 1 >= 0) {
        char temp = newState->board[row - 1][col];
        newState->board[row - 1][col] = BLANK_CHARACTER;
        newState->board[row][col] = temp;
        newState->action = UP;
        return newState;
    }
    else if(move == DOWN  && row + 1 < 3) {
        char temp = newState->board[row + 1][col];
        newState->board[row + 1][col] = BLANK_CHARACTER;
        newState->board[row][col] = temp;
        newState->action = DOWN;
        return newState;
    }
    else if(move == LEFT  && col - 1 >= 0) {
        char temp = newState->board[row][col - 1];
        newState->board[row][col - 1] = BLANK_CHARACTER;
        newState->board[row][col] = temp;
        newState->action = LEFT;
        return newState;
    }
    else if(move == RIGHT && col + 1 < 3) {
        char temp = newState->board[row][col + 1];
        newState->board[row][col + 1] = BLANK_CHARACTER;
        newState->board[row][col] = temp;
        newState->action = RIGHT;
        return newState;
    }

    free(newState);
    return NULL;
}

void destroyState(State **state) {
    free(*state);
    state = NULL;
}

int manhattanDist(State * const curr, State * const goal) {
    int x0, y0; //used for indexing each symbol in `curr`
    int x1, y1; //correspoinding row and column of symbol from curr[y0, x0] at `goal`
    int dx, dy; //change in x0 and x1, and y0 and y1, respectively
    int sum = 0;

    for(y0 = 0; y0 < 3; ++y0) {
        for(x0 = 0; x0 < 3; ++x0) {
            for(y1 = 0; y1 < 3; ++y1) {
                for(x1 = 0; x1 < 3; ++x1) {
                    if(curr->board[y0][x0] == goal->board[y1][x1]) {
                        dx = (x0 - x1 < 0)? x1 - x0 : x0 - x1;
                        dy = (y0 - y1 < 0)? y1 - y0 : y0 - y1;
                        sum += dx + dy;
                    }
                }
            }
        }
    }

    return sum;
}

char statesMatch(State const *testState, State const *goalState) {
    char row = 3, col;

    while(row--) {
        col = 3;
        while(col--) {
            if(testState->board[row][col] != goalState->board[row][col])
                return 0;
        }
    }

    return 1;
}

//=============================================================================================================================================================

Node* createNode(unsigned int d, unsigned int h, State *s, Node *p) {
    Node *newNode = malloc(sizeof(Node));
    if(newNode) {
        newNode->depth = d;
        newNode->hCost = h;
        newNode->state = s;
        newNode->parent = p;
        newNode->children = NULL;
        ++nodesGenerated; //update counter
    }
    return newNode;
}

void destroyTree(Node *node) {
    if(node->children == NULL) {
        free(node->state);
        free(node);
        return;
    }

    ListNode *listNode = node->children->head;
    ListNode *nextNode;

    while(listNode) {
        nextNode = listNode->nextNode;
        destroyTree(listNode->currNode);
        listNode = nextNode;
    }

    free(node->children);
    free(node);
}

char pushNode(Node *node, NodeList** const list) {
    if(!node)
        return 0;

    ListNode *doublyNode = malloc(sizeof(ListNode));
    if(!doublyNode)
        return 0;

    doublyNode->currNode = node;

    if(*list && !(*list)->nodeCount) {
        (*list)->head = doublyNode;
        (*list)->tail = doublyNode;
        doublyNode->nextNode = NULL;
        doublyNode->prevNode = NULL;
        ++(*list)->nodeCount;
        return 1;
    }

    if(*list == NULL) {
        *list = malloc(sizeof(NodeList));
        if(*list == NULL)
            return 0;

        (*list)->nodeCount = 0;
        (*list)->head = NULL;
        (*list)->tail = doublyNode;
    }
    else {
        (*list)->head->prevNode = doublyNode;
    }

    doublyNode->nextNode = (*list)->head;
    doublyNode->prevNode = NULL;
    (*list)->head = doublyNode;

    ++(*list)->nodeCount;

    return 1;
}

NodeList* getChildren(Node *parent, State *goalState) {
    NodeList *childrenPtr = NULL;
    State *testState = NULL;
    Node *child = NULL;

    if(parent->state->action != DOWN && (testState = createState(parent->state, UP))) {
        child = createNode(parent->depth + 1, manhattanDist(testState, goalState), testState, parent);
        pushNode(child, &parent->children);
        pushNode(child, &childrenPtr);
    }
    if(parent->state->action != UP && (testState = createState(parent->state, DOWN))) {
        child = createNode(parent->depth + 1, manhattanDist(testState, goalState), testState, parent);
        pushNode(child, &parent->children);
        pushNode(child, &childrenPtr);
    }
    if(parent->state->action != RIGHT && (testState = createState(parent->state, LEFT))) {
        child = createNode(parent->depth + 1, manhattanDist(testState, goalState), testState, parent);
        pushNode(child, &parent->children);
        pushNode(child, &childrenPtr);
    }
    if(parent->state->action != LEFT && (testState = createState(parent->state, RIGHT))) {
        child = createNode(parent->depth + 1, manhattanDist(testState, goalState), testState, parent);
        pushNode(child, &parent->children);
        pushNode(child, &childrenPtr);
    }

    return childrenPtr;
}

int totalCost(Node * const node) {
    return node->depth + node->hCost;
}

//=============================================================================================================================================================

void destroySolution(SolutionPath **list) {
    SolutionPath *next;
    while(*list) {
        next = (*list)->next;
        free(*list);
        *list = next;
    }
    *list = NULL;
}

Node* popNode(NodeList** const list) {
    if(!*list || (*list)->nodeCount == 0)
        return NULL;

    Node *popped = (*list)->tail->currNode;
    ListNode *prevNode = (*list)->tail->prevNode;

    free((*list)->tail);

    if((*list)->nodeCount == 1) {
        (*list)->head = NULL;
    }
	else {
		prevNode->nextNode = NULL;
	}

    (*list)->tail = prevNode;
    --(*list)->nodeCount;
    return popped;
}

void pushList(NodeList **toAppend, NodeList *list) {
    if(!*toAppend || !list || !(*toAppend)->head || (*toAppend)->head == list->head) {
        return;
    }

    if(!list->nodeCount) {
        list->head = (*toAppend)->head;
        list->tail = (*toAppend)->tail;
    }
    else {
        (*toAppend)->tail->nextNode = list->head;
        list->head->prevNode = (*toAppend)->tail;
		list->head = (*toAppend)->head;
    }

    list->nodeCount += (*toAppend)->nodeCount;

    free(*toAppend);
    *toAppend = NULL;
}

int totalCost(Node *); //forward declaration for the next function

void pushListInOrder(NodeList **toAppend, NodeList *list) {
    if(!*toAppend || !list || !(*toAppend)->head || (*toAppend)->head == list->head) {
        return;
    }

    if(!list->nodeCount) {
        pushNode(popNode(toAppend), &list);
    }

    ListNode *toAppendNode; //list node to place in `list`
    ListNode *listNode;     //for traversing each node in `list`
    Node *node;

     while((toAppendNode = (*toAppend)->head)) {
        listNode = list->head;

        while(listNode && totalCost(toAppendNode->currNode) < totalCost(listNode->currNode)) {
            listNode = listNode->nextNode;
        }

        ListNode *temp = toAppendNode->nextNode;

        if(!listNode) {
            list->tail->nextNode = toAppendNode;
            toAppendNode->prevNode = list->tail;
            toAppendNode->nextNode = NULL;
            list->tail = toAppendNode;
        }
        else {
            if(listNode->prevNode) {
                toAppendNode->prevNode = listNode->prevNode;
                toAppendNode->nextNode = listNode;
                listNode->prevNode->nextNode = toAppendNode;
                listNode->prevNode = toAppendNode;
            }
            else {
                toAppendNode->nextNode = list->head;
                toAppendNode->prevNode = NULL;
                list->head->prevNode = toAppendNode;
                list->head = toAppendNode;
            }
        }

        (*toAppend)->head = temp;
        --(*toAppend)->nodeCount;
        ++list->nodeCount;
    }

    free(*toAppend);
    *toAppend = NULL;
}

//=============================================================================================================================================================

SolutionPath* BFS_search(State *, State *);
SolutionPath* AStar_search(State *, State *);

//=============================================================================================================================================================
void inputState(State * const state) {
    state->action = NOT_APPLICABLE;
    char row, col;
    int symbol;

    char isNumUsed[9] = { 0 };

    for(row = 0; row < 3; ++row) {
        for(col = 0; col < 3; ++col) {
            printf("    board[%i][%i]: ", row, col);

            scanf("%i", &symbol);

            if(symbol >= 0 && symbol < 9) {
                if(!isNumUsed[symbol]) {
                    state->board[row][col] = symbol + '0';
                    isNumUsed[symbol] = 1;
                }
                else {
                    printf("    ERROR: Number %c is already used. Try again with different input.\n", symbol);
                    --col;
                }
            }
            else {
                printf("    ERROR: Invalid input. Enter a number from 0 to 8.\n");
                --col;
            }
        }
    }
    printf("\n");
}

void printBoard(char const board[][3]) {
    char row, col;

    for(row = 0; row < 3; ++row) {
        for(col = 0; col < 3; ++col) {
            printf(" %c ", board[row][col]);
        }
        printf("\n");
    }

}

void printSolution(struct SolutionPath *path) {
    if(!path) {
        printf("No solution found.\n");
        return;
    }

	if(!path->next) {
		printf("No moves needed. The initial state is already the goal state.\n");
		return;
	}

    printf("SOLUTION: (Relative to the space character)\n");

    char *move[4] = { "UP", "DOWN", "LEFT", "RIGHT" };
    int counter = 1;

    for(path = path->next; path; path = path->next, ++counter) {
        printf("%i. Move %s\n", counter, move[path->action]);
    }

    printf(
        "DETAILS:\n"
        " - Solution length : %i\n"
        " - Nodes expanded  : %i\n"
        " - Nodes generated : %i\n",
        solutionLength, nodesExpanded, nodesGenerated);
}
//====================================================================================================================================================================

int main(void) {
    printf("\n 8 Block Puzzle\n");
    printf( "Instructions:\n"
        "    Enter the initial and goal state of the 8-puzzle board. Input\n"
        "    either integers 0-8, 0 representing the space character, to assign\n"
        "    symbols toeach board[row][col].\n\n"
    );   
    State initial;           //initial board state
    State goalState;         //goal board configuration
    
    SolutionPath *bfs;
    SolutionPath *aStar;

    printf("INITIAL STATE:\n");
    inputState(&initial);

    printf("\nGOAL STATE:\n");
    inputState(&goalState);

    printf("INITIAL BOARD STATE:\n");
    printBoard(initial.board);

    printf("GOAL BOARD STATE:\n");
    printBoard(goalState.board);

    aStar = AStar_search(&initial, &goalState);
    printf("\n-------------------------- USING A* ALGORITHM --------------------------\n");
    printSolution(aStar);

    nodesExpanded = 0;
    nodesGenerated = 0;
    solutionLength = 0;

    bfs = BFS_search(&initial, &goalState);
    printf("\n------------------------- USING BFS ALGORITHM --------------------------\n");
    printSolution(bfs);

    destroySolution(&bfs);
    destroySolution(&aStar);

    return 0;
}

SolutionPath* BFS_search(State *initial, State *goal) {
    NodeList *queue = NULL;
    NodeList *children = NULL;
    Node *node = NULL;


    pushNode(createNode(0, manhattanDist(initial, goal), initial, NULL), &queue);
    Node *root = queue->head->currNode; //for deallocating the generated tree

    while(queue->nodeCount > 0) {
        node = popNode(&queue);

        if(statesMatch(node->state, goal))
            break;

        children = getChildren(node, goal);
        ++nodesExpanded;

        pushList(&children, queue);
    }


    SolutionPath *pathHead = NULL;
    SolutionPath *newPathNode = NULL;

    while(node) {
        newPathNode = malloc(sizeof(SolutionPath));
        newPathNode->action = node->state->action;
        newPathNode->next = pathHead;
        pathHead = newPathNode;

        ++solutionLength;
        node = node->parent;
    }

    --solutionLength; //uncount the root node

    destroyTree(root);

    return pathHead;
}

SolutionPath* AStar_search(State *initial, State *goal) {
    NodeList *queue = NULL;
    NodeList *children = NULL;
    Node *node = NULL;


    pushNode(createNode(0, manhattanDist(initial, goal), initial, NULL), &queue);
    Node *root = queue->head->currNode; //for deallocating generated tree

    while(queue->nodeCount > 0) {
        node = popNode(&queue);

        if(statesMatch(node->state, goal))
            break;

        children = getChildren(node, goal);
        ++nodesExpanded;

        pushListInOrder(&children, queue);
    }


    SolutionPath *pathHead = NULL;
    SolutionPath *newPathNode = NULL;

    while(node) {
        newPathNode = malloc(sizeof(SolutionPath));
        newPathNode->action = node->state->action;
        newPathNode->next = pathHead;
        pathHead = newPathNode;

        ++solutionLength;
        node = node->parent;
    }

    --solutionLength; //uncount the root node

    destroyTree(root);

    return pathHead;
}