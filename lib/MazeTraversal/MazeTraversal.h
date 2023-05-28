/**
 * @file MazeTraversal.cpp
 * @brief Library for navigating a maze using Depth First Search (DFS). 
 * @details Uses a stack to implement DFS. The stack is implemented as a dynamic array.
 * @note Please see here for Arduino Libraries conventions: https://docs.arduino.cc/learn/contributions/arduino-creating-library-guide
 * @author Majd Hamdan
 * @date April 15, 2023.
 * 
 */

#ifndef MazeTraversal_h
#define MazeTraversal_h

#include <Arduino.h>

class MazeTraversal
{
public:
    // row column indices of the cell
    struct Cell
    {
        int row;
        int col;
    };
    // Constructor
    MazeTraversal(int numRows, int numCols);
    // Methods
    bool initilizeTraversal();
    int traverse(int startRow, int startCol, int endRow, int endCol);
    // For maze given as an array
    bool logicTraverse(int startRow, int startCol, int endRow, int endCol);

    void printDirection(int d);

    int state = 0; // 0 = traverse. 1 = backtrack.
    Cell position;  

private:
    /* Here we assume the maze is given as a 2D matrix for demonstration purposes.
    Since we can't yet explore the maze, the maze is given to the program.
    This doesn't mean the code only works for this maze, this method is general.
    Once our mouse can explore a physical maze, this will be later adjusted.
    */
    int numRows = 500;
    int numCols = 500;
    int *maze;     // 2d array to represent the maze. 1 represent a passage, 0 a wall.
    int **visited; // 2d array to represent visited cells. true = visited, false= not yet explored.
    int **numVisited; // 2d array to track wich cells have been visited in order.

    // stack for DFS
    Cell *stack;
    int *path_stack;
    // top of stack index
    int top;
    int path_top;
    // stack methods
    void push(Cell cell);
    Cell pop();
    void path_push(int d);
    int path_pop();

    
    bool isDestination(int row, int col);
    bool dfs(Cell currentCell, Cell endCell); // For maze given as array
    void resizeVisited(int newNumRows, int newNumCols);

 

    int getReverseDirection(int d);


    // free memory
    void Clean();



    // for programmed maze tesing purposes
    void printCell(Cell cell);
    bool canGo(int row, int col);
    void printMaze();
    void printVisited();
    void printNumVisited();
    void printStack();
    void printPathStack();
    


};

#endif