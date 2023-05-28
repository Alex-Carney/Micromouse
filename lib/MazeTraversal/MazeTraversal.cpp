/**
 * @file MazeTraversal.cpp
 * @brief Library for navigating a maze using Depth First Search (DFS). 
 * @details Uses a stack to implement DFS. The stack is implemented as a dynamic array.
 * @note Please see here for Arduino Libraries conventions: https://docs.arduino.cc/learn/contributions/arduino-creating-library-guide
 * @author Majd Hamdan
 * @date April 15, 2023.
 * 
 */
 


#include "MazeTraversal.h"

MazeTraversal::MazeTraversal(int numRows, int numCols, int *maze)
{
    this->numRows = numRows;
    this->numCols = numCols;
    this->maze = maze;


    /*+++++++++++++++++++++++++++++++++++++++
    +++ Allocate memory for visited array +++
    +++++++++++++++++++++++++++++++++++++++++*/
    /*
    At compilation time, we don't really know the size of the actual maze. Our data structures
    should be able to expand as we explore more of the maze. Then, we create dynamic arrays
    that can be re-sized at run time. This is why we don't define the visited array  as
    bool visited[numRows][numColumns]. Pointers is what we need.
    */
    visited = new bool *[numRows];
    for (int i = 0; i < numRows; i++)
    {
        visited[i] = new bool[numCols];
        for (int j = 0; j < numCols; j++)
        {
            visited[i][j] = false;
        }
    }
    
    numVisited = new int *[numRows];
    for (int i = 0; i < numRows; i++)
    {
        numVisited[i] = new int[numCols];
        for (int j = 0; j < numCols; j++)
        {
            numVisited[i][j] = 0;
        }
    }

    // in the worst case senario, we explore every possible cell in the 2d array-> stack size = numRows * numCols
    stack = new Cell[numRows * numCols];
    top = 0;
}




void MazeTraversal::resizeVisited(int newNumRows, int newNumCols)
{
    // Allocate memory for new visited array
    bool **newVisited = new bool *[newNumRows];
    for (int i = 0; i < newNumRows; i++)
    {
        newVisited[i] = new bool[newNumCols];
        for (int j = 0; j < newNumCols; j++)
        {
            newVisited[i][j] = false;
        }
    }

    // Copy values from old visited array to new visited array
    for (int i = 0; i < numRows; i++)
    {
        for (int j = 0; j < numCols; j++)
        {
            newVisited[i][j] = visited[i][j];
        }
    }

    // Deallocate memory for old visited array
    for (int i = 0; i < numRows; i++)
    {
        delete[] visited[i];
    }
    delete[] visited;

    // Resize stack to match new size of visited array
    Cell *newStack = new Cell[newNumRows * newNumCols];
    memcpy(newStack, stack, numRows * numCols * sizeof(Cell));
    delete[] stack;
    stack = newStack;

    // Set member variables to new values
    visited = newVisited;
    numRows = newNumRows;
    numCols = newNumCols;
}


bool MazeTraversal::initilizeTraversal(){

    // clear visited array and stack
    for (int i = 0; i < numRows; i++)
    {
        for (int j = 0; j < numCols; j++)
        {
            visited[i][j] = false;
        }
    }
    top = 0;
    Cell startCell = {0, 0}; // assume starting point has corrdinates (row = 0, col = 0)
    push(startCell);

    return true;
}





// TODO: Different handling when mouse is ready. isDestination() should be used.
bool MazeTraversal::logicTraverse(int startRow, int startCol, int endRow, int endCol)
{
    
    // clear visited array and stack
    for (int i = 0; i < numRows; i++)
    {
        for (int j = 0; j < numCols; j++)
        {
            visited[i][j] = false;
        }
    }
    top = 0;
    Cell startCell = {startRow, startCol};
    push(startCell);
    Cell endCell = {endRow, endCol};

    return dfs(startCell, endCell);
}


int MazeTraversal::traverse(int path_right, int path_left, int path_forward, int path_back){

    // Initialize variables
    int currentRow;
    int currentCol;
    int order = 1;
    int direction = 0; // which direction to turn. 1 = forward, 2 = right, 3 = left, 4 = back
    unsigned long initial_time;

    if (top > 0){
        initial_time = micros();
        Serial.println("Starting loop: ");
        Serial.print("    I am at: ");
        printCell(stack[top-1]);
        Serial.print("    right:");
        Serial.println(path_right);
        Serial.print("    left:");
        Serial.println(path_left);
        Serial.print("    forward:");
        Serial.println(path_forward);
        Serial.print("    back:");
        Serial.println(path_back);
        

        // Pop the top cell from the stack and mark it as visited
        currentRow = stack[top-1].row;
        currentCol = stack[top-1].col;
        
        pop();
        visited[currentRow][currentCol] = true;
        numVisited[currentRow][currentCol] = order;
        order++;
        

        // Check if we have reached the end of the maze
        if (isDestination()) {
            Serial.println("Success!");
            printNumVisited();
            printMaze();
            return true;
            // Maze is solved, exit loop
        }

        /*############ Check adjacent cells and add unvisited ones to the stack ############*/ 
        // Check cell to the top
        if (currentRow > 0 && !visited[currentRow-1][currentCol] && path_forward) {
            push({currentRow-1, currentCol});
            direction = 1;
            Serial.print("  Added top: ");
            printCell(stack[top-1]);
        }

        // Check cell to the right
        if (currentCol < numCols-1 && !visited[currentRow][currentCol+1] && path_right) {
            push({currentRow, currentCol+1});
            direction = 2;
            Serial.print("    Added right: ");
            printCell(stack[top-1]);
        }

        // Check cell to the bottom
        if (currentRow < numRows-1 && !visited[currentRow+1][currentCol] && path_back) {
            push({currentRow+1, currentCol});
            direction = 4;
            Serial.print("    Added bottom: ");
            printCell(stack[top-1]);
        }
        // Check cell to the left
        if (currentCol > 0 && !visited[currentRow][currentCol-1] && path_left) {
            // Add cell to the left
            push({currentRow, currentCol-1});
            direction = 3;
            Serial.print("    Added left: ");
            printCell(stack[top-1]);
        }

        // Serial.println(micros() - initial_time);  

        printVisited();
        printStack();  


        if (currentRow == 2 && currentCol == 8 ){
            direction = 2;
        }    

    }else{
        return 10;
    }

    // Process top to know which direction to turn



    return direction;

}


/* 
    Stack implementation. Written in C++ to use C++ libs. 
*/
bool MazeTraversal::dfs(Cell currentCell, Cell endCell) {

    // Initialize variables
    int currentRow;
    int currentCol;
    int mazeEndRow = endCell.row;
    int mazeEndCol = endCell.col;
    int order = 1;
    unsigned long initial_time;

    // Loop until the stack is empty
    while (top > 0) {
        initial_time = micros();
        // Serial.println("Starting loop: ");
        // Serial.print("    I am at: ");
        // printCell(stack[top-1]);

        // Pop the top cell from the stack and mark it as visited
        currentRow = stack[top-1].row;
        currentCol = stack[top-1].col;
        pop();
        visited[currentRow][currentCol] = true;
        numVisited[currentRow][currentCol] = order;
        order++;

        // printVisited();

        // Check if we have reached the end of the maze
        if (currentRow == mazeEndRow && currentCol == mazeEndCol) {
            Serial.println("Success!");
            printNumVisited();
            printMaze();
            return true;
            // Maze is solved, exit loop
            break;
        }

        /*############ Check adjacent cells and add unvisited ones to the stack ############*/ 
        // Check cell to the top
        if (currentRow > 0 && !visited[currentRow-1][currentCol] && canGo(currentRow-1, currentCol)) {
            push({currentRow-1, currentCol});
            // Serial.print("  Added top: ");
            // printCell(stack[top-1]);
        }

        // Check cell to the right
        if (currentCol < numCols-1 && !visited[currentRow][currentCol+1] && canGo(currentRow, currentCol+1)) {
            push({currentRow, currentCol+1});
            // Serial.print("    Added right: ");
            // printCell(stack[top-1]);
        }

        // Check cell to the bottom
        if (currentRow < numRows-1 && !visited[currentRow+1][currentCol] && canGo(currentRow+1, currentCol)) {
            push({currentRow+1, currentCol});
            // Serial.print("    Added bottom: ");
            // printCell(stack[top-1]);
        }
        // Check cell to the left
        if (currentCol > 0 && !visited[currentRow][currentCol-1] && canGo(currentRow, currentCol-1)) {
            // Add cell to the left
            push({currentRow, currentCol-1});
            // Serial.print("    Added left: ");
            // printCell(stack[top-1]);
        }

        Serial.println(micros() - initial_time);

    }

 return false;
}



// Free used memory
void MazeTraversal::Clean(){
    for (int i = 0; i < numRows; i++) {
        delete[] visited[i];
    }
    delete[] visited;
    delete[] stack;
}



void MazeTraversal::push(Cell cell)
{
    stack[top++] = cell;
}

MazeTraversal::Cell MazeTraversal::pop()
{
    return stack[--top];
}


// TODO: Different handling when mouse is ready. Need sensor data.
bool MazeTraversal::isPassage(Cell cell)
{
    if (cell.row < 0 || cell.row >= numRows || cell.col < 0 || cell.col >= numCols)
    {
        return false; // out of bounds
    }
    return *(maze + cell.row*numCols + cell.col) == 1;
}

bool MazeTraversal::isVisited(Cell cell)
{
    if (cell.row < 0 || cell.row >= numRows || cell.col < 0 || cell.col >= numCols)
    {
        return true; // out of bounds
    }
    return visited[cell.row][cell.col];
}

// TODO: Different handling when mouse is ready. End cell is not known. Need sensor data.
bool MazeTraversal::isDestination()
{
    // cell.row == endRow && cell.col == endCol;
    return false;
}








// TODO: write when arduino is ready. Need sensors. 
bool MazeTraversal::canGoLeft(){
    return true;
}

bool MazeTraversal::canGoRight(){
    return true;
}

bool MazeTraversal::canGoUp(){
    return true;
}

bool MazeTraversal::canGoDown(){
    return true;
}

void MazeTraversal::goLeft(){
    return;
}

void MazeTraversal::goRight(){
    return;
}

void MazeTraversal::goUp(){
    return;
}

void MazeTraversal::goDown(){
    return;
}



void MazeTraversal::printCell(Cell cell){
    Serial.print("row: ");
    Serial.print(cell.row);
    Serial.print(" col: ");
    Serial.println(cell.col);
    return;
}


void MazeTraversal::printVisited(){
    
    for (int i = 0; i < numRows; i++){
        Serial.print("        ");
        for (int j = 0; j < numCols; j++){
            if (visited[i][j])
                Serial.print(visited[i][j]);
            else
                Serial.print(" ");
            Serial.print(" ");
        }
        Serial.println();
    }
    return;

}


void MazeTraversal::printNumVisited(){
    for (int i = 0; i < numRows; i++){
        Serial.print("        ");
        for (int j = 0; j < numCols; j++){
            if (numVisited[i][j] != 0){
                 if (numVisited[i][j] < 10)
                    Serial.print("0");
                Serial.print(numVisited[i][j]);
               
            }else
                Serial.print("  ");
            Serial.print(" ");
        }
        Serial.println();
    }
    return;

}

void MazeTraversal::printMaze(){
   
    for (int i = 0; i < numRows; i++){
        for (int j = 0; j < numCols; j++){
            Serial.print(" ");
            Serial.print(*(maze+numCols*i + j));
            Serial.print(" ");
        }
        Serial.println();

    }
    return;
}

void MazeTraversal::printStack(){
    Serial.println("Stack: ");
    for (int i = 0; i < top; i++){
        Serial.print("    ");
        printCell(stack[i]);
    }
    Serial.println();
    return;
}

bool MazeTraversal::canGo(int row, int col){
    return *(maze+numCols*row + col) == 1;
}