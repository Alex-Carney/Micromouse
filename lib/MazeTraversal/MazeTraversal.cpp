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

MazeTraversal::MazeTraversal(int numRows, int numCols)
{
    this->numRows = numRows;
    this->numCols = numCols;


    /*+++++++++++++++++++++++++++++++++++++++
    +++ Allocate memory for visited array +++
    +++++++++++++++++++++++++++++++++++++++++*/
    /*
    At compilation time, we don't really know the size of the actual maze. Our data structures
    should be able to expand as we explore more of the maze. Then, we create dynamic arrays
    that can be re-sized at run time. This is why we don't define the visited array  as
    bool visited[numRows][numColumns]. Pointers is what we need.
    */
    visited = new int *[numRows];
    for (int i = 0; i < numRows; i++)
    {
        visited[i] = new int [numCols];
        for (int j = 0; j < numCols; j++)
        {
            visited[i][j] = 0;
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
    path_stack = new int[numRows * numCols];
    top = 0;
    path_top = 0;
}





bool MazeTraversal::initilizeTraversal(){

    // clear visited array and stack
    for (int i = 0; i < numRows; i++)
    {
        for (int j = 0; j < numCols; j++)
        {
            visited[i][j] = 0;
        }
    }
    top = 0;
    Cell startCell = {numCols/2, numCols/2}; // assume starting point has corrdinates (row = 0, col = 0)
    push(startCell);
    path_top = 0;

    return true;
}


int MazeTraversal::traverse(int path_right, int path_left, int path_forward, int path_back){

    // Initialize variables
    int currentRow;
    int currentCol;
    int dest_row; // for backtracking
    int dest_col; // for backtracking
    int order = 1; // for printing the path
    int direction = 0; // which direction to turn. 1 = forward, 2 = right, 3 = left, 4 = back
    unsigned long initial_time;

    bool backtrack = true;

    switch (state)
    {
    // traversal    
    case 0:
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
            if (visited[currentRow][currentCol] == 1){
                return 10;
            }
            visited[currentRow][currentCol] = 1;
            numVisited[currentRow][currentCol] = order;
            order++;


            

            // Check if we have reached the end of the maze
            if (isDestination(currentRow, currentCol)) {
                Serial.println("Success!");
                // printNumVisited();
                // printMaze();
                return 10;
                // Maze is solved, exit loop
            }

            

            /*############ Check adjacent cells and add unvisited ones to the stack ############*/ 
            // Check cell to the top
            if (currentRow > 0 && visited[currentRow-1][currentCol] != 1 && path_forward) {
                push({currentRow-1, currentCol});
                direction = 1;
                // Serial.print("  Added top: ");
                // printCell(stack[top-1]);
                visited[currentRow-1][currentCol] = 2; // mark as not visited
                backtrack = false;
            }

            

            // Check cell to the right
            if (currentCol < numCols-1 && visited[currentRow][currentCol+1] != 1 && path_right) {
                push({currentRow, currentCol+1});
                direction = 2;
                // Serial.print("    Added right: ");
                // printCell(stack[top-1]);
                visited[currentRow][currentCol+1] = 2; // mark as not visited
                backtrack = false;
            }

            // Check cell to the bottom
            if (currentRow < numRows-1 && visited[currentRow+1][currentCol] != 1 && path_back) {
                push({currentRow+1, currentCol});
                direction = 4;
                // Serial.print("    Added bottom: ");
                // printCell(stack[top-1]);
                visited[currentRow+1][currentCol] = 2; // mark as not visited
                backtrack = false;
            }
            // Check cell to the left
            if (currentCol > 0 && visited[currentRow][currentCol-1] != 1 && path_left) {
                // Add cell to the left
                push({currentRow, currentCol-1});
                direction = 3;
                // Serial.print("    Added left: ");
                // printCell(stack[top-1]);
                visited[currentRow][currentCol-1] = 2; // mark as not visited
                backtrack = false;
            }

            


            //#############3 Backtracking ###############
            // When adjancent cells are all visisted, backtrack to the top of the stack.
            if (backtrack){
                // Serial.println("   ########################## entered Backtracking");
                int previous_direction = path_pop();
                direction =  getReverseDirection(previous_direction);
                state = 1; // switch to backtracking state
                if (direction == 1){
                    position = {currentRow - 1, currentCol};
                }else if (direction == 2){
                    position = {currentRow, currentCol+1};
                }else if(direction == 3){
                    position = {currentRow, currentCol-1};
                }else if(direction == 4){
                    position = {currentRow + 1, currentCol};
                }
                visited[currentRow][currentCol] = 3; // mark as backtracked                
            }

            // Keep track of the path taken
            if (!backtrack){
                path_push(direction);
            }


            
            
    
        }else{
            return 10;
        }
        break;

    case 1:
        Serial.println("########################## Backtracking");
        currentRow = position.row;
        currentCol = position.col;
        dest_row = stack[top-1].row;
        dest_col = stack[top-1].col;

        backtrack = true;
    
        // Check cell to the top
        if (currentRow > 0 &&  path_forward) {
            if (currentRow - 1 == dest_row && currentCol == dest_col){
                backtrack = false;
                direction = 1;
            }
        }

        

        // Check cell to the right
        if (currentCol < numCols-1  && path_right) {
            if (currentRow == dest_row && currentCol + 1 == dest_col){
                backtrack = false;
                direction = 2;
            }

        }

        // Check cell to the bottom
        if (currentRow < numRows-1  && path_back) {
            if (currentRow + 1 == dest_row && currentCol == dest_col){
                backtrack = false;
                direction = 4;
            }
        }
        // Check cell to the left
        if (currentCol > 0 && path_left) {
            if (currentRow == dest_row && currentCol - 1 == dest_col){
                backtrack = false;
                direction = 3;
            }
        }

        if (backtrack){
            int previous_direction = path_pop();
            direction =  getReverseDirection(previous_direction);
            if (direction == 1){
                position = {currentRow - 1, currentCol};
            }else if (direction == 2){
                position = {currentRow, currentCol+1};
            }else if(direction == 3){
                position = {currentRow, currentCol-1};
            }else if(direction == 4){
                position = {currentRow + 1, currentCol};
            }
            visited[currentRow][currentCol] = 3; // mark as backtracked    
                       
        }else{
            state = 0;
            path_push(direction);
        }

        

        break;
    
    default:
        break;
    }


    

    // Process top to know which direction to turn

    // path_push(direction);
    printVisited();
    printStack();
    printPathStack();  
    

    return direction;

}



// TODO: Different handling when mouse is ready. isDestination() should be used.
bool MazeTraversal::logicTraverse(int startRow, int startCol, int endRow, int endCol)
{
    
    // clear visited array and stack
    for (int i = 0; i < numRows; i++)
    {
        for (int j = 0; j < numCols; j++)
        {
            visited[i][j] = 0;
        }
    }
    top = 0;
    Cell startCell = {startRow, startCol};
    push(startCell);
    Cell endCell = {endRow, endCol};

    return dfs(startCell, endCell);
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
        visited[currentRow][currentCol] = 1;
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

void MazeTraversal::path_push(int d)
{
    path_stack[path_top++] = d;
}

int MazeTraversal::path_pop()
{
    return path_stack[--path_top];
}





// TODO: Different handling when mouse is ready. End cell is not known. Need sensor data.
bool MazeTraversal::isDestination(int row, int col)
{   
    if (row == 30 && col == 32)
    {
        return true;
    }
    // cell.row == endRow && cell.col == endCol;
    return false;
}





// void MazeTraversal::resizeVisited(int newNumRows, int newNumCols)
// {
//     // Allocate memory for new visited array
//     bool **newVisited = new bool *[newNumRows];
//     for (int i = 0; i < newNumRows; i++)
//     {
//         newVisited[i] = new bool[newNumCols];
//         for (int j = 0; j < newNumCols; j++)
//         {
//             newVisited[i][j] = false;
//         }
//     }

//     // Copy values from old visited array to new visited array
//     for (int i = 0; i < numRows; i++)
//     {
//         for (int j = 0; j < numCols; j++)
//         {
//             newVisited[i][j] = visited[i][j];
//         }
//     }

//     // Deallocate memory for old visited array
//     for (int i = 0; i < numRows; i++)
//     {
//         delete[] visited[i];
//     }
//     delete[] visited;

//     // Resize stack to match new size of visited array
//     Cell *newStack = new Cell[newNumRows * newNumCols];
//     memcpy(newStack, stack, numRows * numCols * sizeof(Cell));
//     delete[] stack;
//     stack = newStack;

//     // Set member variables to new values
//     visited = newVisited;
//     numRows = newNumRows;
//     numCols = newNumCols;
// }









void MazeTraversal::printCell(Cell cell){
    Serial.print("row: ");
    Serial.print(cell.row - numRows/2);
    Serial.print(" col: ");
    Serial.println(cell.col - numCols/2);
    return;
}

void MazeTraversal::printDirection(int d){
    if (d == 1){
        Serial.println("Forward");
    }else if (d == 2){
        Serial.println("Right");
    }else if (d == 3){
        Serial.println("Left");
    }else if (d == 4){
        Serial.println("Back");
    }
}

int MazeTraversal::getReverseDirection(int d){
    if (d == 1){
        return 4;
    }else if (d == 2){
        return 3;
    }else if (d == 3){
        return 2;
    }else if (d == 4){
        return 1;
    }
    return 0;
}

void MazeTraversal::printVisited(){
    int r = numRows/2 + 8;
    int c = numCols/2 + 10;
    for (int i = numRows/2; i < r; i++){
        Serial.print("        ");
        for (int j = numCols/2; j < c; j++){
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

void MazeTraversal::printPathStack(){
    Serial.println("Path Stack: ");
    for (int i = 0; i < path_top; i++){
        Serial.print("    ");
        printDirection(path_stack[i]);
    }
    Serial.println();
    return;
}

bool MazeTraversal::canGo(int row, int col){
    return *(maze+numCols*row + col) == 1;
}