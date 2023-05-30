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
            visited[i][j] = 5;
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
    path_stack = new pathCell[numRows * numCols];
    top = 0;
    path_top = 0;
}





bool MazeTraversal::initilizeTraversal(){

    // clear visited array and stack
    for (int i = 0; i < numRows; i++)
    {
        for (int j = 0; j < numCols; j++)
        {
            visited[i][j] = 5;
        }
    }
    top = 0;
    Cell startCell = {numCols/2, numCols/2}; // assume starting point has corrdinates (row = 0, col = 0)
    push(startCell);
    path_top = 0;
    path_push({North, 1});
    cur_row = numRows/2;
    cur_col = numCols/2;

    return true;
}


int MazeTraversal::traverse(int path_right, int path_left, int path_forward, int path_back, long encoder_val){

    // Initialize variables

    int order = 1; // for printing the path
    int direction = 0; // which direction to turn. 1 = North, 2 = East, 3 = West, 4 = South
    unsigned long initial_time;
    int curr_orientation = orientation;
    int path_north, path_east, path_west, path_south;
    pathCell currentCell;


    bool backtrack = true;

    Serial.println("####################################Starting loop: ");
    Serial.print("    My orientaion is: ");
    printDirection(orientation);

    int steps = encoder_val;


    switch (state)
    {
        // traversal    
        case 0:
            if (path_top > 0){
                initial_time = micros();
                
                // convert to maze frame
                Path path = convertToMazeOrientation(path_right, path_left, path_forward, path_back);
                path_east = path.right;
                path_west = path.left;
                path_north = path.forward;
                path_south = path.back;
                
                
                // update path stack
                if (path_north) {
                    path_push({North, 0});
                    backtrack = false;
                    direction = North;
                }

                if (path_east) {
                    path_push({East, 0});
                    backtrack = false;
                    direction = East;
                }

                // Check cell to the bottom
                if (path_south) {
                    path_push({South, 0});
                    backtrack = false;
                    direction = South;
                }
                // Check cell to the left
                if (path_west) {
                    path_push({West, 0});
                    backtrack = false;
                    direction = West;
                }

                

                if (!backtrack){
                    path_stack[path_top-1].visited = 1;    
                }else{
                    Serial.println("Backtracking first#########################################");
                    // TODO: checkk if turn automatically
                    int previous_direction = path_stack[path_top-1].direction;
                    direction = getReverseDirection(previous_direction);
                    path_pop();    
                    state = 1;       
                }

            }else{
                return 10;
            }
            break;

        case 1:
            Serial.println("########################## Backtracking");
            
            currentCell = path_stack[path_top-1];

            if (currentCell.visited){
                path_pop();
                direction = getReverseDirection(currentCell.direction);
            }else{
                path_stack[path_top-1].visited = 1;
                direction = currentCell.direction;
                state = 0;
            }           

            break;
        
        default:
            break;
    }

    int i;
    if (backtrack == true || state == 1)
        i = 0;
    else 
        i = 1;
    if (orientation == North){
        while (i <= steps){
            visited[cur_row-i][cur_col] = North;
            i++;
        }
        cur_row -= steps;
        // visited[cur_row][cur_col] = North;
    }else if (orientation == East){
        while (i <= steps){
            visited[cur_row][cur_col+i] = East;
            i++;
        }
        cur_col += steps;
        // visited[cur_row][cur_col] = East;
    }else if (orientation == South){
        while (i <= steps){
            visited[cur_row+i][cur_col] = South;
            i++;
        }
        cur_row += steps;
        // visited[cur_row][cur_col] = South;
    }else if (orientation == West){
        while (i <= steps){
            visited[cur_row][cur_col-i] = West;
            i++;
        }
        cur_col -= steps;
        // visited[cur_row][cur_col] = West;
    }


    

    // Process top to know which direction to turn

    // path_push(direction);

    printPathStack();  
    printVisited();
    int mdirection = convertToMouseOrientation(direction);
    Serial.print("Going to: ");
    printMouseDirection(mdirection);
    updateMouseOrientation(mdirection);

    return mdirection;

}




int MazeTraversal::getMouseOrientation(){
    return orientation;
}

// here d is the direction in the mouse frame
void MazeTraversal::updateMouseOrientation(int d){
    if (orientation == North){
        orientation = d;
    }else if (orientation == East){
        if (d == North){
            orientation = East;
        }else if (d == East){
            orientation = South;
        }else if (d == West){
            orientation = North;
        }else if (d == South){
            orientation = West;
        }
    } else if (orientation == 3){
        if (d == 1){
            orientation = 3;
        }else if (d == 2){
            orientation = 1;
        }else if (d == 3){
            orientation = 4;
        }else if (d == 4){
            orientation = 2;
        }
    } else if (orientation == 4){
        if (d == 1){
            orientation = 4;
        }else if (d == 2){
            orientation = 3;
        }else if (d == 3){
            orientation = 2;
        }else if (d == 4){
            orientation = 1;
        }     
    }
}


// 1 = North, 2 = East, 3 = West, 4 = South
int MazeTraversal::convertToMouseOrientation(int d){
    // if mouse pointing to the south of the logical maze
    if (orientation == 4){
        if (d == 1){
            return 4;
        }else if (d == 2){
            return 3;
        }else if (d == 3){
            return 2;
        }else if (d == 4){
            return 1;
        }
    } 
    // if mouse is pointing to the north of the logical maze (default)
    else if (orientation == 1){
        return d;
    } 
    // if mouse is pointing to the east of the logical maze
    else if (orientation == 2){
        if (d == 1){
            return 3;
        }else if (d == 2){
            return 1;
        }else if (d == 3){
            return 4;
        }else if (d == 4){
            return 2;
        }
    } 
    // if the mouse is pointing to the west of the logical maze
    else if (orientation == 3){
        if (d == 1){
            return 2;
        }else if (d == 2){
            return 4;
        }else if (d == 3){
            return 1;
        }else if (d == 4){
            return 3;
        }
    }

    return 10;

}


MazeTraversal::Path MazeTraversal::convertToMazeOrientation(int path_right, int path_left, int path_forward, int path_back){

    Path possible_paths = {path_right, path_left, path_forward, path_back};

    if (orientation == North){
        return possible_paths;
    } else if (orientation == East){
        possible_paths.back = path_right;
        possible_paths.forward = path_left;
        possible_paths.right = path_forward;
        possible_paths.left = path_back;
    } else if (orientation == West){
        possible_paths.forward = path_right;
        possible_paths.back = path_left;
        possible_paths.left = path_forward;
        possible_paths.right = path_back;
    } else if (orientation == South){
        possible_paths.left = path_right;
        possible_paths.right = path_left;
        possible_paths.back = path_forward;
        possible_paths.forward = path_back;
    }

    return possible_paths;
    
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





void MazeTraversal::push(Cell cell)
{
    stack[top++] = cell;
}

MazeTraversal::Cell MazeTraversal::pop()
{
    return stack[--top];
}

void MazeTraversal::path_push(MazeTraversal::pathCell cell)
{
    path_stack[path_top++] = cell;
}

MazeTraversal::pathCell MazeTraversal::path_pop()
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



void MazeTraversal::printCell(Cell cell){
    Serial.print("row: ");
    Serial.print(cell.row - numRows/2);
    Serial.print(" col: ");
    Serial.println(cell.col - numCols/2);
    return;
}

void MazeTraversal::printDirection(int d){
    if (d == North){
        Serial.println("North");
    }else if (d == East){
        Serial.println("East");
    }else if (d == West){
        Serial.println("West");
    }else if (d == South){
        Serial.println("South");
    }
}
void MazeTraversal::printMouseDirection(int d){
    if (d == North){
        Serial.println("Forward");
    }else if (d == East){
        Serial.println("Right");
    }else if (d == West){
        Serial.println("Left");
    }else if (d == South){
        Serial.println("Back");
    }
}



void MazeTraversal::printnlDirection(int d){
    if (d == 1){
        Serial.print("North");
    }else if (d == 2){
        Serial.print("East");
    }else if (d == 3){
        Serial.print("West");
    }else if (d == 4){
        Serial.print("South");
    }
}


void MazeTraversal::printVisited(){
    int r = numRows;
    int c = numCols;
    for (int i = 0; i < r; i++){
        Serial.print("        ");
        for (int j = 0; j < c; j++){
            if (visited[i][j] != 5)
                Serial.print(visited[i][j]);
                // if (visited[i][j] == North)
                //     Serial.print("N");
                // else if (visited[i][j] == East)
                //     Serial.print("E");
                // else if (visited[i][j] == West)
                //     Serial.print("W");
                // else if (visited[i][j] == South)
                //     Serial.print("S");
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
        Serial.print("    {");
        printnlDirection(path_stack[i].direction);
        Serial.print(", ");
        Serial.print(path_stack[i].visited);
        Serial.println("}");
    }
    Serial.println();
    return;
}

bool MazeTraversal::canGo(int row, int col){
    return *(maze+numCols*row + col) == 1;
}