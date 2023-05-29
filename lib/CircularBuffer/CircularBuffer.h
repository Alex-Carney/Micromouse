#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

/**
 * @brief Implements 2D CircularBuffer data structure
 * Note, this also includes the implementation...
 * "The error you're seeing is likely due to the fact that C++ 
 * templates are typically implemented in header files. 
 * The compiler needs to have access to the entire 
 * template definition (not just the declaration) 
 * in order to generate code for each instantiation of the template." 
 * @author Alex Carney and GPT4
 */
template <typename T>
// class CircularBuffer
// {
// private:
//     int steps;  // Number of steps to store
//     int values; // Number of values per step
//     T **buffer; // Stores the values
//     int index;  // Points to the current location in the buffer

// public:
//     CircularBuffer<T>(int steps, int values);
//     ~CircularBuffer<T>();

//     void addValues(T *newValues);              // Takes an array of 'values' integers
//     T getValue(int stepsBack, int valueIndex); // Gets a specific value
// };
class CircularBuffer
{
private:
    int steps;  // Number of steps to store
    int values;  // Number of values per step
    T** buffer;  // Stores the values
    int index;  // Points to the current location in the buffer
public:
CircularBuffer(int steps, int values) : steps(steps), values(values), index(0)
{
    // Allocate memory for the buffer
    buffer = new T *[steps];
    for (int i = 0; i < steps; i++)
    {
        buffer[i] = new T[values];
        // Initialize the buffer to zero
        for (int j = 0; j < values; j++)
        {
            buffer[i][j] = 0;
        }
    }
}

~CircularBuffer()
{
    // Deallocate memory for the buffer
    for (int i = 0; i < steps; i++)
    {
        delete[] buffer[i];
    }
    delete[] buffer;
}

void addValues(T *newValues)
{
    // Add the values to the buffer at the current index
    for (int i = 0; i < values; i++)
    {
        buffer[index][i] = newValues[i];
    }

    // Update the index
    index = (index + 1) % steps;
}

T getValue(int stepsBack, int valueIndex)
{
    // Get the value from the specified step and index
    return buffer[(index - stepsBack + steps) % steps][valueIndex];
}

void clear()
{
    for (int i = 0; i < steps; i++)
    {
        for (int j = 0; j < values; j++)
        {
            buffer[i][j] = 0;
        }
    }
}

void print() const
    {
        for (int i = 0; i < steps; i++)
        {
            if( i == index ) {
                Serial.print("-> ");
            } else {
                Serial.print("   ");
            }
            for (int j = 0; j < values; j++)
            {
                Serial.print(buffer[i][j]);
                Serial.print(" ");
            }
            Serial.println();
        }
    }

};

#endif // CIRCULARBUFFER_H
