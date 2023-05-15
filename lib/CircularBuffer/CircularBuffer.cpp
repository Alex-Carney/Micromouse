#include "CircularBuffer.h"
#include <stdlib.h> // for dynamic memory allocation

template <typename T>
CircularBuffer<T>::CircularBuffer(int steps, int values) : steps(steps), values(values), index(0)
{
    // Allocate memory for the buffer
    buffer = new int *[steps];
    for (int i = 0; i < steps; i++)
    {
        buffer[i] = new int[values];
        // Initialize the buffer to zero
        for (int j = 0; j < values; j++)
        {
            buffer[i][j] = 0;
        }
    }
}

template <typename T>
CircularBuffer<T>::~CircularBuffer()
{
    // Deallocate memory for the buffer
    for (int i = 0; i < steps; i++)
    {
        delete[] buffer[i];
    }
    delete[] buffer;
}

template <typename T>
void CircularBuffer<T>::addValues(T *newValues)
{
    // Add the values to the buffer at the current index
    for (int i = 0; i < values; i++)
    {
        buffer[index][i] = newValues[i];
    }

    // Update the index
    index = (index + 1) % steps;
}

template <typename T>
T CircularBuffer<T>::getValue(int stepsBack, int valueIndex)
{
    // Get the value from the specified step and index
    return buffer[(index - stepsBack + steps) % steps][valueIndex];
}
