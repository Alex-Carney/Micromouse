#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

/**
 * @brief Implements 2D CircularBuffer data structure
 * @author Alex Carney and GPT4
 */
template <typename T>
class CircularBuffer
{
private:
    int steps;  // Number of steps to store
    int values; // Number of values per step
    T **buffer; // Stores the values
    int index;  // Points to the current location in the buffer

public:
    CircularBuffer(int steps, int values);
    ~CircularBuffer();

    void addValues(T *newValues);              // Takes an array of 'values' integers
    T getValue(int stepsBack, int valueIndex); // Gets a specific value
};

#endif // CIRCULARBUFFER_H
