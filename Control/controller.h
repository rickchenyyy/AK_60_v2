
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

class Motor;    // Forward declaration, prevents "motor.h" from including "controller.h"

#define MAX_COEFFS 10
#define MAX_CONTROLLER_NAME 40

class Controller{
public:
    Controller(const char* name);
    float torqueCommand(Motor *motor);
private:
    char controller_name[MAX_CONTROLLER_NAME];  
};

/**
 * @class Buffer
 * @brief A class to manage a circular buffer of fixed size.
 * 
 * 
 * @example
 * Buffer buf(10);
 * buf.push_front(3.14);
 * buf.push_front(2.71);
 * buf.print();
 * double value = buf[0];
 * 
 * @note The maximum size of the buffer is defined by MAX_COEFFS.
 *
 */
class Buffer {
friend class DiscreteTF;
public:
    Buffer(int size);
    void push_front(double value);
    void pop_back();
    void print();
    double& operator[](int index);
    const double& operator[](int index) const;

private:
    double buffer[MAX_COEFFS];
    int size;
    int current_size;
    double* p_front;
    double* p_tail;
};

/**
 * @class DiscreteTF
 * @brief A class to represent a discrete transfer function for signal processing.
 *
 * @example
 * // Define coefficients for the transfer function
 * double a[] = {1.0, -0.5};
 * double b[] = {0.5, 0.5};
 * 
 * // Create an instance of DiscreteTF
 * DiscreteTF tf(a, 2, b, 2);
 * 
 * // Input a signal value and get the filtered output
 * double output = tf.output(1.0);
 *
 * @note The size of the coefficient arrays should not exceed MAX_COEFFS.
 */
class DiscreteTF {
public:
    // Constructor
    DiscreteTF(const double* a, int a_size, const double* b, int b_size);

    // Function to input one signal value at a time and get the filtered output
    double output(double u_n);

    double a[MAX_COEFFS]; // Coefficients for the output side
    double b[MAX_COEFFS]; // Coefficients for the input side
    int a_size;
    int b_size;

    Buffer y; // History of output values
    Buffer u; // History of input values
};

#endif // CONTROLLER_H
