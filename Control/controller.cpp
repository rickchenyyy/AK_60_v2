#include "controller.h"
#include "motor.h"

float Controller::torqueCommand(Motor *motor)
{
    if (strcmp(controller_name, "position_pd_w_compensation") == 0)
    {
        double pos_err = motor->pos_command - motor->pos_feedback;
        if (fabs(pos_err) < MIN_POS_RESLOUTION)
        {
            pos_err = 0;
        }
        static double b[] = {25.71, -25.22};
        static double a[] = {1, -0.8365};
        // static double b[] = {1, 1};
        // static double a[] = {1};
        static DiscreteTF tf(a, 2, b, 2);
        static double torque_comp = 0.0;

        if (motor->pos_command > 0)
        {
            if (pos_err > 0.02 || pos_err < -0.02)
            {
                torque_comp = 0.2 * pos_err;
            }
        }
        return float(tf.output(pos_err) + torque_comp);
    }
    else if (strcmp(controller_name, "position_pi") == 0)
    {
        return 0.1;
    }
    else
    {
        printf("Error in Controller::torqueCommand: unknown controller name\n");
    }
}

Controller::Controller(const char *name)
{
    strcpy(controller_name, name);
}

// DiscreteTF class implementation
DiscreteTF::DiscreteTF(const double *a, int a_size, const double *b, int b_size)
    : a_size(a_size), b_size(b_size), y(a_size), u(b_size)
{
    memcpy(this->a, a, a_size * sizeof(double));
    memcpy(this->b, b, b_size * sizeof(double));
}

double DiscreteTF::output(double u_n)
{
    double y_n = 0.0;
    if (u.current_size < b_size)
    {
        u.push_front(u_n);
    }
    else
    {
        u.pop_back();
        u.push_front(u_n);
    }
    // printf("u: ");
    // u.print();
    // printf("y: ");
    // y.print();
    for (int i = 0; i < b_size; ++i)
    {
        y_n += b[i] * u[i];
    }
    for (int i = 1; i < a_size; ++i)
    {
        y_n -= a[i] * y[i - 1];
    }
    // Update history
    if (y.current_size < a_size)
    {
        y.push_front(y_n);
    }
    else
    {
        y.pop_back();
        y.push_front(y_n);
    }

    return y_n;
}

// Constructor
Buffer::Buffer(int size) : size(size), current_size(0), p_front(buffer), p_tail(buffer)
{
    // Initialize buffer with zeros
    for (int i = 0; i < MAX_COEFFS; ++i)
    {
        buffer[i] = 0.0;
    }
}
void Buffer::push_front(double value)
{
    if (current_size < size)
    {
        *p_front = value;
        p_front = (p_front == buffer + size - 1) ? buffer : p_front + 1;
        ++current_size;
    }
    else
    {
        // Buffer is full, overwrite the oldest value
        *p_front = value;
        p_front = (p_front == buffer + size - 1) ? buffer : p_front + 1;
        p_tail = (p_tail == buffer + size - 1) ? buffer : p_tail + 1;
    }
}

void Buffer::pop_back()
{
    if (current_size > 0)
    {
        p_tail = (p_tail == buffer + size - 1) ? buffer : p_tail + 1;
        --current_size;
    }
}

void Buffer::print()
{
    printf("Buffer contents (from front to tail): ");
    double *ptr = (p_front == buffer) ? buffer + size - 1 : p_front - 1;
    for (int i = 0; i < current_size; ++i)
    {
        printf("%f ", *ptr);
        ptr = (ptr == buffer) ? buffer + size - 1 : ptr - 1;
    }
    printf("\n");
}

double &Buffer::operator[](int index)
{
    return buffer[(p_front - buffer - 1 - index + size) % size];
}

const double &Buffer::operator[](int index) const
{
    return buffer[(p_front - buffer - 1 - index + size) % size];
}

/*  Testing Buffer and DiscreteTF classes
void test_discrete_tf() {
    // Coefficients
    const double a[] = {1, 0.8};  // a0, a1
    const double b[] = {0.2, 0.1};  // b0, b1

    // Create the Discrete Transfer Function object
    DiscreteTF tf(a, 2, b, 2);

    // Input signal (you can change this with any signal you want)
    double input_signal[10] = {1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001};

    std::cout << "Filtered output data: " << std::endl;
    for (int i = 0; i < 10; ++i) {
        double output = tf.output(input_signal[i]);
        std::cout << "y[" << i << "] = " << output << std::endl;
    }
}

void testBuffer() {
    Buffer buffer(MAX_COEFFS);

    // Test adding elements to the buffer
    std::cout << "Adding elements to the buffer:" << std::endl;
    for (int i = 0; i < MAX_COEFFS; ++i) {
        buffer.push_front(i + 1);
        // buffer.print();
    }

    // Test accessing elements using operator[]
    std::cout << "Accessing elements using operator[]:" << std::endl;
    for (int i = 0; i < MAX_COEFFS; ++i) {
        std::cout << "buffer[" << i << "] = " << buffer[i] << std::endl;
    }

    // Test overwriting elements when buffer is full
    std::cout << "Overwriting elements when buffer is full:" << std::endl;
    for (int i = 0; i < 5; ++i) {
        buffer.push_front(i + 11);
        buffer.print();
    }

    // Test popping elements from the back
    std::cout << "Popping elements from the back:" << std::endl;
    for (int i = 0; i < 5; ++i) {
        buffer.pop_back();
        buffer.print();
    }
}
*/
