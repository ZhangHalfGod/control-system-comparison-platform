// Controller base class definition
#ifndef _CONTROLLER_BASE_H_
#define _CONTROLLER_BASE_H_

// Controller base class, defines unified interface
class Controller {
public:
    // Virtual destructor
    virtual ~Controller() = default;
    
    // Calculate controller output - pure virtual function
    virtual double compute(double setpoint, double process_val) = 0;
    
    // Reset controller state - virtual function
    virtual void reset() = 0;
};

#endif // _CONTROLLER_BASE_H_