// 极简测试文件
#include <stdio.h>
#include "pid_controller.hpp"

int main() {
    printf("PID Test\n");
    
    PIDController pid(2.0, 1.0, 0.5, 0.01);
    
    double setpoint = 1.0;
    double process_val = 0.0;
    
    for (int i = 0; i < 10; i++) {
        double output = pid.compute(setpoint, process_val);
        printf("Iteration %d: setpoint=%.2f, process_val=%.2f, output=%.2f\n", 
               i, setpoint, process_val, output);
        
        process_val += output * 0.1;
    }
    
    printf("Test completed\n");
    return 0;
}