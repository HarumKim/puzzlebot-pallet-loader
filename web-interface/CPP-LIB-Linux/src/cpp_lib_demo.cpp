//author AECL
#define DEBUG

#include <iostream>
#include <stdio.h>

#include "cpp_lib_demo.h"

bool Mult100(double* val_in, int val_in_size, double* val_out, int val_out_size){
    for(int i=0; i<val_out_size; i++){
        val_out[i] = val_in[i]*100.0;
    }
    return true;
}

#ifdef DEBUG
int main(int argc, char** argv){
    std::cout << "START" << std::endl;
    double test_in_vals[2] = {1.0, 2.0};
    double* test_out_vals = new double[2];

    bool res = false;

    res = Mult100(test_in_vals, 2, test_out_vals, 2);
    if(res){
        std::cout << "OK" << std::endl;
        std::cout << test_out_vals[0] << std::endl;
        std::cout << test_out_vals[1] << std::endl;
    }
    else{
        std::cout << "NG" << std::endl;
    }
    return 0;
}
#endif
