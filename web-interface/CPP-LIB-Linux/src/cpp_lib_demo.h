// author: AECL

#pragma once

#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C"
#endif

#ifndef CPP_LIB_DEMO_H_
#define CPP_LIB_DEMO_H_

EXPORT bool Mult100(double* val_in,  int val_in_size, double* val_out,  int val_out_size);

#endif /* CPP_LIB_DEMO_H_ */