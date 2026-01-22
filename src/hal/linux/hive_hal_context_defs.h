// x86-64 Context Structure
//
// Stores callee-saved registers per System V AMD64 ABI:
// rbx, rbp, r12-r15, and rsp

#ifndef HIVE_HAL_CONTEXT_DEFS_H
#define HIVE_HAL_CONTEXT_DEFS_H

typedef struct {
    void *rsp; // Stack pointer
    void *rbx;
    void *rbp;
    void *r12;
    void *r13;
    void *r14;
    void *r15;
} hive_context;

#endif // HIVE_HAL_CONTEXT_DEFS_H
