#include <stdio.h>
#include <stdlib.h>

#include "tcache.h"

// ==============================================================
// ===========  THIS FILE WILL BE REPLACED BY GRADER ============
// ===========  THIS FILE WILL BE REPLACED BY GRADER ============
// ===========  THIS FILE WILL BE REPLACED BY GRADER ============
// ==============================================================

uint8_t memory[HW11_MEM_SIZE];

void check_address(uint64_t mem_addr) {
    if (mem_addr >= HW11_MEM_SIZE) {
        fprintf(stderr, "Memory address out of bounds: 0x%lx\n", mem_addr);
        exit(EXIT_FAILURE);
    }
}

uint8_t read_memory(uint64_t mem_addr) {
    check_address(mem_addr);
    return memory[mem_addr];
}

void write_memory(uint64_t mem_addr, uint8_t value) {
    check_address(mem_addr);
    memory[mem_addr] = value;
}