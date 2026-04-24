#ifndef TCACHE_H
#define TCACHE_H

#include <stdint.h>

// ==============================================================
// ===========  THIS FILE WILL BE REPLACED BY GRADER ============
// ===========  THIS FILE WILL BE REPLACED BY GRADER ============
// ===========  THIS FILE WILL BE REPLACED BY GRADER ============
// ==============================================================

// ==============================================================
// ====================== HARNESS CODE ==========================
// ==============================================================

#define HW11_L1_SIZE (32 * 1024)  // 32 KB
#define HW11_L1_INSTR_ASSOC 1
#define HW11_L1_DATA_ASSOC 2

#define HW11_L2_SIZE (2 * 1024 * 1024)  // 2 MB
#define HW11_L2_ASSOC 4

#define HW11_MEM_SIZE (16 * 1024 * 1024)  // 16 MB

// cache line struct
typedef struct {
    uint8_t valid;
    uint8_t modified;
	uint64_t tag;
    uint8_t data[64];
} cache_line_t;

// logging struct to fill out for stats
typedef struct {
    uint64_t accesses;
    uint64_t misses;
} cache_stats_t;

// helper functions for memory access, we will implement these our end
uint8_t read_memory(uint64_t mem_addr);
void write_memory(uint64_t mem_addr, uint8_t value);

typedef enum {
    INSTR,
    DATA
} mem_type_t;

typedef enum {
	LRU,
	RANDOM
} replacement_policy_e;

// grader may insert stuff here

// ==============================================================
// ====================== STUDENT TODO ==========================
// ==============================================================

void init_cache(replacement_policy_e policy);

uint8_t read_cache(uint64_t mem_addr, mem_type_t type);

void write_cache(uint64_t mem_addr, uint8_t value, mem_type_t type);

cache_stats_t get_l1_instr_stats();
cache_stats_t get_l1_data_stats();
cache_stats_t get_l2_stats();

cache_line_t* get_l1_instr_cache_line(uint64_t mem_addr);
cache_line_t* get_l1_data_cache_line(uint64_t mem_addr);
cache_line_t* get_l2_cache_line(uint64_t mem_addr);

#endif  // TCACHE_H