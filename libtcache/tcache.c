#include "tcache.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* ── Address layout (MSB→LSB): tag | index | offset ─────────────────── */

#define OFFSET_BITS  6
#define LINE_SIZE    64
#define OFFSET_MASK  ((uint64_t)(LINE_SIZE - 1))
#define BASE_ADDR(a) ((a) & ~OFFSET_MASK)
#define OFFSET(a)    ((uint8_t)((a) & OFFSET_MASK))

/* L1 Instruction: direct-mapped, 512 sets */
#define L1I_INDEX_BITS 9
#define L1I_SETS       (1 << L1I_INDEX_BITS)
#define L1I_INDEX(a)   ((uint32_t)(((a) >> OFFSET_BITS) & (L1I_SETS - 1)))
#define L1I_TAG(a)     ((a) >> (OFFSET_BITS + L1I_INDEX_BITS))
#define L1I_BASE(t,i)  (((uint64_t)(t) << (OFFSET_BITS + L1I_INDEX_BITS)) | ((uint64_t)(i) << OFFSET_BITS))

/* L1 Data: 2-way set-associative, 256 sets */
#define L1D_INDEX_BITS 8
#define L1D_SETS       (1 << L1D_INDEX_BITS)
#define L1D_INDEX(a)   ((uint32_t)(((a) >> OFFSET_BITS) & (L1D_SETS - 1)))
#define L1D_TAG(a)     ((a) >> (OFFSET_BITS + L1D_INDEX_BITS))
#define L1D_BASE(t,i)  (((uint64_t)(t) << (OFFSET_BITS + L1D_INDEX_BITS)) | ((uint64_t)(i) << OFFSET_BITS))

/* L2: 4-way set-associative, 8192 sets */
#define L2_INDEX_BITS  13
#define L2_SETS        (1 << L2_INDEX_BITS)
#define L2_INDEX(a)    ((uint32_t)(((a) >> OFFSET_BITS) & (L2_SETS - 1)))
#define L2_TAG(a)      ((a) >> (OFFSET_BITS + L2_INDEX_BITS))
#define L2_BASE(t,i)   (((uint64_t)(t) << (OFFSET_BITS + L2_INDEX_BITS))  | ((uint64_t)(i) << OFFSET_BITS))

/* ── Storage ──────────────────────────────────────────────────────────── */

static cache_line_t l1i[L1I_SETS][HW11_L1_INSTR_ASSOC];
static cache_line_t l1d[L1D_SETS][HW11_L1_DATA_ASSOC];
static cache_line_t l2c[L2_SETS][HW11_L2_ASSOC];

/* LRU: L1D uses a single bit per set (which way is LRU).
   L2 uses ages per way: 0 = MRU, ASSOC-1 = LRU. */
static uint8_t l1d_lru[L1D_SETS];
static uint8_t l2_age[L2_SETS][HW11_L2_ASSOC];

static cache_stats_t l1i_stats, l1d_stats, l2_stats;
static replacement_policy_e g_policy;

/* ── LRU helpers ─────────────────────────────────────────────────────── */

static void l1d_touch(uint32_t idx, int way) {
    l1d_lru[idx] = 1 - way;
}

static int l1d_pick_victim(uint32_t idx) {
    for (int w = 0; w < HW11_L1_DATA_ASSOC; w++)
        if (!l1d[idx][w].valid) return w;
    if (g_policy == RANDOM) return rand() % HW11_L1_DATA_ASSOC;
    return l1d_lru[idx];
}

static void l2_touch(uint32_t idx, int way) {
    uint8_t old = l2_age[idx][way];
    for (int w = 0; w < HW11_L2_ASSOC; w++)
        if (w != way && l2_age[idx][w] < old)
            l2_age[idx][w]++;
    l2_age[idx][way] = 0;
}

static int l2_lru_way(uint32_t idx) {
    for (int w = 0; w < HW11_L2_ASSOC; w++)
        if (l2_age[idx][w] == HW11_L2_ASSOC - 1) return w;
    return 0;
}

static int l2_pick_victim(uint32_t idx) {
    for (int w = 0; w < HW11_L2_ASSOC; w++)
        if (!l2c[idx][w].valid) return w;
    if (g_policy == RANDOM) return rand() % HW11_L2_ASSOC;
    return l2_lru_way(idx);
}

/* ── Forward declarations (coherence helpers call l2_writeback) ──────── */
static void l2_writeback(uint64_t base, cache_line_t *dirty);

/* ── Coherence helpers ───────────────────────────────────────────────────
   These are NOT cache accesses and do not increment L1 stats.
   A dirty-line flush to L2 DOES count as an L2 access (write from L1).   */

/* If L1I has base dirty, write it back to L2 (L2 access) and clear
   the modified bit.  The L1I line stays valid (flush, not evict). */
static void coherence_flush_l1i(uint64_t base) {
    uint32_t i = L1I_INDEX(base);
    if (l1i[i][0].valid && l1i[i][0].tag == L1I_TAG(base) &&
            l1i[i][0].modified) {
        l2_writeback(base, &l1i[i][0]);
        l1i[i][0].modified = 0;
    }
}

/* If L1D has base dirty (any way), write it back to L2 and clear modified. */
static void coherence_flush_l1d(uint64_t base) {
    uint32_t i = L1D_INDEX(base);
    uint64_t t = L1D_TAG(base);
    for (int w = 0; w < HW11_L1_DATA_ASSOC; w++) {
        if (l1d[i][w].valid && l1d[i][w].tag == t && l1d[i][w].modified) {
            l2_writeback(base, &l1d[i][w]);
            l1d[i][w].modified = 0;
            break;
        }
    }
}

/* Flush L1I (if dirty) then invalidate it.  Called on DATA writes. */
static void coherence_invalidate_l1i(uint64_t base) {
    uint32_t i = L1I_INDEX(base);
    if (l1i[i][0].valid && l1i[i][0].tag == L1I_TAG(base)) {
        if (l1i[i][0].modified)
            l2_writeback(base, &l1i[i][0]);   /* L2 access */
        l1i[i][0].valid = l1i[i][0].modified = 0;
    }
}

/* Flush L1D (if dirty) then invalidate it.  Called on INSTR writes. */
static void coherence_invalidate_l1d(uint64_t base) {
    uint32_t i = L1D_INDEX(base);
    uint64_t t = L1D_TAG(base);
    for (int w = 0; w < HW11_L1_DATA_ASSOC; w++) {
        if (l1d[i][w].valid && l1d[i][w].tag == t) {
            if (l1d[i][w].modified)
                l2_writeback(base, &l1d[i][w]);   /* L2 access */
            l1d[i][w].valid = l1d[i][w].modified = 0;
            break;
        }
    }
}

/* ── L2 eviction (inclusive: must flush L1 first) ───────────────────── */

static void l2_evict_line(uint32_t idx, int way) {
    cache_line_t *vl = &l2c[idx][way];
    if (!vl->valid) return;

    uint64_t vbase = L2_BASE(vl->tag, idx);
    bool l1_wrote = false;

    /* L1I — forced writeback counts as an L2 access */
    {
        uint32_t i = L1I_INDEX(vbase);
        if (l1i[i][0].valid && l1i[i][0].tag == L1I_TAG(vbase)) {
            if (l1i[i][0].modified) {
                l2_stats.accesses++;           /* forced L1→L2 writeback */
                for (int b = 0; b < LINE_SIZE; b++)
                    write_memory(vbase + b, l1i[i][0].data[b]);
                l1_wrote = true;
            }
            l1i[i][0].valid = l1i[i][0].modified = 0;
        }
    }

    /* L1D — forced writeback counts as an L2 access */
    {
        uint32_t i  = L1D_INDEX(vbase);
        uint64_t dt = L1D_TAG(vbase);
        for (int w = 0; w < HW11_L1_DATA_ASSOC; w++) {
            if (l1d[i][w].valid && l1d[i][w].tag == dt) {
                if (l1d[i][w].modified) {
                    l2_stats.accesses++;       /* forced L1→L2 writeback */
                    for (int b = 0; b < LINE_SIZE; b++)
                        write_memory(vbase + b, l1d[i][w].data[b]);
                    l1_wrote = true;
                }
                l1d[i][w].valid = l1d[i][w].modified = 0;   /* always back-invalidate */
                break;
            }
        }
    }

    if (!l1_wrote && vl->modified)
        for (int b = 0; b < LINE_SIZE; b++)
            write_memory(vbase + b, vl->data[b]);

    vl->valid = vl->modified = 0;
}

/* ── L2 access (called by L1 on miss or writeback) ───────────────────── */

static cache_line_t *l2_fetch(uint64_t base) {
    uint32_t idx = L2_INDEX(base);
    uint64_t tag = L2_TAG(base);

    l2_stats.accesses++;

    for (int w = 0; w < HW11_L2_ASSOC; w++) {
        if (l2c[idx][w].valid && l2c[idx][w].tag == tag) {
            if (g_policy == LRU) l2_touch(idx, w);
            return &l2c[idx][w];
        }
    }

    l2_stats.misses++;
    int v = l2_pick_victim(idx);
    l2_evict_line(idx, v);

    cache_line_t *line = &l2c[idx][v];
    for (int b = 0; b < LINE_SIZE; b++)
        line->data[b] = read_memory(base + b);
    line->valid    = 1;
    line->modified = 0;
    line->tag      = tag;
    if (g_policy == LRU) l2_touch(idx, v);
    return line;
}

static void l2_writeback(uint64_t base, cache_line_t *dirty) {
    uint32_t idx = L2_INDEX(base);
    uint64_t tag = L2_TAG(base);

    l2_stats.accesses++;

    for (int w = 0; w < HW11_L2_ASSOC; w++) {
        if (l2c[idx][w].valid && l2c[idx][w].tag == tag) {
            memcpy(l2c[idx][w].data, dirty->data, LINE_SIZE);
            l2c[idx][w].modified = 1;
            if (g_policy == LRU) l2_touch(idx, w);
            return;
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

void init_cache(replacement_policy_e policy) {
    g_policy = policy;
    memset(l1i,      0, sizeof(l1i));
    memset(l1d,      0, sizeof(l1d));
    memset(l2c,      0, sizeof(l2c));
    memset(l1d_lru,  0, sizeof(l1d_lru));
    for (int i = 0; i < L2_SETS; i++)
        for (int w = 0; w < HW11_L2_ASSOC; w++)
            l2_age[i][w] = w;
    memset(&l1i_stats, 0, sizeof(l1i_stats));
    memset(&l1d_stats, 0, sizeof(l1d_stats));
    memset(&l2_stats,  0, sizeof(l2_stats));
}

uint8_t read_cache(uint64_t mem_addr, mem_type_t type) {
    if (type == INSTR) {
        uint32_t idx  = L1I_INDEX(mem_addr);
        uint64_t tag  = L1I_TAG(mem_addr);
        uint8_t  off  = OFFSET(mem_addr);
        uint64_t base = BASE_ADDR(mem_addr);

        l1i_stats.accesses++;

        /* Hit — L1D cannot be dirty for this block (DATA writes invalidate L1I) */
        if (l1i[idx][0].valid && l1i[idx][0].tag == tag)
            return l1i[idx][0].data[off];

        /* Miss */
        l1i_stats.misses++;

        /* Evict current L1I occupant if dirty */
        if (l1i[idx][0].valid && l1i[idx][0].modified)
            l2_writeback(L1I_BASE(l1i[idx][0].tag, idx), &l1i[idx][0]);

        /* Coherence: if L1D has a dirty version, flush it to L2 first so the
           fetch below gets up-to-date data. */
        coherence_flush_l1d(base);

        cache_line_t *l2l = l2_fetch(base);
        memcpy(l1i[idx][0].data, l2l->data, LINE_SIZE);
        l1i[idx][0].valid    = 1;
        l1i[idx][0].modified = 0;
        l1i[idx][0].tag      = tag;

        return l1i[idx][0].data[off];

    } else { /* DATA */
        uint32_t idx  = L1D_INDEX(mem_addr);
        uint64_t tag  = L1D_TAG(mem_addr);
        uint8_t  off  = OFFSET(mem_addr);
        uint64_t base = BASE_ADDR(mem_addr);

        l1d_stats.accesses++;

        /* Hit */
        for (int w = 0; w < HW11_L1_DATA_ASSOC; w++) {
            if (l1d[idx][w].valid && l1d[idx][w].tag == tag) {
                if (g_policy == LRU) l1d_touch(idx, w);
                return l1d[idx][w].data[off];
            }
        }

        /* Miss */
        l1d_stats.misses++;
        int v = l1d_pick_victim(idx);

        if (l1d[idx][v].valid && l1d[idx][v].modified)
            l2_writeback(L1D_BASE(l1d[idx][v].tag, idx), &l1d[idx][v]);

        /* Coherence: flush dirty L1I to L2 before fetching */
        coherence_flush_l1i(base);

        cache_line_t *l2l = l2_fetch(base);
        memcpy(l1d[idx][v].data, l2l->data, LINE_SIZE);
        l1d[idx][v].valid    = 1;
        l1d[idx][v].modified = 0;
        l1d[idx][v].tag      = tag;
        if (g_policy == LRU) l1d_touch(idx, v);

        return l1d[idx][v].data[off];
    }
}

void write_cache(uint64_t mem_addr, uint8_t value, mem_type_t type) {
    if (type == INSTR) {
        uint32_t idx  = L1I_INDEX(mem_addr);
        uint64_t tag  = L1I_TAG(mem_addr);
        uint8_t  off  = OFFSET(mem_addr);
        uint64_t base = BASE_ADDR(mem_addr);

        l1i_stats.accesses++;

        /* Hit */
        if (l1i[idx][0].valid && l1i[idx][0].tag == tag) {
            l1i[idx][0].data[off] = value;
            l1i[idx][0].modified  = 1;
            /* Coherence: L1D now has stale data for this block */
            coherence_invalidate_l1d(base);
            return;
        }

        /* Miss — write-allocate */
        l1i_stats.misses++;

        if (l1i[idx][0].valid && l1i[idx][0].modified)
            l2_writeback(L1I_BASE(l1i[idx][0].tag, idx), &l1i[idx][0]);

        /* Coherence: flush dirty L1D to L2 before fetching */
        coherence_flush_l1d(base);

        cache_line_t *l2l = l2_fetch(base);
        memcpy(l1i[idx][0].data, l2l->data, LINE_SIZE);
        l1i[idx][0].valid     = 1;
        l1i[idx][0].tag       = tag;
        l1i[idx][0].data[off] = value;
        l1i[idx][0].modified  = 1;

        /* Coherence: invalidate L1D (already flushed above, so just set valid=0) */
        coherence_invalidate_l1d(base);

    } else { /* DATA */
        uint32_t idx  = L1D_INDEX(mem_addr);
        uint64_t tag  = L1D_TAG(mem_addr);
        uint8_t  off  = OFFSET(mem_addr);
        uint64_t base = BASE_ADDR(mem_addr);

        l1d_stats.accesses++;

        /* Hit */
        for (int w = 0; w < HW11_L1_DATA_ASSOC; w++) {
            if (l1d[idx][w].valid && l1d[idx][w].tag == tag) {
                l1d[idx][w].data[off] = value;
                l1d[idx][w].modified  = 1;
                if (g_policy == LRU) l1d_touch(idx, w);
                /* Coherence: L1I now has stale data for this block */
                coherence_invalidate_l1i(base);
                return;
            }
        }

        /* Miss — write-allocate */
        l1d_stats.misses++;
        int v = l1d_pick_victim(idx);

        if (l1d[idx][v].valid && l1d[idx][v].modified)
            l2_writeback(L1D_BASE(l1d[idx][v].tag, idx), &l1d[idx][v]);

        /* Coherence: flush dirty L1I to L2 before fetching */
        coherence_invalidate_l1i(base);

        cache_line_t *l2l = l2_fetch(base);
        memcpy(l1d[idx][v].data, l2l->data, LINE_SIZE);
        l1d[idx][v].valid     = 1;
        l1d[idx][v].tag       = tag;
        l1d[idx][v].data[off] = value;
        l1d[idx][v].modified  = 1;
        if (g_policy == LRU) l1d_touch(idx, v);
    }
}

/* ── Stats ───────────────────────────────────────────────────────────── */

cache_stats_t get_l1_instr_stats() { return l1i_stats; }
cache_stats_t get_l1_data_stats()  { return l1d_stats; }
cache_stats_t get_l2_stats()       { return l2_stats;  }

/* ── Cache line lookup (no access counted) ───────────────────────────── */

cache_line_t *get_l1_instr_cache_line(uint64_t mem_addr) {
    uint32_t idx = L1I_INDEX(mem_addr);
    if (l1i[idx][0].valid && l1i[idx][0].tag == L1I_TAG(mem_addr))
        return &l1i[idx][0];
    return NULL;
}

cache_line_t *get_l1_data_cache_line(uint64_t mem_addr) {
    uint32_t idx = L1D_INDEX(mem_addr);
    uint64_t tag = L1D_TAG(mem_addr);
    for (int w = 0; w < HW11_L1_DATA_ASSOC; w++)
        if (l1d[idx][w].valid && l1d[idx][w].tag == tag)
            return &l1d[idx][w];
    return NULL;
}

cache_line_t *get_l2_cache_line(uint64_t mem_addr) {
    uint32_t idx = L2_INDEX(mem_addr);
    uint64_t tag = L2_TAG(mem_addr);
    for (int w = 0; w < HW11_L2_ASSOC; w++)
        if (l2c[idx][w].valid && l2c[idx][w].tag == tag)
            return &l2c[idx][w];
    return NULL;
}
