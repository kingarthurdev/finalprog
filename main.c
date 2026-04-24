#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tcache.h"

/* tcache_backend.c exposes this for post-flush memory verification */
extern uint8_t memory[HW11_MEM_SIZE];

/* ── Test framework ──────────────────────────────────────────────────── */

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  PASS: %s\n", msg); g_pass++; } \
    else       { printf("  FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } \
} while (0)

static void section(const char *name) {
    printf("\n=== %s ===\n", name);
}

/* ── Address layout constants ─────────────────────────────────────────
   Cache geometry (MSB→LSB  tag | index | offset):
     L1I: 1-way, 512 sets  →  stride = 2^(6+9)  = 32768  (same set)
     L1D: 2-way, 256 sets  →  stride = 2^(6+8)  = 16384  (same set)
     L2:  4-way, 8192 sets →  stride = 2^(6+13) = 524288 (same set)

   Addresses using L2_STRIDE also share the same L1D set (since L1D index
   bits are a subset of L2 index bits), making them conflict at both levels.
   Addresses using L1D_STRIDE only conflict at L1D (different L2 sets).    */

#define LINE      64
#define L1D_STRIDE (1 << 14)    /* 16384  – same L1D set, different L2 sets */
#define L1I_STRIDE (1 << 15)    /* 32768  – same L1I set */
#define L2_STRIDE  (1 << 19)    /* 524288 – same L2 set AND same L1D set   */

/* Three addresses that thrash L1D (2-way) without conflicting in L2. */
static const uint64_t P[3] = { 0, L1D_STRIDE, 2 * L1D_STRIDE };

/* Five addresses that thrash both L1D (2-way) and L2 (4-way). */
static const uint64_t Q[6] = {
    0,
    1 * (uint64_t)L2_STRIDE,
    2 * (uint64_t)L2_STRIDE,
    3 * (uint64_t)L2_STRIDE,
    4 * (uint64_t)L2_STRIDE,
    5 * (uint64_t)L2_STRIDE,   /* Q[5] used only in write-back test */
};

/* ── 1. Basic read / write correctness ───────────────────────────────── */

static void test_basic_rw(void) {
    section("Basic read/write correctness");
    init_cache(LRU);

    /* Write a byte and read it back from the same cache line */
    write_cache(0x100, 0xAB, DATA);
    CHECK(read_cache(0x100, DATA) == 0xAB, "single byte roundtrip");

    /* Multiple bytes in the same line */
    write_cache(0x200,    0x11, DATA);
    write_cache(0x200+1,  0x22, DATA);
    write_cache(0x200+63, 0xFF, DATA);
    CHECK(read_cache(0x200,    DATA) == 0x11, "first byte in line");
    CHECK(read_cache(0x200+1,  DATA) == 0x22, "second byte in line");
    CHECK(read_cache(0x200+63, DATA) == 0xFF, "last byte in line");

    /* Instruction read from a zero-initialised memory region */
    CHECK(read_cache(0x0, INSTR) == 0x00, "instr read from zero-init memory");

    /* L1 cache-line lookup helpers */
    CHECK(get_l1_data_cache_line(0x100) != NULL, "L1D has line for 0x100");
    CHECK(get_l2_cache_line(0x100)      != NULL, "L2  has line for 0x100 (inclusive)");
    CHECK(get_l1_data_cache_line(0xDEAD0000) == NULL, "L1D correctly returns NULL");
    CHECK(get_l2_cache_line(0xDEAD0000)      == NULL, "L2  correctly returns NULL");
}

/* ── 2. Stats: hit/miss counting ─────────────────────────────────────── */

static void test_stats(void) {
    section("Hit/miss stat counting");
    init_cache(LRU);

    /* First access: miss; second: hit */
    read_cache(0x0, DATA);
    read_cache(0x0, DATA);
    cache_stats_t d = get_l1_data_stats();
    CHECK(d.accesses == 2, "2 L1D accesses after two reads");
    CHECK(d.misses   == 1, "1 L1D miss (cold), 1 hit");

    cache_stats_t l2 = get_l2_stats();
    CHECK(l2.accesses == 1, "1 L2 access for the L1D miss");
    CHECK(l2.misses   == 1, "1 L2 miss (cold)");

    /* Write miss (write-allocate) then read hit */
    write_cache(0x4000, 0x99, DATA);   /* different set from 0x0 */
    read_cache (0x4000, DATA);
    d = get_l1_data_stats();
    CHECK(d.accesses == 4, "4 total L1D accesses");
    CHECK(d.misses   == 2, "2 total L1D misses (one per cold line)");

    /* L1I stats are separate */
    read_cache(0x0, INSTR);
    cache_stats_t i = get_l1_instr_stats();
    CHECK(i.accesses == 1 && i.misses == 1, "L1I and L1D stats are independent");
}

/* ── 3. LRU ordering in 2-way L1D ────────────────────────────────────── */

static void test_lru_l1d(void) {
    section("LRU ordering in 2-way L1D");
    init_cache(LRU);

    /* Load P[0] and P[1] into the same L1D set (way 0 and way 1). */
    read_cache(P[0], DATA);   /* cold miss: loads into way 0, LRU = way 1 */
    read_cache(P[1], DATA);   /* cold miss: loads into way 1, LRU = way 0 */

    CHECK(get_l1_data_cache_line(P[0]) != NULL, "P[0] in L1D after load");
    CHECK(get_l1_data_cache_line(P[1]) != NULL, "P[1] in L1D after load");

    /* Re-access P[0] to make it MRU; P[1] becomes LRU. */
    read_cache(P[0], DATA);
    CHECK(get_l1_data_cache_line(P[0]) != NULL, "P[0] still in L1D (hit)");

    /* P[2] conflicts with P[0] and P[1] in L1D. LRU (P[1]) is evicted. */
    read_cache(P[2], DATA);
    CHECK(get_l1_data_cache_line(P[2]) != NULL, "P[2] loaded");
    CHECK(get_l1_data_cache_line(P[1]) == NULL, "P[1] evicted (was LRU)");
    CHECK(get_l1_data_cache_line(P[0]) != NULL, "P[0] kept (was MRU)");

    /* L2 must still hold all three (different L2 sets → no L2 conflict). */
    CHECK(get_l2_cache_line(P[0]) != NULL, "P[0] in L2");
    CHECK(get_l2_cache_line(P[1]) != NULL, "P[1] in L2 even after L1D eviction");
    CHECK(get_l2_cache_line(P[2]) != NULL, "P[2] in L2");
}

/* ── 4. Write-back chain: L1D → L2 → memory ─────────────────────────── */

static void test_writeback_chain(void) {
    section("Write-back chain (L1D → L2 → memory)");
    memset(memory, 0, HW11_MEM_SIZE);
    init_cache(LRU);

    /* Write a distinctive value to Q[0] (base address 0). */
    write_cache(Q[0], 0xDE, DATA);

    cache_line_t *cl = get_l1_data_cache_line(Q[0]);
    CHECK(cl != NULL && cl->data[0] == 0xDE && cl->modified, "Q[0] dirty in L1D");
    CHECK(memory[Q[0]] == 0x00, "memory NOT yet updated (write-back policy)");

    /* Load Q[1]-Q[5] sequentially.  Each L1D miss eventually forces write-backs
       up through L2 and finally to memory once L2 evicts Q[0].
       Sequence of events (all via LRU):
         Q[1]: cold miss → L1D[way1]=Q[1]
         Q[2]: cold miss → evict L1D LRU=Q[0] (dirty) → writeback to L2 (Q[0] dirty in L2)
         Q[3]: cold miss → evict L1D LRU=Q[1]
         Q[4]: cold miss → evict L1D LRU=Q[2]
         Q[5]: cold miss → L2 must evict LRU. After the Q[0] writeback touched Q[0] in
               L2 and later accesses pushed other lines, analysis shows Q[0] becomes the
               L2 LRU after loading Q[4]; loading Q[5] evicts Q[0] from L2 → memory write. */
    read_cache(Q[1], DATA);
    read_cache(Q[2], DATA);
    read_cache(Q[3], DATA);
    read_cache(Q[4], DATA);

    CHECK(get_l1_data_cache_line(Q[0]) == NULL, "Q[0] evicted from L1D");
    CHECK(get_l2_cache_line(Q[0]) != NULL,       "Q[0] still in L2 after L1D eviction");

    cache_line_t *l2l = get_l2_cache_line(Q[0]);
    CHECK(l2l != NULL && l2l->data[0] == 0xDE && l2l->modified,
          "Q[0] dirty with correct value in L2");
    CHECK(memory[Q[0]] == 0x00, "memory still clean (Q[0] not yet evicted from L2)");

    /* One more conflicting access evicts Q[0] from L2 → flushes to memory. */
    read_cache(Q[5], DATA);

    CHECK(get_l2_cache_line(Q[0]) == NULL, "Q[0] evicted from L2");
    CHECK(memory[Q[0]] == 0xDE,            "memory updated after L2 eviction");
    CHECK(read_cache(Q[0], DATA) == 0xDE,  "Q[0] re-read from memory via L2 load");
}

/* ── 5. L2 inclusivity: L2 eviction invalidates L1D ─────────────────── */

static void test_l2_inclusivity(void) {
    section("L2 inclusivity enforcement (RANDOM policy, seed=1)");

    /* Use a fixed seed so the test is deterministic.
       With srand(1), rand() produces the sequence:
         call 1 (L1D pick_victim for Q[2]): 1804289383 % 2 = 1  → evict way1
         call 2 (L1D pick_victim for Q[3]): 846930886  % 2 = 0  → evict way0
         call 3 (L1D pick_victim for Q[4]): 1681692777 % 2 = 1  → evict way1
         call 4 (L2  pick_victim for Q[4]): 1714636915 % 4 = 3  → evict L2 way3 = Q[3]
       After loading Q[0]-Q[3]: L1D holds Q[2](way0) and Q[3](way1) (from rand calls 1-2).
       Loading Q[4]:
         - L1D evicts way1=Q[3] (rand call 3), but BEFORE overwriting, l2_fetch is called
         - L2 randomly picks way3=Q[3] as victim (rand call 4)
         - Q[3] IS in L1D (way1), so l2_evict_line invalidates L1D way1
         - Q[4] is then stored in L1D way1 (the now-invalid slot)              */
    srand(1);
    init_cache(RANDOM);

    read_cache(Q[0], DATA);
    read_cache(Q[1], DATA);
    read_cache(Q[2], DATA);   /* rand()%2 = 1 → L1D evicts Q[1], loads Q[2] */
    read_cache(Q[3], DATA);   /* rand()%2 = 0 → L1D evicts Q[0], loads Q[3] */

    /* At this point L1D has Q[2](way0) and Q[3](way1), L2 has Q[0]-Q[3]. */
    CHECK(get_l1_data_cache_line(Q[2]) != NULL, "Q[2] in L1D before trigger");
    CHECK(get_l1_data_cache_line(Q[3]) != NULL, "Q[3] in L1D before trigger");

    read_cache(Q[4], DATA);   /* L2 rand evicts Q[3] which is still in L1D */

    CHECK(get_l1_data_cache_line(Q[3]) == NULL,
          "Q[3] invalidated in L1D when L2 evicted it (inclusivity)");
    CHECK(get_l2_cache_line(Q[3])      == NULL,
          "Q[3] no longer in L2");
    CHECK(get_l1_data_cache_line(Q[4]) != NULL, "Q[4] loaded into vacated L1D slot");

    /* Data integrity: Q[4] was zero-initialised, should read back as 0 */
    CHECK(read_cache(Q[4], DATA) == 0x00, "Q[4] data correct after inclusivity eviction");
}

/* ── 6. Policy comparison ────────────────────────────────────────────── */

typedef struct { cache_stats_t l1d, l2; } run_stats_t;

/* Run N rounds of cyclic access to `n_addrs` addresses under `policy`.
   All addresses map to the same L1D set (and optionally the same L2 set). */
static run_stats_t run_cyclic(replacement_policy_e policy,
                               const uint64_t *addrs, int n_addrs,
                               int rounds, mem_type_t mtype) {
    init_cache(policy);
    for (int r = 0; r < rounds; r++)
        for (int i = 0; i < n_addrs; i++)
            read_cache(addrs[i], mtype);
    return (run_stats_t){ get_l1_data_stats(), get_l2_stats() };
}

/* Run N distinct addresses once each, then repeat K times (locality). */
static run_stats_t run_locality(replacement_policy_e policy, int rounds) {
    static const uint64_t local_addrs[16] = {
        0x0000, 0x0A00, 0x1400, 0x1E00, 0x2800, 0x3200, 0x3C00, 0x4600,
        0x5000, 0x5A00, 0x6400, 0x6E00, 0x7800, 0x8200, 0x8C00, 0x9600,
    };
    init_cache(policy);
    for (int r = 0; r < rounds; r++)
        for (int i = 0; i < 16; i++)
            read_cache(local_addrs[i], DATA);
    return (run_stats_t){ get_l1_data_stats(), get_l2_stats() };
}

static const char *policy_name(replacement_policy_e p) {
    return p == LRU ? "LRU   " : "RANDOM";
}

static void print_stats(replacement_policy_e p, const char *workload,
                        run_stats_t s) {
    printf("  [%s] %-28s  L1D: %4lu acc / %4lu miss  |  "
           "L2: %4lu acc / %4lu miss\n",
           policy_name(p), workload,
           s.l1d.accesses, s.l1d.misses,
           s.l2.accesses,  s.l2.misses);
}

static void test_policy_comparison(void) {
    section("Replacement policy comparison: LRU vs RANDOM");

    int rounds = 20;
    srand(42);   /* fixed seed for reproducibility */

    puts("\n  --- Workload 1: L1D thrash (3 addrs, 2-way L1D, no L2 conflict) ---");
    puts("  [LRU is worst-case: every access after warmup is a miss]");
    {
        run_stats_t lru = run_cyclic(LRU,    P, 3, rounds, DATA);
        srand(42);
        run_stats_t rnd = run_cyclic(RANDOM, P, 3, rounds, DATA);
        print_stats(LRU,    "L1D thrash x20 rounds", lru);
        print_stats(RANDOM, "L1D thrash x20 rounds", rnd);
        printf("  => L1D miss delta (LRU - RANDOM): %ld\n",
               (long)lru.l1d.misses - (long)rnd.l1d.misses);
    }

    puts("\n  --- Workload 2: L2 thrash (5 addrs, 4-way L2 + 2-way L1D conflict) ---");
    puts("  [LRU is worst-case: cyclic access to assoc+1 lines]");
    {
        run_stats_t lru = run_cyclic(LRU,    Q, 5, rounds, DATA);
        srand(42);
        run_stats_t rnd = run_cyclic(RANDOM, Q, 5, rounds, DATA);
        print_stats(LRU,    "L2 thrash x20 rounds",  lru);
        print_stats(RANDOM, "L2 thrash x20 rounds",  rnd);
        printf("  => L1D miss delta (LRU - RANDOM): %ld\n",
               (long)lru.l1d.misses - (long)rnd.l1d.misses);
        printf("  => L2  miss delta (LRU - RANDOM): %ld\n",
               (long)lru.l2.misses  - (long)rnd.l2.misses);
    }

    puts("\n  --- Workload 3: High locality (16 addrs, all in distinct L1D sets) ---");
    puts("  [Both policies identical: only cold misses, then all hits]");
    {
        run_stats_t lru = run_locality(LRU,    rounds);
        srand(42);
        run_stats_t rnd = run_locality(RANDOM, rounds);
        print_stats(LRU,    "locality x20 rounds",   lru);
        print_stats(RANDOM, "locality x20 rounds",   rnd);
        CHECK(lru.l1d.misses == rnd.l1d.misses,
              "locality workload: both policies have equal L1D misses");
    }

    puts("\n  --- Workload 4: Hot addr + cold stream (LRU wins) ---");
    puts("  [Pattern: A C1 A C2 A C3 ... in same 2-way L1D set]");
    puts("  [LRU always evicts the cold Ci; RANDOM may evict hot A (~50% each time)]");
    {
        /* A = P[0], cold addresses = P[1] + k*LINE each in their own cache line
           but same L1D set index as P[0].  Use a large enough stride that they
           stay distinct lines yet map to the same L1D set.
           P[1] + k*LINE with k=0..19 all share L1D set because:
             (P[1] + k*64) >> 6 == (P[1] >> 6) + k, and adding k to the quotient
             changes only the tag, not the lower 8 index bits iff k < 256.       */
        uint64_t hot = P[0];  /* the frequently-used address */

        /* Each cold address uses L1D_STRIDE spacing so all 20 land in the
           same L1D set (only tag bits change), guaranteed to conflict with
           hot A.  They hit different L2 sets so L2 never evicts any of them. */

        /* LRU run */
        init_cache(LRU);
        read_cache(hot, DATA);                                   /* cold miss on A */
        for (int k = 0; k < 20; k++) {
            uint64_t cold = P[1] + (uint64_t)k * L1D_STRIDE;   /* same L1D set */
            read_cache(cold, DATA);                              /* cold miss */
            read_cache(hot,  DATA);                              /* LRU: always a hit */
        }
        cache_stats_t lru_hot = get_l1_data_stats();

        /* RANDOM run */
        srand(42);
        init_cache(RANDOM);
        read_cache(hot, DATA);
        for (int k = 0; k < 20; k++) {
            uint64_t cold = P[1] + (uint64_t)k * L1D_STRIDE;
            read_cache(cold, DATA);
            read_cache(hot,  DATA);   /* RANDOM: ~50% chance of evicting A */
        }
        cache_stats_t rnd_hot = get_l1_data_stats();

        printf("  [LRU   ] hot+cold stream: %lu acc / %lu miss\n",
               lru_hot.accesses, lru_hot.misses);
        printf("  [RANDOM] hot+cold stream: %lu acc / %lu miss\n",
               rnd_hot.accesses, rnd_hot.misses);
        printf("  => RANDOM extra misses vs LRU: %ld  (negative = RANDOM lost)\n",
               (long)lru_hot.misses - (long)rnd_hot.misses);
        CHECK(lru_hot.misses <= rnd_hot.misses,
              "LRU has no more misses than RANDOM on hot+cold workload");
    }

    puts("\n  --- Workload 5: L1I thrash (3 addrs, direct-mapped L1I) ---");
    puts("  [Direct-mapped: LRU and RANDOM both always miss after warmup]");
    {
        uint64_t instr_addrs[3] = { 0, L1I_STRIDE, 2 * L1I_STRIDE };
        (void)run_cyclic(LRU,    instr_addrs, 3, rounds, INSTR);
        srand(42);
        (void)run_cyclic(RANDOM, instr_addrs, 3, rounds, INSTR);
        /* For L1I stats we need get_l1_instr_stats; re-run and collect */
        init_cache(LRU);
        for (int r = 0; r < rounds; r++)
            for (int i = 0; i < 3; i++)
                read_cache(instr_addrs[i], INSTR);
        cache_stats_t li_lru = get_l1_instr_stats();

        init_cache(RANDOM); srand(42);
        for (int r = 0; r < rounds; r++)
            for (int i = 0; i < 3; i++)
                read_cache(instr_addrs[i], INSTR);
        cache_stats_t li_rnd = get_l1_instr_stats();

        printf("  [LRU   ] L1I thrash x20: %lu acc / %lu miss\n",
               li_lru.accesses, li_lru.misses);
        printf("  [RANDOM] L1I thrash x20: %lu acc / %lu miss\n",
               li_rnd.accesses, li_rnd.misses);
        CHECK(li_lru.misses == li_rnd.misses,
              "L1I direct-mapped: policy makes no difference");
    }
}

/* ── main ────────────────────────────────────────────────────────────── */

int main(void) {
    test_basic_rw();
    test_stats();
    test_lru_l1d();
    test_writeback_chain();
    test_l2_inclusivity();
    test_policy_comparison();

    printf("\n%s: %d passed, %d failed\n",
           g_fail == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED",
           g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
