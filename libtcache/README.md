# Name:
Arthur Chen

# EID:
ac93957

# How to compile and run tests:

```bash
bash build.sh
./build/hw11 #runs tests
```

Or manually:

```bash
cmake -S . -B build
make -C build
./build/hw11
```

# Testing methodology and replacement policy results

Five workloads were designed to isolate specific behaviors of LRU vs. RANDOM replacement across the L1D (2-way, 256 sets) and L2 (4-way, 8192 sets) caches. The address stride constants (`L1D_STRIDE = 16384`, `L2_STRIDE = 524288`) were computed from the cache geometry so that accesses could be precisely targeted to conflict within a specific set at L1D, L2, or both simultaneously.

**Thrashing workloads** (cycling through associativity+1 addresses in the same set) showed the starkest difference: LRU suffers 100% miss rate because it deterministically evicts the next needed line, while RANDOM avoids this worst case by chance. In the L1D thrash test (3 addresses, 2-way), LRU missed all 60 accesses vs. RANDOM's 39. In the L2 thrash test (5 addresses, 4-way), LRU missed all 100 L2 accesses vs. RANDOM's 45. **High-locality workloads** (16 addresses each in distinct sets) showed no difference — both policies see only cold misses then hit every subsequent access. The **hot+cold stream** workload (one hot address interleaved with 20 unique cold addresses in the same 2-way set) reversed the advantage: LRU reliably evicts the cold line and keeps the hot one, yielding 21 misses vs. RANDOM's 32. The **direct-mapped L1I** workload confirmed that replacement policy is irrelevant with associativity 1 — both policies always miss identically. In summary, LRU is superior for workloads with temporal locality but pathologically bad for cyclic thrashing patterns, where RANDOM's non-determinism provides a significant advantage.
