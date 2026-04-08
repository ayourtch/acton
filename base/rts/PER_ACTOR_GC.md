# Per-Actor Garbage Collector

This document describes the per-actor garbage collector implemented in
`actor_gc.{h,c}` and integrated through `common.c` and `rts.c`. The
implementation runs alongside Boehm GC: arena allocation is the fast path,
Boehm is the fallback and root for non-arena objects.

Status: experimental, on the `ayourtch-hack` branch. Tracks issue
[#2710](https://github.com/actonlang/acton/issues/2710).

## Motivation

Acton's runtime currently uses [Boehm GC](https://www.hboehm.info/gc/) as a
single global garbage collector. This has well-known drawbacks for an actor
language:

1. **Stop-the-world pauses.** Every collection scans the entire process heap
   and pauses every actor on every worker thread.
2. **Penalty for "mice" actors.** A workload with one large "elephant" actor
   and many lightweight "mice" actors pays elephant-sized GC cost on every
   mouse, because the elephant's heap dominates collection time.
3. **No locality.** A mouse touching a few KB of working set still triggers
   global mark/sweep over hundreds of MB of unrelated state.

The goal of this work is to give each actor its own collectable heap that can
be marked and swept independently of every other actor — keeping mice cheap
even when elephants are present.

## Approach

We allocate a single large `mmap` region (default 8 GB virtual, lazily backed
by physical pages) at startup, and carve per-actor *arenas* out of it. Each
arena gets its own thread-local bump allocation buffer (TLAB), its own object
list, and its own collection cycle.

Boehm is not removed. It still owns:
- Actor structs (`$Actor` objects, allocated via `GC_malloc`)
- Messages (`B_Msg` objects)
- A handful of root structures and external library state

Arena allocation routes through `acton_malloc` / `acton_malloc_atomic` /
`acton_malloc_leaf`. When an actor is currently executing, allocations go to
its arena. Otherwise (initialization, library code, idle threads) they fall
back to Boehm. The fallback is what makes the design incremental: we only
need to be correct on the arena fast path; everything else continues to work
exactly as before.

## Architecture

### Memory layout

```
[ region_base ────────────── region_end ]   (single mmap, 8 GB virtual)
        │
        ├── TLAB chunk (256 KB) for actor A
        │      ├── obj header │ payload │
        │      ├── obj header │ payload │
        │      └── ...
        ├── TLAB chunk (256 KB) for actor B
        ├── TLAB chunk (256 KB) for actor C
        └── ...
```

A global atomic bump pointer hands out 256 KB *chunks* to actors. Within a
chunk, the actor bumps lock-free with no atomics — actors are
single-threaded during message processing, so the per-actor bump is safe.

This is essentially the JVM's TLAB pattern: amortize the cost of one global
CAS over thousands of allocations.

### Object header

Every arena allocation is prefixed with a 24-byte header
(`actor_gc_obj_t`):

```c
struct actor_gc_obj {
    actor_gc_obj_t *next;       // linked list (live objects or free list)
    actor_gc_arena_t *owner;    // which arena owns this object
    uint32_t size;              // payload size, excluding header
    uint16_t flags;             // MARK | LEAF | VISITED
    uint16_t ext_refcount;      // cross-actor reference count (atomic)
};
```

The header is invariant under arena GC. When sweeping, we link the block
into the free list and write a sentinel for corruption detection. The header
is rewritten when the block is reused.

### Per-arena state

```c
struct actor_gc_arena {
    actor_gc_obj_t *objects;        // live objects (singly linked)
    actor_gc_obj_t *free_list;      // freed blocks available for reuse
    actor_gc_obj_t *limbo;          // swept last cycle, await promotion
    char *local_bump;               // TLAB current pointer
    char *local_bump_end;           // TLAB chunk end
    void **boehm_pin_set;           // Boehm objects to keep alive
    void **foreign_refs;            // arena pointers we reference in OTHER arenas
    /* ... counters, index, threshold ... */
};
```

## Allocation path

`acton_malloc(size)` → `actor_gc_alloc(arena, size)` → `arena_alloc_internal`:

1. Try free list (best-fit by size). Verify sentinel before reuse.
2. If TLAB chunk has room, bump within it (no atomics).
3. Otherwise, CAS a fresh 256 KB chunk from the global region and bump
   inside it.
4. Initialize the header, link into the objects list, return the payload.

This path is intentionally minimal. Most allocations take only a few
instructions and do not touch shared cache lines.

## Collection

Collection is triggered at actor message boundaries (`$RDONE` / `$RFAIL`,
before `ENQ_ready`) when `total_bytes > collect_threshold`. The actor is
quiescent at this point — no other thread is running its handler.

### Phase 0: limbo promotion

Objects swept in the *previous* cycle are promoted from limbo to the free
list **only if their `ext_refcount == 0`**. If another actor incremented the
refcount via `track_outgoing_refs` since the sweep, the object is rescued
back to the live list. This is the fix for the cross-actor sweep race.

### Phase 1: build sorted index

A flat array indexed by payload range, used for O(log n) pointer→object
lookup during marking. The objects list is in descending allocation order
(LIFO), so we just copy and reverse — O(n), no qsort.

### Phase 2: BFS mark from roots

Roots are the actor's own struct (scanned for arena pointers).

We trace through arena objects, marking everything reachable. We do **not**
trace through Boehm objects: profiling shows that scanning Boehm objects to
find arena-pointers-on-the-other-side adds 200K+ object visits per
collection while finding 0–1 additional arena pointers. The Boehm objects
discovered during root scanning are still recorded in the arena's
`boehm_pin_set` (a `GC_malloc`'d array that Boehm sees as a root) so they
stay alive across the actor's next message handler.

`GC_disable()` is held over the entire BFS to keep Boehm from collecting
anything we touch via `GC_base()` while we're traversing. This is correct
but tightens the no-GC window — the duration is microseconds because we no
longer trace through Boehm objects.

### Phase 3: sweep

Walk the objects list. For each object that is unmarked AND has
`ext_refcount == 0`, unlink it and put it on `limbo`. (Limbo, not free
list, to give one collection cycle of grace for in-flight cross-actor
references.) Marked or externally-referenced objects stay on the live list.

## Cross-actor references

When actor A sends a message to actor B, the message contains a continuation
which may reference arena objects in A's arena. Without protection, A's GC
would sweep them while they are still in flight.

`track_outgoing_refs` runs on the sender's thread before the message is
enqueued. It walks the continuation, finds every arena pointer, and atomically
increments `ext_refcount` on each one. The sweep phase respects
`ext_refcount`. When B finishes processing the message, the foreign refs are
diffed against the previous set and the unused ones are decremented.

The recursive scan (`actor_gc_track_refs_recursive`) uses `AGC_FLAG_VISITED`
to avoid cycles. The flag is cleared after the batch via
`actor_gc_clear_visited`.

## Critical invariants

1. **Arena payload alignment.** Real arena payload addresses always satisfy
   `(addr & 0xF) == 0x8`. Allocations are 16-byte aligned and the header is
   24 bytes, so payloads are at offset `(16 N) + 24 = (16 M) + 8`.
   `actor_gc_is_arena_ptr` enforces this. Without the alignment check,
   conservative scanning of message contents can pick up integers that
   happen to fall in the arena address range and dereference them as object
   headers, crashing on misaligned access.

2. **Boehm pointers must reach Boehm via `acton_realloc`.** The bigint code
   allocates limbs via `acton_malloc_atomic` (which routes to the arena),
   then later passes them to `bsdnt_realloc`. We pass `acton_realloc` (not
   raw `GC_realloc`) to `bsdnt_replace_allocator` and `xmlMemSetup` so that
   arena pointers are detected and copied to Boehm before realloc.

3. **Free list rescue.** Limbo objects must be checked for `ext_refcount`
   before promotion to the free list. A reference may have appeared between
   sweep and promotion (window is at least one collection cycle, usually
   much longer).

4. **Quiescent collection.** Collection only runs at `$RDONE` / `$RFAIL`,
   never during message handling. Other threads' allocations into other
   arenas continue concurrently — they cannot touch this arena's state.

## Configuration

Environment variables:

| Variable | Effect |
|----------|--------|
| `ACTON_NO_ARENA=1` | Disable arena allocation entirely (Boehm-only). For A/B benchmarking. |
| `ACTON_GC_DEBUG=1` | Print region map at startup and per-collection profiling lines on stderr. |
| `ACTON_GC_ROOTS=1` | Enable legacy Boehm root scanning (default off, replaced by pin sets). |

Compile-time constants in `actor_gc.c`:

- `INITIAL_THRESHOLD` — first collection trigger (default 4 MB)
- `THRESHOLD_GROWTH` — grow factor when reclaim ratio is low (default 2x)
- `MIN_RECLAIM_RATIO` — minimum free ratio before growing threshold (0.25)
- `TLAB_CHUNK_SIZE` — per-arena bump chunk size (default 256 KB)
- `DEFAULT_REGION_SIZE` — virtual mmap size (default 8 GB)

## Benchmark

`/tmp/test_gc_bench/gc_stress` is an allocation-dominated benchmark: 50
churner actors, each retaining ~2 MB of live data and rapidly creating and
discarding garbage via `list(range(5000))` calls. Total live heap is ~100
MB across all actors, which forces Boehm to collect frequently.

Results on Apple M1 (gc_stress v2, 50 actors × 100 rounds):

| Metric | Arena | Boehm-only | Delta |
|--------|-------|-----------|-------|
| Wall time | 229.5 s | 330.8 s | **-30.6%** |
| Per-actor median | 42.9 s | 57.0 s | -24.7% |
| Per-actor mean | 38.7 s | 50.9 s | -23.9% |
| Stddev | 7.4 s | 12.2 s | -39.3% |
| Spread (slowest − fastest) | 21.2 s | 46.5 s | **-54.4%** |

The spread reduction is the most interesting number: per-actor times are
dramatically more uniform under arena GC, which is the whole point of
giving each actor its own collector.

BFS mark phase time per collection: ~0.02 ms (after skipping Boehm
worklist), down from ~268 ms when we were tracing through Boehm objects.

## What's not done yet

- **Ownership transfer.** When actor A's GC finds an object that is
  unreachable locally but has `ext_refcount > 0`, it leaves the object in
  A's arena. A more aggressive design would transfer ownership to the
  referencing actor B so A's arena can shrink. Deferred — the object is
  in use either way, so this is a memory locality optimization, not a
  correctness or footprint issue.
- **Reducing remaining Boehm load.** Actor structs and `B_Msg` objects
  still go through Boehm. Routing more of those to per-arena allocation
  would shrink Boehm's heap and its collection cost further.
- **Workload coverage.** The current benchmark is allocation-dominated;
  the GC overhead is only ~30% of total runtime. To validate this work
  on more workloads, we need either GC-dominated benchmarks or
  application-level measurements.
- **Distributed runtime interaction.** The DDB and serialization paths
  have not been examined under arena GC; the cross-process semantics of
  arena pointers need more thought before this is enabled in distributed
  mode.

## Files

- `actor_gc.h` — public API and data structures
- `actor_gc.c` — arena management, allocation, mark/sweep, refcount tracking
- `common.c` — `acton_malloc` family, allocator routing, `acton_realloc`
- `rts.c` — `ACTOR_GC_COLLECT_NOW` macro, `track_outgoing_refs`, init

The build pulls all of these into one translation unit through
`__builtin__.ext.c → rts.c → actor_gc.c`.
