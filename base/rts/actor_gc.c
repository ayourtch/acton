// Per-actor garbage collector with ownership tracking
//
// Uses a single large mmap'd region for all arena allocations.
// Free-list + bump allocation within the region.
// Per-actor mark-sweep collection respects ext_refcount for cross-actor refs.
//
// NOTE: This file is #include'd from rts.c (not compiled separately).

#include <sys/mman.h>
#include <time.h>
#include "actor_gc.h"

// --- Global arena region ---

static char *region_base = NULL;
static size_t region_size = 0;
static char *region_end = NULL;
static volatile char *bump_ptr = NULL;

#define DEFAULT_REGION_SIZE (8UL * 1024 * 1024 * 1024)

#define INITIAL_THRESHOLD (4 * 1024 * 1024)
#define THRESHOLD_GROWTH 2
#define MIN_RECLAIM_RATIO 0.25

#define ARENA_ALIGN 16
#define ALIGN_UP(x, a) (((x) + (a) - 1) & ~((a) - 1))

// Boehm root scanning (legacy — replaced by per-arena pin sets)
#define ROOT_CHUNK_SIZE (1024 * 1024)
static volatile char *roots_registered_up_to = NULL;
static int actor_gc_no_roots = 1;  // pin sets replace root scanning

// Debug output flag (set by ACTON_GC_DEBUG=1)
static int actor_gc_debug = 0;

// Stubs for diagnostic APIs (callers in rts.c still reference these)
void actor_gc_set_val_roots(actor_gc_root_t *roots, int n) { (void)roots; (void)n; }
void actor_gc_print_sweep_ring(void) {}

// --- Thread-local current arena ---

static __thread actor_gc_arena_t *current_arena = NULL;

void actor_gc_set_current(actor_gc_arena_t *arena) {
    current_arena = arena;
}

actor_gc_arena_t *actor_gc_get_current(void) {
    return current_arena;
}

// --- Actor-to-arena hash table ---

#define ARENA_HT_SIZE 4096

typedef struct {
    void *actor_ptr;
    actor_gc_arena_t *arena;
} arena_ht_entry_t;

static arena_ht_entry_t arena_ht[ARENA_HT_SIZE];
static volatile int arena_ht_lock = 0;

static void ht_lock(void) {
    while (__sync_lock_test_and_set(&arena_ht_lock, 1)) {}
}
static void ht_unlock(void) {
    __sync_lock_release(&arena_ht_lock);
}

static unsigned ht_hash(void *ptr) {
    uintptr_t v = (uintptr_t)ptr;
    v = (v >> 4) * 2654435761U;
    return (unsigned)(v & (ARENA_HT_SIZE - 1));
}

actor_gc_arena_t *actor_gc_register(void *actor_ptr) {
    if (!region_base) return NULL;
    actor_gc_arena_t *arena = (actor_gc_arena_t *)calloc(1, sizeof(actor_gc_arena_t));
    if (!arena) return NULL;
    actor_gc_arena_init(arena);

    ht_lock();
    unsigned idx = ht_hash(actor_ptr);
    for (unsigned i = 0; i < ARENA_HT_SIZE; i++) {
        unsigned slot = (idx + i) & (ARENA_HT_SIZE - 1);
        if (arena_ht[slot].actor_ptr == NULL) {
            arena_ht[slot].actor_ptr = actor_ptr;
            arena_ht[slot].arena = arena;
            ht_unlock();
            return arena;
        }
    }
    ht_unlock();
    free(arena);
    return NULL;
}

actor_gc_arena_t *actor_gc_lookup(void *actor_ptr) {
    if (!region_base) return NULL;
    unsigned idx = ht_hash(actor_ptr);
    for (unsigned i = 0; i < ARENA_HT_SIZE; i++) {
        unsigned slot = (idx + i) & (ARENA_HT_SIZE - 1);
        void *key = arena_ht[slot].actor_ptr;
        if (key == actor_ptr) return arena_ht[slot].arena;
        if (key == NULL) return NULL;
    }
    return NULL;
}

void actor_gc_unregister(void *actor_ptr) {
    ht_lock();
    unsigned idx = ht_hash(actor_ptr);
    for (unsigned i = 0; i < ARENA_HT_SIZE; i++) {
        unsigned slot = (idx + i) & (ARENA_HT_SIZE - 1);
        if (arena_ht[slot].actor_ptr == actor_ptr) {
            actor_gc_arena_destroy(arena_ht[slot].arena);
            free(arena_ht[slot].arena);
            arena_ht[slot].actor_ptr = NULL;
            arena_ht[slot].arena = NULL;
            ht_unlock();
            return;
        }
        if (arena_ht[slot].actor_ptr == NULL) break;
    }
    ht_unlock();
}

// --- Global region init ---

int actor_gc_global_init(size_t size) {
    if (size == 0) size = DEFAULT_REGION_SIZE;
#ifdef __linux__
    region_base = (char *)mmap(NULL, size, PROT_READ | PROT_WRITE,
                               MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
#else
    region_base = (char *)mmap(NULL, size, PROT_READ | PROT_WRITE,
                               MAP_PRIVATE | MAP_ANON, -1, 0);
#endif
    if (region_base == MAP_FAILED) {
        region_base = NULL;
        return -1;
    }
    region_size = size;
    region_end = region_base + size;
    bump_ptr = region_base;
    roots_registered_up_to = region_base;

    const char *no_roots_env = getenv("ACTON_GC_NO_ROOTS");
    if (no_roots_env && no_roots_env[0] == '1') {
        actor_gc_no_roots = 1;
    }
    const char *roots_env = getenv("ACTON_GC_ROOTS");
    if (roots_env && roots_env[0] == '1') {
        actor_gc_no_roots = 0;
    }

    // Only print banner when ACTON_GC_DEBUG=1 to avoid breaking tests that check stderr
    const char *debug_env = getenv("ACTON_GC_DEBUG");
    if (debug_env && debug_env[0] == '1') {
        actor_gc_debug = 1;
        fprintf(stderr, "ACTOR_GC: region %p - %p (%zu MB)%s\n",
                region_base, region_end, size / (1024*1024),
                actor_gc_no_roots ? " [pin set mode]" : " [root scanning ON]");
    }
    return 0;
}

bool actor_gc_is_arena_ptr(void *ptr) {
    return region_base && ptr >= (void *)region_base && ptr < (void *)region_end;
}

// --- Arena lifecycle ---

void actor_gc_arena_init(actor_gc_arena_t *arena) {
    arena->objects = NULL;
    arena->free_list = NULL;
    arena->limbo = NULL;
    arena->total_bytes = 0;
    arena->num_objects = 0;
    arena->free_bytes = 0;
    arena->limbo_bytes = 0;
    arena->collect_threshold = INITIAL_THRESHOLD;
    arena->collections = 0;
    arena->bytes_freed = 0;
    arena->index = NULL;
    arena->index_count = 0;
    arena->index_cap = 0;
    arena->boehm_pin_set = NULL;
    arena->pin_set_count = 0;
    arena->pin_set_cap = 0;
    arena->local_bump = NULL;
    arena->local_bump_end = NULL;
    arena->foreign_refs = NULL;
    arena->foreign_refs_count = 0;
    arena->foreign_refs_cap = 0;
}

void actor_gc_arena_destroy(actor_gc_arena_t *arena) {
    arena->objects = NULL;
    arena->free_list = NULL;
    arena->limbo = NULL;
    arena->total_bytes = 0;
    arena->num_objects = 0;
    arena->free_bytes = 0;
    if (arena->index) {
        free(arena->index);
        arena->index = NULL;
    }
    arena->index_count = 0;
    arena->index_cap = 0;
    // boehm_pin_set is GC_malloc'd — just NULL the pointer, Boehm will collect it
    arena->boehm_pin_set = NULL;
    arena->pin_set_count = 0;
    arena->pin_set_cap = 0;
    if (arena->foreign_refs) {
        free(arena->foreign_refs);
        arena->foreign_refs = NULL;
    }
    arena->foreign_refs_count = 0;
    arena->foreign_refs_cap = 0;
}

// --- Boehm root scanning ---
// Root scanning is replaced by per-arena pin sets (built during BFS mark phase).
// The pin set is a GC_malloc'd array of Boehm pointers discovered during marking,
// so Boehm treats it as containing valid references and keeps those objects alive.

static void ensure_roots_registered(char *up_to) {
    (void)up_to;
    // No-op: replaced by per-arena boehm_pin_set built during collection.
}

// --- Bump allocator ---

static void *region_bump_alloc(size_t total_size) {
    total_size = ALIGN_UP(total_size, ARENA_ALIGN);
    char *old;
    char *new_ptr;
    do {
        old = (char *)bump_ptr;
        new_ptr = old + total_size;
        if (new_ptr > region_end) return NULL;
    } while (!__sync_bool_compare_and_swap(&bump_ptr, old, new_ptr));
    ensure_roots_registered(new_ptr);
    return old;
}

// --- Allocation: free list + bump ---

// Sentinel written to payload of free-list blocks for corruption detection.
// If this value is not seen when reusing, something wrote to a freed block.
#define AGC_FREELIST_SENTINEL 0xDEADB00FDEADBA11ULL


// Per-arena TLAB chunk size: each arena gets a private chunk from the global
// region and bumps within it without any atomics. One global CAS per chunk.
#define TLAB_CHUNK_SIZE (256 * 1024)  // 256KB per chunk

static inline void *arena_alloc_internal(actor_gc_arena_t *arena, size_t size, uint16_t flags) {
    if (!region_base) return NULL;

    size_t aligned_size = ALIGN_UP(size, ARENA_ALIGN);
    size_t total = sizeof(actor_gc_obj_t) + aligned_size;

    // Try free list first (best-fit from previously collected objects).
    // Objects go sweep→limbo→free_list with ext_refcount check at promotion.
    {
        actor_gc_obj_t **prev_free = &arena->free_list;
        actor_gc_obj_t *cur = arena->free_list;
        while (cur) {
            if (cur->size >= aligned_size) {
                // Check sentinel for corruption detection
                if (cur->size >= sizeof(uint64_t)) {
                    uint64_t sentinel = *(uint64_t *)actor_gc_payload(cur);
                    if (sentinel != AGC_FREELIST_SENTINEL) {
                        // Corrupted block — skip it
                        prev_free = &cur->next;
                        cur = cur->next;
                        continue;
                    }
                }
                // Remove from free list
                *prev_free = cur->next;
                arena->free_bytes -= cur->size;
                // Re-initialize and add to objects list
                cur->flags = flags;
                cur->ext_refcount = 0;
                cur->next = arena->objects;
                arena->objects = cur;
                arena->num_objects++;
                arena->total_bytes += cur->size;
                // Zero the payload
                memset(actor_gc_payload(cur), 0, cur->size);
                return actor_gc_payload(cur);
            }
            prev_free = &cur->next;
            cur = cur->next;
        }
    }

    // Per-arena bump (TLAB): try local chunk first, no atomics needed.
    // Actors are single-threaded during execution, so this is safe.
    void *block = NULL;
    if (arena->local_bump && arena->local_bump + total <= arena->local_bump_end) {
        block = arena->local_bump;
        arena->local_bump += ALIGN_UP(total, ARENA_ALIGN);
    } else {
        // Local chunk exhausted or not yet allocated: grab a new one.
        // Use a chunk at least as big as the requested allocation.
        size_t chunk_sz = total > TLAB_CHUNK_SIZE ? ALIGN_UP(total, ARENA_ALIGN) : TLAB_CHUNK_SIZE;
        char *chunk = (char *)region_bump_alloc(chunk_sz);
        if (!chunk) return NULL;
        block = chunk;
        arena->local_bump = chunk + ALIGN_UP(total, ARENA_ALIGN);
        arena->local_bump_end = chunk + chunk_sz;
    }

    actor_gc_obj_t *obj = (actor_gc_obj_t *)block;
    obj->next = NULL;
    obj->size = (uint32_t)aligned_size;
    obj->owner = arena;
    obj->flags = flags;
    obj->ext_refcount = 0;

    obj->next = arena->objects;
    arena->objects = obj;
    arena->total_bytes += aligned_size;
    arena->num_objects++;

    return actor_gc_payload(obj);
}

void *actor_gc_alloc(actor_gc_arena_t *arena, size_t size) {
    return arena_alloc_internal(arena, size, 0);
}

// Allocate a leaf object (no outgoing GC pointers — skip scanning in mark phase).
void *actor_gc_alloc_leaf(actor_gc_arena_t *arena, size_t size) {
    return arena_alloc_internal(arena, size, AGC_FLAG_LEAF);
}

size_t actor_gc_obj_size(void *ptr) {
    return actor_gc_header(ptr)->size;
}

void *actor_gc_realloc(actor_gc_arena_t *arena, void *old_ptr, size_t new_size) {
    size_t old_size = actor_gc_obj_size(old_ptr);
    if (new_size <= old_size) return old_ptr;  // Shrink: no-op (keep existing block)
    void *new_ptr = actor_gc_alloc(arena, new_size);
    if (!new_ptr) return NULL;
    memcpy(new_ptr, old_ptr, old_size);
    // Old block stays in object list, will be swept later
    return new_ptr;
}

// --- Sorted index for O(log n) pointer lookup ---

// Build sorted index of all objects in the arena. Called once per collection.
// Objects list is in reverse allocation order (newest first). Since bump
// allocation produces monotonically increasing addresses, the list is in
// descending address order. We populate the index and reverse it — O(n)
// instead of O(n log n) qsort.
static void build_object_index(actor_gc_arena_t *arena) {
    // Ensure capacity
    if (arena->num_objects > arena->index_cap) {
        size_t new_cap = arena->num_objects + (arena->num_objects >> 1);  // 1.5x
        if (new_cap < 64) new_cap = 64;
        actor_gc_index_entry_t *new_idx = (actor_gc_index_entry_t *)realloc(
            arena->index, new_cap * sizeof(actor_gc_index_entry_t));
        if (!new_idx) return;  // OOM — collection will skip marking
        arena->index = new_idx;
        arena->index_cap = new_cap;
    }

    // Populate (descending order from objects list)
    size_t i = 0;
    actor_gc_obj_t *obj = arena->objects;
    while (obj && i < arena->index_cap) {
        void *payload = actor_gc_payload(obj);
        arena->index[i].payload_start = (uintptr_t)payload;
        arena->index[i].payload_end = (uintptr_t)payload + obj->size;
        arena->index[i].obj = obj;
        i++;
        obj = obj->next;
    }
    arena->index_count = i;

    // Reverse to ascending order — O(n) swap
    for (size_t lo = 0, hi = i; lo + 1 < hi; lo++, hi--) {
        actor_gc_index_entry_t tmp = arena->index[lo];
        arena->index[lo] = arena->index[hi - 1];
        arena->index[hi - 1] = tmp;
    }
}

// Binary search: find the object whose payload range contains `ptr`.
// Returns the object header, or NULL if not found.
static inline actor_gc_obj_t *index_lookup(actor_gc_arena_t *arena, uintptr_t ptr) {
    size_t lo = 0, hi = arena->index_count;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if (arena->index[mid].payload_start <= ptr) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    // lo is now the first entry with payload_start > ptr.
    // The candidate is lo-1 (the last entry with payload_start <= ptr).
    if (lo == 0) return NULL;
    actor_gc_index_entry_t *e = &arena->index[lo - 1];
    if (ptr >= e->payload_start && ptr < e->payload_end) {
        return e->obj;
    }
    return NULL;
}

// --- Conservative mark phase ---
//
// Mark phase must find ALL arena pointers reachable from GC roots, including
// those behind mixed arena→Boehm→arena and Boehm→arena chains. The typical
// problematic chain is:
//
//   actor struct (Boehm)
//     → elephants: B_list (ARENA - allocated via acton_malloc)
//       → data: $WORD* (ARENA - data array also via acton_malloc)
//         → Elephant ptr (BOEHM - actor struct is GC_malloc)
//           → report: $action (ARENA - callback closure in owner's arena)
//
// We use a unified worklist approach: both arena objects and Boehm objects
// are queued for scanning. For each range scanned:
//   - arena pointers: mark them, push payload to arena worklist
//   - Boehm pointers: push base to Boehm worklist (visited tracking via hash table)
//
// This handles all transitions: root→arena, root→Boehm, arena→Boehm, Boehm→arena.

// Open-addressing hash table for visited Boehm base pointers.
// Returns:  1 = newly inserted (not yet visited)
//           0 = already in table (duplicate)
//          -1 = table completely full (overflow — caller must grow and retry)
static inline int boehm_ht_insert(void **ht, int ht_cap, void *base) {
    uintptr_t h = ((uintptr_t)base >> 3) * 2654435761ULL;
    unsigned slot = (unsigned)(h & (unsigned)(ht_cap - 1));
    for (unsigned i = 0; i < (unsigned)ht_cap; i++) {
        unsigned s = (slot + i) & (unsigned)(ht_cap - 1);
        if (ht[s] == NULL) { ht[s] = base; return 1; }
        if (ht[s] == base) { return 0; }
    }
    return -1;  // table full: overflow
}

// Rehash boehm_ht into a table of double capacity.
// Returns true on success, false on OOM (old table left unchanged).
static bool boehm_ht_grow(void ***ht_ptr, int *cap_ptr) {
    int old_cap = *cap_ptr;
    int new_cap = old_cap * 2;
    void **new_ht = (void **)calloc(new_cap, sizeof(void *));
    if (!new_ht) return false;
    void **old_ht = *ht_ptr;
    for (int i = 0; i < old_cap; i++) {
        if (!old_ht[i]) continue;
        void *base = old_ht[i];
        uintptr_t h = ((uintptr_t)base >> 3) * 2654435761ULL;
        unsigned slot = (unsigned)(h & (unsigned)(new_cap - 1));
        for (unsigned j = 0; j < (unsigned)new_cap; j++) {
            unsigned s = (slot + j) & (unsigned)(new_cap - 1);
            if (new_ht[s] == NULL) { new_ht[s] = base; break; }
        }
    }
    free(old_ht);
    *ht_ptr = new_ht;
    *cap_ptr = new_cap;
    return true;
}

// Unified mark phase: scan range for arena ptrs (mark+queue) and Boehm ptrs (queue).
// arena_wl/boehm_wl are dynamic worklists passed in/out.
// boehm_ht is passed as void*** so it can be grown in place if it gets too full.
// Returns false on OOM (caller should fall back to simple scan).
static bool mark_range(actor_gc_arena_t *arena, void *start, size_t size,
                        void ***arena_wl, int *arena_wl_n, int *arena_wl_cap,
                        void ***boehm_wl, int *boehm_wl_n, int *boehm_wl_cap,
                        void ***boehm_ht, int *boehm_ht_cap,
                        int *boehm_ht_overflow) {
    uintptr_t addr = (uintptr_t)start;
    uintptr_t end = addr + size;
    addr = (addr + sizeof(void *) - 1) & ~(sizeof(void *) - 1);

    // Cache Boehm heap bounds for fast pre-filtering (avoids expensive GC_base calls).
    // These are Boehm GC globals (declared in gc/gc_mark.h).
    extern void *GC_least_plausible_heap_addr;
    extern void *GC_greatest_plausible_heap_addr;
    uintptr_t boehm_lo = (uintptr_t)GC_least_plausible_heap_addr;
    uintptr_t boehm_hi = (uintptr_t)GC_greatest_plausible_heap_addr;

    while (addr + sizeof(void *) <= end) {
        uintptr_t candidate = *(uintptr_t *)addr;

        if (candidate >= (uintptr_t)region_base && candidate < (uintptr_t)region_end) {
            // Arena pointer: mark and queue payload for further scanning
            actor_gc_obj_t *obj = index_lookup(arena, candidate);
            if (obj && !(obj->flags & AGC_FLAG_MARK)) {
                obj->flags |= AGC_FLAG_MARK;
                if (!(obj->flags & AGC_FLAG_LEAF)) {
                    // Push obj ptr onto arena worklist (dequeue reads payload+size)
                    if (*arena_wl_n >= *arena_wl_cap) {
                        int nc = *arena_wl_cap * 2;
                        void **nw = (void **)realloc(*arena_wl, nc * sizeof(void *));
                        if (!nw) return false;
                        *arena_wl = nw; *arena_wl_cap = nc;
                    }
                    (*arena_wl)[(*arena_wl_n)++] = (void *)obj;
                }
            }
        } else if (candidate >= boehm_lo && candidate < boehm_hi) {
            // Within Boehm heap bounds: worth calling GC_base
            void *gb = GC_base((void *)candidate);
            if (gb) {
                // Proactively grow hash table if >50% full
                if (*boehm_wl_n + 1 > *boehm_ht_cap / 2) {
                    if (!boehm_ht_grow(boehm_ht, boehm_ht_cap)) {
                        return false; // OOM
                    }
                }
                int ins = boehm_ht_insert(*boehm_ht, *boehm_ht_cap, gb);
                if (ins == 1) {
                    // Newly visited: push (base, size) pair to Boehm worklist.
                    // Capture GC_size now while the object is known-valid, to
                    // avoid calling GC_size later when it may have been collected.
                    size_t bsz = GC_size(gb);
                    if (*boehm_wl_n + 1 >= *boehm_wl_cap) {
                        int nc = *boehm_wl_cap * 2;
                        void **nw = (void **)realloc(*boehm_wl, nc * sizeof(void *));
                        if (!nw) return false;
                        *boehm_wl = nw; *boehm_wl_cap = nc;
                    }
                    (*boehm_wl)[(*boehm_wl_n)++] = gb;
                    (*boehm_wl)[(*boehm_wl_n)++] = (void *)bsz;
                } else if (ins == -1) {
                    (*boehm_ht_overflow)++;
                }
            }
        }
        addr += sizeof(void *);
    }
    return true;
}

// BFS mark phase stats (populated per collection, read by collect_full)
typedef struct {
    int arena_marked_from_roots;   // arena objects marked during root scanning
    int arena_marked_from_boehm;   // arena objects marked during Boehm worklist scanning
    int boehm_objects_visited;     // unique Boehm objects traversed
    double bfs_time_ms;            // wall time of entire BFS mark phase
} bfs_stats_t;

static void mark_from_roots_bfs(actor_gc_arena_t *arena,
                                  actor_gc_root_t *roots, int num_roots,
                                  bfs_stats_t *stats) {
    struct timespec bfs_start, bfs_end;
    clock_gettime(CLOCK_MONOTONIC, &bfs_start);

    memset(stats, 0, sizeof(*stats));

    // Prevent Boehm from collecting during BFS. Without this, Boehm can
    // collect objects between discovery (GC_base) and scanning, causing
    // stale memory reads that miss arena pointers → live objects swept → crash.
    GC_disable();

    // Boehm visited hash table — starts small, grows dynamically (see mark_range)
    int boehm_ht_cap = 512;
    void **boehm_ht = (void **)calloc(boehm_ht_cap, sizeof(void *));
    int boehm_ht_overflow = 0;  // count of OOM-overflow events (should stay 0)

    // Arena worklist: obj ptrs waiting to have their payloads scanned
    int arena_wl_cap = 512;
    void **arena_wl = (void **)malloc(arena_wl_cap * sizeof(void *));
    int arena_wl_head = 0, arena_wl_n = 0;

    // Boehm worklist: base ptrs waiting to be scanned
    int boehm_wl_cap = 512;
    void **boehm_wl = (void **)malloc(boehm_wl_cap * sizeof(void *));
    int boehm_wl_head = 0, boehm_wl_n = 0;

    if (!boehm_ht || !arena_wl || !boehm_wl) {
        // OOM fallback: simple arena-only scan
        for (int i = 0; i < num_roots; i++) {
            uintptr_t addr = (uintptr_t)roots[i].start;
            uintptr_t end = addr + roots[i].size;
            addr = (addr + sizeof(void*)-1) & ~(sizeof(void*)-1);
            while (addr + sizeof(void*) <= end) {
                uintptr_t c = *(uintptr_t *)addr;
                if (c >= (uintptr_t)region_base && c < (uintptr_t)region_end) {
                    actor_gc_obj_t *obj = index_lookup(arena, c);
                    if (obj && !(obj->flags & AGC_FLAG_MARK)) {
                        obj->flags |= AGC_FLAG_MARK;
                        // no further scanning in fallback
                    }
                }
                addr += sizeof(void*);
            }
        }
        goto cleanup;
    }

    // Seed from roots
    for (int i = 0; i < num_roots; i++) {
        if (!mark_range(arena, roots[i].start, roots[i].size,
                        &arena_wl, &arena_wl_n, &arena_wl_cap,
                        &boehm_wl, &boehm_wl_n, &boehm_wl_cap,
                        &boehm_ht, &boehm_ht_cap, &boehm_ht_overflow)) goto cleanup;
    }

    // Record how many arena objects were marked directly from roots
    stats->arena_marked_from_roots = arena_wl_n;

    // Process arena worklist only. Skip Boehm worklist: profiling shows
    // scanning Boehm objects finds <1 additional arena pointer while visiting
    // 200K+ Boehm objects (99% of collection time for ~0% of marks).
    // Boehm objects discovered during root/arena scanning are still pinned
    // via the pin set below, just not scanned for further arena pointers.
    while (arena_wl_head < arena_wl_n) {
        actor_gc_obj_t *obj = (actor_gc_obj_t *)arena_wl[arena_wl_head++];
        if (!mark_range(arena, actor_gc_payload(obj), obj->size,
                        &arena_wl, &arena_wl_n, &arena_wl_cap,
                        &boehm_wl, &boehm_wl_n, &boehm_wl_cap,
                        &boehm_ht, &boehm_ht_cap, &boehm_ht_overflow)) goto cleanup;
    }

    int boehm_unique = boehm_wl_n / 2;  // worklist stores (base, size) pairs

    if (boehm_ht_overflow > 0) {
        fprintf(stderr, "AGC BUG: boehm_ht overflow %d time(s) during BFS "
                "(final cap=%d, boehm_unique=%d) — objects may have been missed!\n",
                boehm_ht_overflow, boehm_ht_cap, boehm_unique);
    }

    // Build pin set: extract base pointers from (base, size) pairs.
    // Allocated via GC_malloc so Boehm sees the pointers and keeps the objects alive.
    if (boehm_unique > 0) {
        size_t need = (size_t)boehm_unique;
        if (need > arena->pin_set_cap) {
            size_t cap = arena->pin_set_cap ? arena->pin_set_cap : 64;
            while (cap < need) cap *= 2;
            arena->boehm_pin_set = (void **)GC_malloc(cap * sizeof(void *));
            arena->pin_set_cap = cap;
        }
        // Extract base pointers (every other entry)
        for (int i = 0; i < boehm_unique; i++) {
            arena->boehm_pin_set[i] = boehm_wl[i * 2];
        }
        arena->pin_set_count = need;
        // Zero out the rest so Boehm doesn't see stale pointers
        if (need < arena->pin_set_cap) {
            memset(arena->boehm_pin_set + need, 0,
                   (arena->pin_set_cap - need) * sizeof(void *));
        }
    } else {
        // No Boehm objects referenced — clear pin set
        arena->boehm_pin_set = NULL;
        arena->pin_set_count = 0;
        arena->pin_set_cap = 0;
    }

    // Total arena objects marked = all that ended up on the arena worklist
    // arena_marked_from_boehm = total - roots (those discovered via Boehm intermediaries)
    stats->arena_marked_from_boehm = arena_wl_n - stats->arena_marked_from_roots;
    stats->boehm_objects_visited = boehm_unique;

cleanup:
    GC_enable();
    free(boehm_ht);
    free(arena_wl);
    free(boehm_wl);

    clock_gettime(CLOCK_MONOTONIC, &bfs_end);
    stats->bfs_time_ms = (bfs_end.tv_sec - bfs_start.tv_sec) * 1000.0
                       + (bfs_end.tv_nsec - bfs_start.tv_nsec) / 1e6;
}

// Simple arena-only scan (used by arena→arena following within scan_range_mark)
static void scan_range_mark(actor_gc_arena_t *arena, void *start, size_t size) {
    uintptr_t addr = (uintptr_t)start;
    uintptr_t end = addr + size;
    addr = (addr + sizeof(void *) - 1) & ~(sizeof(void *) - 1);

    while (addr + sizeof(void *) <= end) {
        uintptr_t candidate = *(uintptr_t *)addr;

        if (candidate >= (uintptr_t)region_base && candidate < (uintptr_t)region_end) {
            actor_gc_obj_t *obj = index_lookup(arena, candidate);
            if (obj && !(obj->flags & AGC_FLAG_MARK)) {
                obj->flags |= AGC_FLAG_MARK;
                if (!(obj->flags & AGC_FLAG_LEAF)) {
                    scan_range_mark(arena, actor_gc_payload(obj), obj->size);
                }
            }
        }
        addr += sizeof(void *);
    }
}

// --- Full collection with ext_refcount ---

void actor_gc_collect_full(actor_gc_arena_t *arena, actor_gc_root_t *roots, int num_roots) {
    if (!arena->objects) return;

    arena->collections++;

    // Phase 0a: Promote limbo → free_list (with ext_refcount check).
    // Objects in limbo were swept in the PREVIOUS collection cycle.
    // Check ext_refcount: if another actor acquired a reference since the
    // sweep (via track_outgoing_refs), rescue the object back to live list.
    {
        actor_gc_obj_t *limbo_obj = arena->limbo;
        while (limbo_obj) {
            actor_gc_obj_t *next = limbo_obj->next;
            if (__sync_fetch_and_add(&limbo_obj->ext_refcount, 0) > 0) {
                // Object was referenced by another actor since sweep.
                // Rescue: put back on live objects list.
                limbo_obj->next = arena->objects;
                arena->objects = limbo_obj;
                arena->num_objects++;
                arena->total_bytes += limbo_obj->size;
            } else {
                // Truly dead: promote to free_list for reuse.
                if (limbo_obj->size >= sizeof(uint64_t)) {
                    *(uint64_t *)actor_gc_payload(limbo_obj) = AGC_FREELIST_SENTINEL;
                }
                limbo_obj->next = arena->free_list;
                arena->free_list = limbo_obj;
                arena->free_bytes += limbo_obj->size;
            }
            limbo_obj = next;
        }
        arena->limbo = NULL;
        arena->limbo_bytes = 0;
    }

    // Phase 0b: Build sorted index for O(log n) pointer lookup.
    build_object_index(arena);
    if (arena->index_count == 0) return;  // OOM or empty

    // Phase 1: Clear marks
    actor_gc_obj_t *obj = arena->objects;
    while (obj) {
        obj->flags &= ~AGC_FLAG_MARK;
        obj = obj->next;
    }

    // Phase 2: Mark from roots via BFS through Boehm heap
    struct timespec collect_start, collect_end;
    clock_gettime(CLOCK_MONOTONIC, &collect_start);

    bfs_stats_t bfs_stats;
    mark_from_roots_bfs(arena, roots, num_roots, &bfs_stats);

    // Phase 3: Sweep — free objects that are unmarked AND have no external refs
    actor_gc_obj_t **prev = &arena->objects;
    obj = arena->objects;
    size_t freed = 0, freed_count = 0;

    while (obj) {
        actor_gc_obj_t *next = obj->next;
        if (obj->flags & AGC_FLAG_MARK) {
            prev = &obj->next;
        } else if (__sync_fetch_and_add(&obj->ext_refcount, 0) > 0) {
            prev = &obj->next;
        } else {
            // Dead: unreachable locally, no foreign references.
            // Remove from objects list and add to limbo (deferred free).
            // Objects go to limbo first, then get promoted to free_list
            // on the NEXT collection cycle. This ensures any concurrent
            // ENQ_msg + track_outgoing_refs has time to complete.
            *prev = next;
            freed += obj->size;
            freed_count++;
            arena->limbo_bytes += obj->size;

            obj->next = arena->limbo;
            arena->limbo = obj;
        }
        obj = next;
    }

    arena->total_bytes = (freed <= arena->total_bytes) ? arena->total_bytes - freed : 0;
    arena->num_objects = (freed_count <= arena->num_objects) ? arena->num_objects - freed_count : 0;
    arena->bytes_freed += freed;

    // Adjust threshold
    double reclaim_ratio = (arena->total_bytes + freed > 0)
        ? (double)freed / (double)(arena->total_bytes + freed) : 0.0;
    if (reclaim_ratio < MIN_RECLAIM_RATIO) {
        arena->collect_threshold = arena->total_bytes * THRESHOLD_GROWTH;
        if (arena->collect_threshold < INITIAL_THRESHOLD)
            arena->collect_threshold = INITIAL_THRESHOLD;
    }

    if (actor_gc_debug) {
        clock_gettime(CLOCK_MONOTONIC, &collect_end);
        double total_ms = (collect_end.tv_sec - collect_start.tv_sec) * 1000.0
                        + (collect_end.tv_nsec - collect_start.tv_nsec) / 1e6;
        int total_arena_marked = bfs_stats.arena_marked_from_roots + bfs_stats.arena_marked_from_boehm;
        fprintf(stderr, "AGC[%p] #%zu: marked=%d (roots=%d boehm=%d) boehm_visited=%d "
                "freed=%zu/%zu BFS=%.2fms total=%.2fms\n",
                (void *)arena, arena->collections,
                total_arena_marked,
                bfs_stats.arena_marked_from_roots,
                bfs_stats.arena_marked_from_boehm,
                bfs_stats.boehm_objects_visited,
                freed, freed + arena->total_bytes,
                bfs_stats.bfs_time_ms, total_ms);
    }
}

// --- Cross-actor reference tracking ---

void actor_gc_ext_ref(void *arena_ptr) {
    if (!actor_gc_is_arena_ptr(arena_ptr)) return;
    actor_gc_obj_t *hdr = actor_gc_header(arena_ptr);
    __sync_fetch_and_add(&hdr->ext_refcount, 1);
}

void actor_gc_ext_unref(void *arena_ptr) {
    if (!actor_gc_is_arena_ptr(arena_ptr)) return;
    actor_gc_obj_t *hdr = actor_gc_header(arena_ptr);
    // Guard against underflow: only decrement if > 0
    uint16_t old;
    do {
        old = hdr->ext_refcount;
        if (old == 0) return;  // already zero, don't underflow
    } while (!__sync_bool_compare_and_swap(&hdr->ext_refcount, old, old - 1));
}

// Recursively increment ext_refcount on an arena object and everything it
// points to. Uses AGC_FLAG_VISITED to avoid infinite loops on cycles.
void actor_gc_track_refs_recursive(void *arena_ptr) {
    if (!actor_gc_is_arena_ptr(arena_ptr)) return;

    actor_gc_obj_t *hdr = actor_gc_header(arena_ptr);
    if (hdr->flags & AGC_FLAG_VISITED) return;  // Already visited
    hdr->flags |= AGC_FLAG_VISITED;

    __sync_fetch_and_add(&hdr->ext_refcount, 1);

    // Scan payload for more arena pointers (unless leaf)
    if (hdr->flags & AGC_FLAG_LEAF) return;

    uintptr_t addr = (uintptr_t)arena_ptr;
    uintptr_t end = addr + hdr->size;
    addr = (addr + sizeof(void *) - 1) & ~(sizeof(void *) - 1);

    while (addr + sizeof(void *) <= end) {
        void *candidate = *(void **)addr;
        if (actor_gc_is_arena_ptr(candidate)) {
            actor_gc_track_refs_recursive(candidate);
        }
        addr += sizeof(void *);
    }
}

// Clear VISITED flags on all objects in an arena
void actor_gc_clear_visited(actor_gc_arena_t *arena) {
    if (!arena) return;
    actor_gc_obj_t *obj = arena->objects;
    while (obj) {
        obj->flags &= ~AGC_FLAG_VISITED;
        obj = obj->next;
    }
}

// --- Foreign reference management ---

// Helper: add a pointer to the foreign_refs array
static void foreign_refs_add(actor_gc_arena_t *arena, void *ptr) {
    if (arena->foreign_refs_count >= arena->foreign_refs_cap) {
        int new_cap = arena->foreign_refs_cap == 0 ? 32 : arena->foreign_refs_cap * 2;
        void **new_arr = (void **)realloc(arena->foreign_refs, new_cap * sizeof(void *));
        if (!new_arr) return;  // OOM, skip
        arena->foreign_refs = new_arr;
        arena->foreign_refs_cap = new_cap;
    }
    arena->foreign_refs[arena->foreign_refs_count++] = ptr;
}

// Comparison function for sorting pointers
static int ptr_cmp(const void *a, const void *b) {
    uintptr_t pa = *(const uintptr_t *)a;
    uintptr_t pb = *(const uintptr_t *)b;
    return (pa > pb) - (pa < pb);
}

void actor_gc_update_foreign_refs(actor_gc_arena_t *arena,
                                   void *actor_start, size_t actor_size) {
    // Build new foreign refs list by scanning the actor struct
    void **old_refs = arena->foreign_refs;
    int old_count = arena->foreign_refs_count;

    // Allocate new list
    arena->foreign_refs = NULL;
    arena->foreign_refs_count = 0;
    arena->foreign_refs_cap = 0;

    // Scan actor struct for arena pointers owned by OTHER arenas
    uintptr_t addr = (uintptr_t)actor_start;
    uintptr_t end = addr + actor_size;
    addr = (addr + sizeof(void *) - 1) & ~(sizeof(void *) - 1);

    while (addr + sizeof(void *) <= end) {
        void *candidate = *(void **)addr;
        if (actor_gc_is_arena_ptr(candidate)) {
            actor_gc_obj_t *hdr = actor_gc_header(candidate);
            if (hdr->owner != arena) {
                foreign_refs_add(arena, candidate);
            }
        }
        addr += sizeof(void *);
    }

    // Sort both lists for efficient diff
    if (old_count > 0)
        qsort(old_refs, old_count, sizeof(void *), ptr_cmp);
    if (arena->foreign_refs_count > 0)
        qsort(arena->foreign_refs, arena->foreign_refs_count, sizeof(void *), ptr_cmp);

    // Decrement ext_refcount for pointers in old but not in new
    int oi = 0, ni = 0;
    while (oi < old_count) {
        if (ni >= arena->foreign_refs_count || old_refs[oi] < arena->foreign_refs[ni]) {
            // old_refs[oi] was dropped
            actor_gc_ext_unref(old_refs[oi]);
            oi++;
        } else if (old_refs[oi] == arena->foreign_refs[ni]) {
            // Still referenced — no change
            oi++;
            ni++;
        } else {
            // new ref (already accounted for by track_outgoing_refs)
            ni++;
        }
    }

    if (old_refs) free(old_refs);
}

// --- Legacy promote API (kept for fallback/compatibility) ---

void *actor_gc_promote_one(void *arena_ptr) {
    if (!actor_gc_is_arena_ptr(arena_ptr))
        return arena_ptr;

    actor_gc_obj_t *hdr = actor_gc_header(arena_ptr);

    // Check if already promoted (forwarding pointer)
    if (hdr->flags & AGC_FLAG_VISITED) {
        return (void *)hdr->next;
    }

    size_t obj_size = hdr->size;
    void *boehm_copy = GC_malloc(obj_size);
    if (!boehm_copy) return arena_ptr;

    memcpy(boehm_copy, arena_ptr, obj_size);

    hdr->flags |= AGC_FLAG_VISITED;
    hdr->next = (actor_gc_obj_t *)boehm_copy;

    actor_gc_promote_region(boehm_copy, obj_size);

    return boehm_copy;
}

void actor_gc_promote_region(void *start, size_t size) {
    if (!region_base) return;
    uintptr_t addr = (uintptr_t)start;
    uintptr_t end = addr + size;
    addr = (addr + sizeof(void *) - 1) & ~(sizeof(void *) - 1);

    while (addr + sizeof(void *) <= end) {
        void *candidate = *(void **)addr;
        if (actor_gc_is_arena_ptr(candidate)) {
            void *promoted = actor_gc_promote_one(candidate);
            if (promoted != candidate) {
                *(void **)addr = promoted;
            }
        }
        addr += sizeof(void *);
    }
}
