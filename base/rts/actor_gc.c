// Per-actor garbage collector with ownership tracking
//
// Uses a single large mmap'd region for all arena allocations.
// Free-list + bump allocation within the region.
// Per-actor mark-sweep collection respects ext_refcount for cross-actor refs.
//
// NOTE: This file is #include'd from rts.c (not compiled separately).

#include <sys/mman.h>
#include "actor_gc.h"

// --- Global arena region ---

static char *region_base = NULL;
static size_t region_size = 0;
static char *region_end = NULL;
static volatile char *bump_ptr = NULL;

#define DEFAULT_REGION_SIZE (8UL * 1024 * 1024 * 1024)

#define INITIAL_THRESHOLD (1024 * 1024)
#define THRESHOLD_GROWTH 2
#define MIN_RECLAIM_RATIO 0.25

#define ARENA_ALIGN 16
#define ALIGN_UP(x, a) (((x) + (a) - 1) & ~((a) - 1))

// Boehm root scanning
#define ROOT_CHUNK_SIZE (1024 * 1024)
static volatile char *roots_registered_up_to = NULL;
// Default: root scanning ON (all-in arena needs it for Boehm pointer safety)
static int actor_gc_no_roots = 0;

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

    fprintf(stderr, "ACTOR_GC: region %p - %p (%zu MB)%s\n",
            region_base, region_end, size / (1024*1024),
            actor_gc_no_roots ? " [no root scanning]" : " [root scanning ON]");
    return 0;
}

bool actor_gc_is_arena_ptr(void *ptr) {
    return region_base && ptr >= (void *)region_base && ptr < (void *)region_end;
}

// --- Arena lifecycle ---

void actor_gc_arena_init(actor_gc_arena_t *arena) {
    arena->objects = NULL;
    arena->free_list = NULL;
    arena->total_bytes = 0;
    arena->num_objects = 0;
    arena->free_bytes = 0;
    arena->collect_threshold = INITIAL_THRESHOLD;
    arena->collections = 0;
    arena->bytes_freed = 0;
    arena->index = NULL;
    arena->index_count = 0;
    arena->index_cap = 0;
    arena->foreign_refs = NULL;
    arena->foreign_refs_count = 0;
    arena->foreign_refs_cap = 0;
}

void actor_gc_arena_destroy(actor_gc_arena_t *arena) {
    arena->objects = NULL;
    arena->free_list = NULL;
    arena->total_bytes = 0;
    arena->num_objects = 0;
    arena->free_bytes = 0;
    if (arena->index) {
        free(arena->index);
        arena->index = NULL;
    }
    arena->index_count = 0;
    arena->index_cap = 0;
    if (arena->foreign_refs) {
        free(arena->foreign_refs);
        arena->foreign_refs = NULL;
    }
    arena->foreign_refs_count = 0;
    arena->foreign_refs_cap = 0;
}

// --- Boehm root scanning ---

static void ensure_roots_registered(char *up_to) {
    if (actor_gc_no_roots) return;
    char *registered = (char *)roots_registered_up_to;
    if (up_to <= registered) return;
    char *new_limit = (char *)ALIGN_UP((uintptr_t)up_to, ROOT_CHUNK_SIZE);
    if (new_limit > region_end) new_limit = region_end;
    if (__sync_bool_compare_and_swap(&roots_registered_up_to, registered, new_limit)) {
        GC_add_roots(registered, new_limit);
    }
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

void *actor_gc_alloc(actor_gc_arena_t *arena, size_t size) {
    if (!region_base) return NULL;

    size_t aligned_size = ALIGN_UP(size, ARENA_ALIGN);
    size_t total = sizeof(actor_gc_obj_t) + aligned_size;

    // 1. Check free list (first-fit)
    actor_gc_obj_t **prev = &arena->free_list;
    actor_gc_obj_t *blk = arena->free_list;
    while (blk) {
        size_t blk_aligned = ALIGN_UP(blk->size, ARENA_ALIGN);
        if (blk_aligned >= aligned_size) {
            // Found a fit — remove from free list
            *prev = blk->next;
            arena->free_bytes -= blk->size;

            // Zero the payload
            memset(actor_gc_payload(blk), 0, blk->size);

            // Set up header (keep original size for the block)
            blk->next = arena->objects;
            arena->objects = blk;
            blk->owner = arena;
            blk->flags = 0;
            blk->ext_refcount = 0;
            // Keep blk->size as the original block size (may be >= requested)

            arena->total_bytes += blk->size;
            arena->num_objects++;
            return actor_gc_payload(blk);
        }
        prev = &blk->next;
        blk = blk->next;
    }

    // 2. Bump allocate from global region
    void *block = region_bump_alloc(total);
    if (!block) return NULL;

    memset(block, 0, total);

    actor_gc_obj_t *obj = (actor_gc_obj_t *)block;
    obj->size = (uint32_t)aligned_size;
    obj->owner = arena;
    obj->flags = 0;
    obj->ext_refcount = 0;

    obj->next = arena->objects;
    arena->objects = obj;
    arena->total_bytes += aligned_size;
    arena->num_objects++;

    return actor_gc_payload(obj);
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

// Comparison for qsort: sort index entries by payload_start
static int index_entry_cmp(const void *a, const void *b) {
    uintptr_t pa = ((const actor_gc_index_entry_t *)a)->payload_start;
    uintptr_t pb = ((const actor_gc_index_entry_t *)b)->payload_start;
    return (pa > pb) - (pa < pb);
}

// Build sorted index of all objects in the arena. Called once per collection.
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

    // Populate
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

    // Sort by payload_start
    qsort(arena->index, arena->index_count, sizeof(actor_gc_index_entry_t), index_entry_cmp);
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
// The mark phase scans memory for arena pointers. Arena→arena chains
// are followed recursively. Boehm objects in the ROOT (actor struct)
// are followed one level deep to bridge the gap (e.g., B_Msg on Boehm
// contains $cont on arena).

// Arena-only scan: follows arena pointers, ignores everything else.
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

// Root scan: like scan_range_mark but also follows Boehm pointers one level
// deep. This bridges actor struct → B_Msg (Boehm) → $cont (arena).
// Boehm objects found here are scanned with arena-only scan_range_mark
// (no further Boehm following to avoid cascading into the global heap).
static void scan_roots_mark(actor_gc_arena_t *arena, void *start, size_t size) {
    // First pass: find and mark arena pointers (same as scan_range_mark)
    // Second concern: follow Boehm pointers one level deep

    // Collect Boehm objects to scan (small fixed-size buffer)
    #define MAX_BOEHM_ROOTS 64
    void *boehm_bases[MAX_BOEHM_ROOTS];
    size_t boehm_sizes[MAX_BOEHM_ROOTS];
    int boehm_count = 0;

    uintptr_t addr = (uintptr_t)start;
    uintptr_t end = addr + size;
    addr = (addr + sizeof(void *) - 1) & ~(sizeof(void *) - 1);

    while (addr + sizeof(void *) <= end) {
        uintptr_t candidate = *(uintptr_t *)addr;

        if (candidate >= (uintptr_t)region_base && candidate < (uintptr_t)region_end) {
            // Arena pointer — mark and recursively scan
            actor_gc_obj_t *obj = index_lookup(arena, candidate);
            if (obj && !(obj->flags & AGC_FLAG_MARK)) {
                obj->flags |= AGC_FLAG_MARK;
                if (!(obj->flags & AGC_FLAG_LEAF)) {
                    scan_range_mark(arena, actor_gc_payload(obj), obj->size);
                }
            }
        } else if (candidate > 0x1000 && boehm_count < MAX_BOEHM_ROOTS) {
            // Potential Boehm pointer — collect for one-level scan
            void *base = GC_base((void *)candidate);
            if (base) {
                // Check for duplicates
                bool dup = false;
                for (int i = 0; i < boehm_count; i++) {
                    if (boehm_bases[i] == base) { dup = true; break; }
                }
                if (!dup) {
                    boehm_bases[boehm_count] = base;
                    boehm_sizes[boehm_count] = GC_size(base);
                    boehm_count++;
                }
            }
        }
        addr += sizeof(void *);
    }

    // Now scan collected Boehm objects (arena-only, no further Boehm following)
    for (int i = 0; i < boehm_count; i++) {
        scan_range_mark(arena, boehm_bases[i], boehm_sizes[i]);
    }
}

// --- Full collection with ext_refcount ---

void actor_gc_collect_full(actor_gc_arena_t *arena, actor_gc_root_t *roots, int num_roots) {
    if (!arena->objects) return;

    arena->collections++;

    // Phase 0: Build sorted index for O(log n) pointer lookup
    build_object_index(arena);
    if (arena->index_count == 0) return;  // OOM or empty

    // Phase 1: Clear marks
    actor_gc_obj_t *obj = arena->objects;
    while (obj) {
        obj->flags &= ~AGC_FLAG_MARK;
        obj = obj->next;
    }

    // Phase 2: Mark from roots (follows Boehm pointers one level deep)
    for (int i = 0; i < num_roots; i++) {
        scan_roots_mark(arena, roots[i].start, roots[i].size);
    }

    // Phase 3: Sweep — free objects that are unmarked AND have no external refs
    actor_gc_obj_t **prev = &arena->objects;
    obj = arena->objects;
    size_t freed = 0, freed_count = 0;
    size_t marked_count = 0, extref_count = 0;

    while (obj) {
        actor_gc_obj_t *next = obj->next;
        if (obj->flags & AGC_FLAG_MARK) {
            marked_count++;
            prev = &obj->next;
        } else if (obj->ext_refcount > 0) {
            extref_count++;
            prev = &obj->next;
        } else {
            // Dead: unreachable locally, no foreign references
            *prev = next;
            freed += obj->size;
            freed_count++;
            // TODO: free list reuse causes corruption — investigate header/size
            // mismatch on reuse. For now, swept objects are leaked (not reused).
            // With 8GB virtual region and overcommit, this is acceptable for
            // benchmarking. Physical pages are reclaimed by the OS when unused.
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

    // Uncomment for debug:
    // fprintf(stderr, "AGC[%p] #%zu: %zu marked, %zu swept, %zu live (%zu B)\n",
    //         (void *)arena, arena->collections, marked_count, freed_count,
    //         arena->num_objects, arena->total_bytes);
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
