// Per-actor arena garbage collector implementation
//
// Uses a single large mmap'd region for all arena allocations.
// Bump-pointer allocation within the region, with per-actor object lists
// for mark-sweep collection.
//
// Arenas are stored in a hash table keyed by actor pointer, NOT embedded
// in the $Actor struct. This avoids struct layout issues with generated code.
//
// NOTE: This file is #include'd from rts.c (not compiled separately).
// Most system headers are already available from the rts.c inclusion chain.

#include <sys/mman.h>
#include "actor_gc.h"

// GC_add_roots is available from gc.h, included by the parent translation unit

// Global arena region
static char *region_base = NULL;
static size_t region_size = 0;
static char *region_end = NULL;
static volatile char *bump_ptr = NULL;  // next free position

// Default: 1GB virtual region (physical pages allocated on demand)
#define DEFAULT_REGION_SIZE (1UL * 1024 * 1024 * 1024)

// Initial per-actor collection threshold
#define INITIAL_THRESHOLD (256 * 1024)
#define THRESHOLD_GROWTH 2
#define MIN_RECLAIM_RATIO 0.25

// Alignment for all allocations (16 bytes for SIMD compatibility)
#define ARENA_ALIGN 16
#define ALIGN_UP(x, a) (((x) + (a) - 1) & ~((a) - 1))

// Track the high-water mark of arena memory registered with Boehm as roots.
#define ROOT_CHUNK_SIZE (1024 * 1024)  // Register 1MB at a time
static volatile char *roots_registered_up_to = NULL;

// Runtime flag: skip Boehm root scanning of arena memory.
// Default ON (no root scanning) because leaf-only arena has no Boehm pointers.
// Set ACTON_GC_ROOTS=1 to force root scanning (for debugging or if non-leaf
// types are arena-allocated).
static int actor_gc_no_roots = 1;

// --- Thread-local current arena ---
static __thread actor_gc_arena_t *current_arena = NULL;

void actor_gc_set_current(actor_gc_arena_t *arena) {
    current_arena = arena;
}

actor_gc_arena_t *actor_gc_get_current(void) {
    return current_arena;
}

// --- Actor-to-arena hash table ---
// Simple open-addressing hash table keyed by actor pointer.
// Protected by a spinlock for thread safety.

#define ARENA_HT_SIZE 4096  // must be power of 2

typedef struct {
    void *actor_ptr;            // key (NULL = empty slot)
    actor_gc_arena_t *arena;    // value
} arena_ht_entry_t;

static arena_ht_entry_t arena_ht[ARENA_HT_SIZE];
static volatile int arena_ht_lock = 0;

static void ht_lock(void) {
    while (__sync_lock_test_and_set(&arena_ht_lock, 1)) {
        // spin
    }
}

static void ht_unlock(void) {
    __sync_lock_release(&arena_ht_lock);
}

static unsigned ht_hash(void *ptr) {
    uintptr_t v = (uintptr_t)ptr;
    // Mix bits: actor pointers are aligned, so shift away low bits
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
    // Hash table full — shouldn't happen with 4096 slots
    free(arena);
    return NULL;
}

actor_gc_arena_t *actor_gc_lookup(void *actor_ptr) {
    if (!region_base) return NULL;

    // Lock-free read: entries are only inserted, never moved or reused.
    // An entry's actor_ptr is written atomically (pointer-sized store).
    // We may see a NULL arena for a just-inserted entry, which is safe
    // (we'd just return NULL and miss one dispatch — not a correctness issue).
    unsigned idx = ht_hash(actor_ptr);
    for (unsigned i = 0; i < ARENA_HT_SIZE; i++) {
        unsigned slot = (idx + i) & (ARENA_HT_SIZE - 1);
        void *key = arena_ht[slot].actor_ptr;
        if (key == actor_ptr) {
            return arena_ht[slot].arena;
        }
        if (key == NULL) {
            return NULL;
        }
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
        if (arena_ht[slot].actor_ptr == NULL) {
            break;
        }
    }
    ht_unlock();
}

// --- Global region management ---

int actor_gc_global_init(size_t size) {
    if (size == 0) size = DEFAULT_REGION_SIZE;
    // Reserve virtual address space. MAP_NORESERVE means no swap is reserved;
    // physical pages are allocated only when touched.
#ifdef __linux__
    region_base = (char *)mmap(NULL, size, PROT_READ | PROT_WRITE,
                               MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
#else
    // macOS doesn't have MAP_NORESERVE but overcommits by default
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

    // Check environment for root scanning override
    const char *roots_env = getenv("ACTON_GC_ROOTS");
    if (roots_env && roots_env[0] == '1') {
        actor_gc_no_roots = 0;
        fprintf(stderr, "ACTOR_GC: region %p - %p (%zu MB) [BOEHM ROOT SCANNING ENABLED]\n",
                region_base, region_end, size / (1024*1024));
    } else {
        fprintf(stderr, "ACTOR_GC: region %p - %p (%zu MB)\n",
                region_base, region_end, size / (1024*1024));
    }
    return 0;
}

bool actor_gc_is_arena_ptr(void *ptr) {
    return region_base && ptr >= (void *)region_base && ptr < (void *)region_end;
}

void actor_gc_arena_init(actor_gc_arena_t *arena) {
    arena->objects = NULL;
    arena->total_bytes = 0;
    arena->num_objects = 0;
    arena->collect_threshold = INITIAL_THRESHOLD;
    arena->collections = 0;
    arena->bytes_freed = 0;
}

// Ensure Boehm knows about arena memory up to the given address.
// Arena-allocated objects may contain pointers to Boehm-managed objects.
// Without this, Boehm could collect objects only referenced from arena memory.
//
// NOTE: This is DISABLED when promote-on-flush is active. The promote path
// copies arena objects to Boehm before message delivery, so Boehm only needs
// to manage its own heap. Arena→Boehm references within an actor are safe
// because the actor struct itself (on Boehm) is the root, and Boehm traces
// from there. Arena objects are reachable from the actor struct via Boehm
// pointers, not the other way around... EXCEPT when arena objects contain
// pointers to Boehm objects (vtables, strings allocated pre-arena, etc.).
//
// TODO: For full correctness without root scanning, we need to ensure that
// Boehm objects referenced ONLY from arena memory are kept alive. Options:
// 1. Re-enable root scanning (current fallback)
// 2. Pin Boehm objects when stored into arena memory
// 3. Only arena-allocate leaf objects (no outgoing Boehm pointers)
static void ensure_roots_registered(char *up_to) {
    if (actor_gc_no_roots) return;

    char *registered = (char *)roots_registered_up_to;
    if (up_to <= registered) return;

    // Round up to ROOT_CHUNK_SIZE boundary for less frequent calls
    char *new_limit = (char *)ALIGN_UP((uintptr_t)up_to, ROOT_CHUNK_SIZE);
    if (new_limit > region_end) new_limit = region_end;

    if (__sync_bool_compare_and_swap(&roots_registered_up_to, registered, new_limit)) {
        GC_add_roots(registered, new_limit);
    }
}

// Thread-safe bump allocation from the global region
void *region_bump_alloc(size_t total_size) {
    total_size = ALIGN_UP(total_size, ARENA_ALIGN);
    char *old;
    char *new_ptr;
    do {
        old = (char *)bump_ptr;
        new_ptr = old + total_size;
        if (new_ptr > region_end) {
            return NULL;
        }
    } while (!__sync_bool_compare_and_swap(&bump_ptr, old, new_ptr));

    // Register new arena memory with Boehm so it scans for GC pointers
    ensure_roots_registered(new_ptr);

    return old;
}

void *actor_gc_alloc(actor_gc_arena_t *arena, size_t size) {
    if (!region_base) return NULL;  // global init failed
    size_t total = sizeof(actor_gc_obj_t) + size;
    void *block = region_bump_alloc(total);
    if (!block) return NULL;  // region full, caller should fall back to Boehm

    // Zero the allocation (matching GC_malloc behavior)
    memset(block, 0, ALIGN_UP(total, ARENA_ALIGN));

    actor_gc_obj_t *obj = (actor_gc_obj_t *)block;
    obj->size = (uint32_t)size;
    obj->marked = 0;

    // Link into actor's object list
    obj->next = arena->objects;
    arena->objects = obj;
    arena->total_bytes += size;
    arena->num_objects++;

    return actor_gc_payload(obj);
}

size_t actor_gc_obj_size(void *ptr) {
    actor_gc_obj_t *hdr = actor_gc_header(ptr);
    return hdr->size;
}

void *actor_gc_realloc(actor_gc_arena_t *arena, void *old_ptr, size_t new_size) {
    size_t old_size = actor_gc_obj_size(old_ptr);
    void *new_ptr = actor_gc_alloc(arena, new_size);
    if (!new_ptr) return NULL;
    size_t copy_size = old_size < new_size ? old_size : new_size;
    memcpy(new_ptr, old_ptr, copy_size);
    // Old block stays in the object list, will be swept later
    return new_ptr;
}

// Conservatively scan a memory range for pointers into the arena region.
static void scan_range(actor_gc_arena_t *arena, void *start, size_t size) {
    uintptr_t addr = (uintptr_t)start;
    uintptr_t end = addr + size;
    addr = (addr + sizeof(void *) - 1) & ~(sizeof(void *) - 1);

    while (addr + sizeof(void *) <= end) {
        void *candidate = *(void **)addr;

        if (actor_gc_is_arena_ptr(candidate)) {
            // candidate points into our region. Find the object it belongs to
            // by checking if it's a valid payload pointer.
            // Walk the object list to find it (could optimize with a bitmap later).
            actor_gc_obj_t *obj = arena->objects;
            while (obj) {
                void *payload = actor_gc_payload(obj);
                if (candidate >= payload && (char *)candidate < (char *)payload + obj->size) {
                    if (!obj->marked) {
                        obj->marked = 1;
                        // Recursively scan the newly marked object
                        scan_range(arena, payload, obj->size);
                    }
                    break;
                }
                obj = obj->next;
            }
        }
        addr += sizeof(void *);
    }
}

void actor_gc_collect(actor_gc_arena_t *arena, actor_gc_root_t *roots, int num_roots) {
    if (!arena->objects) return;

    arena->collections++;

    // Phase 1: Clear all marks
    actor_gc_obj_t *obj = arena->objects;
    while (obj) {
        obj->marked = 0;
        obj = obj->next;
    }

    // Phase 2: Mark from roots
    for (int i = 0; i < num_roots; i++) {
        scan_range(arena, roots[i].start, roots[i].size);
    }

    // Phase 3: Sweep - unlink unmarked objects
    // Note: we can't free individual bump-allocated blocks. Instead we just
    // unlink them from the object list so they won't be scanned again.
    // The memory is "leaked" within the region until the actor is destroyed.
    // TODO: use a free list or compaction for better memory reuse.
    actor_gc_obj_t **prev = &arena->objects;
    obj = arena->objects;
    size_t freed = 0;
    size_t freed_count = 0;

    while (obj) {
        actor_gc_obj_t *next = obj->next;
        if (!obj->marked) {
            *prev = next;
            freed += obj->size;
            freed_count++;
            // Can't actually free bump-allocated memory, but we stop tracking it
        } else {
            prev = &obj->next;
        }
        obj = next;
    }

    arena->total_bytes -= freed;
    arena->num_objects -= freed_count;
    arena->bytes_freed += freed;

    // Adjust threshold
    double reclaim_ratio = (arena->total_bytes + freed > 0)
        ? (double)freed / (double)(arena->total_bytes + freed)
        : 0.0;
    if (reclaim_ratio < MIN_RECLAIM_RATIO) {
        arena->collect_threshold = arena->total_bytes * THRESHOLD_GROWTH;
        if (arena->collect_threshold < INITIAL_THRESHOLD)
            arena->collect_threshold = INITIAL_THRESHOLD;
    }
}

void actor_gc_arena_destroy(actor_gc_arena_t *arena) {
    // Individual blocks can't be freed (bump allocator).
    // Just drop the object list.
    arena->objects = NULL;
    arena->total_bytes = 0;
    arena->num_objects = 0;
}

// --- Promote arena objects to Boehm heap ---

// Promote a single arena object to Boehm. Returns the new Boehm pointer.
// If the object has already been promoted (forwarding pointer stored in header),
// returns the existing Boehm copy. Recursively promotes arena pointers within.
void *actor_gc_promote_one(void *arena_ptr) {
    if (!actor_gc_is_arena_ptr(arena_ptr))
        return arena_ptr;  // Already on Boehm or not an arena ptr

    actor_gc_obj_t *hdr = actor_gc_header(arena_ptr);

    // Check if already promoted: we reuse the 'marked' field as a forwarding flag.
    // If marked == 2, the 'next' field holds the Boehm copy pointer.
    if (hdr->marked == 2) {
        return (void *)hdr->next;  // Return existing forwarding pointer
    }

    size_t obj_size = hdr->size;

    // Allocate on Boehm heap (GC_malloc returns zeroed, pointer-traced memory)
    void *boehm_copy = GC_malloc(obj_size);
    if (!boehm_copy) return arena_ptr;  // OOM fallback — keep arena ptr

    // Copy object data
    memcpy(boehm_copy, arena_ptr, obj_size);

    // Install forwarding pointer BEFORE recursing (breaks cycles)
    hdr->marked = 2;
    hdr->next = (actor_gc_obj_t *)boehm_copy;

    // Recursively promote any arena pointers within the copied object
    actor_gc_promote_region(boehm_copy, obj_size);

    return boehm_copy;
}

// Conservatively scan a region for arena pointers and promote them in-place.
void actor_gc_promote_region(void *start, size_t size) {
    if (!region_base) return;

    uintptr_t addr = (uintptr_t)start;
    uintptr_t end = addr + size;
    // Align to pointer boundary
    addr = (addr + sizeof(void *) - 1) & ~(sizeof(void *) - 1);

    while (addr + sizeof(void *) <= end) {
        void *candidate = *(void **)addr;
        if (actor_gc_is_arena_ptr(candidate)) {
            void *promoted = actor_gc_promote_one(candidate);
            if (promoted != candidate) {
                *(void **)addr = promoted;  // Update pointer in-place
            }
        }
        addr += sizeof(void *);
    }
}
