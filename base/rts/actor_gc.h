// Per-actor garbage collector with ownership tracking
//
// Memory architecture:
// - One large virtual memory region reserved at startup (1GB, overcommit)
// - Physical pages allocated on demand
// - Any pointer in [region_base, region_end) is arena-owned: O(1) check
// - Each actor gets a per-actor arena within this region
// - Allocation: free list first, then bump from region, then Boehm fallback
//
// Ownership model:
// - Every arena object has an owner (the arena that allocated it)
// - Objects stay in the owner's arena for their lifetime (no copying)
// - Cross-actor references tracked via ext_refcount on the object header
// - On message send: increment ext_refcount on referenced arena objects
// - On message done: decrement ext_refcount for dropped foreign references
// - Collection: sweep only objects with mark=0 AND ext_refcount=0
//
// Integration:
// - Arenas stored in hash table keyed by actor pointer (NOT in $Actor struct)
// - Thread-local caches current arena for fast access
// - acton_malloc routes to arena when an actor is executing

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

// --- Object header ---

// Flags for actor_gc_obj_t.flags
#define AGC_FLAG_MARK      0x0001   // Reachable from local roots
#define AGC_FLAG_LEAF      0x0002   // No outgoing pointers (skip scanning)
#define AGC_FLAG_VISITED   0x0004   // Temporary: used during ref tracking traversal

// Forward declaration
typedef struct actor_gc_arena actor_gc_arena_t;

// Object header prepended to every arena allocation.
typedef struct actor_gc_obj {
    struct actor_gc_obj *next;      // linked list in owner's arena (objects or free_list)
    actor_gc_arena_t *owner;        // which arena owns this object
    uint32_t size;                  // usable payload size (excluding header)
    uint16_t flags;                 // AGC_FLAG_* bits
    uint16_t ext_refcount;          // cross-actor reference count (atomic)
} actor_gc_obj_t;  // 24 bytes

// Per-actor arena
struct actor_gc_arena {
    actor_gc_obj_t *objects;        // linked list of all live objects
    actor_gc_obj_t *free_list;      // freed blocks available for reuse
    size_t total_bytes;             // total live bytes allocated (payload only)
    size_t num_objects;             // number of live objects
    size_t free_bytes;              // bytes available in free list
    size_t collect_threshold;       // trigger collection when total_bytes exceeds this
    size_t collections;             // number of collections performed
    size_t bytes_freed;             // total bytes freed across all collections
    // Foreign reference tracking
    void **foreign_refs;            // array of foreign arena ptrs this actor references
    int foreign_refs_count;         // current count
    int foreign_refs_cap;           // capacity
};

// --- Global region management ---

// Initialize the global arena region. Call once at startup.
int actor_gc_global_init(size_t region_size);

// O(1) check: is this pointer within the arena region?
bool actor_gc_is_arena_ptr(void *ptr);

// --- Per-actor arena lifecycle ---

void actor_gc_arena_init(actor_gc_arena_t *arena);
void actor_gc_arena_destroy(actor_gc_arena_t *arena);

// --- Allocation ---

// Allocate from arena: checks free list, then bump, returns zeroed memory.
// Returns NULL if region exhausted (caller should fall back to Boehm).
void *actor_gc_alloc(actor_gc_arena_t *arena, size_t size);

// Realloc within arena: allocate new, copy, old block goes to free list on next sweep.
void *actor_gc_realloc(actor_gc_arena_t *arena, void *old_ptr, size_t new_size);

// Get the size of an arena-allocated object
size_t actor_gc_obj_size(void *ptr);

// --- Collection ---

typedef struct {
    void *start;
    size_t size;
} actor_gc_root_t;

// Full collection: mark from roots, sweep unmarked objects with ext_refcount==0.
// Swept objects are moved to the free list for reuse.
void actor_gc_collect_full(actor_gc_arena_t *arena, actor_gc_root_t *roots, int num_roots);

// --- Cross-actor reference tracking ---

// Increment ext_refcount (thread-safe, atomic)
void actor_gc_ext_ref(void *arena_ptr);

// Decrement ext_refcount (thread-safe, atomic)
void actor_gc_ext_unref(void *arena_ptr);

// Recursively increment ext_refcount on arena_ptr and all arena objects
// reachable from it. Uses AGC_FLAG_VISITED to avoid cycles.
// Call actor_gc_clear_visited() after a batch of these calls.
void actor_gc_track_refs_recursive(void *arena_ptr);

// Clear AGC_FLAG_VISITED on all objects visited during track_refs_recursive.
// Pass the arena that owns the objects, or NULL to clear based on a visited list.
void actor_gc_clear_visited(actor_gc_arena_t *arena);

// --- Foreign reference management ---

// After an actor finishes processing a message, scan the actor struct for
// foreign arena pointers and update ext_refcounts accordingly.
// old_refs/old_count: previous foreign refs (from last call). Pass NULL/0 on first call.
// actor_start/actor_size: memory region of the actor struct to scan.
// Updates arena->foreign_refs with the new set.
void actor_gc_update_foreign_refs(actor_gc_arena_t *arena,
                                   void *actor_start, size_t actor_size);

// --- Actor-to-arena mapping ---

actor_gc_arena_t *actor_gc_register(void *actor_ptr);
actor_gc_arena_t *actor_gc_lookup(void *actor_ptr);
void actor_gc_unregister(void *actor_ptr);

// --- Thread-local current arena ---

void actor_gc_set_current(actor_gc_arena_t *arena);
actor_gc_arena_t *actor_gc_get_current(void);

// --- Legacy promote API (kept for fallback) ---

void actor_gc_promote_region(void *start, size_t size);
void *actor_gc_promote_one(void *arena_ptr);

// --- Header/payload helpers ---

static inline actor_gc_obj_t *actor_gc_header(void *ptr) {
    return (actor_gc_obj_t *)((char *)ptr - sizeof(actor_gc_obj_t));
}

static inline void *actor_gc_payload(actor_gc_obj_t *obj) {
    return (void *)((char *)obj + sizeof(actor_gc_obj_t));
}
