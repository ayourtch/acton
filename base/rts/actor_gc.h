// Per-actor arena garbage collector
//
// Memory architecture:
// - One large virtual memory region is reserved at startup (e.g. 1GB)
// - Physical pages are only allocated on demand (mmap overcommit)
// - Any pointer within [region_base, region_base+region_size) is arena-owned
// - This gives O(1) pointer ownership checks
// - Each actor gets a sub-arena within this region via bump allocation
// - If the region is exhausted, allocations fall back to Boehm GC
//
// Integration:
// - Arenas are stored in a hash table keyed by actor pointer (NOT in $Actor struct)
// - A thread-local caches the current actor's arena for fast access
// - acton_malloc/acton_realloc route to the arena when an actor is executing

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

// Object header prepended to every arena allocation.
// Keeps it simple: linked list per actor for mark-sweep.
typedef struct actor_gc_obj {
    struct actor_gc_obj *next;  // linked list of all objects in this actor's arena
    uint32_t size;              // usable size (excluding header)
    uint32_t marked;            // mark bit for GC
} actor_gc_obj_t;

// Per-actor arena: a bump pointer into the global region + object list for GC
typedef struct actor_gc_arena {
    actor_gc_obj_t *objects;    // linked list of all allocated objects
    size_t total_bytes;         // total bytes allocated
    size_t num_objects;         // number of live objects
    size_t collect_threshold;   // trigger collection when total_bytes exceeds this
    size_t collections;         // number of collections performed
    size_t bytes_freed;         // total bytes freed across all collections
} actor_gc_arena_t;

// Initialize the global arena region. Call once at startup.
// Returns 0 on success, -1 on failure.
int actor_gc_global_init(size_t region_size);

// O(1) check: is this pointer within the arena region?
bool actor_gc_is_arena_ptr(void *ptr);

// Initialize a per-actor arena
void actor_gc_arena_init(actor_gc_arena_t *arena);

// Allocate memory from the global region, tracked by the given actor arena.
// Returns zeroed memory. Falls back to NULL if region is exhausted.
void *actor_gc_alloc(actor_gc_arena_t *arena, size_t size);

// Realloc within arena: allocate new, copy old data, leave old for sweep.
// old_ptr must be an arena pointer.
void *actor_gc_realloc(actor_gc_arena_t *arena, void *old_ptr, size_t new_size);

// Get the size of an arena-allocated object (from its header)
size_t actor_gc_obj_size(void *ptr);

// Run conservative mark-sweep collection on an arena.
typedef struct {
    void *start;
    size_t size;
} actor_gc_root_t;

void actor_gc_collect(actor_gc_arena_t *arena, actor_gc_root_t *roots, int num_roots);

// Free all memory in an arena (used when actor is destroyed)
void actor_gc_arena_destroy(actor_gc_arena_t *arena);

// --- Actor-to-arena mapping (hash table, separate from $Actor struct) ---

// Register an arena for an actor. The arena is heap-allocated by this function.
// Returns the new arena, or NULL on failure.
actor_gc_arena_t *actor_gc_register(void *actor_ptr);

// Look up the arena for an actor. Returns NULL if not registered.
actor_gc_arena_t *actor_gc_lookup(void *actor_ptr);

// Unregister and destroy an actor's arena.
void actor_gc_unregister(void *actor_ptr);

// --- Thread-local current arena (set/cleared alongside SET_SELF) ---

// Set the current thread's active arena (call when SET_SELF is called).
// Pass NULL to clear (when SET_SELF(NULL)).
void actor_gc_set_current(actor_gc_arena_t *arena);

// Get the current thread's active arena. Returns NULL if no actor is executing.
actor_gc_arena_t *actor_gc_get_current(void);

// --- Promote arena objects to Boehm heap ---

// Conservatively scan a memory region for arena pointers. For each arena
// pointer found, copy the object to the Boehm heap and update the pointer
// in-place. Recurses into promoted objects to handle transitive references.
// This is used before message delivery to ensure the receiving actor doesn't
// hold dangling references into the sender's arena.
void actor_gc_promote_region(void *start, size_t size);

// Promote a single arena-allocated object to Boehm. Returns the new Boehm
// pointer. Recursively promotes any arena pointers within the object.
void *actor_gc_promote_one(void *arena_ptr);

// Header/payload helpers
static inline actor_gc_obj_t *actor_gc_header(void *ptr) {
    return (actor_gc_obj_t *)((char *)ptr - sizeof(actor_gc_obj_t));
}

static inline void *actor_gc_payload(actor_gc_obj_t *obj) {
    return (void *)((char *)obj + sizeof(actor_gc_obj_t));
}
