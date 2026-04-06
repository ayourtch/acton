#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef __linux__
#include <dlfcn.h>
#endif

#include <mbedtls/platform.h>
#define LIBXML_STATIC
#include <libxml/xmlmemory.h>
#include <tlsuv/tlsuv.h>

#include "rts/common.h"
#include "rts/actor_gc.h"

#if defined(_WIN32) || defined(_WIN64)

#include <math.h>

// strndup() is not available on Windows
char *strndup( const char *s1, size_t n)
{
    char *copy= (char*)malloc( n+1 );
    memcpy( copy, s1, n );
    copy[n] = 0;
    return copy;
};
#endif

typedef struct {
    acton_malloc_func malloc;
    acton_malloc_func malloc_atomic;
    acton_realloc_func realloc;
    acton_calloc_func calloc;
    acton_free_func free;
    acton_strdup_func strdup;
    acton_strndup_func strndup;
} acton__allocator_t;

static acton__allocator_t acton__allocator = {
    malloc,
    malloc,
    realloc,
    calloc,
    free,
    strdup,
    strndup
};

void acton_noop_free(void *ptr) {
}

void *GC_calloc(size_t count, size_t size) {
    return GC_malloc(count*size);
}

void acton_init_alloc() {
    // UV & TLSUV are used for IO, which is always done on the GC-heap. We don't
    // have any constants related to UV, so we can always use the GC

    uv_replace_allocator(GC_malloc,
                         GC_realloc,
                         GC_calloc,
                         acton_noop_free);

    tlsuv_set_allocator(GC_malloc,
                            GC_realloc,
                            GC_calloc,
                            acton_noop_free);
}

int acton_replace_allocator(acton_malloc_func malloc_func,
                            acton_malloc_func malloc_atomic_func,
                            acton_realloc_func realloc_func,
                            acton_calloc_func calloc_func,
                            acton_free_func free_func,
                            acton_strdup_func strdup_func,
                            acton_strndup_func strndup_func) {
    if (malloc_func == NULL || malloc_atomic_func == NULL ||
        realloc_func == NULL || calloc_func == NULL ||
        free_func == NULL || strdup_func == NULL ||
        strndup_func == NULL
        ) {
        return -1;
    }

    acton__allocator.malloc = malloc_func;
    acton__allocator.malloc_atomic = malloc_atomic_func;
    acton__allocator.realloc = realloc_func;
    acton__allocator.calloc = calloc_func;
    acton__allocator.free = free_func;
    acton__allocator.strdup = strdup_func;
    acton__allocator.strndup = strndup_func;

    bsdnt_replace_allocator(acton__allocator.malloc,
                            acton__allocator.realloc,
                            acton__allocator.free);

    xmlMemSetup(acton__allocator.free,
                acton__allocator.malloc,
                acton__allocator.realloc,
                acton__allocator.strdup);

    mbedtls_platform_set_calloc_free(acton__allocator.calloc,
                                     acton_noop_free);

    return 0;
}

void* acton_malloc(size_t size) {
    return acton__allocator.calloc(1, size);
}

void* acton_malloc_atomic(size_t size) {
    return acton__allocator.malloc_atomic(size);
}

// Allocate from per-actor arena if available, otherwise from Boehm atomic.
// Use this ONLY for leaf objects that contain no outgoing GC pointers
// (e.g., B_int, B_float — only have a static vtable pointer and scalar data).
// Since leaf objects don't reference GC-managed memory, Boehm doesn't need
// to scan them, and they don't need to be registered as roots.
void* acton_malloc_leaf(size_t size) {
    actor_gc_arena_t *arena = actor_gc_get_current();
    if (arena) {
        void *p = actor_gc_alloc(arena, size);
        if (p) return p;
    }
    return acton__allocator.calloc(1, size);
}

void* acton_realloc(void* ptr, size_t size) {
    // Safety: arena pointers must never be passed to GC_realloc.
    // With leaf-only arena allocation, this shouldn't happen (leaf types
    // don't use realloc), but guard against it.
    if (ptr && actor_gc_is_arena_ptr(ptr)) {
        void *boehm_ptr = acton__allocator.calloc(1, size);
        if (boehm_ptr) {
            size_t old_size = actor_gc_obj_size(ptr);
            size_t copy_size = old_size < size ? old_size : size;
            memcpy(boehm_ptr, ptr, copy_size);
        }
        return boehm_ptr;
    }
    return acton__allocator.realloc(ptr, size);
}


void* acton_calloc(size_t count, size_t size) {
    return acton__allocator.calloc(count, size);
}

void acton_free(void* ptr) {
    int saved_errno;

    /* The system allocator the assumption that errno is not modified but custom
     * allocators may not be so careful.
     */
    saved_errno = errno;
    acton__allocator.free(ptr);
    errno = saved_errno;
}

char *acton_strdup(const char *s) {
    return acton__allocator.strdup(s);
}

char *acton_strndup(const char *s, size_t n) {
    return acton__allocator.strndup(s, n);
}


void* acton_gc_malloc(size_t size) {
    return GC_malloc(size);
}

void* acton_gc_malloc_atomic(size_t size) {
    return GC_malloc_atomic(size);
}

void* acton_gc_realloc(void* ptr, size_t size) {
    return GC_realloc(ptr, size);
}

void* acton_gc_calloc(size_t count, size_t size) {
    return GC_calloc(count, size);
}

void acton_gc_free(void* ptr) {
    return GC_free(ptr);
}

char *acton_gc_strdup(const char *s) {
    return GC_strdup(s);
}

char *acton_gc_strndup(const char *s, size_t n) {
    return GC_strndup(s, n);
}
