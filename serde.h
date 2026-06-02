/*
 * serde.h - single-header persistent allocator + columnar scan (little-endian).
 * Define SERDE_IMPLEMENTATION in exactly one translation unit before including.
 *
 * Copyright (c) 2019 Praveen Vaddadi <thynktank@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef SERDE_H
#define SERDE_H

#define SERDE_VERSION_MAJOR 0
#define SERDE_VERSION_MINOR 1
#define SERDE_VERSION_PATCH 0
#define SERDE_VERSION "0.1.0"


#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "serde.h targets little-endian architectures only."
#endif

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include <assert.h>
#include <errno.h>
#include <execinfo.h>
#include <fcntl.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef __SSE4_1__
#include <pmmintrin.h>
#include <smmintrin.h>
#endif
#ifdef __SSE2__
#include <emmintrin.h>
#endif

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif
#if defined(__GNUC__) || __has_builtin(__builtin_expect)
#define SD_LIKELY(x) __builtin_expect((x), 1)
#define SD_UNLIKELY(x) __builtin_expect((x), 0)
#else
#define SD_LIKELY(x) (x)
#define SD_UNLIKELY(x) (x)
#endif

#ifndef SD_TRACE_DEPTH
#define SD_TRACE_DEPTH 2048
#endif
#define sd_print_trace(fp)                                                                         \
    do {                                                                                           \
        void *frames[SD_TRACE_DEPTH];                                                              \
        size_t nbytes = backtrace(frames, SD_TRACE_DEPTH);                                         \
        backtrace_symbols_fd(frames, nbytes, fileno(fp));                                          \
        abort();                                                                                   \
    } while (0)
#ifndef NDEBUG
#define SD_ASSERT(X, fmt, ...)                                                                     \
    do {                                                                                           \
        if (SD_UNLIKELY(!(X))) {                                                                   \
            fprintf(stderr, "serde[%s:%d %s]: (" #X ") " fmt "\n", __FILE__, __LINE__,             \
                    __FUNCTION__, ##__VA_ARGS__);                                                  \
            sd_print_trace(stderr);                                                                \
        }                                                                                          \
    } while (0)
#else
#define SD_ASSERT(X, fmt, ...) ((void)0)
#endif
#define SD_CEILDIV(X, Y) (((X) + (Y)-1) / (Y))

enum {
    SD_PAGE_BITS = 12,
    SD_REGION_BITS = 21,
    SD_ARENA_BITS = 45,
    SD_TYPE_SLOTS = 2048,
    SD_DISPATCH_TRIES = 128
};
#define SD_PAGE_SIZE ((size_t)1 << SD_PAGE_BITS)
#define SD_REGION_SIZE ((size_t)1 << SD_REGION_BITS)
#define SD_ARENA_SIZE ((uint64_t)1 << SD_ARENA_BITS)
#define SD_REGION_WORDS (SD_ARENA_SIZE / SD_REGION_SIZE / 64)
#define SD_MARKMAP_BYTES (SD_REGION_WORDS * 8)
#define SD_MARKMAP_REGIONS ((SD_MARKMAP_BYTES + SD_REGION_SIZE - 1) / SD_REGION_SIZE)
#define SD_META_REGIONS (1 + 2 * SD_MARKMAP_REGIONS)
#define SD_FORMAT_REV 0x44524553u

typedef uintptr_t sd_ref;
typedef _Atomic int16_t sd_a16;
typedef _Atomic uint8_t sd_au8;
typedef _Atomic uint16_t sd_au16;
typedef _Atomic uint32_t sd_au32;
typedef _Atomic uint64_t sd_au64;

typedef enum sd_tag_kind {
    SD_K_SLAB = 0x6,
    SD_K_WIDESLAB = 0x9,
    SD_K_CHUNK = 0x3,
    SD_K_REGION = 0x5,
    SD_K_GIANT = 0xA
} sd_tag_kind;

typedef union sd_tag {
    struct {
        sd_tag_kind kind : 4;
        uint32_t pad_cells : 28;
    } any;
    struct {
        sd_tag_kind kind : 4;
        uint16_t cell_bytes : 12;
        uint16_t pad_cells;
    } slab_any;
    struct {
        sd_tag_kind kind : 4;
        uint16_t cell_bytes : 12;
        uint8_t sd_tls_lane : 4;
        uint16_t pad_cells : 12;
    } slab;
    struct {
        sd_tag_kind kind : 4;
        uint16_t cell_bytes : 12;
        uint16_t pad_cells;
    } wideslab;
    struct {
        sd_tag_kind kind : 4;
        uint16_t page_run : 12;
        uint16_t rsv;
    } chunk;
    struct {
        sd_tag_kind kind : 4;
        uint32_t rsv : 28;
    } region;
    struct {
        sd_tag_kind kind : 4;
        uint32_t region_run : 28;
    } giant;
    uint32_t word;
    uint64_t align_to;
} sd_tag;

enum { SD_SPAN_LISTED = 0, SD_SPAN_UNLISTED = 1 };
typedef enum sd_step { SD_DONE = 0, SD_NEXT = 1, SD_REDO = 2 } sd_step;

typedef struct sd_slab {
    sd_tag tag;
    sd_ref chain;
    sd_au16 cell_used;
    sd_a16 latch;
    sd_au8 listed;
    uint8_t scan_hint;
    uint8_t mark_pad;
    uint8_t mark_head;
    uint8_t mark_words;
    const uint32_t pad : 24;
} sd_slab;

typedef struct __attribute__((aligned(16))) sd_region {
    sd_tag tag;
    sd_ref chain;
    sd_au64 head_mark[8];
    sd_au64 used_mark[8];
    sd_a16 latch;
    sd_au8 listed;
    const int8_t pad;
    char pad2[8];
} sd_region;

typedef struct sd_slab_list {
    sd_a16 latch;
    sd_ref pending;
    sd_ref slab;
} sd_slab_list;

typedef struct sd_region_list {
    sd_a16 latch;
    sd_ref pending;
    sd_ref region;
} sd_region_list;

typedef struct sd_chunk_blob {
    const sd_tag tag;
} sd_chunk_blob;

typedef struct sd_giant_blob {
    const sd_tag tag;
} sd_giant_blob;

typedef union sd_chunk_ref {
    uintptr_t word_addr;
    sd_tag *tag;
    sd_slab *slab;
    sd_chunk_blob *chunk_blob;
} sd_chunk_ref __attribute__((__transparent_union__));

typedef union sd_region_ref {
    uintptr_t word_addr;
    sd_tag *tag;
    sd_region *region;
    sd_giant_blob *giant_blob;
} sd_region_ref __attribute__((__transparent_union__));

typedef struct sd_roots {
    sd_region_list region_lane;
    sd_slab_list wideslab_lanes[3];
    sd_slab_list slab_lanes[16][16];
} sd_roots;

typedef struct serde_arena {
    uint32_t format_rev;
    int map_flags;
    int backing_fd;
    uint16_t region_count;
    sd_a16 latch;
    sd_ref head_mark_ref;
    sd_ref used_mark_ref;
    sd_ref roots[8];
    sd_roots freelists;
    sd_region region;
} serde_arena;

typedef struct sd_cursor {
    sd_region_list *rlane;
    sd_slab_list *slane;
    sd_region_ref rspan;
    sd_chunk_ref cspan;
} sd_cursor;

serde_arena *serde_open(const char *file_path, int flags);
serde_arena *serde_open_anon(void);
bool serde_format(serde_arena *arena, int backing_fd, size_t first_bytes);
void serde_sync(serde_arena *arena);
void serde_close(serde_arena *arena);
int serde_fd(serde_arena *arena);
void serde_set_root(serde_arena *arena, void *mem, int slot_ix);
void *serde_get_root(serde_arena *arena, int slot_ix);
void *serde_alloc(serde_arena *arena, size_t nbytes) __attribute__((malloc));
void *serde_calloc(serde_arena *arena, size_t count, size_t nbytes) __attribute__((malloc));
void *serde_alloc_on(serde_arena *arena, size_t nbytes, int lane) __attribute__((malloc));
void *serde_calloc_on(serde_arena *arena, size_t count, size_t nbytes, int lane)
    __attribute__((malloc));
void *serde_realloc(serde_arena *arena, void *cell, size_t want_bytes);
void *serde_realloc_on(serde_arena *arena, void *cell, size_t want_bytes, int lane,
                       size_t hint_old);
void serde_free(void *cell);
void serde_free_fast(void *cell);
size_t serde_usable_size(void *cell);
size_t serde_usable_size_fast(void *cell);

static inline __attribute__((always_inline, flatten)) serde_arena *serde_arena_of(void *cell) {
    return (serde_arena *)((uintptr_t)cell & ~(SD_ARENA_SIZE - 1));
}
static inline __attribute__((always_inline, flatten)) sd_ref serde_ref(void *cell) {
    return cell ? (sd_ref)((uintptr_t)cell & (SD_ARENA_SIZE - 1)) : 0;
}
static inline __attribute__((always_inline, flatten)) void *serde_deref(void *in_arena, sd_ref r) {
    if (r == 0 || in_arena == NULL)
        return NULL;
    return (void *)((uintptr_t)serde_arena_of(in_arena) + r);
}

#if defined(__AVX2__)
#define SERDE_HAVE_COLUMN 1
static inline size_t serde_col_words(size_t rows, int bits) { return (rows >> 6) * (size_t)bits; }
void serde_col_pack(const uint32_t *codes, uint64_t *packed, size_t rows, int bits);
void serde_col_unpack(const uint64_t *packed, uint32_t *codes, size_t rows, int bits);
void serde_col_scan_range(const uint64_t *packed, uint64_t *result, size_t rows, int bits,
                          uint32_t lo, uint32_t hi);
static inline sd_ref serde_col_build(serde_arena *a, const uint32_t *codes, size_t rows, int bits) {
    uint64_t *packed = (uint64_t *)serde_alloc(a, serde_col_words(rows, bits) * sizeof(uint64_t));
    if (!packed)
        return 0;
    serde_col_pack(codes, packed, rows, bits);
    return serde_ref(packed);
}
#else
#define SERDE_HAVE_COLUMN 0
#endif

#endif

#ifdef SERDE_IMPLEMENTATION
#ifndef SERDE_IMPL_ONCE
#define SERDE_IMPL_ONCE

static inline __attribute__((always_inline, flatten)) bool
bytes_equal(const void *x1, const void *x2, size_t count) {
    uintptr_t a1, a2;
    a1 = (uintptr_t)x1;
    a2 = (uintptr_t)x2;

#ifdef __SSE4_1__
    for (; count >= 16; count -= 16, a1 += 16, a2 += 16) {
        __m128i w1 = _mm_lddqu_si128((void *)a1);
        __m128i w2 = _mm_lddqu_si128((void *)a2);
        __m128i cmpv = _mm_cmpeq_epi64(w1, w2);
        int ok = _mm_movemask_pd((__m128d)cmpv);
        if (ok != 0x3)
            return false;
    }
#elif defined(__SSE2__)
    for (; count >= 16; count -= 16, a1 += 16, a2 += 16) {
        __m128i w1 = _mm_loadu_si128((void *)a1);
        __m128i w2 = _mm_loadu_si128((void *)a2);
        __m128i cmpv = _mm_cmpeq_epi32(w1, w2);
        int ok = _mm_movemask_epi8(cmpv);
        if (ok != 0xFFFF)
            return false;
    }
#else
    for (; count >= 8; count -= 8, a1 += 8, a2 += 8) {
        uint64_t *w1 = (uint64_t *)a1;
        uint64_t *w2 = (uint64_t *)a2;
        if (*w1 != *w2)
            return false;
    }
#endif

    uint64_t *q1, *q2, *qe1, *qe2;
    uint32_t *h1, *h2, *he1, *he2;
    uint16_t *s1x, *s2x;
    uint8_t *b1, *b2;

    switch (count) {
    case 15:
        q1 = (uint64_t *)a1;
        q2 = (uint64_t *)a2;
        qe1 = (uint64_t *)(a1 + 7);
        qe2 = (uint64_t *)(a2 + 7);
        return *q1 == *q2 && *qe1 == *qe2;
    case 14:
        q1 = (uint64_t *)a1;
        q2 = (uint64_t *)a2;
        qe1 = (uint64_t *)(a1 + 6);
        qe2 = (uint64_t *)(a2 + 6);
        return *q1 == *q2 && *qe1 == *qe2;
    case 13:
        q1 = (uint64_t *)a1;
        q2 = (uint64_t *)a2;
        qe1 = (uint64_t *)(a1 + 5);
        qe2 = (uint64_t *)(a2 + 5);
        return *q1 == *q2 && *qe1 == *qe2;
    case 12:
        q1 = (uint64_t *)a1;
        q2 = (uint64_t *)a2;
        qe1 = (uint64_t *)(a1 + 4);
        qe2 = (uint64_t *)(a2 + 4);
        return *q1 == *q2 && *qe1 == *qe2;
    case 11:
        q1 = (uint64_t *)a1;
        q2 = (uint64_t *)a2;
        qe1 = (uint64_t *)(a1 + 3);
        qe2 = (uint64_t *)(a2 + 3);
        return *q1 == *q2 && *qe1 == *qe2;
    case 10:
        q1 = (uint64_t *)a1;
        q2 = (uint64_t *)a2;
        qe1 = (uint64_t *)(a1 + 2);
        qe2 = (uint64_t *)(a2 + 2);
        return *q1 == *q2 && *qe1 == *qe2;
    case 9:
        q1 = (uint64_t *)a1;
        q2 = (uint64_t *)a2;
        qe1 = (uint64_t *)(a1 + 1);
        qe2 = (uint64_t *)(a2 + 1);
        return *q1 == *q2 && *qe1 == *qe2;
    case 8:
        q1 = (uint64_t *)a1;
        q2 = (uint64_t *)a2;
        return *q1 == *q2;
    case 7:
        h1 = (uint32_t *)a1;
        h2 = (uint32_t *)a2;
        he1 = (uint32_t *)(a1 + 3);
        he2 = (uint32_t *)(a2 + 3);
        return *h1 == *h2 && *he1 == *he2;
    case 6:
        h1 = (uint32_t *)a1;
        h2 = (uint32_t *)a2;
        he1 = (uint32_t *)(a1 + 2);
        he2 = (uint32_t *)(a2 + 2);
        return *h1 == *h2 && *he1 == *he2;
    case 5:
        h1 = (uint32_t *)a1;
        h2 = (uint32_t *)a2;
        he1 = (uint32_t *)(a1 + 1);
        he2 = (uint32_t *)(a2 + 1);
        return *h1 == *h2 && *he1 == *he2;
    case 4:
        h1 = (uint32_t *)a1;
        h2 = (uint32_t *)a2;
        return *h1 == *h2;
    case 3:
        s1x = (uint16_t *)a1;
        s2x = (uint16_t *)a2;
        b1 = (uint8_t *)(a1 + 2);
        b2 = (uint8_t *)(a2 + 2);
        return *s1x == *s2x && *b1 == *b2;
    case 2:
        s1x = (uint16_t *)a1;
        s2x = (uint16_t *)a2;
        return *s1x == *s2x;
    case 1:
        b1 = (uint8_t *)a1;
        b2 = (uint8_t *)a2;
        return *b1 == *b2;
    default:
        return true;
    }
}

static inline __attribute__((always_inline, flatten)) int first_one_run(uint32_t x, int n) {
    int s;
    while (n > 1) {
        s = n >> 1;
        x = x & (x >> s);
        n = n - s;
    }
    if (x == 0)
        return -1;
    return __builtin_ctz(x);
}

static inline __attribute__((always_inline, flatten)) int first_zero_run(uint32_t x, int n) {
    return first_one_run(~x, n);
}

static inline __attribute__((always_inline, flatten)) int first_one_run64(uint64_t x, int n) {
    int s;
    while (n > 1) {
        s = n >> 1;
        x = x & (x >> s);
        n = n - s;
    }
    if (x == 0)
        return -1;
    return __builtin_ctzl(x);
}

static inline __attribute__((always_inline, flatten)) int first_zero_run64(uint64_t x, int n) {
    return first_one_run64(~x, n);
}

static inline __attribute__((always_inline, flatten)) bool sd_latch_share(sd_a16 *latch_p) {
    int16_t lv = atomic_load_explicit(latch_p, memory_order_relaxed);
    do {
        if (lv < 0)
            return false;
    } while (!atomic_compare_exchange_weak_explicit(latch_p, &lv, lv + 1, memory_order_acquire,
                                                    memory_order_relaxed));
    return true;
}

static inline __attribute__((always_inline, flatten)) void sd_latch_unshare(sd_a16 *latch_p) {
    atomic_fetch_sub_explicit(latch_p, 1, memory_order_release);
}

static inline __attribute__((always_inline, flatten)) bool sd_latch_reserve(sd_a16 *latch_p) {
    return atomic_fetch_or_explicit(latch_p, 1 << 15, memory_order_acq_rel) > 0;
}

static inline __attribute__((always_inline, flatten)) bool sd_latch_reserved(sd_a16 *latch_p) {
    return atomic_load_explicit(latch_p, memory_order_acquire) < 0;
}

static inline __attribute__((always_inline, flatten)) bool sd_latch_share_reserve(sd_a16 *latch_p) {
    int16_t v = atomic_load_explicit(latch_p, memory_order_relaxed);
    do {
        if (v < 0)
            return false;
    } while (!atomic_compare_exchange_weak_explicit(latch_p, &v, (int16_t)((v + 1) | (1 << 15)),
                                                    memory_order_acquire, memory_order_relaxed));
    return true;
}

static inline __attribute__((always_inline, flatten)) void sd_latch_enter(sd_a16 *latch_p) {
    while (atomic_load_explicit(latch_p, memory_order_relaxed) > INT16_MIN + 1)
        ;
    atomic_fetch_sub_explicit(latch_p, 1, memory_order_acq_rel);
}

static inline __attribute__((always_inline, flatten)) void sd_latch_leave(sd_a16 *latch_p) {
    atomic_store_explicit(latch_p, 1, memory_order_release);
}

static inline __attribute__((always_inline, flatten)) void sd_latch_release(sd_a16 *latch_p) {
    atomic_store_explicit(latch_p, 0, memory_order_release);
}

bool lane_serve_cell(sd_cursor *cur, sd_tag tag, void **cell)
    __attribute__((visibility("internal")));

bool arena_serve_chunk(sd_cursor *cur, sd_tag tag, unsigned int page_need, bool reserve_head)
    __attribute__((visibility("internal")));

sd_step slab_take_cell(sd_cursor *cur, void **cell) __attribute__((visibility("internal")));

sd_step region_take_chunk(sd_cursor *cur, unsigned int page_need, bool reserve_head)
    __attribute__((visibility("internal")));

sd_step region_take_slab(sd_cursor *cur, unsigned int page_need, bool reserve_head)
    __attribute__((visibility("internal")));

bool arena_take_region(serde_arena *arena, sd_cursor *cur) __attribute__((visibility("internal")));

bool arena_take_giant(serde_arena *arena, sd_cursor *cur, unsigned int region_need)
    __attribute__((visibility("internal")));

bool arena_take_small_giant(serde_arena *arena, sd_cursor *cur, unsigned int region_need)
    __attribute__((visibility("internal")));

bool arena_take_large_giant(serde_arena *arena, sd_cursor *cur, unsigned int region_need)
    __attribute__((visibility("internal")));

void slab_drop_cell(sd_slab *slab, void *cell) __attribute__((visibility("internal")));

void region_drop_chunk(sd_region *region, sd_chunk_ref cspan)
    __attribute__((visibility("internal")));

void arena_drop_region(sd_region_ref rspan) __attribute__((visibility("internal")));

void serde_free_fast(void *cell) __attribute__((visibility("default")));

static inline __attribute__((always_inline, flatten)) void slab_list_push(sd_slab_list *slab_lanes,
                                                                          sd_slab *slab) {
    serde_arena *arena = serde_arena_of(slab_lanes);
    sd_ref slab_off = serde_ref(slab);
    sd_ref *slot = &slab_lanes->slab;

    while (*slot && *slot < slab_off) {

        sd_slab *walk = (sd_slab *)serde_deref(arena, *slot);
        slot = &walk->chain;
    }

    if (*slot != slab_off) {

        if (*slot > slab_off)
            slab->chain = *slot;
        else
            slab->chain = 0;

        *slot = slab_off;
    }

    atomic_store_explicit(&slab->listed, SD_SPAN_LISTED, memory_order_release);
}

static inline __attribute__((always_inline, flatten)) void slab_list_drop(sd_slab_list *slab_lanes,
                                                                          sd_slab *slab) {
    serde_arena *arena = serde_arena_of(slab_lanes);
    sd_ref slab_off = serde_ref(slab);
    sd_ref *slot = &slab_lanes->slab;

    while (*slot && *slot != slab_off) {
        sd_slab *walk = (sd_slab *)serde_deref(arena, *slot);
        slot = &walk->chain;
    }

    if (*slot) {

        *slot = slab->chain;
        slab->chain = 0;
    }

    atomic_store_explicit(&slab->listed, SD_SPAN_UNLISTED, memory_order_release);
}

static inline __attribute__((always_inline, flatten)) void
region_list_push(sd_region_list *region_lane, sd_region *region) {
    serde_arena *arena = serde_arena_of(region_lane);
    sd_ref region_off = serde_ref(region);
    sd_ref *slot = &region_lane->region;

    while (*slot && *slot < region_off) {
        sd_region *walk = (sd_region *)serde_deref(arena, *slot);
        slot = &walk->chain;
    }

    if (*slot != region_off) {
        if (*slot > region_off)
            region->chain = *slot;
        else
            region->chain = 0;

        *slot = region_off;
    }

    atomic_store_explicit(&region->listed, SD_SPAN_LISTED, memory_order_release);
}

static inline __attribute__((always_inline, flatten)) void
region_list_drop(sd_region_list *region_lane, sd_region *region) {
    serde_arena *arena = serde_arena_of(region_lane);
    sd_ref region_off = serde_ref(region);
    sd_ref *slot = &region_lane->region;

    while (*slot && *slot != region_off) {
        sd_region *walk = (sd_region *)serde_deref(arena, *slot);
        slot = &walk->chain;
    }

    if (*slot) {
        *slot = region->chain;
        region->chain = 0;
    }

    atomic_store_explicit(&region->listed, SD_SPAN_UNLISTED, memory_order_release);
}

void region_setup(sd_region *region, sd_tag tag) __attribute__((visibility("internal")));

void slab_setup(sd_slab *slab, sd_tag tag, unsigned int page_need)
    __attribute__((visibility("internal")));

void arena_blank_marks(serde_arena *arena, sd_au64 *used_mark, sd_au64 *head_mark)
    __attribute__((visibility("internal")));

void region_blank_marks(sd_region *region, sd_au64 *used_mark, sd_au64 *head_mark)
    __attribute__((visibility("internal")));

void slab_blank_marks(sd_slab *slab, sd_au64 *marks) __attribute__((visibility("internal")));

static inline __attribute__((always_inline, flatten)) sd_region *region_holder_of(void *cell) {
    serde_arena *arena;
    uintptr_t arena_base, cell_addr;

    arena = serde_arena_of(cell);
    arena_base = (uintptr_t)arena;
    cell_addr = (uintptr_t)cell;

    if (cell_addr - arena_base < SD_REGION_SIZE)
        return &arena->region;

    return (sd_region *)(cell_addr & ~(SD_REGION_SIZE - 1));
}

static inline __attribute__((always_inline, flatten)) uintptr_t
region_base_of(sd_region_ref header) {
    return header.word_addr & ~(SD_REGION_SIZE - 1);
}

static inline __attribute__((always_inline, flatten)) uintptr_t chunk_base_of(sd_chunk_ref header) {
    return header.word_addr & ~(SD_PAGE_SIZE - 1);
}

sd_region_ref region_span_at(void *cell) __attribute__((visibility("internal")));

sd_chunk_ref region_chunk_at(sd_region *region, void *cell) __attribute__((visibility("internal")));

sd_slab_list *slab_lane_of(sd_slab *slab) __attribute__((visibility("internal")));

sd_region_list *region_lane_of(sd_region *region) __attribute__((visibility("internal")));

void region_setup(sd_region *region, sd_tag tag) {
    region->tag = tag;
    region->latch = 0;

    region->chain = 0;

    region_blank_marks(region, region->used_mark, region->head_mark);
}

void slab_setup(sd_slab *slab, sd_tag tag, unsigned int page_need) {
    uintptr_t slab_base;
    unsigned int cell_bytes, cell_used, mark_words, pad_cells, head_cells;
    size_t span_bytes, mark_reach;
    uint64_t *marks;

    span_bytes = (size_t)page_need * SD_PAGE_SIZE;
    slab_base = chunk_base_of(slab);
    cell_bytes = tag.slab_any.cell_bytes < 16 ? 16 : tag.slab_any.cell_bytes;
    cell_used = span_bytes / cell_bytes;
    mark_words = SD_CEILDIV(cell_used, 64);
    mark_reach = (size_t)mark_words * 64UL * cell_bytes;
    pad_cells = SD_CEILDIV(mark_reach - span_bytes, cell_bytes);
    head_cells = SD_CEILDIV(
        (uintptr_t)slab - slab_base + sizeof(sd_slab) + mark_words * sizeof(sd_au64), cell_bytes);
    slab->tag = tag;
    slab->mark_words = mark_words;
    slab->mark_head = head_cells;
    slab->mark_pad = pad_cells;
    slab->scan_hint = 0;
    slab->latch = 0;
    slab->cell_used = 0;

    slab->chain = 0;

    marks = (uint64_t *)((uintptr_t)slab + sizeof(sd_slab));
    slab_blank_marks(slab, (sd_au64 *)marks);
}

void arena_blank_marks(serde_arena *arena, sd_au64 *used_mark, sd_au64 *head_mark) {

    (void)head_mark;

    SD_ASSERT(arena->region_count > 0, "serde: arena reports zero regions");

    size_t hi_page = arena->region_count - 1;
    size_t hi_word = hi_page / 64;
    size_t hi_bit = hi_page % 64;

    for (size_t word_ix = 0; word_ix < hi_word; word_ix++) {
        atomic_store_explicit(&used_mark[word_ix], ~0ULL, memory_order_relaxed);
    }

    if (hi_word < SD_REGION_WORDS) {

        uint64_t mk = (hi_bit == 63) ? ~0ULL : ((1ULL << (hi_bit + 1)) - 1);
        atomic_store_explicit(&used_mark[hi_word], mk, memory_order_relaxed);
    }
}

void region_blank_marks(sd_region *region, sd_au64 *used_mark, sd_au64 *head_mark) {
    serde_arena *arena;
    arena = serde_arena_of(region);

    memset((uint64_t *)used_mark, 0, 8 * sizeof(uint64_t));
    memset((uint64_t *)head_mark, 0, 8 * sizeof(uint64_t));

    int hdr_bytes, hdr_pages, hdr_word, hdr_bit;
    if (region == &arena->region) {

        hdr_bytes = (int)sizeof(serde_arena);
    } else {

        hdr_bytes = (int)sizeof(sd_region);
    }

    hdr_pages = SD_CEILDIV(hdr_bytes, SD_PAGE_SIZE);
    hdr_word = hdr_pages / 64;
    hdr_bit = hdr_pages % 64;
    memset((uint64_t *)used_mark, 0xff, hdr_word * sizeof(uint64_t));
    if (hdr_bit)
        used_mark[hdr_word] |= (1ULL << hdr_bit) - 1;
}
void slab_blank_marks(sd_slab *slab, sd_au64 *marks) {
    int head_word, head_bit;
    head_word = slab->mark_head / 64;
    head_bit = slab->mark_head % 64;

    memset((uint64_t *)marks, 0, slab->mark_words * sizeof(uint64_t));
    if (head_word)
        memset((uint64_t *)marks, 0xff, head_word * sizeof(uint64_t));
    if (head_bit)
        marks[head_word] |= (1ULL << head_bit) - 1;

    if (slab->mark_pad)
        marks[slab->mark_words - 1] |= ~((1ULL << (64 - slab->mark_pad)) - 1);
}

sd_region_ref region_span_at(void *cell) {
    serde_arena *arena;
    sd_region_ref rspan_p;
    uintptr_t arena_base, cell_addr, off, off_region, off_word, off_bit;
    uint64_t marks, mk, masked_w;
    sd_au64 *used_mark;
    sd_au64 *head_mark;
    int lead0;

    arena = serde_arena_of(cell);
    while (!sd_latch_share(&arena->latch))
        ;

    used_mark = serde_deref(arena, arena->used_mark_ref);
    head_mark = serde_deref(arena, arena->head_mark_ref);
    (void)used_mark;
    arena_base = (uintptr_t)arena;
    cell_addr = (uintptr_t)cell;
    off = cell_addr - arena_base;

    off_region = off >> SD_REGION_BITS;
    off_word = off_region / 64;
    off_bit = off_region % 64;
    SD_ASSERT(off_word < SD_REGION_WORDS, "serde: ref %p past region bitmap (word %" PRIuPTR, cell,
              off_word);

    SD_ASSERT(atomic_load_explicit(&used_mark[off_word], memory_order_relaxed) & (1ULL << off_bit),
              "Addr %p located in %" PRIuPTR " serde_arena bitmap "
              "has value 0x%" PRIx64 "\n",
              cell, off_word, used_mark[off_word]);

    if (off < SD_REGION_SIZE) {
        rspan_p.region = &arena->region;
        sd_latch_unshare(&arena->latch);
        return rspan_p;
    }

    if (atomic_load_explicit(&head_mark[off_word], memory_order_relaxed) & (1ULL << off_bit)) {
        rspan_p.word_addr = arena_base + off_region * SD_REGION_SIZE;
        sd_latch_unshare(&arena->latch);
        return rspan_p;
    }

    mk = (1ULL << off_bit) - 1;
    masked_w = atomic_load_explicit(&head_mark[off_word], memory_order_relaxed) & mk;
    lead0 = __builtin_clzl(masked_w);

    if (masked_w && lead0) {
        rspan_p.word_addr = arena_base + (off_word * 64 + (64 - lead0 - 1)) * SD_REGION_SIZE;

        sd_latch_unshare(&arena->latch);
        return rspan_p;
    }

    size_t word_ix = off_word;
    while (word_ix > 0) {
        word_ix--;
        marks = atomic_load_explicit(&head_mark[word_ix], memory_order_relaxed);
        if (marks) {
            lead0 = __builtin_clzl(marks);
            rspan_p.word_addr = arena_base + (word_ix * 64 + (64 - lead0 - 1)) * SD_REGION_SIZE;

            sd_latch_unshare(&arena->latch);
            return rspan_p;
        }
    }
    SD_ASSERT(0, "serde: no region owns %p", cell);
    sd_latch_unshare(&arena->latch);
    rspan_p.word_addr = 0;
    return rspan_p;
}

sd_chunk_ref region_chunk_at(sd_region *region, void *cell) {
    sd_chunk_ref cspan_p;
    uintptr_t region_base, cell_addr, off, off_page, off_word, off_bit;
    uint64_t marks, mk, masked_w;
    int lead0;
    while (!sd_latch_share(&region->latch))
        ;
    region_base = region_base_of(region);
    cell_addr = (uintptr_t)cell;
    off = cell_addr - region_base;
    SD_ASSERT(off >= 0 && off < SD_REGION_SIZE, "serde: %p not inside region %p\n", cell, region);

    off_page = off >> SD_PAGE_BITS;
    off_word = off_page / 64;
    off_bit = off_page % 64;

    SD_ASSERT(atomic_load_explicit(&region->used_mark[off_word], memory_order_relaxed) &
                  (1ULL << off_bit),
              "Addr %p located in %" PRIuPTR " rgn-bitmap word "
              "has value %" PRIx64 "\n",
              cell, off_word, region->used_mark[off_word]);

    if (atomic_load_explicit(&region->head_mark[off_word], memory_order_relaxed) &
        (1ULL << off_bit)) {
        cspan_p.word_addr = region_base + off_page * SD_PAGE_SIZE;
        goto chunk_shift;
    }

    mk = (1ULL << off_bit) - 1;
    masked_w = atomic_load_explicit(&region->head_mark[off_word], memory_order_relaxed) & mk;
    lead0 = __builtin_clzl(masked_w);

    if (masked_w && lead0) {
        cspan_p.word_addr = region_base + (off_word * 64 + (64 - lead0 - 1)) * SD_PAGE_SIZE;
        goto chunk_shift;
    }

    for (int word_ix = (int)off_word - 1; word_ix >= 0; word_ix--) {
        marks = atomic_load_explicit(&region->head_mark[word_ix], memory_order_relaxed);
        if (marks) {
            lead0 = __builtin_clzl(marks);
            cspan_p.word_addr = region_base + (word_ix * 64 + (64 - lead0 - 1)) * SD_PAGE_SIZE;
            goto chunk_shift;
        }
    }
    SD_ASSERT(0, "serde: no chunk owns %p (region %p)\n", cell, region);
    sd_latch_unshare(&region->latch);
    cspan_p.word_addr = 0;
    return cspan_p;

chunk_shift:
    if (cspan_p.word_addr == region_base)
        cspan_p.word_addr = region_base + sizeof(sd_region);

    sd_latch_unshare(&region->latch);

    return cspan_p;
}

sd_slab_list *slab_lane_of(sd_slab *slab) {
    serde_arena *arena;
    int th, klass, cell_bytes, slab_id;

    arena = serde_arena_of(slab);
    switch (slab->tag.any.kind) {
    case SD_K_SLAB:
        klass = SD_CEILDIV(slab->tag.slab.cell_bytes, 16) - 1;
        th = slab->tag.slab.sd_tls_lane;
        return &arena->freelists.slab_lanes[klass][th];
    case SD_K_WIDESLAB:
        cell_bytes = slab->tag.wideslab.cell_bytes;
        slab_id = cell_bytes == 512 ? 0 : cell_bytes == 1024 ? 1 : 2;
        return &arena->freelists.wideslab_lanes[slab_id];
    default:
        SD_ASSERT(0, "serde: corrupt slab tag %d\n", slab->tag.any.kind);
        return NULL;
    }
}

sd_region_list *region_lane_of(sd_region *region) {
    serde_arena *arena;
    arena = serde_arena_of(region);
    SD_ASSERT(region->tag.any.kind == SD_K_REGION, "serde: corrupt span tag %d\n",
              region->tag.any.kind);
    return &arena->freelists.region_lane;
}

static __thread int sd_tls_lane = -1;
static sd_au32 sd_lane_rr = 0;

extern void arena_grow_to(serde_arena *arena, size_t nbytes);

void *serde_alloc(serde_arena *arena, size_t nbytes) {
    if (sd_tls_lane == -1)
        sd_tls_lane = atomic_fetch_add_explicit(&sd_lane_rr, 1, memory_order_acquire) % 16;

    return serde_alloc_on(arena, nbytes, sd_tls_lane);
}

void *serde_calloc(serde_arena *arena, size_t count, size_t nbytes) {
    if (sd_tls_lane == -1)
        sd_tls_lane = atomic_fetch_add_explicit(&sd_lane_rr, 1, memory_order_acquire) % 16;

    return serde_calloc_on(arena, count, nbytes, sd_tls_lane);
}

void *serde_alloc_on(serde_arena *arena, size_t nbytes, int lane) {
    sd_cursor cur;
    void *cell;
    sd_tag tag;
    unsigned int klass, page_total;

    SD_ASSERT(nbytes > 0, "serde: requested size is zero");

    lane %= 16;

    cur.rlane = &arena->freelists.region_lane;
    if (nbytes <= 256) {

        klass = (nbytes - 1) / 16;

        tag.slab.kind = SD_K_SLAB;
        tag.slab.cell_bytes = (klass + 1) * 16;
        tag.slab.sd_tls_lane = lane;
        cur.slane = &arena->freelists.slab_lanes[klass][lane];
        if (lane_serve_cell(&cur, tag, &cell))
            return cell;
        else
            return NULL;
    } else if (nbytes <= 2048) {
        tag.wideslab.kind = SD_K_WIDESLAB;
        if (nbytes <= 512) {
            cur.slane = &arena->freelists.wideslab_lanes[0];
            tag.wideslab.cell_bytes = 512;
        } else if (nbytes <= 1024) {
            cur.slane = &arena->freelists.wideslab_lanes[1];
            tag.wideslab.cell_bytes = 1024;
        } else {
            cur.slane = &arena->freelists.wideslab_lanes[2];
            tag.wideslab.cell_bytes = 2048;
        }
        if (lane_serve_cell(&cur, tag, &cell))
            return cell;
        else
            return NULL;
    } else if (nbytes <= (SD_REGION_SIZE - SD_PAGE_SIZE)) {
        page_total = SD_CEILDIV(nbytes + sizeof(sd_tag), SD_PAGE_SIZE);
        tag.region.kind = SD_K_REGION;
        if (!arena_serve_chunk(&cur, tag, page_total, true))
            return NULL;
        cur.cspan.tag->word = 0;
        cur.cspan.tag->chunk.kind = SD_K_CHUNK;
        cur.cspan.tag->chunk.page_run = page_total;
        cell = (void *)(cur.cspan.word_addr + sizeof(sd_tag));
        return cell;
    } else {
        page_total = SD_CEILDIV(nbytes + sizeof(sd_tag), SD_REGION_SIZE);
        if (!arena_take_giant(arena, &cur, page_total))
            return NULL;
        cur.rspan.tag->word = 0;
        cur.rspan.tag->giant.kind = SD_K_GIANT;
        cur.rspan.tag->giant.region_run = page_total;
        cell = (void *)(cur.rspan.word_addr + sizeof(sd_tag));
        return cell;
    }
}

void *serde_calloc_on(serde_arena *arena, size_t count, size_t nbytes, int lane) {
    void *cell;
    size_t tot_bytes;

    tot_bytes = count * nbytes;
    cell = serde_alloc_on(arena, tot_bytes, lane);

    if (cell)
        memset(cell, 0x00, tot_bytes);

    return cell;
}

static inline __attribute__((always_inline, flatten)) bool
try_claim_run(sd_au64 *marks, size_t mark_len, size_t lo_word, size_t lo_bit, size_t need_pages,
              sd_a16 *latch) {
    size_t word_ix = lo_word;
    size_t bit_ix = lo_bit;
    size_t left_pages = need_pages;
    size_t scan_pages;
    uint64_t mk;

    while (bit_ix >= 64) {
        word_ix++;
        bit_ix -= 64;
    }
    if (word_ix >= mark_len)
        return false;

    {
        size_t pword = word_ix, pbit = bit_ix, pl = left_pages;

        scan_pages = (64 - pbit) < pl ? (64 - pbit) : pl;
        mk = (scan_pages == 64) ? ~0ULL : ((1ULL << scan_pages) - 1) << pbit;
        if (atomic_load_explicit(&marks[pword], memory_order_relaxed) & mk)
            return false;
        pl -= scan_pages;
        pword++;

        while (pl > 0) {
            if (pword >= mark_len)
                return false;
            scan_pages = pl < 64 ? pl : 64;
            mk = (scan_pages == 64) ? ~0ULL : (1ULL << scan_pages) - 1;
            if (atomic_load_explicit(&marks[pword], memory_order_relaxed) & mk)
                return false;
            pl -= scan_pages;
            pword++;
        }
    }

    while (!sd_latch_share_reserve(latch))
        ;
    sd_latch_enter(latch);

    word_ix = lo_word;
    bit_ix = lo_bit;
    left_pages = need_pages;
    while (bit_ix >= 64) {
        word_ix++;
        bit_ix -= 64;
    }

    if (word_ix >= mark_len)
        goto bail;

    scan_pages = (64 - bit_ix) < left_pages ? (64 - bit_ix) : left_pages;

    if (scan_pages == 64) {
        mk = ~0ULL;
    } else {
        mk = ((1ULL << scan_pages) - 1) << bit_ix;
    }
    if (atomic_load_explicit(&marks[word_ix], memory_order_relaxed) & mk) {
        goto bail;
    }
    left_pages -= scan_pages;
    word_ix++;
    bit_ix = 0;

    while (left_pages > 0) {
        if (word_ix >= mark_len)
            goto bail;
        scan_pages = left_pages < 64 ? left_pages : 64;
        mk = (scan_pages == 64) ? ~0ULL : (1ULL << scan_pages) - 1;

        if (atomic_load_explicit(&marks[word_ix], memory_order_relaxed) & mk) {
            goto bail;
        }
        left_pages -= scan_pages;
        word_ix++;
    }

    word_ix = lo_word;
    bit_ix = lo_bit;
    left_pages = need_pages;

    while (bit_ix >= 64) {
        word_ix++;
        bit_ix -= 64;
    }

    scan_pages = (64 - bit_ix) < left_pages ? (64 - bit_ix) : left_pages;

    if (scan_pages == 64) {
        mk = ~0ULL;
    } else {
        mk = ((1ULL << scan_pages) - 1) << bit_ix;
    }
    atomic_fetch_or_explicit(&marks[word_ix], mk, memory_order_release);
    left_pages -= scan_pages;
    word_ix++;
    bit_ix = 0;

    while (left_pages > 0) {
        scan_pages = left_pages < 64 ? left_pages : 64;
        mk = (scan_pages == 64) ? ~0ULL : (1ULL << scan_pages) - 1;
        atomic_fetch_or_explicit(&marks[word_ix], mk, memory_order_release);
        left_pages -= scan_pages;
        word_ix++;
    }

    sd_latch_release(latch);
    return true;

bail:
    sd_latch_release(latch);
    return false;
}

int serde_fd(serde_arena *arena) { return arena ? arena->backing_fd : -1; }

static void clear_giant_marks(serde_arena *arena, uintptr_t region_base2, unsigned int page_run) {
    sd_au64 *used_mark = serde_deref(arena, arena->used_mark_ref);
    sd_au64 *head_mark = serde_deref(arena, arena->head_mark_ref);
    uintptr_t arena_base = (uintptr_t)arena;

    uintptr_t off = region_base2 - arena_base;
    unsigned int off_region = off / SD_REGION_SIZE;
    unsigned int word_ix = off_region / 64;
    unsigned int bit_ix = off_region % 64;
    unsigned int left_pages = page_run;

    atomic_fetch_and_explicit(&head_mark[word_ix], ~(1ULL << bit_ix), memory_order_relaxed);

    while (left_pages > 0) {
        unsigned int wbits = (64 - bit_ix) < left_pages ? (64 - bit_ix) : left_pages;
        uint64_t mk;
        if (wbits == 64) {
            mk = ~0ULL;
        } else {
            mk = ((1ULL << wbits) - 1) << bit_ix;
        }
        atomic_fetch_and_explicit(&used_mark[word_ix], ~mk, memory_order_release);
        left_pages -= wbits;
        word_ix++;
        bit_ix = 0;
    }
}

static void *try_regrow_giant(serde_arena *arena, sd_region_ref rspan, unsigned int prior_pages,
                              unsigned int grown_pages) {
#ifndef __linux__

    return NULL;
#else

    if (arena->map_flags != MAP_PRIVATE) {
        return NULL;
    }

    uintptr_t arena_base = (uintptr_t)arena;
    uintptr_t old_region_base = rspan.word_addr;
    size_t prev_bytes = (size_t)prior_pages * SD_REGION_SIZE;
    size_t want_bytes = (size_t)grown_pages * SD_REGION_SIZE;
    sd_cursor cur;

    while (!sd_latch_share_reserve(&arena->latch))
        ;
    sd_latch_enter(&arena->latch);

    bool hit = false;
    if (grown_pages <= 32) {

        sd_au64 *used_mark = serde_deref(arena, arena->used_mark_ref);
        sd_au64 *head_mark = serde_deref(arena, arena->head_mark_ref);

        for (size_t word_ix = 0; word_ix < SD_REGION_WORDS; word_ix++) {
            uint64_t old_w = atomic_load_explicit(&used_mark[word_ix], memory_order_relaxed);
            if (old_w == ~0ULL)
                continue;

            uint64_t probe_w = word_ix == 0 ? old_w | 0x01 : old_w;
            int bit_ix = first_zero_run64(probe_w, grown_pages);
            if (bit_ix == -1)
                continue;

            uint64_t new_w = old_w | (((1ULL << grown_pages) - 1) << bit_ix);
            if (atomic_compare_exchange_strong_explicit(&used_mark[word_ix], &old_w, new_w,
                                                        memory_order_acquire,
                                                        memory_order_relaxed)) {
                atomic_fetch_or_explicit(&head_mark[word_ix], 1ULL << bit_ix, memory_order_relaxed);
                cur.rspan.word_addr = arena_base + (64 * word_ix + bit_ix) * SD_REGION_SIZE;
                hit = true;
                break;
            }
        }
    }

    if (!hit) {

        hit = arena_take_large_giant(arena, &cur, grown_pages);
    }

    if (!hit) {
        sd_latch_release(&arena->latch);
        return NULL;
    }

    uintptr_t new_region_base = cur.rspan.word_addr;

    size_t need_bytes = new_region_base - arena_base + want_bytes;
    arena_grow_to(arena, need_bytes);

    void *ok = mremap((void *)old_region_base, prev_bytes, want_bytes,
                      MREMAP_MAYMOVE | MREMAP_FIXED, (void *)new_region_base);

    if (ok == MAP_FAILED) {

        clear_giant_marks(arena, new_region_base, grown_pages);
        sd_latch_release(&arena->latch);
        return NULL;
    }

    clear_giant_marks(arena, old_region_base, prior_pages);

    sd_latch_release(&arena->latch);

    sd_tag *new_tag = (sd_tag *)new_region_base;
    new_tag->word = 0;
    new_tag->giant.kind = SD_K_GIANT;
    new_tag->giant.region_run = grown_pages;

    return (void *)(new_region_base + sizeof(sd_tag));
#endif
}

void *serde_realloc_on(serde_arena *arena, void *cell, size_t want_bytes, int lane,
                       size_t hint_old) {

    if (cell == NULL) {
        return serde_alloc_on(arena, want_bytes, lane);
    }

    if (want_bytes == 0) {
        serde_free(cell);
        return NULL;
    }

    size_t old_cap = hint_old;
    if (old_cap == 0) {
        old_cap = serde_usable_size(cell);
        if (old_cap == 0) {
            SD_ASSERT(0, "serde: realloc of bad pointer %p", cell);
            return NULL;
        }
    }

    if (want_bytes <= old_cap) {

        return cell;
    }

    sd_region_ref rspan;
    {
        uintptr_t av = (uintptr_t)cell - (uintptr_t)arena;
        uintptr_t av_region = av >> SD_REGION_BITS;
        if (av < SD_REGION_SIZE) {
            rspan.region = &arena->region;
        } else {
            sd_au64 *hmarks = serde_deref(arena, arena->head_mark_ref);
            uintptr_t wi = av_region / 64;
            uintptr_t bi2 = av_region % 64;
            uint64_t rmark_w = atomic_load_explicit(&hmarks[wi], memory_order_relaxed);

            if (rmark_w & (1ULL << bi2)) {
                rspan.word_addr = (uintptr_t)arena + av_region * SD_REGION_SIZE;
            } else {
                uint64_t rmask = (1ULL << bi2) - 1;
                uint64_t rmasked = rmark_w & rmask;
                if (rmasked) {
                    int lead = __builtin_clzl(rmasked);
                    rspan.word_addr = (uintptr_t)arena + (wi * 64 + (63 - lead)) * SD_REGION_SIZE;
                } else {
                    size_t wx = wi;
                    rspan.word_addr = 0;
                    while (wx > 0) {
                        wx--;
                        uint64_t mw = atomic_load_explicit(&hmarks[wx], memory_order_relaxed);
                        if (mw) {
                            int lead = __builtin_clzl(mw);
                            rspan.word_addr =
                                (uintptr_t)arena + (wx * 64 + (63 - lead)) * SD_REGION_SIZE;
                            break;
                        }
                    }
                    if (!rspan.word_addr) {
                        SD_ASSERT(0, "serde: realloc cannot locate region for %p", cell);
                        return NULL;
                    }
                }
            }
        }
    }

    if (rspan.tag->any.kind == SD_K_GIANT) {

        unsigned int prior_pages = rspan.tag->giant.region_run;
        unsigned int grown_pages = SD_CEILDIV(want_bytes + sizeof(sd_tag), SD_REGION_SIZE);
        sd_au64 *used_mark = serde_deref(arena, arena->used_mark_ref);

        unsigned int need_pages = grown_pages - prior_pages;

        uintptr_t region_base2 = rspan.word_addr;
        uintptr_t off = region_base2 - (uintptr_t)arena;
        unsigned int off_region = off / SD_REGION_SIZE;
        unsigned int off_word = off_region / 64;
        unsigned int off_bit = off_region % 64;

        if (try_claim_run(used_mark, SD_REGION_WORDS, off_word, off_bit + prior_pages, need_pages,
                          &arena->latch)) {

            size_t need_bytes = region_base2 - (uintptr_t)arena + (grown_pages * SD_REGION_SIZE);
            arena_grow_to(arena, need_bytes);

            rspan.tag->giant.region_run = grown_pages;
            return cell;
        }

        void *remap_at = try_regrow_giant(arena, rspan, prior_pages, grown_pages);
        if (remap_at != NULL) {
            return remap_at;
        }

    } else {

        sd_chunk_ref cspan;
        {
            uintptr_t rbase = rspan.word_addr & ~(SD_REGION_SIZE - 1);
            uintptr_t cw = (uintptr_t)cell - rbase;
            uintptr_t c_page2 = cw >> SD_PAGE_BITS;
            uintptr_t c_word2 = c_page2 / 64;
            uintptr_t c_bit2 = c_page2 % 64;
            sd_region *rg = rspan.region;
            uint64_t cmark_w = atomic_load_explicit(&rg->head_mark[c_word2], memory_order_relaxed);
            if (cmark_w & (1ULL << c_bit2)) {
                cspan.word_addr = rbase + c_page2 * SD_PAGE_SIZE;
            } else {
                uint64_t cmask = (1ULL << c_bit2) - 1;
                uint64_t cmasked = cmark_w & cmask;
                if (cmasked) {
                    int lead = __builtin_clzl(cmasked);
                    cspan.word_addr = rbase + (c_word2 * 64 + (63 - lead)) * SD_PAGE_SIZE;
                } else {
                    cspan.word_addr = 0;
                    for (int i = (int)c_word2 - 1; i >= 0; i--) {
                        uint64_t mw = atomic_load_explicit(&rg->head_mark[i], memory_order_relaxed);
                        if (mw) {
                            int lead = __builtin_clzl(mw);
                            cspan.word_addr = rbase + (i * 64 + (63 - lead)) * SD_PAGE_SIZE;
                            break;
                        }
                    }
                    if (!cspan.word_addr) {
                        SD_ASSERT(0, "serde: realloc cannot locate chunk for %p", cell);
                        return NULL;
                    }
                }
            }
            if (cspan.word_addr == rbase)
                cspan.word_addr = rbase + sizeof(sd_region);
        }

        if (cspan.tag->any.kind == SD_K_CHUNK) {

            unsigned int prior_pages = cspan.tag->chunk.page_run;
            unsigned int grown_pages = SD_CEILDIV(want_bytes + sizeof(sd_tag), SD_PAGE_SIZE);

            unsigned int need_pages = grown_pages - prior_pages;

            uintptr_t chunk_base = chunk_base_of(cspan);
            uintptr_t region_base = region_base_of(rspan.region);
            uintptr_t off = chunk_base - region_base;
            unsigned int off_page = off / SD_PAGE_SIZE;
            unsigned int off_word = off_page / 64;
            unsigned int off_bit = off_page % 64;

            if (try_claim_run(rspan.region->used_mark, 8, off_word, off_bit + prior_pages,
                              need_pages, &rspan.region->latch)) {

                cspan.tag->chunk.page_run = grown_pages;
                return cell;
            }

        } else if (cspan.tag->any.kind == SD_K_SLAB || cspan.tag->any.kind == SD_K_WIDESLAB) {
        }
    }

    void *new_cell = serde_alloc_on(arena, want_bytes, lane);
    if (new_cell == NULL) {
        return NULL;
    }

    memcpy(new_cell, cell, old_cap);

    serde_free_fast(cell);

    return new_cell;
}

void *serde_realloc(serde_arena *arena, void *cell, size_t want_bytes) {
    if (sd_tls_lane == -1)
        sd_tls_lane = atomic_fetch_add_explicit(&sd_lane_rr, 1, memory_order_acquire) % 16;

    return serde_realloc_on(arena, cell, want_bytes, sd_tls_lane, 0);
}

static inline void slab_list_defer(sd_slab_list *q, sd_slab *nd) {
    sd_ref nd_off = serde_ref(nd);
    sd_ref prev_top;

    _Atomic(sd_ref) *top_slot = (_Atomic(sd_ref) *)&q->pending;
    prev_top = atomic_load_explicit(top_slot, memory_order_relaxed);

    do {
        nd->chain = prev_top;
    } while (!atomic_compare_exchange_weak_explicit(top_slot, &prev_top, nd_off,
                                                    memory_order_release, memory_order_relaxed));
}

static inline void slab_list_reclaim(sd_slab_list *q) {
    serde_arena *arena = serde_arena_of(q);
    _Atomic(sd_ref) *top_slot = (_Atomic(sd_ref) *)&q->pending;

    sd_ref top_off = atomic_exchange_explicit(top_slot, 0, memory_order_acquire);

    sd_slab *top = (sd_slab *)serde_deref(arena, top_off);
    while (top) {
        sd_ref next_off = top->chain;

        slab_list_push(q, top);
        top = (sd_slab *)serde_deref(arena, next_off);
    }
}

static inline void region_list_defer(sd_region_list *q, sd_region *nd) {
    sd_ref nd_off = serde_ref(nd);
    sd_ref prev_top;

    _Atomic(sd_ref) *top_slot = (_Atomic(sd_ref) *)&q->pending;

    prev_top = atomic_load_explicit(top_slot, memory_order_relaxed);
    do {
        nd->chain = prev_top;
    } while (!atomic_compare_exchange_weak_explicit(top_slot, &prev_top, nd_off,
                                                    memory_order_release, memory_order_relaxed));
}

static inline void region_list_reclaim(sd_region_list *q) {
    serde_arena *arena = serde_arena_of(q);
    _Atomic(sd_ref) *top_slot = (_Atomic(sd_ref) *)&q->pending;

    sd_ref top_off = atomic_exchange_explicit(top_slot, 0, memory_order_acquire);

    sd_region *top = (sd_region *)serde_deref(arena, top_off);
    while (top) {
        sd_ref next_off = top->chain;

        region_list_push(q, top);
        top = (sd_region *)serde_deref(arena, next_off);
    }
}

bool lane_serve_cell(sd_cursor *cur, sd_tag slab_tag, void **cell) {
    unsigned int page_need;
    sd_tag region_tag = {};
    int tries = 0;

    serde_arena *arena = serde_arena_of(cur->slane);

again:
    if (tries++ > SD_DISPATCH_TRIES)
        return false;

    while (!sd_latch_share(&cur->slane->latch))
        ;

    _Atomic(sd_ref) *head_slot = (_Atomic(sd_ref) *)&cur->slane->slab;

    sd_ref walk_off = atomic_load_explicit(head_slot, memory_order_acquire);

    sd_slab *walk2 = (sd_slab *)serde_deref(arena, walk_off);

    while (walk2) {
        cur->cspan.slab = walk2;

        sd_ref next_off = walk2->chain;
        sd_slab *next_nd = (sd_slab *)serde_deref(arena, next_off);

        switch (slab_take_cell(cur, cell)) {
        case SD_DONE:
            sd_latch_unshare(&cur->slane->latch);
            return true;
        case SD_REDO:
            sd_latch_unshare(&cur->slane->latch);
            goto again;
        case SD_NEXT:

            walk_off = next_off;
            walk2 = next_nd;
            break;
        }
    }

    if (!sd_latch_reserve(&cur->slane->latch)) {
        sd_latch_unshare(&cur->slane->latch);
        goto again;
    }
    sd_latch_enter(&cur->slane->latch);

    slab_list_reclaim(cur->slane);

    switch (slab_tag.any.kind) {
    case SD_K_SLAB:
    case SD_K_WIDESLAB:
        region_tag.region.kind = SD_K_REGION;
        break;
    default:
        SD_ASSERT(false, "Unknown slab kind %d", slab_tag.any.kind);
    }
    if (slab_tag.slab_any.cell_bytes <= 32)
        page_need = 1;
    else if (slab_tag.slab_any.cell_bytes <= 64)
        page_need = 4;
    else if (slab_tag.slab_any.cell_bytes <= 256)
        page_need = 8;
    else if (slab_tag.slab_any.cell_bytes < 1024)
        page_need = 16;
    else
        page_need = 32;
    if (!arena_serve_chunk(cur, region_tag, page_need, false)) {
        sd_latch_release(&cur->slane->latch);
        return false;
    }
    slab_setup(cur->cspan.slab, slab_tag, page_need);
    slab_list_push(cur->slane, cur->cspan.slab);
    sd_latch_release(&cur->slane->latch);
    goto again;
}

bool arena_serve_chunk(sd_cursor *cur, sd_tag tag, unsigned int page_need, bool reserve_head) {
    int tries = 0;
    serde_arena *arena = serde_arena_of(cur->rlane);

again:
    if (tries++ > SD_DISPATCH_TRIES)
        return false;

    while (!sd_latch_share(&cur->rlane->latch))
        ;

    _Atomic(sd_ref) *head_slot = (_Atomic(sd_ref) *)&cur->rlane->region;

    sd_ref walk_off = atomic_load_explicit(head_slot, memory_order_acquire);

    sd_region *walk2 = (sd_region *)serde_deref(arena, walk_off);

    while (walk2) {
        cur->rspan.region = walk2;

        sd_ref next_off = walk2->chain;
        sd_region *next_nd = (sd_region *)serde_deref(arena, next_off);

        switch (region_take_chunk(cur, page_need, reserve_head)) {
        case SD_DONE:
            sd_latch_unshare(&cur->rlane->latch);
            return true;
        case SD_REDO:
            sd_latch_unshare(&cur->rlane->latch);
            goto again;
        case SD_NEXT:

            walk_off = next_off;
            walk2 = next_nd;
            break;
        }
    }

    if (!sd_latch_reserve(&cur->rlane->latch)) {
        sd_latch_unshare(&cur->rlane->latch);
        goto again;
    }
    sd_latch_enter(&cur->rlane->latch);

    region_list_reclaim(cur->rlane);

    if (!arena_take_region(serde_arena_of(cur->rlane), cur)) {
        sd_latch_release(&cur->rlane->latch);
        return false;
    }
    region_setup(cur->rspan.region, tag);
    region_list_push(cur->rlane, cur->rspan.region);
    sd_latch_release(&cur->rlane->latch);
    goto again;
}

sd_step slab_take_cell(sd_cursor *cur, void **cell) {
    sd_slab *slab;
    uintptr_t slab_base;
    uint64_t old_w, new_w;
    uint16_t cell_bytes, used_old, used_new, cell_cap;
    sd_au64 *marks;
    uint64_t word_ix, bit_ix;

    slab = cur->cspan.slab;
    slab_base = chunk_base_of(slab);

    if (!sd_latch_share(&slab->latch))
        return SD_NEXT;

    cell_cap = slab->mark_words * 64L - slab->mark_head - slab->mark_pad;
    used_old = atomic_load_explicit(&slab->cell_used, memory_order_relaxed);
    do {
        if (used_old >= cell_cap)
            goto slab_is_full;
        used_new = used_old + 1;
    } while (!atomic_compare_exchange_weak_explicit(&slab->cell_used, &used_old, used_new,
                                                    memory_order_acq_rel, memory_order_relaxed));

    cell_bytes = slab->tag.slab_any.cell_bytes;
    marks = (sd_au64 *)((uintptr_t)slab + sizeof(sd_slab));
    word_ix = slab->scan_hint;

    while (1) {
        old_w = atomic_load_explicit(&marks[word_ix], memory_order_relaxed);
        do {
            if (old_w == ~0ULL)
                goto skip_word;
            new_w = (old_w + 1);
            bit_ix = __builtin_ctzl(new_w);
            new_w |= old_w;
        } while (!atomic_compare_exchange_strong_explicit(
            &marks[word_ix], &old_w, new_w, memory_order_relaxed, memory_order_relaxed));
        *cell = (void *)(slab_base + (word_ix * 64UL + bit_ix) * cell_bytes);
        slab->scan_hint = word_ix;
        sd_latch_unshare(&slab->latch);
        return SD_DONE;

    skip_word:
        word_ix++;
        word_ix %= slab->mark_words;
    }

slab_is_full:
    if (atomic_load_explicit(&slab->listed, memory_order_acquire) == SD_SPAN_UNLISTED) {
        sd_latch_unshare(&slab->latch);
        return SD_REDO;
    }
    if (!sd_latch_reserve(&cur->slane->latch)) {
        sd_latch_unshare(&slab->latch);
        return SD_REDO;
    }
    sd_latch_enter(&cur->slane->latch);
    slab_list_drop(cur->slane, slab);
    sd_latch_leave(&cur->slane->latch);
    sd_latch_unshare(&slab->latch);
    return SD_REDO;
}

sd_step region_take_chunk(sd_cursor *cur, unsigned int page_need, bool reserve_head) {
    sd_region *region;
    uintptr_t region_base;
    int run_word, run_bit, word_ix, pages_left2;
    uint64_t *used_mark;

    if (page_need <= 32)
        return region_take_slab(cur, page_need, reserve_head);

    region = cur->rspan.region;
    region_base = region_base_of(region);

    if (!sd_latch_share_reserve(&region->latch))
        return SD_NEXT;
    sd_latch_enter(&region->latch);

    used_mark = (uint64_t *)(region->used_mark);
    run_word = 0;

    while (1) {
        if (run_word >= 8)
            goto maybe_full;
        if (used_mark[run_word] & (1ULL << 63)) {
            run_word++;
            continue;
        }
        pages_left2 = page_need;
        word_ix = run_word;
        run_bit = used_mark[run_word] == 0 ? 0 : 64 - __builtin_clzl(used_mark[run_word]);
        if (reserve_head && run_word == 0 && run_bit == 0)
            run_bit = 1;
        if (pages_left2 <= 64 - run_bit)
            goto hit;

        pages_left2 -= 64 - run_bit;
        word_ix++;

        while (1) {
            if (word_ix >= 8)
                goto maybe_full;
            if (pages_left2 > 64) {
                if (used_mark[word_ix] != 0ULL) {
                    run_word++;
                    break;
                }
                word_ix++;
                pages_left2 -= 64;
                continue;
            } else if (pages_left2 == 64) {
                if (atomic_load_explicit(&used_mark[word_ix], memory_order_relaxed) != 0ULL) {
                    run_word = word_ix + 1;
                    break;
                }
                word_ix++;
                pages_left2 -= 64;
                goto hit;
            } else if (pages_left2 <
                       (used_mark[word_ix] == 0 ? 64 : __builtin_ctzl(used_mark[word_ix]))) {
                goto hit;
            }
            run_word++;
            break;
        }
    }

hit:
    cur->cspan.word_addr = region_base + (64 * run_word + run_bit) * SD_PAGE_SIZE;
    if (run_word == 0 && run_bit == 0)
        cur->cspan.word_addr += sizeof(sd_region);
    atomic_fetch_or_explicit(&region->head_mark[run_word], 1ULL << run_bit, memory_order_release);
    if (word_ix == run_word) {
        used_mark[run_word] |=
            (pages_left2 >= 64 ? ~0ULL : (((1ULL << pages_left2) - 1) << run_bit));
    } else {
        used_mark[run_word] |= ~((1ULL << run_bit) - 1) | (1ULL << run_bit);
        if (pages_left2)
            used_mark[word_ix] |= (1ULL << pages_left2) - 1;
        for (int wx = run_word + 1; wx < word_ix; wx++)
            used_mark[wx] = ~0ULL;
    }

    sd_latch_release(&region->latch);
    return SD_DONE;

maybe_full:
    for (int wx = 0; wx < 8; wx++) {
        if (used_mark[wx] != ~0ULL) {
            sd_latch_release(&region->latch);
            return SD_NEXT;
        }
    }
    while (1) {
        if (atomic_load_explicit(&region->listed, memory_order_acquire) == SD_SPAN_UNLISTED) {
            sd_latch_release(&region->latch);
            return SD_REDO;
        }
        if (sd_latch_reserve(&cur->rlane->latch))
            break;
    }
    sd_latch_enter(&cur->rlane->latch);
    if (atomic_load_explicit(&region->listed, memory_order_acquire) == SD_SPAN_UNLISTED) {
        sd_latch_leave(&cur->rlane->latch);
        sd_latch_release(&region->latch);
        return SD_REDO;
    }
    region_list_drop(cur->rlane, region);
    sd_latch_leave(&cur->rlane->latch);
    sd_latch_release(&region->latch);
    return SD_REDO;
}

sd_step region_take_slab(sd_cursor *cur, unsigned int page_need, bool reserve_head) {
    sd_region *region;
    uintptr_t region_base;
    int run_word, run_bit;
    uint64_t old_w, new_w;

    region = cur->rspan.region;
    region_base = region_base_of(region);

    if (!sd_latch_share_reserve(&region->latch))
        return SD_NEXT;
    sd_latch_enter(&region->latch);

    for (run_word = 0; run_word < 8; run_word++) {
        old_w = atomic_load_explicit(&region->used_mark[run_word], memory_order_relaxed);

        if (old_w == ~0ULL)
            continue;

        new_w = run_word == 0 && reserve_head ? old_w | 0x01 : old_w;
        run_bit = first_zero_run64(new_w, page_need);
        if (run_bit == -1)
            continue;

        new_w = old_w | ((1ULL << page_need) - 1) << run_bit;

        atomic_store_explicit(&region->used_mark[run_word], new_w, memory_order_relaxed);
        atomic_fetch_or_explicit(&region->head_mark[run_word], 1ULL << run_bit,
                                 memory_order_relaxed);

        sd_latch_release(&region->latch);

        cur->cspan.word_addr = region_base + (64 * run_word + run_bit) * SD_PAGE_SIZE;
        if (run_word == 0 && run_bit == 0)
            cur->cspan.word_addr += sizeof(sd_region);
        return SD_DONE;
    }

    for (int word_ix = 0; word_ix < 8; word_ix++) {
        if (atomic_load_explicit(&region->used_mark[word_ix], memory_order_relaxed) != ~0ULL) {

            sd_latch_release(&region->latch);
            return SD_NEXT;
        }
    }

    if (!sd_latch_reserve(&cur->rlane->latch)) {
        sd_latch_release(&region->latch);
        return SD_REDO;
    }
    sd_latch_enter(&cur->rlane->latch);
    region_list_drop(cur->rlane, region);
    sd_latch_leave(&cur->rlane->latch);
    sd_latch_release(&region->latch);
    return SD_REDO;
}

bool arena_take_region(serde_arena *arena, sd_cursor *cur) {
    int cmp_res, rg_bit;
    uint64_t old_w, new_w;
    uintptr_t arena_base;
    size_t region_edge;
    sd_au64 *used_mark;
    sd_au64 *head_mark;

    uint64_t blank_marks[SD_REGION_WORDS];

    arena_base = (uintptr_t)arena;
    used_mark = serde_deref(arena, arena->used_mark_ref);
    head_mark = serde_deref(arena, arena->head_mark_ref);

again:
    while (!sd_latch_share(&arena->latch))
        ;

    for (size_t rg_word = 0; rg_word < SD_REGION_WORDS; rg_word++) {
        old_w = atomic_load_explicit(&used_mark[rg_word], memory_order_relaxed);
        while (1) {
            if (old_w == ~0ULL)
                break;
            new_w = old_w + 1;
            rg_bit = __builtin_ctzl(new_w);
            new_w |= old_w;

            if (atomic_compare_exchange_weak_explicit(&used_mark[rg_word], &old_w, new_w,
                                                      memory_order_acquire, memory_order_relaxed)) {
                atomic_fetch_or_explicit(&head_mark[rg_word], 1ULL << rg_bit, memory_order_relaxed);
                sd_latch_unshare(&arena->latch);

                if (rg_word == 0 && rg_bit == 0) {
                    cur->rspan.region = &arena->region;
                } else {
                    cur->rspan.word_addr = arena_base + (64 * rg_word + rg_bit) * SD_REGION_SIZE;
                    region_edge = cur->rspan.word_addr + SD_REGION_SIZE - arena_base;
                    arena_grow_to(arena, region_edge);
                }
                return true;
            }
        }
    }

    if (!sd_latch_reserve(&arena->latch)) {
        sd_latch_unshare(&arena->latch);
        goto again;
    }

    sd_latch_enter(&arena->latch);
    memset(blank_marks, 0xFF, sizeof(blank_marks));
    cmp_res = memcmp(used_mark, blank_marks, sizeof(blank_marks));
    sd_latch_release(&arena->latch);

    if (cmp_res)
        goto again;

    return false;
}

bool arena_take_giant(serde_arena *arena, sd_cursor *cur, unsigned int region_need) {
    bool ok;
    uintptr_t arena_base;
    size_t region_edge;

    arena_base = (uintptr_t)arena;

    if (region_need <= 32) {
        ok = arena_take_small_giant(arena, cur, region_need);
    } else {
        while (!sd_latch_share_reserve(&arena->latch))
            ;
        ok = arena_take_large_giant(arena, cur, region_need);
        sd_latch_release(&arena->latch);
    }

    if (!ok)
        return ok;

    region_edge = cur->rspan.word_addr + SD_REGION_SIZE * region_need - arena_base;

    arena_grow_to(arena, region_edge);
    return ok;
}

bool arena_take_small_giant(serde_arena *arena, sd_cursor *cur, unsigned int region_need) {
    int gt_bit;
    uint64_t old_w, new_w;
    uintptr_t arena_base;
    bool ok;
    sd_au64 *used_mark;
    sd_au64 *head_mark;

    arena_base = (uintptr_t)arena;
    used_mark = serde_deref(arena, arena->used_mark_ref);
    head_mark = serde_deref(arena, arena->head_mark_ref);

again:
    while (!sd_latch_share(&arena->latch))
        ;

    for (size_t gt_word = 0; gt_word < SD_REGION_WORDS; gt_word++) {
        old_w = atomic_load_explicit(&used_mark[gt_word], memory_order_relaxed);
        while (1) {
            if (old_w == ~0ULL)
                break;
            new_w = gt_word == 0 ? old_w | 0x01 : old_w;
            gt_bit = first_zero_run64(new_w, region_need);
            if (gt_bit == -1)
                break;
            new_w = old_w | ((1ULL << region_need) - 1) << gt_bit;

            if (atomic_compare_exchange_weak_explicit(&used_mark[gt_word], &old_w, new_w,
                                                      memory_order_acquire, memory_order_relaxed)) {
                atomic_fetch_or_explicit(&head_mark[gt_word], 1ULL << gt_bit, memory_order_relaxed);
                sd_latch_unshare(&arena->latch);
                cur->rspan.word_addr = arena_base + (64 * gt_word + gt_bit) * SD_REGION_SIZE;
                return true;
            }
        }
    }

    if (!sd_latch_reserve(&arena->latch)) {
        sd_latch_unshare(&arena->latch);
        goto again;
    }

    sd_latch_enter(&arena->latch);
    ok = arena_take_large_giant(arena, cur, region_need);
    sd_latch_release(&arena->latch);
    return ok;
}

bool arena_take_large_giant(serde_arena *arena, sd_cursor *cur, unsigned int region_need) {
    size_t word_head, word_it, region_need2, bit_head;
    uintptr_t arena_base;
    sd_au64 *used_mark;
    sd_au64 *head_mark;

    arena_base = (uintptr_t)arena;
    used_mark = serde_deref(arena, arena->used_mark_ref);
    head_mark = serde_deref(arena, arena->head_mark_ref);
    word_head = 0;

    while (1) {
        if (word_head >= SD_REGION_WORDS)
            return false;
        if (word_head < SD_REGION_WORDS &&
            (atomic_load_explicit(&used_mark[word_head], memory_order_relaxed) & (1ULL << 63))) {
            word_head++;
            continue;
        }

        region_need2 = region_need;
        word_it = word_head;
        bit_head = atomic_load_explicit(&used_mark[word_head], memory_order_relaxed) == 0ULL
                       ? 0
                       : 64 - __builtin_clzl(atomic_load_explicit(&used_mark[word_head],
                                                                  memory_order_relaxed));
        if (word_head == 0 && bit_head == 0)
            bit_head = 1;
        if (region_need2 <= 64 - bit_head)
            goto hit;

        region_need2 -= 64 - bit_head;
        word_it++;

        while (1) {
            if (word_it >= SD_REGION_WORDS)
                return false;
            if (region_need2 > 64) {
                if (atomic_load_explicit(&used_mark[word_it], memory_order_relaxed) != 0ULL) {
                    word_head = word_it + 1;
                    break;
                }
                word_it++;
                region_need2 -= 64;
                continue;
            } else if (region_need2 == 64) {

                if (word_it >= SD_REGION_WORDS ||
                    atomic_load_explicit(&used_mark[word_it], memory_order_relaxed) != 0ULL) {
                    word_head = word_it + 1;
                    break;
                }
                word_it++;
                region_need2 -= 64;
                goto hit;
            } else {

                if (region_need2 <=
                    (size_t)(atomic_load_explicit(&used_mark[word_it], memory_order_relaxed) == 0ULL
                                 ? 64
                                 : __builtin_ctzl(atomic_load_explicit(&used_mark[word_it],
                                                                       memory_order_relaxed)))) {
                    goto hit;
                }
                word_head = word_it + 1;
                break;
            }
        }
    }
hit:
    cur->rspan.word_addr = arena_base + (64 * word_head + bit_head) * SD_REGION_SIZE;
    atomic_fetch_or_explicit(&head_mark[word_head], 1ULL << bit_head, memory_order_release);
    if (word_it - word_head == 0) {

        atomic_fetch_or_explicit(&used_mark[word_head], ((1ULL << region_need) - 1) << bit_head,
                                 memory_order_release);
    } else {

        atomic_fetch_or_explicit(&used_mark[word_head], ~((1ULL << bit_head) - 1),
                                 memory_order_release);

        for (size_t word_ix = word_head + 1; word_ix < word_it; word_ix++)
            atomic_store_explicit(&used_mark[word_ix], ~0ULL, memory_order_release);

        if (region_need2 > 0 && word_it < SD_REGION_WORDS)
            atomic_fetch_or_explicit(&used_mark[word_it], (1ULL << region_need2) - 1,
                                     memory_order_release);
    }
    return true;
}

void serde_free(void *cell) {

    if (cell == NULL) {
        return;
    }

    sd_region_ref rspan;
    sd_chunk_ref cspan;

    rspan = region_span_at(cell);
    if (rspan.tag->any.kind == SD_K_GIANT) {
        arena_drop_region(rspan);
        return;
    }
    cspan = region_chunk_at(rspan.region, cell);
    if (cspan.tag->any.kind == SD_K_CHUNK) {
        region_drop_chunk(rspan.region, cspan);
        return;
    }
    slab_drop_cell(cspan.slab, cell);
}

void serde_free_fast(void *cell) {
    if (cell == NULL)
        return;

    serde_arena *arena = serde_arena_of(cell);
    uintptr_t arena_base = (uintptr_t)arena;
    uintptr_t cell_addr = (uintptr_t)cell;
    uintptr_t off = cell_addr - arena_base;

    uintptr_t off_region = off >> SD_REGION_BITS;
    uintptr_t off_word = off_region / 64;
    uintptr_t off_bit = off_region % 64;

    sd_region_ref rspan;

    if (off < SD_REGION_SIZE) {
        rspan.region = &arena->region;
    } else {
        sd_au64 *head_mark = serde_deref(arena, arena->head_mark_ref);

        uint64_t rmarks = atomic_load_explicit(&head_mark[off_word], memory_order_relaxed);
        if (rmarks & (1ULL << off_bit)) {
            rspan.word_addr = arena_base + off_region * SD_REGION_SIZE;
        } else {
            uint64_t rmask = (1ULL << off_bit) - 1;
            uint64_t masked_w = rmarks & rmask;
            if (masked_w) {
                int lead0 = __builtin_clzl(masked_w);
                rspan.word_addr = arena_base + (off_word * 64 + (63 - lead0)) * SD_REGION_SIZE;
            } else {
                size_t word_ix = off_word;
                rspan.word_addr = 0;
                while (word_ix > 0) {
                    word_ix--;
                    uint64_t marks =
                        atomic_load_explicit(&head_mark[word_ix], memory_order_relaxed);
                    if (marks) {
                        int lead0 = __builtin_clzl(marks);
                        rspan.word_addr =
                            arena_base + (word_ix * 64 + (63 - lead0)) * SD_REGION_SIZE;
                        break;
                    }
                }
                if (!rspan.word_addr) {
                    serde_free(cell);
                    return;
                }
            }
        }
    }

    if (rspan.tag->any.kind == SD_K_GIANT) {
        arena_drop_region(rspan);
        return;
    }

    sd_region *region = rspan.region;
    uintptr_t region_base = rspan.word_addr & ~(SD_REGION_SIZE - 1);
    uintptr_t c_off = cell_addr - region_base;
    uintptr_t c_page = c_off >> SD_PAGE_BITS;
    uintptr_t c_word = c_page / 64;
    uintptr_t c_bit = c_page % 64;

    sd_chunk_ref cspan;

    uint64_t cmarks = atomic_load_explicit(&region->head_mark[c_word], memory_order_relaxed);
    if (cmarks & (1ULL << c_bit)) {
        cspan.word_addr = region_base + c_page * SD_PAGE_SIZE;
    } else {
        uint64_t cmask = (1ULL << c_bit) - 1;
        uint64_t c_masked = cmarks & cmask;
        if (c_masked) {
            int lead = __builtin_clzl(c_masked);
            cspan.word_addr = region_base + (c_word * 64 + (63 - lead)) * SD_PAGE_SIZE;
        } else {
            cspan.word_addr = 0;
            for (int word_ix = (int)c_word - 1; word_ix >= 0; word_ix--) {
                uint64_t marks =
                    atomic_load_explicit(&region->head_mark[word_ix], memory_order_relaxed);
                if (marks) {
                    int lead = __builtin_clzl(marks);
                    cspan.word_addr = region_base + (word_ix * 64 + (63 - lead)) * SD_PAGE_SIZE;
                    break;
                }
            }
            if (!cspan.word_addr) {
                serde_free(cell);
                return;
            }
        }
    }
    if (cspan.word_addr == region_base)
        cspan.word_addr = region_base + sizeof(sd_region);

    if (cspan.tag->any.kind == SD_K_CHUNK) {
        region_drop_chunk(rspan.region, cspan);
        return;
    }
    slab_drop_cell(cspan.slab, cell);
}
size_t serde_usable_size(void *cell) {
    if (!cell)
        return 0;

    sd_region_ref rspan = region_span_at(cell);
    if (!rspan.tag)
        return 0;

    if (rspan.tag->any.kind == SD_K_GIANT) {

        return ((size_t)rspan.tag->giant.region_run * SD_REGION_SIZE) - sizeof(sd_tag);
    }

    sd_chunk_ref cspan = region_chunk_at(rspan.region, cell);
    if (!cspan.tag)
        return 0;

    switch (cspan.tag->any.kind) {
    case SD_K_SLAB:
    case SD_K_WIDESLAB:

        return cspan.tag->slab_any.cell_bytes;
    case SD_K_CHUNK:

        return ((size_t)cspan.tag->chunk.page_run * SD_PAGE_SIZE) - sizeof(sd_tag);
    default:

        return 0;
    }
}

size_t serde_usable_size_fast(void *cell) {
    if (!cell)
        return 0;

    serde_arena *arena = serde_arena_of(cell);
    uintptr_t arena_base = (uintptr_t)arena;
    uintptr_t cell_addr = (uintptr_t)cell;
    uintptr_t off = cell_addr - arena_base;

    uintptr_t off_region = off >> SD_REGION_BITS;
    uintptr_t off_word = off_region / 64;
    uintptr_t off_bit = off_region % 64;

    sd_region_ref rspan;

    if (off < SD_REGION_SIZE) {
        rspan.region = &arena->region;
    } else {
        sd_au64 *head_mark = serde_deref(arena, arena->head_mark_ref);

        uint64_t rmarks = atomic_load_explicit(&head_mark[off_word], memory_order_relaxed);
        if (rmarks & (1ULL << off_bit)) {
            rspan.word_addr = arena_base + off_region * SD_REGION_SIZE;
        } else {
            uint64_t rmask = (1ULL << off_bit) - 1;
            uint64_t masked_w = rmarks & rmask;
            if (masked_w) {
                int lead0 = __builtin_clzl(masked_w);
                rspan.word_addr = arena_base + (off_word * 64 + (63 - lead0)) * SD_REGION_SIZE;
            } else {
                size_t word_ix = off_word;
                rspan.word_addr = 0;
                while (word_ix > 0) {
                    word_ix--;
                    uint64_t marks =
                        atomic_load_explicit(&head_mark[word_ix], memory_order_relaxed);
                    if (marks) {
                        int lead0 = __builtin_clzl(marks);
                        rspan.word_addr =
                            arena_base + (word_ix * 64 + (63 - lead0)) * SD_REGION_SIZE;
                        break;
                    }
                }
                if (!rspan.word_addr)
                    return 0;
            }
        }
    }

    if (rspan.tag->any.kind == SD_K_GIANT) {
        return ((size_t)rspan.tag->giant.region_run * SD_REGION_SIZE) - sizeof(sd_tag);
    }

    sd_region *region = rspan.region;
    uintptr_t region_base = rspan.word_addr & ~(SD_REGION_SIZE - 1);
    uintptr_t c_off = cell_addr - region_base;
    uintptr_t c_page = c_off >> SD_PAGE_BITS;
    uintptr_t c_word = c_page / 64;
    uintptr_t c_bit = c_page % 64;

    sd_chunk_ref cspan;

    uint64_t cmarks = atomic_load_explicit(&region->head_mark[c_word], memory_order_relaxed);
    if (cmarks & (1ULL << c_bit)) {
        cspan.word_addr = region_base + c_page * SD_PAGE_SIZE;
    } else {
        uint64_t cmask = (1ULL << c_bit) - 1;
        uint64_t c_masked = cmarks & cmask;
        if (c_masked) {
            int lead = __builtin_clzl(c_masked);
            cspan.word_addr = region_base + (c_word * 64 + (63 - lead)) * SD_PAGE_SIZE;
        } else {
            cspan.word_addr = 0;
            for (int word_ix = (int)c_word - 1; word_ix >= 0; word_ix--) {
                uint64_t marks =
                    atomic_load_explicit(&region->head_mark[word_ix], memory_order_relaxed);
                if (marks) {
                    int lead = __builtin_clzl(marks);
                    cspan.word_addr = region_base + (word_ix * 64 + (63 - lead)) * SD_PAGE_SIZE;
                    break;
                }
            }
            if (!cspan.word_addr)
                return 0;
        }
    }
    if (cspan.word_addr == region_base)
        cspan.word_addr = region_base + sizeof(sd_region);

    switch (cspan.tag->any.kind) {
    case SD_K_SLAB:
    case SD_K_WIDESLAB:
        return cspan.tag->slab_any.cell_bytes;
    case SD_K_CHUNK:
        return ((size_t)cspan.tag->chunk.page_run * SD_PAGE_SIZE) - sizeof(sd_tag);
    default:
        return 0;
    }
}

void slab_drop_cell(sd_slab *slab, void *cell) {
    uint16_t cell_bytes, cell_cap;
    uintptr_t slab_base, cell_addr, off, off_cell_bytes, off_word, off_bit;
    uint64_t mk, old_w;
    sd_au64 *marks;
    sd_slab_list *slane;
    sd_region *region;

    while (!sd_latch_share(&slab->latch))
        ;
    slab_base = chunk_base_of(slab);
    cell_addr = (uintptr_t)cell;
    slane = slab_lane_of(slab);
    region = region_span_at(slab).region;
    marks = (sd_au64 *)((uintptr_t)slab + sizeof(sd_slab));

    off = cell_addr - slab_base;
    cell_bytes = slab->tag.slab_any.cell_bytes;
    off_cell_bytes = off / cell_bytes;
    SD_ASSERT(off_cell_bytes >= slab->mark_head, "serde: %p resolved onto a header bit\n", cell);
    SD_ASSERT(off_cell_bytes < slab->mark_words * 64UL - slab->mark_pad,
              "serde: %p resolved onto a padding bit\n", cell);
    off_word = off_cell_bytes / 64;
    off_bit = off_cell_bytes % 64;
    cell_cap = slab->mark_words * 64L - slab->mark_head - slab->mark_pad;

    mk = ~(1ULL << off_bit);
    old_w = atomic_fetch_and_explicit(&marks[off_word], mk, memory_order_release);
    (void)old_w;
    SD_ASSERT(old_w & (1ULL << off_bit), "serde: double free of %p\n", cell);
    if (atomic_fetch_sub_explicit(&slab->cell_used, 1, memory_order_acq_rel) == 1) {
        if (!sd_latch_reserve(&slab->latch)) {
            sd_latch_unshare(&slab->latch);
            return;
        }
        sd_latch_enter(&slab->latch);
        if (atomic_load_explicit(&slab->cell_used, memory_order_acquire) != 0) {
            sd_latch_release(&slab->latch);
            return;
        }
        while (1) {
            if (atomic_load_explicit(&slab->listed, memory_order_acquire) == SD_SPAN_UNLISTED) {
                sd_latch_release(&slab->latch);
                region_drop_chunk(region, slab);
                return;
            }
            if (sd_latch_share_reserve(&slane->latch))
                break;
        }
        sd_latch_enter(&slane->latch);
        if (atomic_load_explicit(&slab->listed, memory_order_acquire) == SD_SPAN_UNLISTED) {
            sd_latch_release(&slane->latch);
            sd_latch_unshare(&slab->latch);
            region_drop_chunk(region, slab);
            return;
        }
        slab_list_drop(slane, slab);
        sd_latch_release(&slane->latch);
        region_drop_chunk(region, slab);
        return;
    }

    while (1) {
        if (atomic_load_explicit(&slab->cell_used, memory_order_acquire) > (cell_cap * 3 / 4) ||
            atomic_load_explicit(&slab->listed, memory_order_acquire) == SD_SPAN_LISTED) {
            sd_latch_unshare(&slab->latch);
            return;
        }
        if (sd_latch_reserved(&slab->latch)) {
            sd_latch_unshare(&slab->latch);
            return;
        }
        if (!sd_latch_share_reserve(&slane->latch))
            continue;
        sd_latch_enter(&slane->latch);
        if (atomic_load_explicit(&slab->listed, memory_order_acquire) == SD_SPAN_LISTED) {
            sd_latch_release(&slane->latch);
            sd_latch_unshare(&slab->latch);
            return;
        }
        slab_list_push(slane, slab);
        sd_latch_release(&slane->latch);
        sd_latch_unshare(&slab->latch);
        return;
    }
}

void region_drop_chunk(sd_region *region, sd_chunk_ref cspan) {
    uintptr_t region_base, off, off_page, off_word, off_bit, page_n, cell_bytes, mark_words,
        mark_pad;
    sd_region_list *rlane;
    uint64_t mk, old_w;
    uint64_t used_mark[8], head_mark[8];

    region_base = region_base_of(region);
    rlane = region_lane_of(region);
    region_blank_marks(region, (sd_au64 *)used_mark, (sd_au64 *)head_mark);

    off = chunk_base_of(cspan) - region_base;
    off_page = off / SD_PAGE_SIZE;
    off_word = off_page / 64;
    off_bit = off_page % 64;

    switch (cspan.tag->any.kind) {
    case SD_K_SLAB:
    case SD_K_WIDESLAB:
        cell_bytes = cspan.tag->slab_any.cell_bytes;
        if (cell_bytes < 16)
            cell_bytes = 16;
        mark_words = cspan.slab->mark_words;
        mark_pad = cspan.slab->mark_pad;
        page_n = SD_CEILDIV(cell_bytes * (64 * mark_words - mark_pad), SD_PAGE_SIZE);
        break;
    case SD_K_CHUNK:
        page_n = cspan.tag->chunk.page_run;
        break;
    default:
        SD_ASSERT(0, "serde: corrupt chunk tag %d", cspan.tag->any.kind);
    }

    if (off_bit + page_n <= 64) {

        while (!sd_latch_share_reserve(&region->latch))
            ;
        sd_latch_enter(&region->latch);

        old_w = atomic_fetch_and_explicit(&region->head_mark[off_word], ~(1ULL << off_bit),
                                          memory_order_release);
        (void)old_w;
        SD_ASSERT((old_w & (1ULL << off_bit)) != 0, "header bit didn't match");
        if (page_n >= 64) {
            mk = 0ULL;
        } else {
            mk = ~(((1ULL << page_n) - 1) << off_bit);
        }
        atomic_fetch_and_explicit(&region->used_mark[off_word], mk, memory_order_release);

        if (memcmp(used_mark, region->used_mark, sizeof(used_mark)) == 0) {

            if (memcmp(used_mark, region->used_mark, sizeof(used_mark)) != 0) {
                sd_latch_release(&region->latch);
                return;
            }
            while (1) {
                if (atomic_load_explicit(&region->listed, memory_order_acquire) ==
                    SD_SPAN_UNLISTED) {
                    sd_latch_release(&region->latch);
                    arena_drop_region(region);
                    return;
                }
                if (sd_latch_share_reserve(&rlane->latch))
                    break;
            }
            sd_latch_enter(&rlane->latch);
            if (atomic_load_explicit(&region->listed, memory_order_acquire) == SD_SPAN_UNLISTED) {
                sd_latch_release(&rlane->latch);
                sd_latch_release(&region->latch);
                arena_drop_region(region);
                return;
            }
            region_list_drop(rlane, region);
            sd_latch_release(&rlane->latch);
            sd_latch_release(&region->latch);
            arena_drop_region(region);
            return;
        } else {
            while (1) {
                if (sd_latch_reserved(&region->latch)) {

                    sd_latch_release(&region->latch);
                    return;
                }
                if (atomic_load_explicit(&region->listed, memory_order_acquire) == SD_SPAN_LISTED) {

                    sd_latch_release(&region->latch);
                    return;
                }
                if (sd_latch_share_reserve(&rlane->latch))
                    break;
            }
            sd_latch_enter(&rlane->latch);
            if (atomic_load_explicit(&region->listed, memory_order_acquire) == SD_SPAN_LISTED) {
                sd_latch_release(&rlane->latch);

                sd_latch_release(&region->latch);
                return;
            }
            region_list_push(rlane, region);
            sd_latch_release(&rlane->latch);

            sd_latch_release(&region->latch);
        }
    } else {

        while (!sd_latch_share_reserve(&region->latch))
            ;
        sd_latch_enter(&region->latch);
        atomic_fetch_and_explicit(&region->head_mark[off_word], ~(1ULL << off_bit),
                                  memory_order_relaxed);
        atomic_fetch_and_explicit(&region->used_mark[off_word], (1ULL << off_bit) - 1,
                                  memory_order_relaxed);
        page_n -= (64 - off_bit);
        off_word++;
        while (page_n >= 64) {
            atomic_store_explicit(&region->used_mark[off_word++], 0ULL, memory_order_relaxed);
            page_n -= 64;
        }
        if (page_n) {
            atomic_fetch_and_explicit(&region->used_mark[off_word], ~((1ULL << page_n) - 1),
                                      memory_order_relaxed);
        }

        if (memcmp(used_mark, region->used_mark, sizeof(used_mark)) == 0) {
            while (1) {
                if (atomic_load_explicit(&region->listed, memory_order_acquire) ==
                    SD_SPAN_UNLISTED) {
                    sd_latch_release(&region->latch);
                    arena_drop_region(region);
                    return;
                }
                if (sd_latch_share_reserve(&rlane->latch))
                    break;
            }
            sd_latch_enter(&rlane->latch);
            if (atomic_load_explicit(&region->listed, memory_order_acquire) == SD_SPAN_UNLISTED) {
                sd_latch_release(&rlane->latch);
                sd_latch_release(&region->latch);
                arena_drop_region(region);
                return;
            }
            region_list_drop(rlane, region);
            sd_latch_release(&rlane->latch);
            sd_latch_release(&region->latch);
            arena_drop_region(region);
            return;
        } else {
            while (1) {
                if (atomic_load_explicit(&region->listed, memory_order_acquire) == SD_SPAN_LISTED) {
                    sd_latch_release(&region->latch);
                    return;
                }
                if (sd_latch_share_reserve(&rlane->latch))
                    break;
            }
            sd_latch_enter(&rlane->latch);
            if (atomic_load_explicit(&region->listed, memory_order_acquire) == SD_SPAN_LISTED) {
                sd_latch_release(&rlane->latch);
                sd_latch_release(&region->latch);
                return;
            }
            region_list_push(rlane, region);
            sd_latch_release(&rlane->latch);
            sd_latch_release(&region->latch);
        }
    }
}

void arena_drop_region(sd_region_ref rspan) {
    serde_arena *arena;
    sd_au64 *used_mark;
    sd_au64 *head_mark;
    uintptr_t arena_base, off, off_region, off_word, off_bit;
    uint32_t region_n;
    uint64_t mk;
    sd_tag_kind kind;

    arena = serde_arena_of(rspan.region);
    used_mark = serde_deref(arena, arena->used_mark_ref);
    head_mark = serde_deref(arena, arena->head_mark_ref);
    arena_base = (uintptr_t)arena;

    if (&arena->region == rspan.region) {
        off_word = off_bit = 0;
    } else {
        off = rspan.word_addr - arena_base;
        off_region = off / SD_REGION_SIZE;
        off_word = off_region / 64;
        off_bit = off_region % 64;
        SD_ASSERT(off_word < SD_REGION_WORDS, "serde: free index out of range (word %" PRIuPTR,
                  off_word);
    }

    kind = rspan.tag->any.kind;
    if (kind == SD_K_REGION || (kind == SD_K_GIANT && rspan.tag->giant.region_run == 1)) {
        while (!sd_latch_share(&arena->latch))
            ;
        atomic_fetch_and_explicit(&head_mark[off_word], ~(1ULL << off_bit), memory_order_relaxed);
        atomic_fetch_and_explicit(&used_mark[off_word], ~(1ULL << off_bit), memory_order_release);
        sd_latch_unshare(&arena->latch);
        return;
    }

    region_n = rspan.tag->giant.region_run;
    if (off_bit + region_n <= 64) {
        while (!sd_latch_share(&arena->latch))
            ;
        if (region_n >= 64) {
            mk = 0ULL;
        } else {
            mk = ~(((1ULL << region_n) - 1) << off_bit);
        }
        atomic_fetch_and_explicit(&head_mark[off_word], ~(1ULL << off_bit), memory_order_relaxed);
        atomic_fetch_and_explicit(&used_mark[off_word], mk, memory_order_release);
        sd_latch_unshare(&arena->latch);
        return;
    }

    while (!sd_latch_share_reserve(&arena->latch))
        ;
    sd_latch_enter(&arena->latch);
    atomic_fetch_and_explicit(&head_mark[off_word], ~(1ULL << off_bit), memory_order_relaxed);
    atomic_fetch_and_explicit(&used_mark[off_word], (1ULL << off_bit) - 1, memory_order_relaxed);
    region_n -= (64 - off_bit);
    off_word++;
    while (region_n >= 64) {
        SD_ASSERT(off_word < SD_REGION_WORDS, "serde: free index out of range (mid)");
        atomic_store_explicit(&used_mark[off_word++], 0ULL, memory_order_relaxed);
        region_n -= 64;
    }
    if (region_n) {
        SD_ASSERT(off_word < SD_REGION_WORDS, "serde: free index out of range (tail)");
        atomic_fetch_and_explicit(&used_mark[off_word], ~((1ULL << region_n) - 1),
                                  memory_order_relaxed);
    }
    sd_latch_release(&arena->latch);
    return;
}

pthread_mutex_t sd_open_mutex = PTHREAD_MUTEX_INITIALIZER;

static off_t fd_byte_size(int backing_fd);
static inline __attribute__((always_inline, flatten)) serde_arena *arena_map_open(int backing_fd,
                                                                                  int flags);

serde_arena *serde_open(const char *file_path, int flags) {
    serde_arena *arena;
    int backing_fd;
    pthread_mutex_lock(&sd_open_mutex);

    backing_fd = open(file_path, flags, S_IRUSR | S_IWUSR);
    if (backing_fd == -1) {

        pthread_mutex_unlock(&sd_open_mutex);
        return NULL;
    }

    arena = arena_map_open(backing_fd, flags);
    if (!arena) {
        close(backing_fd);
    }
    pthread_mutex_unlock(&sd_open_mutex);
    return arena;
}

serde_arena *serde_open_anon() {
    serde_arena *arena;
    int backing_fd;
    pthread_mutex_lock(&sd_open_mutex);

    FILE *tmp_fp;
    tmp_fp = tmpfile();
    if (tmp_fp == NULL) {
        printf("serde: tmpfile() for anonymous arena failed");

        pthread_mutex_unlock(&sd_open_mutex);
        return NULL;
    }
    backing_fd = fileno(tmp_fp);

    arena = arena_map_open(backing_fd, O_RDWR);
    if (!arena) {
        close(backing_fd);
    }
    pthread_mutex_unlock(&sd_open_mutex);
    return arena;
}
void serde_sync(serde_arena *arena) {
    int backing_fd = arena->backing_fd;
    off_t arena_bytes;
    pthread_mutex_lock(&sd_open_mutex);

    SD_ASSERT(backing_fd != -1, "All serde_arena should have a matching fd.\n");
    arena_bytes = fd_byte_size(backing_fd);
    if (arena_bytes > 0) {
        if (msync(arena, (size_t)arena_bytes, MS_SYNC) == -1) {
        }
    }

    pthread_mutex_unlock(&sd_open_mutex);
}

bool serde_format(serde_arena *arena, int backing_fd, size_t first_bytes) {
    if (!arena || backing_fd < 0 || first_bytes < SD_REGION_SIZE)
        return false;

    uintptr_t raw_base = (uintptr_t)arena;

    memset(arena, 0, sizeof(serde_arena));

    arena->format_rev = SD_FORMAT_REV;
    arena->backing_fd = backing_fd;
    arena->map_flags = MAP_SHARED;

    arena->region_count = SD_META_REGIONS;
    arena->latch = 0;

    memset(arena->roots, 0, sizeof(arena->roots));

    arena->used_mark_ref = serde_ref((void *)(raw_base + 1 * SD_REGION_SIZE));
    arena->head_mark_ref =
        serde_ref((void *)(raw_base + (1 + SD_MARKMAP_REGIONS) * SD_REGION_SIZE));

    sd_au64 *used_mark = serde_deref(arena, arena->used_mark_ref);
    sd_au64 *head_mark = serde_deref(arena, arena->head_mark_ref);

    arena_blank_marks(arena, used_mark, head_mark);

    memset(&arena->freelists, 0, sizeof(arena->freelists));

    sd_tag tag = {};
    tag.region.kind = SD_K_REGION;
    region_setup(&arena->region, tag);

    region_list_push(&arena->freelists.region_lane, &arena->region);

    return true;
}

void serde_close(serde_arena *arena) {
    serde_sync(arena);
    int backing_fd = arena->backing_fd;

    pthread_mutex_lock(&sd_open_mutex);

    munmap(arena, SD_ARENA_SIZE);
    close(backing_fd);
    pthread_mutex_unlock(&sd_open_mutex);
}

void serde_set_root(serde_arena *arena, void *mem, int slot_ix) {
    SD_ASSERT(slot_ix < 8, "serde: root slot %d outside [0,8)\n", slot_ix);

    if (mem != NULL) {
        SD_ASSERT(arena == serde_arena_of(mem), "serde: root %p is not owned by arena %p\n", mem,
                  arena);
    }

    sd_ref cell_off;
    cell_off = serde_ref(mem);
    arena->roots[slot_ix] = cell_off;
}

void *serde_get_root(serde_arena *arena, int slot_ix) {
    SD_ASSERT(slot_ix < 8, "serde: root slot %d outside [0,8)\n", slot_ix);
    return serde_deref(arena, arena->roots[slot_ix]);
}

void arena_grow_to(serde_arena *arena, size_t nbytes) {
    int arena_fd = arena->backing_fd;
    off_t arena_bytes, grow_edge;
    uintptr_t arena_base;

    pthread_mutex_lock(&sd_open_mutex);

    arena_bytes = fd_byte_size(arena_fd);
    grow_edge = (off_t)nbytes;
    arena_base = (uintptr_t)arena;

    if (arena_bytes < grow_edge) {

        if (ftruncate(arena_fd, grow_edge) == -1) {

            fprintf(stderr, "serde: grow via ftruncate failed: %s\n", strerror(errno));
            abort();
        }
        if (mmap((void *)(arena_base + arena_bytes), grow_edge - arena_bytes,
                 PROT_READ | PROT_WRITE, arena->map_flags | MAP_FIXED, arena_fd,
                 arena_bytes) == MAP_FAILED) {

            fprintf(stderr, "serde: grow via mmap failed: %s\n", strerror(errno));
            abort();
        }
    }
    pthread_mutex_unlock(&sd_open_mutex);
}

static off_t fd_byte_size(int backing_fd) {
    struct stat arena_stat;
    if (fstat(backing_fd, &arena_stat) == -1) {

        return ~0ULL;
    }
    return arena_stat.st_size;
}

static inline serde_arena *arena_map_open(int backing_fd, int flags) {
    serde_arena *arena;
    void *map_at, *al_addr;
    off_t arena_bytes;

    size_t rsv_bytes = SD_ARENA_SIZE * 2;

    int prot_flags;
    if ((flags & O_RDWR) || (flags & O_WRONLY)) {
        prot_flags = PROT_READ | PROT_WRITE;
    } else {
        prot_flags = PROT_READ;
    }

    arena_bytes = fd_byte_size(backing_fd);
    if (arena_bytes > SD_ARENA_SIZE) {
        fprintf(stderr, "serde: backing file exceeds the arena address-space bound.\n");
        return NULL;
    }

    map_at = mmap(NULL, rsv_bytes, PROT_NONE, MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);

    if (map_at == MAP_FAILED) {
        perror("serde: arena address-space reservation failed");
        return NULL;
    }

    uintptr_t raw_base = (uintptr_t)map_at;
    uintptr_t al_base = (raw_base + SD_ARENA_SIZE - 1) & ~(SD_ARENA_SIZE - 1);
    al_addr = (void *)al_base;

    size_t lead_bytes = al_base - raw_base;
    if (lead_bytes > 0) {
        munmap(map_at, lead_bytes);
    }
    size_t tail_bytes = (raw_base + rsv_bytes) - (al_base + SD_ARENA_SIZE);
    if (tail_bytes > 0) {
        munmap((void *)(al_base + SD_ARENA_SIZE), tail_bytes);
    }

    const size_t meta_bytes = SD_META_REGIONS * SD_REGION_SIZE;

    if (arena_bytes == 0) {
        if (!(prot_flags & PROT_WRITE)) {
            fprintf(stderr, "Cannot create new arena with O_RDONLY\n");
            munmap(al_addr, SD_ARENA_SIZE);
            return NULL;
        }

        if (ftruncate(backing_fd, (off_t)meta_bytes) == -1) {
            perror("serde: metadata ftruncate failed");
            munmap(al_addr, SD_ARENA_SIZE);
            return NULL;
        }

        void *file_at =
            mmap(al_addr, meta_bytes, prot_flags, MAP_SHARED | MAP_FIXED, backing_fd, 0);
        if (file_at == MAP_FAILED) {
            perror("serde: metadata mmap failed");
            munmap(al_addr, SD_ARENA_SIZE);
            return NULL;
        }

        arena = (serde_arena *)al_addr;

        arena->format_rev = SD_FORMAT_REV;
        arena->latch = 0;
        arena->region_count = SD_META_REGIONS;
        arena->backing_fd = backing_fd;
        arena->map_flags = MAP_SHARED;
        memset(arena->roots, 0, sizeof(arena->roots));

        arena->used_mark_ref = serde_ref((void *)(al_base + 1 * SD_REGION_SIZE));
        arena->head_mark_ref =
            serde_ref((void *)(al_base + (1 + SD_MARKMAP_REGIONS) * SD_REGION_SIZE));

        sd_au64 *used_mark = serde_deref(arena, arena->used_mark_ref);
        sd_au64 *head_mark = serde_deref(arena, arena->head_mark_ref);
        arena_blank_marks(arena, used_mark, head_mark);

        memset(&arena->freelists, 0, sizeof(arena->freelists));

        sd_tag tag = {};
        tag.region.kind = SD_K_REGION;
        region_setup(&arena->region, tag);

        region_list_push(&arena->freelists.region_lane, &arena->region);
    } else {

        void *file_at =
            mmap(al_addr, arena_bytes, prot_flags, MAP_SHARED | MAP_FIXED, backing_fd, 0);
        if (file_at == MAP_FAILED) {
            perror("serde: remap of existing arena failed");
            munmap(al_addr, SD_ARENA_SIZE);
            return NULL;
        }

        arena = (serde_arena *)al_addr;
        arena->backing_fd = backing_fd;

        if (arena->format_rev != SD_FORMAT_REV) {
            fprintf(stderr,
                    "serde: not a serde arena, or wrong byte order (header 0x%08x != 0x%08x).\n",
                    arena->format_rev, SD_FORMAT_REV);
            munmap(al_addr, SD_ARENA_SIZE);
            return NULL;
        }
    }
    return arena;
}

#if defined(__AVX2__)
#include <immintrin.h>

static inline uint64_t sd__mask8x32(__m256i v) {
    return (uint64_t)(uint32_t)_mm256_movemask_epi8(v);
}

void serde_col_pack(const uint32_t *in, uint64_t *out, size_t rows, int bits) {
    const uint32_t *in_end = in + rows;
    __m128i shf = _mm_cvtsi32_si128(32 - bits);
    __m256i ord = _mm256_set_epi32(7, 3, 6, 2, 5, 1, 4, 0);
    if (rows)
        do {
            __m256i v1 = _mm256_load_si256((const __m256i *)(in + 0));
            __m256i v2 = _mm256_load_si256((const __m256i *)(in + 8));
            __m256i v3 = _mm256_load_si256((const __m256i *)(in + 16));
            __m256i v4 = _mm256_load_si256((const __m256i *)(in + 24));
            __m256i v5 = _mm256_load_si256((const __m256i *)(in + 32));
            __m256i v6 = _mm256_load_si256((const __m256i *)(in + 40));
            __m256i v7 = _mm256_load_si256((const __m256i *)(in + 48));
            __m256i v8 = _mm256_load_si256((const __m256i *)(in + 56));
            in += 64;
            v1 = _mm256_sll_epi32(v1, shf);
            v2 = _mm256_sll_epi32(v2, shf);
            v3 = _mm256_sll_epi32(v3, shf);
            v4 = _mm256_sll_epi32(v4, shf);
            v5 = _mm256_sll_epi32(v5, shf);
            v6 = _mm256_sll_epi32(v6, shf);
            v7 = _mm256_sll_epi32(v7, shf);
            v8 = _mm256_sll_epi32(v8, shf);
            int bit = 0;
            for (;;) {
                __m256i r1 = _mm256_srli_epi32(v1, 24), r2 = _mm256_srli_epi32(v2, 24);
                __m256i r3 = _mm256_srli_epi32(v3, 24), r4 = _mm256_srli_epi32(v4, 24);
                __m256i r12 = _mm256_packus_epi32(r1, r2), r34 = _mm256_packus_epi32(r3, r4);
                __m256i r5 = _mm256_srli_epi32(v5, 24), r6 = _mm256_srli_epi32(v6, 24);
                __m256i r7 = _mm256_srli_epi32(v7, 24), r8 = _mm256_srli_epi32(v8, 24);
                __m256i r56 = _mm256_packus_epi32(r5, r6), r78 = _mm256_packus_epi32(r7, r8);
                __m256i a = _mm256_packus_epi16(r12, r34), b = _mm256_packus_epi16(r56, r78);
                a = _mm256_permutevar8x32_epi32(a, ord);
                b = _mm256_permutevar8x32_epi32(b, ord);
                int pass = bits - bit;
                if (pass > 8)
                    pass = 8;
                bit += pass;
                do {
                    uint64_t lo = sd__mask8x32(a), hi = sd__mask8x32(b);
                    a = _mm256_add_epi8(a, a);
                    b = _mm256_add_epi8(b, b);
                    _mm_stream_si64((long long *)out++, (long long)(lo | (hi << 32)));
                } while (--pass);
                if (bit == bits)
                    break;
                v1 = _mm256_slli_si256(v1, 1);
                v2 = _mm256_slli_si256(v2, 1);
                v3 = _mm256_slli_si256(v3, 1);
                v4 = _mm256_slli_si256(v4, 1);
                v5 = _mm256_slli_si256(v5, 1);
                v6 = _mm256_slli_si256(v6, 1);
                v7 = _mm256_slli_si256(v7, 1);
                v8 = _mm256_slli_si256(v8, 1);
            }
        } while (in != in_end);
}

void serde_col_unpack(const uint64_t *in, uint32_t *out, size_t rows, int bits) {
    const uint64_t *in_end = in + (rows >> 6) * (size_t)bits;
    __m256i off = _mm256_set1_epi64x((long long)0x8040201008040201ull);
    __m256i blo = _mm256_set_epi64x(0x0808080808080808ull, 0, 0x0808080808080808ull, 0);
    __m256i bhi = _mm256_set_epi64x(0x0C0C0C0C0C0C0C0Cull, 0x0404040404040404ull,
                                    0x0C0C0C0C0C0C0C0Cull, 0x0404040404040404ull);
    if (rows)
        do {
            __m256i v1 = _mm256_setzero_si256(), v2 = _mm256_setzero_si256();
            __m256i v3 = _mm256_setzero_si256(), v4 = _mm256_setzero_si256();
            __m256i v5 = _mm256_setzero_si256(), v6 = _mm256_setzero_si256();
            __m256i v7 = _mm256_setzero_si256(), v8 = _mm256_setzero_si256();
            int bit = bits;
            for (;;) {
                __m256i x = _mm256_setzero_si256(), y = _mm256_setzero_si256();
                do {
                    __m128i w = _mm_loadl_epi64((const __m128i *)in++);
                    __m256i lh = _mm256_cvtepu8_epi32(w);
                    __m256i lo = _mm256_shuffle_epi8(lh, blo), hi = _mm256_shuffle_epi8(lh, bhi);
                    lo = _mm256_and_si256(lo, off);
                    hi = _mm256_and_si256(hi, off);
                    x = _mm256_add_epi8(x, x);
                    y = _mm256_add_epi8(y, y);
                    lo = _mm256_cmpeq_epi8(lo, off);
                    hi = _mm256_cmpeq_epi8(hi, off);
                    x = _mm256_sub_epi8(x, lo);
                    y = _mm256_sub_epi8(y, hi);
                } while (--bit & 7);
                __m128i a = _mm256_castsi256_si128(x), b = _mm256_castsi256_si128(y);
                v1 = _mm256_or_si256(v1, _mm256_cvtepu8_epi32(a));
                v2 = _mm256_or_si256(v2, _mm256_cvtepu8_epi32(b));
                a = _mm_srli_si128(a, 8);
                b = _mm_srli_si128(b, 8);
                v3 = _mm256_or_si256(v3, _mm256_cvtepu8_epi32(a));
                v4 = _mm256_or_si256(v4, _mm256_cvtepu8_epi32(b));
                x = _mm256_permute4x64_epi64(x, _MM_SHUFFLE(1, 0, 3, 2));
                y = _mm256_permute4x64_epi64(y, _MM_SHUFFLE(1, 0, 3, 2));
                __m128i c = _mm256_castsi256_si128(x), d = _mm256_castsi256_si128(y);
                v5 = _mm256_or_si256(v5, _mm256_cvtepu8_epi32(c));
                v6 = _mm256_or_si256(v6, _mm256_cvtepu8_epi32(d));
                c = _mm_srli_si128(c, 8);
                d = _mm_srli_si128(d, 8);
                v7 = _mm256_or_si256(v7, _mm256_cvtepu8_epi32(c));
                v8 = _mm256_or_si256(v8, _mm256_cvtepu8_epi32(d));
                if (bit == 0)
                    break;
                v1 = _mm256_slli_si256(v1, 1);
                v2 = _mm256_slli_si256(v2, 1);
                v3 = _mm256_slli_si256(v3, 1);
                v4 = _mm256_slli_si256(v4, 1);
                v5 = _mm256_slli_si256(v5, 1);
                v6 = _mm256_slli_si256(v6, 1);
                v7 = _mm256_slli_si256(v7, 1);
                v8 = _mm256_slli_si256(v8, 1);
            }
            _mm256_stream_si256((__m256i *)(out + 0), v1);
            _mm256_stream_si256((__m256i *)(out + 8), v2);
            _mm256_stream_si256((__m256i *)(out + 16), v3);
            _mm256_stream_si256((__m256i *)(out + 24), v4);
            _mm256_stream_si256((__m256i *)(out + 32), v5);
            _mm256_stream_si256((__m256i *)(out + 40), v6);
            _mm256_stream_si256((__m256i *)(out + 48), v7);
            _mm256_stream_si256((__m256i *)(out + 56), v8);
            out += 64;
        } while (in != in_end);
}

void serde_col_scan_range(const uint64_t *in, uint64_t *out, size_t rows, int bits, uint32_t lo,
                          uint32_t hi) {
    const uint64_t *in_end = in + ((rows * (size_t)bits) >> 6);
    int64_t lo_s = (int64_t)(((uint64_t)lo) << (64 - bits));
    int64_t hi_s = (int64_t)(((uint64_t)hi) << (64 - bits));
    if (rows)
        do {
            const uint64_t *w = in;
            int64_t gt_lo = 0, eq_lo = -1, lt_hi = 0, eq_hi = -1;
            int64_t lo_bit = lo_s, hi_bit = hi_s;
            in += bits;
            do {
                int64_t d = (int64_t)*w;
                int64_t x_lo = d ^ (lo_bit >> 63);
                int64_t x_hi = d ^ (hi_bit >> 63);
                gt_lo |= eq_lo & x_lo & d;
                lt_hi |= eq_hi & x_hi & ~d;
                eq_lo &= ~x_lo;
                eq_hi &= ~x_hi;
                if ((eq_hi | eq_lo) == 0)
                    break;
                lo_bit += lo_bit;
                hi_bit += hi_bit;
            } while (++w != in);
            _mm_stream_si64((long long *)out++, (long long)((lt_hi | eq_hi) & (gt_lo | eq_lo)));
        } while (in != in_end);
}
#endif

#endif
#endif
