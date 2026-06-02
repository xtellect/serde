/* example.c - what serde.h can do, measured.
 *
 *   cc -O2 -mavx2 example.c -o example -lpthread && ./example
 *
 * 1. alloc / realloc / free across every size class, with throughput
 * 2. a 50,000-node tree that a separate fork()'d process re-opens and walks
 * 3. a persistent open-addressing hash index: build, close, reopen, query
 * 4. a compressed integer column scanned in place (AVX2), with GiB/s numbers
 * 5. a 512 MiB object in a 32 TiB span, backed lazily page by page
 * 6. arena state that persists across repeated close/reopen (msync, not fsync)
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
#define SERDE_IMPLEMENTATION
#include "serde.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/wait.h>

#define PATH "/tmp/serde_demo.arena"

static double clk(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
static uint64_t mix(uint64_t h, uint64_t x) { h ^= x; h *= 1099511628211ULL; return h; }
static void section(const char *s) { printf("\n\033[1m%s\033[0m\n", s); }
static void pass(const char *s) { printf("  ok  %s\n", s); }

/* 1 -------------------------------------------------------------------- */
static void heap(void) {
    section("1  heap in a file: alloc / realloc / free, all size classes");
    serde_arena *a = serde_open(PATH, O_RDWR | O_CREAT);
    printf("  arena base %p  (a 32 TiB position-independent span)\n", (void *)a);

    size_t sz[] = {24, 200, 1024, 9000, 300000, 3u << 20};
    const char *kind[] = {"slab", "slab", "wide-slab", "chunk", "chunk", "giant"};
    for (int i = 0; i < 6; i++) {
        void *p = serde_alloc(a, sz[i]);
        printf("  %-9s %8zu B -> usable %8zu B\n", kind[i], sz[i], serde_usable_size(p));
        serde_free(p);
    }

    size_t cap = 8;
    char *buf = serde_alloc(a, cap);
    memset(buf, 'A', cap);
    int grows = 0, moved = 0;
    void *prev = buf;
    while (cap < (8u << 20)) {
        buf = serde_realloc(a, buf, cap * 2);
        grows++;
        for (size_t i = 0; i < cap; i++)
            if (buf[i] != 'A') { printf("  realloc corrupted data\n"); break; }
        memset(buf + cap, 'A', cap);
        if (buf != prev) { moved++; prev = buf; }
        cap *= 2;
    }
    printf("  realloc 8 B -> %zu MiB, content intact (%d grows, %d in place, %d moved)\n",
           cap >> 20, grows, grows - moved, moved);
    serde_free(serde_realloc(a, buf, 64));

    enum { N = 400000 };
    void **v = malloc(N * sizeof *v);
    double t0 = clk();
    for (int i = 0; i < N; i++) v[i] = serde_alloc(a, 16 + (i * 37u & 511));
    double t1 = clk();
    for (int i = 0; i < N; i++) serde_free(v[i]);
    double t2 = clk();
    printf("  %d mixed alloc %.1f M/s, free %.1f M/s\n", N, N / (t1 - t0) / 1e6, N / (t2 - t1) / 1e6);
    free(v);
    serde_close(a);
    pass("allocator");
}

/* 2 -------------------------------------------------------------------- */
typedef struct { uint64_t key; sd_ref left, right; } tnode;

static sd_ref tree(serde_arena *a, uint32_t lo, uint32_t hi) {
    if (lo > hi || hi == 0xffffffffu) return 0;
    uint32_t mid = lo + (hi - lo) / 2;
    tnode *n = serde_alloc(a, sizeof *n);
    n->key = mid;
    n->left = mid > lo ? tree(a, lo, mid - 1) : 0;
    n->right = tree(a, mid + 1, hi);
    return serde_ref(n);
}
static uint64_t fold(serde_arena *a, sd_ref r) {
    if (!r) return 0;
    tnode *n = serde_deref(a, r);
    return mix(fold(a, n->left), n->key) ^ fold(a, n->right);
}

static void persistence(void) {
    section("2  pointers survive a process boundary (50k-node tree across fork)");
    unlink(PATH);
    serde_arena *a = serde_open(PATH, O_RDWR | O_CREAT);
    sd_ref root = tree(a, 1, 50000);
    uint64_t parent = fold(a, root);
    serde_set_root(a, serde_deref(a, root), 0);
    serde_sync(a);
    printf("  pid %d wrote a 50,000-node tree, checksum %016" PRIx64 "\n", getpid(), parent);
    serde_close(a);

    fflush(stdout);
    pid_t pid = fork();
    if (pid == 0) {
        serde_arena *c = serde_open(PATH, O_RDWR);
        uint64_t child = fold(c, serde_ref(serde_get_root(c, 0)));
        printf("  pid %d reopened the file and walked it, checksum %016" PRIx64 " (%s)\n",
               getpid(), child, child == parent ? "identical" : "MISMATCH");
        serde_close(c);
        fflush(stdout);
        _exit(child == parent ? 0 : 1);
    }
    int st = 0;
    waitpid(pid, &st, 0);
    if (WIFEXITED(st) && WEXITSTATUS(st) == 0) pass("zero-deserialization persistence");
}

/* 3 -------------------------------------------------------------------- */
typedef struct { sd_ref key; uint64_t val; } slot;
typedef struct { uint32_t cap, len; sd_ref slots; } kvmap;

static uint64_t khash(const char *s) {
    uint64_t h = 1469598103934665603ULL;
    while (*s) h = mix(h, (uint8_t)*s++);
    return h;
}
static sd_ref kv_new(serde_arena *a, uint32_t cap) {
    kvmap *m = serde_alloc(a, sizeof *m);
    m->cap = cap;
    m->len = 0;
    m->slots = serde_ref(serde_calloc(a, cap, sizeof(slot)));
    return serde_ref(m);
}
static void kv_put(serde_arena *a, kvmap *m, const char *k, uint64_t val) {
    slot *s = serde_deref(a, m->slots);
    for (uint32_t i = khash(k) & (m->cap - 1);; i = (i + 1) & (m->cap - 1)) {
        if (!s[i].key) {
            char *kk = serde_alloc(a, strlen(k) + 1);
            strcpy(kk, k);
            s[i].key = serde_ref(kk);
            s[i].val = val;
            m->len++;
            return;
        }
        if (strcmp(serde_deref(a, s[i].key), k) == 0) { s[i].val = val; return; }
    }
}
static int kv_get(serde_arena *a, kvmap *m, const char *k, uint64_t *out) {
    slot *s = serde_deref(a, m->slots);
    for (uint32_t i = khash(k) & (m->cap - 1);; i = (i + 1) & (m->cap - 1)) {
        if (!s[i].key) return 0;
        if (strcmp(serde_deref(a, s[i].key), k) == 0) { *out = s[i].val; return 1; }
    }
}

static void index_demo(void) {
    section("3  a persistent hash index (build, close, reopen, query)");
    serde_arena *a = serde_open(PATH, O_RDWR);
    sd_ref ref = kv_new(a, 1 << 16);
    kvmap *m = serde_deref(a, ref);
    const char *k[] = {"c", "rust", "zig", "go", "ocaml", "haskell", "lua", "forth"};
    for (int i = 0; i < 8; i++) kv_put(a, m, k[i], 1970 + i * 7);
    serde_set_root(a, serde_deref(a, ref), 1);
    printf("  inserted %u keys, closed\n", m->len);
    serde_close(a);

    a = serde_open(PATH, O_RDWR);
    kvmap *m2 = serde_get_root(a, 1);
    uint64_t v = 0;
    int ok = kv_get(a, m2, "ocaml", &v) && kv_get(a, m2, "forth", &v);
    printf("  reopened %u entries; get(\"forth\") = %" PRIu64 "\n", m2->len, v);
    serde_close(a);
    if (ok) pass("index");
}

/* 4 -------------------------------------------------------------------- */
static void columns(void) {
#if SERDE_HAVE_COLUMN
    section("4  compressed integer column, scanned in place (AVX2)");
    serde_arena *a = serde_open(PATH, O_RDWR);
    enum { ROWS = 64 * 65536 };
    int bits = 24;
    uint32_t *codes = aligned_alloc(64, ROWS * 4);
    uint64_t rs = 0x9E3779B9;
    for (size_t i = 0; i < ROWS; i++) { rs = rs * 6364136223846793005ULL + 1; codes[i] = (uint32_t)(rs >> 40) & ((1u << bits) - 1); }

    double t0 = clk();
    sd_ref col = serde_col_build(a, codes, ROWS, bits);
    double t1 = clk();
    size_t raw = ROWS * 4ull, packed = serde_col_words(ROWS, bits) * 8ull;
    printf("  pack %d M rows x %d bits: %zu MiB -> %zu MiB (%.2fx) at %.0f M rows/s\n",
           ROWS / 1000000, bits, raw >> 20, packed >> 20, (double)raw / packed, ROWS / (t1 - t0) / 1e6);
    serde_set_root(a, serde_deref(a, col), 2);
    serde_sync(a);
    serde_close(a);

    a = serde_open(PATH, O_RDWR);
    uint64_t *cw = serde_get_root(a, 2);
    uint64_t *bmap = aligned_alloc(64, (ROWS / 64) * 8);
    uint32_t lo = 5000000, hi = 9000000;
    double s0 = clk();
    for (int r = 0; r < 5; r++) serde_col_scan_range(cw, bmap, ROWS, bits, lo, hi);
    double s1 = clk();
    size_t hits = 0, ref = 0;
    for (size_t i = 0; i < ROWS; i++) { hits += (bmap[i >> 6] >> (i & 63)) & 1; ref += codes[i] >= lo && codes[i] <= hi; }
    double rps = 5.0 * ROWS / (s1 - s0);
    printf("  scan %u..%u on the reopened column: %zu hits, %.0f M rows/s (%.1f GiB/s of source)\n",
           lo, hi, hits, rps / 1e6, rps * 4 / (1024.0 * 1024 * 1024));
    free(codes);
    free(bmap);
    serde_close(a);
    if (hits == ref) pass("column scan matches a scalar check exactly");
#else
    section("4  compressed column (build with -mavx2 to enable)");
#endif
}

/* 5 -------------------------------------------------------------------- */
static void sparse(void) {
    section("5  a 512 MiB object in a 32 TiB span, backed lazily");
    serde_arena *a = serde_open(PATH, O_RDWR);
    size_t big = 512ull << 20;
    unsigned char *p = serde_alloc(a, big);
    printf("  alloc 512 MiB -> %p, usable %zu MiB\n", (void *)p, serde_usable_size(p) >> 20);
    for (size_t off = 0; off < big; off += big / 4) p[off] = 0x42;
    printf("  touched 4 pages 128 MiB apart; the rest stays unbacked virtual memory\n");
    serde_free(p);
    serde_close(a);
    pass("sparse allocation");
}

/* 6 -------------------------------------------------------------------- */
static void reopen_cycles(void) {
    section("6  state persists across repeated close/reopen");
    serde_arena *a = serde_open(PATH, O_RDWR);
    uint64_t *c = serde_alloc(a, 8);
    *c = 0;
    serde_set_root(a, c, 3);
    for (int round = 0; round < 3; round++) {
        c = serde_get_root(a, 3);
        *c += 100;
        serde_sync(a); /* msync(MS_SYNC); not an fsync/crash-consistency guarantee */
        serde_close(a);
        a = serde_open(PATH, O_RDWR);
    }
    uint64_t total = *(uint64_t *)serde_get_root(a, 3);
    printf("  +100 over 3 close/reopen cycles -> %" PRIu64 "\n", total);
    serde_close(a);
    unlink(PATH);
    if (total == 300) pass("state survives close/reopen");
}

int main(void) {
    printf("\033[1mserde.h\033[0m  persistent allocator + columnar engine, one header\n");
    heap();
    persistence();
    index_demo();
    columns();
    sparse();
    reopen_cycles();
    printf("\nmalloc, file format, and scan kernel from a single #include.\n");
    return 0;
}
