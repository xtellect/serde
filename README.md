# serde.h

I wanted to persist simple CLI state (hashtables, small graphs) without linking
SQLite or LMDB. serde.h is a persistent, file-backed malloc in a single header.

```c
#define SERDE_IMPLEMENTATION   // one .c file
#include "serde.h"
```
```
cc -O2 -mavx2 example.c -o example -lpthread
```

C99, no dependencies. AVX2 is optional (see Column engine).

Allocate as usual, store a few root pointers, close the file. Reopen it later, or
from another process, and the object graph is intact. No serialize step, no load
step.

Also bundled: an AVX2 column scanner. It's an unusual addition to a malloc header,
but I built both for the same project and kept them together.

## How it works

A raw pointer is invalid once the process exits. serde reserves an aligned slab of
address space and stores every link as an offset from the arena base instead of a
machine pointer. `serde_ref(p)` is `p & (SIZE-1)`, `serde_deref` is `base + ref`.
The file is the heap; reopening is one mmap. Store refs, not pointers.

```c
typedef struct { uint64_t key; sd_ref next; } node;   // links are refs

serde_arena *a = serde_open("graph.arena", O_RDWR | O_CREAT);
sd_ref head = 0;
for (int i = 0; i < 1000; i++) {
    node *n = serde_alloc(a, sizeof *n);
    n->key = i;
    n->next = head;
    head = serde_ref(n);
}
serde_set_root(a, serde_deref(a, head), 0);
serde_close(a);

// later run, possibly a different process
a = serde_open("graph.arena", O_RDWR);
for (node *n = serde_get_root(a, 0); n; n = serde_deref(a, n->next))
    use(n->key);   // list is intact
```

`make run` exercises six of these: allocator throughput, a 50k-node tree rebuilt
by a forked child, a persistent hash index, the column scan, a lazily-backed
512 MiB object, and state across close/reopen.

## Tradeoffs vs. SQLite / LMDB

SQLite and LMDB give you ACID, transactions, and crash recovery. serde gives none
of that. It trades them for zero setup and pointers that stay valid across runs.
It's an allocator with a persistence side effect, not a database. Raw mmap returns
the bytes but not the data structures; serde returns the data structures.

## Allocator

`serde_alloc/calloc/realloc/free`, lane-pinned `*_on` variants, `serde_usable_size`.

```c
serde_arena *a = serde_open_anon();   // or serde_open(path, flags)
void *p = serde_alloc(a, n);
p = serde_realloc(a, p, bigger);
serde_free(p);

sd_ref r = serde_ref(p);          // pointer -> offset
void *q = serde_deref(a, r);      // offset -> pointer
serde_set_root(a, q, 0);          // 8 root slots, persisted
serde_sync(a);                    // msync to the file
serde_close(a);
```

Address space is reserved up front with PROT_NONE; the backing file grows lazily,
so you pay only for pages you touch. Small objects come from bitmap slabs striped
across 16 lanes, medium from multi-page chunks, large from multi-region runs.

## Column engine (AVX2)

Needs AVX2. Without it the header drops this part of the API and sets
`SERDE_HAVE_COLUMN` to 0. It packs unsigned integers into a transposed bit-plane
layout; a range scan touches only as many machine words as the values differ,
MSB-first with early-exit, and writes a 1-bit-per-row bitmap.

```c
// packed words live in the arena, so a column survives close/reopen
sd_ref col = serde_col_build(a, codes, rows, bits);
uint64_t bitmap[rows / 64];
serde_col_scan_range(serde_deref(a, col), bitmap, rows, bits, lo, hi);
```

`make check` builds the header standalone, includes it twice (ODR), and compiles
with and without AVX2.

## Bugs found

Fuzzing the allocator with an overlap/aliasing checker turned up two: a `1 << 64`
shift that under-marked page-aligned runs and handed the same block out twice, and
a free-list path that could re-issue a region while it was still live. **Both fixed.**
The column codec is checked against a scalar reference for widths 1 to 32.

## Caveats

- Little-endian, 64-bit, Linux. `#error`s on big-endian; uses mmap/mremap/ftruncate.
  Each arena reserves 32 TiB of address space, not memory. That's the max and the
  default; lower it via `SD_ARENA_BITS` (e.g. 43 = 8 TiB) to fit more arenas.
- Not a database. `serde_sync` is `msync(MS_SYNC)`: no journaling, no fsync
  ordering, no transactions. A crash mid-write can leave a partial arena.
- One writer per arena. In-process allocation is latched; the format assumes a
  single owning process, not concurrent writers.

## Status

Fuzzed and used in my own tooling, not battle-tested in production yet.

License: Apache 2.0. © Praveen Vaddadi <thynktank@gmail.com>
