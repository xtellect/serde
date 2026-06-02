# serde.h - build the demo. `make`, `make run`, `make check`.
#
# Copyright (c) 2019 Praveen Vaddadi <thynktank@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

CC      ?= cc
CFLAGS  ?= -O2 -Wall
LDLIBS  ?= -lpthread

# Enable the AVX2 columnar column codec if the host supports it.
AVX2 := $(shell $(CC) -mavx2 -dM -E -x c /dev/null >/dev/null 2>&1 && echo -mavx2)
CFLAGS += $(AVX2)

.PHONY: all run check clean

all: example

example: example.c serde.h
	$(CC) $(CFLAGS) example.c -o $@ $(LDLIBS)

run: example
	./example

# Make sure the header compiles standalone, included twice (ODR), and w/o AVX2.
check: serde.h example.c
	@printf '#define SERDE_IMPLEMENTATION\n#include "serde.h"\n#include "serde.h"\nint main(void){return 0;}\n' > .odr.c
	$(CC) $(CFLAGS) -c .odr.c -o .odr.o && echo "  ODR + impl: ok"
	$(CC) -O2 -Wall -c -x c -DSERDE_IMPLEMENTATION serde.h -o .noavx.o && echo "  no-AVX2 build: ok"
	$(CC) $(CFLAGS) example.c -o .ex && echo "  example build: ok"
	@rm -f .odr.c .odr.o .noavx.o .ex
	@echo "all checks passed"

clean:
	rm -f example .odr.c .odr.o .noavx.o .ex /tmp/serde_demo.arena
