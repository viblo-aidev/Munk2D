# MunkBench

MunkBench is the standalone Munk2D benchmark suite. It exercises a range of
simulation setups so Munk2D performance and stability changes can be measured
without demo, rendering, or Python dependencies.

The benchmark scenarios are based on the suite from the
[`box2d-optimized`](https://github.com/mtsamis/box2d-optimized) fork of Box2D,
which proposed multiple Box2D engine improvements. The scenes are useful for
tracking relative changes within Munk2D, but raw timings should not be compared
directly with Box2D results unless the resulting simulation state is also
reviewed for equivalence.

Detailed descriptions of each benchmark, their design intent, and the original
performance findings are documented in
[`BENCHMARKS-REFERENCE.md`](BENCHMARKS-REFERENCE.md), adapted from the
Box2D-optimized dissertation.

Build:

```sh
cmake -B build -S . -DBUILD_DEMOS=OFF -DBUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --target munkbench
```

`RelWithDebInfo` is recommended while bringing the port up because it avoids the
project's default `Release` `-ffast-math` flags, which should be investigated
separately before using MunkBench for final release-mode comparisons.

Run the default size for every benchmark:

```sh
./build/benchmarks/munkbench
```

Run selected benchmarks:

```sh
./build/benchmarks/munkbench -b N2 SlowExplosion -s 100
```

Run a size sweep using each benchmark's configured range:

```sh
./build/benchmarks/munkbench -b N2 -s -1
```

Output is CSV:

```csv
version,benchmark,size,init_time,run_time
```

## Validation summaries

MunkBench can emit checkpoint summaries for stability/correctness validation:

```sh
./build/benchmarks/munkbench --summary-json -b N2 -s 25 --checkpoints 0,1,10,100
```

The JSON includes counts, dynamic body bounds, shape bounds, aggregate position
and velocity sums, kinetic energy, max velocities, sleeping body counts, and an
`invalid_values` count for NaN/Inf detection.

## SVG snapshots

A selected benchmark can be rendered to SVG for visual comparison:

```sh
./build/benchmarks/munkbench -b N2 -s 25 --svg n2-step-100.svg --step 100
```

Use `--svg -` to write the SVG to stdout.
