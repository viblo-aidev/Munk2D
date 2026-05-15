# MunkBench

MunkBench is the standalone Munk2D benchmark suite. It ports the benchmark
scenes from Pymunk's `benchmarks/chipmunk.py` so Munk2D performance changes can
be measured without Python.

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
