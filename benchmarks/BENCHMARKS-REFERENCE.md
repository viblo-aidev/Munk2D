# Benchmarks Reference

> Adapted from Section 8.1 of [*"Creating an optimized 2D physics simulation engine"*](https://github.com/mtsamis/optimizing-box2d-dissertation/blob/main/optimizing-box2d.pdf) by Emmanouil Tsamis, Aristotle University of Thessaloniki (2020).

## Purpose

The benchmark suite was developed alongside the optimized Box2D-based physics engine to:

- Identify performance issues in the original Box2D (v2.4.1)
- Evaluate optimization changes with accurate timing
- Compare scaling behavior at different world sizes

All benchmarks were measured on Ubuntu 20.04 with an AMD Ryzen 3600 CPU (3.6GHz) and 16GB DDR4 RAM, compiled with the same compiler and optimization flags. Results were reproduced on Intel hardware with similar outcomes.

Unless otherwise noted, Continuous Collision Detection (CCD) is **off** in the default benchmark runs.

---

## Benchmark Descriptions

### 1. Add Pair

A fast-moving object hits a group of resting circle objects in a zero-gravity world. Tests performance when a very high number of collisions and contacts are suddenly created.

*Adapted from Box2D's Testbed.*

### 2. Multi-fixture

Bodies with greatly varying fixture count. Several table objects are created, each built with increasingly more and smaller non-overlapping fixtures. Those tables are thrown into a U-shaped static object.

### 3. Falling Squares

Stacked squares of unequal size falling against a ground body. Tests performance when fixtures have a great variety in size.

### 4. Falling Circles

The same scene as Falling Squares but with circles instead.

### 5. Slow Explosion

A high number of dynamic bodies moving slowly in a zero-gravity world. Tests performance in scenes with slow-moving objects and very few collisions.

### 6. Tumbler

In each step, new polygon shapes are added inside a rotating box. Tests dynamic body creation.

*Adapted from Box2D's Testbed.*

### 7. Mild N²

A **degenerate scene**. Many composite objects (tables and spaceships from Box2D's Testbed) are created at the same place, one on top of the other. Creates O(n²) contact pairs momentarily. Designed to evaluate worst-case performance.

### 8. N²

A simpler version of Mild N². Several single-fixture bodies are created in the same position, offset by a small value.

### 9. Mostly Static (Single Body)

A huge static square made from a very high number of small static squares. Only a few dynamic bodies exist in a diagonal opening inside the big square. Tests worlds with very high static-to-dynamic body ratio. All small squares are individual fixtures in a single body.

### 10. Mostly Static (Multi Body)

The same scene as above, but all small squares are individual bodies with a single fixture.

### 11. Diagonal

Dynamic bodies falling through a large group of static diagonal lines. The diagonal lines have a very bad AABB approximation. Creates many broad-phase pairs but with few real collisions. Evaluates broad-phase effectiveness.

### 12. Big Mobile

Many bodies and joints in a very wide structure.

*Adapted from Box2D's Testbed.*

### 13. Mixed Static/Dynamic

A casual scene with both static and dynamic bodies, polygons and circles, some movement and collisions.

---

## Performance Context

### General findings from the original work

- For very small worlds (fixture count <200), both libraries (Box2D and the optimized engine) exhibit similar performance.
- For the majority of scenes, the optimized library scales much better to thousands of bodies.
- In scenes made up mostly of static bodies, performance is similar but Box2D scales better to massive worlds.
- The optimized implementation handles large numbers of collisions and contacts better, including degenerate scenes.

### Key benchmark characteristics

| Benchmark              | Type              | Stress area              |
|------------------------|-------------------|--------------------------|
| Add Pair               | Normal            | Sudden collision spike   |
| Multi-fixture          | Normal            | Varying fixture counts   |
| Falling Squares        | Normal            | Varied fixture sizes     |
| Falling Circles        | Normal            | Varied fixture sizes     |
| Slow Explosion         | Normal            | Sparse collisions, dispersion |
| Tumbler                | Normal            | Dynamic body creation    |
| Mild N²                | Degenerate        | O(n²) contacts           |
| N²                     | Degenerate        | O(n²) contacts (simple)  |
| Mostly Static (single) | Normal            | High static/dynamic ratio |
| Mostly Static (multi)  | Normal            | High static/dynamic ratio |
| Diagonal               | Normal            | Broad-phase stress       |
| Big Mobile             | Normal            | Joints + large structure |
| Mixed Static/Dynamic   | Normal            | Mixed general scene      |

## CCD Benchmarks

Selected benchmarks were also tested with CCD enabled. The original work found:

- Box2D's CCD imposes a major performance penalty with poor scaling
- The optimized library's CCD introduces only a slight decrease in performance
- Even with CCD on, the optimized library outperforms Box2D *without* CCD in many cases

CCD-enabled benchmarks included: Add Pair, Mixed Static/Dynamic, Diagonal, and Falling Squares.

## Hardware & Methodology

- **CPU:** AMD Ryzen 3600 @ 3.6GHz
- **RAM:** 16GB DDR4 @ 2993MHz
- **OS:** Ubuntu 20.04
- **Compiler:** Same compiler and optimization flags for both libraries
- **CCD:** Disabled by default in all benchmark runs
- **Measurement:** Consistent environment, no other applications running, adequate cooling and power
- **Steps per benchmark:** Arbitrary step counts chosen per scene to correctly measure execution time