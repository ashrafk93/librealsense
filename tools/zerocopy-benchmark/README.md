# zerocopy-benchmark

Measures the per-frame cost of the GPU-accelerated processing blocks (`align` +
`pointcloud`) and end-to-end frame latency, so the CUDA **zero-copy** work can be compared
across builds.

It exists to answer one question: *does eliminating the capture copy (C1) and the GPU
host↔device round-trips (C2/C3) actually speed things up, and does it stay safe where it
shouldn't engage?*

## The three configurations

| Config     | CMake flags                                             | Copies per frame | Zero-copy active? |
|------------|---------------------------------------------------------|------------------|-------------------|
| `off`      | `BUILD_WITH_CUDA=OFF`                                    | C1 (CPU/SIMD)    | n/a               |
| `cuda`     | `BUILD_WITH_CUDA=ON  BUILD_WITH_CUDA_ZEROCOPY=OFF`       | C1 + C2 + C3     | no (today's path) |
| `zerocopy` | `BUILD_WITH_CUDA=ON  BUILD_WITH_CUDA_ZEROCOPY=ON`        | none*            | only on integrated GPU |

\* Zero-copy only engages at **runtime** on an **integrated GPU** (Jetson / Tegra, where CPU
and GPU share DRAM). On a **discrete** GPU the `zerocopy` build compiles but safely falls
back to the `cuda` copy path — so `cuda` and `zerocopy` will measure the *same* there. That
fallback is the point: it proves discrete/desktop users are never regressed. Real zero-copy
speedup shows up only on a Jetson.

## Prerequisites

- A connected RealSense camera (depth + color; e.g. D455, D435, D401).
- To build the `cuda` / `zerocopy` configs: the CUDA toolkit (`nvcc`) and an NVIDIA driver.
- CMake ≥ 3.8, a C++14 compiler.

## Quick start — build & run all three

From the repo root:

```bash
tools/zerocopy-benchmark/build-matrix.sh --seconds 30
```

On a machine where `nvcc` is not on `PATH` (point CMake at the toolkit explicitly):

```bash
tools/zerocopy-benchmark/build-matrix.sh --seconds 30 \
  --extra-cmake "-DCMAKE_CUDA_COMPILER=/usr/local/cuda-13.0/bin/nvcc -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-13.0"
```

This does a full from-source build of `librealsense2` into `build-zc/<config>/` for **each**
config, building the **benchmark** (fast — only the SDK + the tool). It then, per config, runs
the benchmark for `--seconds` and tees output to `build-zc/<config>/benchmark.log`, and prints a
comparison table at the end.

### `--with-examples` (off by default)

By default the matrix builds **only the benchmark** to stay fast. Pass **`--with-examples`** to
also build, per config, the **`realsense-viewer`** and the **`rs-gpu-frame`** example (which link
that config's library — e.g. the `zerocopy` viewer/example exercise the zero-copy path live). With
this flag the per-config run also executes `rs-gpu-frame` once to record whether the public
`get_gpu_data()` API returns a device pointer (`build-zc/<config>/gpu-frame.log`), and the depth
summary's **`gpu-ptr`** column shows `yes`/`no` (it shows `n/a` without the flag, since the example
isn't built).

> The viewer pulls in DQT/OpenGL/imgui and is by far the slowest part — it roughly multiplies
> each config's build time, which is why it's off by default. Use `--with-examples` only when you
> want the viewer/example binaries (e.g. to eyeball zero-copy live).

Each config's viewer (when built) is at `build-zc/<config>/Release/realsense-viewer`; run it with
that config's library on the load path:
```bash
LD_LIBRARY_PATH=build-zc/zerocopy/Release build-zc/zerocopy/Release/realsense-viewer
```

### Options

| Flag                    | Meaning                                                        |
|-------------------------|----------------------------------------------------------------|
| `--configs "a b c"`     | Which configs to build/run (default `"off cuda zerocopy"`).   |
| `--seconds N`           | Benchmark duration per config (default 20).                    |
| `--jobs N`              | Parallel build jobs (default `nproc`).                         |
| `--with-examples`       | Also build the `realsense-viewer` + `rs-gpu-frame` per config (slow; off by default). |
| `--build-only`          | Build every config, do not run (no camera needed).             |
| `--run-only`            | Skip building; run already-built configs.                      |
| `--extra-cmake "..."`   | Extra flags passed verbatim to every `cmake` configure.        |

Examples:

```bash
# Only compare today's CUDA path vs zero-copy:
tools/zerocopy-benchmark/build-matrix.sh --configs "cuda zerocopy" --seconds 30

# Just verify all three compile (no camera):
tools/zerocopy-benchmark/build-matrix.sh --build-only

# Re-run benchmarks without rebuilding:
tools/zerocopy-benchmark/build-matrix.sh --run-only --seconds 60
```

## Running a single config by hand

The matrix script is only a convenience wrapper. To build one config yourself:

```bash
cmake -S . -B build-zc/zerocopy -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TOOLS=ON -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
      -DBUILD_WITH_CUDA=ON -DBUILD_WITH_CUDA_ZEROCOPY=ON \
      -DCMAKE_CUDA_COMPILER=/usr/local/cuda-13.0/bin/nvcc \
      -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-13.0
cmake --build build-zc/zerocopy --target rs-zerocopy-benchmark -j$(nproc)
```

Then run it, pointing the loader at that build's library so it doesn't pick up a
system-installed one:

```bash
BIN=$(find build-zc/zerocopy -name rs-zerocopy-benchmark -type f | head -1)
LD_LIBRARY_PATH="$(dirname "$BIN")/../src:build-zc/zerocopy/src" "$BIN" --seconds 30
```

### Benchmark options

```
rs-zerocopy-benchmark [--seconds N] [--no-align] [--no-pointcloud] [--width W --height H]
```

## Reading the output

```
--- results (warmup 30 frames excluded) ---
  align.process()        avg=  X.XXX  min=  ...  max=  ...  ms  (n=...)
  pointcloud.calculate() avg=  X.XXX  ...
  frame interval         avg=  X.XXX  ...
  effective FPS            XX.XX
```

- **align.process() / pointcloud.calculate()** — GPU processing-block cost per frame. This
  is where C2/C3 elimination shows up: on a Jetson, `zerocopy` should be lower than `cuda`.
- **frame interval / effective FPS** — end-to-end throughput. C1 elimination (Layer A) shows
  up here and as reduced CPU load.
- First 30 frames are excluded (allocation, one-time CUDA init, auto-exposure warmup).

### What to expect

- **Jetson (integrated):** `zerocopy` < `cuda` < `off` (or `off` competitive for trivial ops).
- **Discrete GPU (desktop/laptop):** `cuda` ≈ `zerocopy` (zero-copy correctly stays off);
  both should beat `off` for the heavier per-pixel work (pointcloud, align).

To confirm zero-copy actually engaged, run with librealsense logging on and look for the
integrated-GPU log line, or simply compare `cuda` vs `zerocopy` timings on the target.
