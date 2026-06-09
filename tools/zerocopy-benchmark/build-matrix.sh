#!/usr/bin/env bash
# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#
# Build the SDK in each CUDA / zero-copy configuration and (optionally) run
# rs-zerocopy-benchmark against each, so the three modes can be compared on the same
# Jetson + camera.
#
# Configurations:
#   off       BUILD_WITH_CUDA=OFF                              (CPU/SIMD baseline)
#   cuda      BUILD_WITH_CUDA=ON  BUILD_WITH_CUDA_ZEROCOPY=OFF (today's CUDA path)
#   zerocopy  BUILD_WITH_CUDA=ON  BUILD_WITH_CUDA_ZEROCOPY=ON  (Layers A/B/C)
#
# Usage:
#   tools/zerocopy-benchmark/build-matrix.sh [--configs "off cuda zerocopy"]
#                                            [--seconds N] [--jobs N]
#                                            [--build-only] [--run-only]
#                                            [--extra-cmake "..."]
#
# Each config builds into build-zc/<config>. Benchmark output is teed to
# build-zc/<config>/benchmark.log and a comparison summary is printed at the end.
#
# Run from the repo root (or anywhere; the script locates the repo root itself).

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

CONFIGS="off cuda zerocopy"
SECONDS_RUN=20
JOBS="$(nproc 2>/dev/null || echo 4)"
BUILD=1
RUN=1
EXTRA_CMAKE=""
RGB_NN=0          # --rgb-nn: run the RGB upload-vs-direct comparison instead of align+pointcloud
RES=""            # --res WxH: color resolution for --rgb-nn (e.g. 1280x720)
DUAL=0            # --dual: dual-RGB (both color streams, D401 GMSL) in --rgb-nn mode
WITH_EXAMPLES=0   # --with-examples: also build realsense-viewer + rs-gpu-frame (slow). Default off.

while [[ $# -gt 0 ]]; do
    case "$1" in
        --configs)        CONFIGS="$2"; shift 2;;
        --seconds)        SECONDS_RUN="$2"; shift 2;;
        --jobs)           JOBS="$2"; shift 2;;
        --build-only)     RUN=0; shift;;
        --run-only)       BUILD=0; shift;;
        --extra-cmake)    EXTRA_CMAKE="$2"; shift 2;;
        --rgb-nn)         RGB_NN=1; shift;;
        --dual)           DUAL=1; shift;;
        --res)            RES="$2"; shift 2;;
        --with-examples)  WITH_EXAMPLES=1; shift;;
        *) echo "unknown arg: $1"; exit 2;;
    esac
done

# Build the per-config benchmark command line (align+pointcloud, or the RGB upload-vs-direct mode).
bench_args() {
    if [[ $RGB_NN -eq 1 ]]; then
        local r=""
        [[ -n "$RES" ]] && r="--width ${RES%x*} --height ${RES#*x}"
        local d=""
        [[ $DUAL -eq 1 ]] && d="--dual"
        echo "--rgb-nn --seconds $SECONDS_RUN $r $d"
    else
        echo "--seconds $SECONDS_RUN"
    fi
}

cmake_flags_for() {
    case "$1" in
        off)      echo "-DBUILD_WITH_CUDA=OFF";;
        cuda)     echo "-DBUILD_WITH_CUDA=ON -DBUILD_WITH_CUDA_ZEROCOPY=OFF";;
        zerocopy) echo "-DBUILD_WITH_CUDA=ON -DBUILD_WITH_CUDA_ZEROCOPY=ON";;
        *) echo ""; return 1;;
    esac
}

build_one() {
    local cfg="$1" dir="build-zc/$1"
    local flags; flags="$(cmake_flags_for "$cfg")" || { echo "!! unknown config $cfg"; return 1; }

    # By default build only the benchmark tool (fast). --with-examples also builds the
    # realsense-viewer + rs-gpu-frame example, which needs BUILD_EXAMPLES + BUILD_GRAPHICAL_EXAMPLES
    # ON (see tools/CMakeLists.txt) and is much slower (DQT/OpenGL/imgui).
    local ex_flags targets
    if [[ $WITH_EXAMPLES -eq 1 ]]; then
        ex_flags="-DBUILD_EXAMPLES=ON -DBUILD_GRAPHICAL_EXAMPLES=ON"
        targets="rs-zerocopy-benchmark realsense-viewer rs-gpu-frame"
    else
        ex_flags="-DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF"
        targets="rs-zerocopy-benchmark"
    fi

    echo "==================================================================="
    echo "  BUILD [$cfg]  ->  $dir   (examples: $( [[ $WITH_EXAMPLES -eq 1 ]] && echo on || echo off ))"
    echo "  flags: $flags $ex_flags $EXTRA_CMAKE"
    echo "==================================================================="
    cmake -S . -B "$dir" -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_TOOLS=ON $ex_flags \
          $flags $EXTRA_CMAKE || return 1
    cmake --build "$dir" --target $targets -j "$JOBS" || return 1
}

run_one() {
    local cfg="$1" dir="build-zc/$1"
    local bin; bin="$(find "$dir" -name rs-zerocopy-benchmark -type f 2>/dev/null | head -1)"
    if [[ -z "$bin" ]]; then echo "!! benchmark binary not found for $cfg (build it first)"; return 1; fi
    echo "==================================================================="
    echo "  RUN [$cfg]  ($SECONDS_RUN s)$( [[ $RGB_NN -eq 1 ]] && echo '  [rgb-nn]' )"
    echo "==================================================================="
    local libdir; libdir="$(dirname "$bin")"
    local ldp="$libdir:$libdir/../../src:$dir/src:${LD_LIBRARY_PATH:-}"

    # rgb-nn mode: write the per-config detailed table to the log only, so the console shows
    # just the single cross-build summary at the end (the per-config tables were confusing).
    if [[ $RGB_NN -eq 1 ]]; then
        echo "  running rgb-nn (H2D-copy + get_gpu_data, ${SECONDS_RUN}s each) -> $dir/rgb-nn.log"
        LD_LIBRARY_PATH="$ldp" "$bin" $(bench_args) > "$dir/rgb-nn.log" 2>&1
        return 0
    fi

    # Use the freshly-built library, not any system-installed one.
    LD_LIBRARY_PATH="$ldp" "$bin" $(bench_args) 2>&1 | tee "$dir/benchmark.log"

    # Demonstrate the public GPU-pointer API for this config: a non-null gpu= means zero-copy
    # is live (integrated GPU + zerocopy build); null means the upload fallback path.
    local gpubin; gpubin="$(find "$dir" -name rs-gpu-frame -type f 2>/dev/null | head -1)"
    if [[ -n "$gpubin" ]]; then
        echo "  --- get_gpu_data() check [$cfg] ---"
        LD_LIBRARY_PATH="$libdir:${LD_LIBRARY_PATH:-}" "$gpubin" 2>&1 | grep -E "Color frame|GPU pointer|No GPU" | tee "$dir/gpu-frame.log"
    fi
}

declare -A RESULT
for cfg in $CONFIGS; do
    if [[ $BUILD -eq 1 ]]; then build_one "$cfg" || { echo "!! build failed: $cfg"; RESULT[$cfg]="BUILD-FAILED"; continue; }; fi
    if [[ $RUN -eq 1 ]];  then run_one  "$cfg" || RESULT[$cfg]="RUN-FAILED"; fi
done

if [[ $RUN -eq 1 && $RGB_NN -eq 1 ]]; then
    echo ""
    echo "##### feed RGB to GPU: H2D-copy vs get_gpu_data (per camera) #####"
    echo "  (averages per frame over ~${SECONDS_RUN}s per method, 30-frame warmup excluded; total = H2D + kernel)"
    printf "%-10s | %-13s | %-10s | %-10s | %-10s | %-11s | %-9s\n" "build" "method" "total(ms)" "H2D(ms)" "kernel(ms)" "CPU(%1core)" "gpu-used"
    printf -- "-----------+---------------+------------+------------+------------+-------------+----------\n"
    # extract the first float from a '|'-delimited field
    field_num() { echo "$1" | awk -F'|' -v c="$2" '{print $c}' | grep -oE '[0-9]+\.[0-9]+' | head -1; }
    for cfg in $CONFIGS; do
        log="build-zc/$cfg/rgb-nn.log"
        if [[ "${RESULT[$cfg]:-}" == BUILD-FAILED ]]; then printf "%-10s | %s\n" "$cfg" "BUILD FAILED"; continue; fi
        if [[ ! -f "$log" ]]; then printf "%-10s | %s\n" "$cfg" "no log"; continue; fi
        if grep -q 'requires a CUDA build' "$log"; then printf "%-10s | %s\n" "$cfg" "n/a (non-CUDA build)"; continue; fi
        # detailed rows:  method | total | H2D | kernel | FPS | CPU
        local_fellback=0; grep -q 'no GPU ptr' "$log" && local_fellback=1
        for method in H2D-copy get_gpu_data; do
            row=$(grep -m1 "^  $method " "$log")
            t=$(field_num "$row" 2)
            h=$(field_num "$row" 3)
            k=$(field_num "$row" 4)
            cp=$(field_num "$row" 6)
            gu="-"
            [[ "$method" == get_gpu_data ]] && { [[ $local_fellback -eq 1 ]] && gu="no" || gu="yes"; }
            printf "%-10s | %-13s | %-10s | %-10s | %-10s | %-11s | %-9s\n" "$cfg" "$method" "${t:-?}" "${h:-?}" "${k:-?}" "${cp:-?}" "$gu"
        done
        saved=$(grep -m1 'H2D saved/frame' "$log" | sed -n 's/.*: \([0-9.]*\) ms.*/\1/p')
        [[ -n "$saved" ]] && printf "%-10s | %s\n" "$cfg" ">> H2D saved/frame: ${saved} ms (x cameras x fps = aggregate)"
    done
    echo "#################################################################"
elif [[ $RUN -eq 1 ]]; then
    echo ""
    echo "############### COMPARISON SUMMARY ###############"
    printf "%-10s | %-13s | %-13s | %-7s | %-12s | %-8s\n" "config" "align avg(ms)" "pc avg(ms)" "FPS" "CPU(%1core)" "gpu-ptr"
    printf -- "-----------+---------------+---------------+---------+--------------+---------\n"
    # avg is column 2 of the per-call table row "  <block> | avg | min | max | n"
    col2() { echo "$1" | awk -F'|' '{print $2}' | grep -oE '[0-9]+\.[0-9]+' | head -1; }
    for cfg in $CONFIGS; do
        log="build-zc/$cfg/benchmark.log"
        if [[ "${RESULT[$cfg]:-}" == BUILD-FAILED ]]; then printf "%-10s | %s\n" "$cfg" "BUILD FAILED"; continue; fi
        if [[ ! -f "$log" ]]; then printf "%-10s | %s\n" "$cfg" "no log"; continue; fi
        a=$(col2 "$(grep -m1 '^  align.process()' "$log")")
        p=$(col2 "$(grep -m1 '^  pointcloud.calculate()' "$log")")
        f=$(grep -m1 'effective FPS' "$log" | grep -oE '[0-9]+\.[0-9]+' | head -1)
        c=$(grep -m1 'process CPU'   "$log" | grep -oE '[0-9]+\.[0-9]+' | head -1)
        if [[ -f "build-zc/$cfg/gpu-frame.log" ]]; then
            g="no"; grep -q 'GPU pointer available' "build-zc/$cfg/gpu-frame.log" && g="yes"
        else
            g="n/a"   # rs-gpu-frame not built (run without --with-examples)
        fi
        printf "%-10s | %-13s | %-13s | %-7s | %-12s | %-8s\n" "$cfg" "${a:-?}" "${p:-?}" "${f:-?}" "${c:-?}" "$g"
    done
    echo "##################################################"
fi
