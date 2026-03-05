# 🏗️ 12-Month s-e-roadmap

**Target:** SE  
**Core Stack:** C++ · Linux · Concurrency · Hardware Architecture · AI/ML Integration  
**Start Date:** 2026-02-11  

---

## The Full System Stack (Reference Model)

```
┌──────────────────────────────────────────────────────────────────┐
│                        APPLICATION                               │
│  User-space services, frameworks, AI/ML inference pipelines      │
│  AI Role: Model serving, intelligent scheduling, auto-tuning     │
├──────────────────────────────────────────────────────────────────┤
│                    RUNTIME / LIBRARIES                            │
│  libc, libstdc++, CUDA Runtime, TensorRT, DPDK, allocators       │
│  AI Role: AI-driven compiler optimization (MLIR, XLA, TVM),      │
│           auto-vectorization, operator fusion, JIT compilation    │
├──────────────────────────────────────────────────────────────────┤
│                         KERNEL                                    │
│  Scheduler, VFS, memory manager, network stack, cgroups           │
│  AI Role: ML-based CPU/IO scheduling, anomaly detection,          │
│           predictive page fault handling, smart OOM decisions      │
├──────────────────────────────────────────────────────────────────┤
│                         DRIVER                                    │
│  GPU drivers, NIC drivers, storage drivers, DMA engines           │
│  AI Role: Self-healing drivers, adaptive power management,        │
│           automated verification, device fingerprinting           │
├────────────────────────────────────────────────────────���─────────┤
│                        FIRMWARE                                   │
│  BIOS/UEFI, microcontroller FW, GPU microcode, NIC firmware       │
│  AI Role: TinyML on-device, adaptive thermal/power control,       │
│           firmware-level intrusion detection                      │
├──────────────────────────────────────────────────────────────────┤
│                   HARDWARE (Silicon)                              │
│  CPU/GPU cores, caches (L1/L2/L3), DMA engines,                  │
│  PCIe/CXL interconnect, DRAM controllers, NPUs/TPUs              │
│  AI Role: Hardware prefetch prediction, neural branch prediction, │
│           in-silicon inference accelerators, cache policy learning │
└──────────────────────────────────────────────────────────────────┘
```

---

## 🤖 AI Agents Across the Stack — Interlinking Architecture

```
┌────────────────────────────────────────���────────────────────────────────┐
│                        ORCHESTRATOR AGENT                               │
│  Coordinates all layer-specific agents, resolves cross-layer conflicts  │
│  Example: "Cache agent says thrashing → tell Scheduler agent to        │
│            reduce thread count → tell Power agent to throttle"          │
└────────────┬──────────┬──────────┬──────────┬──────────┬───────────────┘
             │          │          │          │          │
    ┌────────▼───┐ ┌────▼─────┐ ┌─▼────────┐│ ┌────────▼───┐ ┌─────────▼──┐
    │ APP AGENT  │ │ COMPILER │ │ KERNEL   ││ │ DRIVER     │ │ FIRMWARE   │
    │            │ │ AGENT    │ │ AGENT    ││ │ AGENT      │ │ AGENT      │
    │ • Workload │ │ • Auto-  │ │ • Smart  ││ │ • Self-    │ │ • TinyML   │
    │   profiling│ │   tuning │ │   sched  ││ │   healing  │ │   on-chip  │
    │ • Model    │ │ • Op     │ │ • Anomaly││ │ • Adaptive │ │ • Thermal  │
    │   selection│ │   fusion │ │   detect ││ │   power    │ │   control  │
    │ • Batch    │ │ • Target │ │ • OOM    ││ │ • HW       │ │ • Intrusion│
    │   sizing   │ │   codegen│ │   predict││ │   compat   │ │   detect   │
    └────────────┘ └──────────┘ └─────────┘│ └────────────┘ └────────────┘
                                           │
                                  ┌────────▼───────┐
                                  │ HARDWARE AGENT │
                                  │ • Cache policy │
                                  │ • Prefetch     │
                                  │ • DMA schedule │
                                  │ • Power states │
                                  └────────────────┘

COMMUNICATION BUS: shared telemetry (metrics, counters, events)
                   via shared memory rings, eBPF, sysfs, perf_events
```

### Problems AI Solves at Each Layer

| Layer | Problem | AI Solution | Agent Role |
|-------|---------|-------------|------------|
| **Application** | Workload varies → inefficient resource use | Predict load patterns, auto-scale batch sizes | App Agent profiles, signals Kernel Agent |
| **Runtime/Compiler** | Hand-tuning kernels for each HW target | Auto-tuning (TVM/XLA), operator fusion, quantization | Compiler Agent generates target-specific code |
| **Kernel** | Static schedulers can't adapt to dynamic loads | ML-based scheduler (Google's ghOSt), predictive paging | Kernel Agent adjusts policy, signals Driver Agent |
| **Driver** | Crashes, compatibility bugs, power waste | Self-healing (auto-restart + root cause), adaptive DVFS | Driver Agent monitors health, reports to Orchestrator |
| **Firmware** | Security threats, thermal runaway on edge devices | On-device TinyML for anomaly detection, thermal prediction | Firmware Agent acts locally with <1ms latency |
| **Hardware** | Cache thrashing, branch misprediction, idle DMA | Neural prefetcher, learned branch prediction, DMA scheduling | Hardware Agent (in-silicon or FPGA co-processor) |

---

## 📊 Skill Matrix & Target Company Coverage

| Skill Area | Qualcomm Needs | NVIDIA Needs | AMD Needs | Covered In |
|-----------|---------------|-------------|----------|-----------|
| C++ (modern, systems) | ✅ Critical | ✅ Critical | ✅ Critical | Phase 1 |
| ARM architecture | ✅ Critical | ⚠️ Nice to have | ⚠️ Nice to have | Phase 4 |
| CUDA/GPU compute | ⚠️ Some roles | ✅ Critical | ✅ Critical (HIP) | Phase 4 |
| Linux kernel/OS | ✅ Critical | ✅ Important | ✅ Important | Phase 2 |
| Concurrency/atomics | ✅ Critical | ✅ Critical | ✅ Critical | Phase 3 |
| Lock-free algorithms | ✅ Important | ✅ Important | ✅ Important | Phase 3, 5 |
| Cache/memory perf | ✅ Critical | ✅ Critical | ✅ Critical | Phase 4 |
| DMA/PCIe | ✅ Critical | ✅ Important | ✅ Important | Phase 4 |
| AI/ML integration | ⚠️ Growing | ✅ Critical | ✅ Critical | Phase 5 |
| eBPF/tracing | ⚠️ Nice to have | ✅ Important | ✅ Important | Phase 4 |
| Technical leadership | ✅ Required | ✅ Required | ✅ Required | Phase 6 |
| Design docs/RFCs | ✅ Required | ✅ Required | ✅ Required | Every phase |

---

## Phase 0: Environment, Toolchain & Mindset (Week 0–1)

**Goal:** Production-grade dev environment. System-layer thinking from day one.

### Setup

```bash
# Ubuntu 22.04 / Fedora 38+
sudo apt install gcc g++ clang clang-tools cmake ninja-build \
  gdb lldb valgrind linux-tools-common linux-tools-generic \
  git docker.io python3 python3-pip

# Sanitizers (built into GCC/Clang — verify)
# ASan, TSan, UBSan, MSan

# Cross-compilation (ARM)
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu qemu-user

# GPU (if NVIDIA GPU available)
# Install CUDA toolkit + nvcc

# Profiling
sudo apt install linux-perf hotspot flamegraph
```

### Deliverables

- [ ] CMakeLists.txt template with ASan/TSan/UBSan toggles, cross-compile target, benchmark target
- [ ] Stack diagram printed/pinned to wall
- [ ] All tools installed and verified

### Mindset (ongoing forever)

```
Every bug          → "Which layer? CPU / cache / OS / memory / my code?"
Every optimization → "Where is the bottleneck? Measure first."
Every API          → "Who owns this resource? What is the lifetime?"
Every struct       → "What is the memory layout? Padding? Cache line?"
```

---

## Phase 1: C++ as a Systems Language (Month 1–2)

**Goal:** Master C++ ownership, lifetime, layout, and zero-cost abstractions. NO concurrency yet.

### Month 1: Memory, Ownership, Layout

| Week | Topic | Deliverable |
|------|-------|-------------|
| 1 | Stack vs heap, RAII, destructor semantics, rule of 5 | RAII wrappers for FILE*, socket, mmap |
| 2 | Smart pointers (unique_ptr, shared_ptr, weak_ptr), custom deleters | Refactor a raw-pointer codebase to smart pointers |
| 3 | Memory layout: sizeof, alignof, offsetof, padding, cache lines | Write a tool that prints struct layout + padding |
| 4 | Allocators: placement new, memory pools, arena allocators | Implement a fixed-size memory pool allocator |

### Month 2: Move Semantics, Templates, Compile-Time

| Week | Topic | Deliverable |
|------|-------|-------------|
| 5 | Rvalue refs, std::move, move ctors/assignment, noexcept | Benchmark: copy vs move for 1M objects |
| 6 | Perfect forwarding, universal references, std::forward | Implement `make_unique` from scratch |
| 7 | Templates: specialization, SFINAE, type_traits, concepts (C++20) | Implement `Optional<T>` supporting move-only types |
| 8 | Compile-time: constexpr, consteval, template metaprogramming | Compile-time hash map or type-safe unit system |

### Mini-Projects

1. `PoolAllocator<T>` — fixed-block allocator with RAII, move semantics, and alignment control
2. `Optional<T>` — supports move-only types, emplacement, strong exception safety
3. Struct layout analyzer — prints field offsets, padding bytes, total size, cache-line crossings

### Acceptance Criteria

- [ ] Zero leaks (Valgrind clean)
- [ ] Move constructors are `noexcept`
- [ ] APIs express ownership clearly in signatures
- [ ] Templates give clear compile-time errors (concepts or static_assert)
- [ ] Benchmarks show move >> copy for non-trivial types
- [ ] Design doc: 1-page writeup of allocator design decisions

### Resources

- *Effective Modern C++* — Scott Meyers
- cppreference.com
- Compiler Explorer (godbolt.org) for assembly inspection

---

## Phase 2: Linux / OS Internals (Month 3–4)

**Goal:** Understand how your C++ code maps to OS primitives. Process model, virtual memory, I/O.

### Month 3: Processes, Memory, Signals

| Week | Topic | Deliverable |
|------|-------|-------------|
| 9 | Process model: fork, exec, wait, PID, file descriptor table | Mini shell that forks+execs commands |
| 10 | Virtual memory: pages, page tables, TLB, mmap, mprotect | Program that maps files, measures page fault cost |
| 11 | Heap internals: malloc/free, sbrk, mmap threshold, fragmentation | Custom malloc benchmarked vs glibc |
| 12 | Signals, signal handlers, async-signal-safety | Signal-safe logging daemon |

### Month 4: I/O, Networking, Profiling Tools

| Week | Topic | Deliverable |
|------|-------|-------------|
| 13 | File descriptors, read/write, buffered vs unbuffered I/O | High-throughput file copier (compare buffered vs mmap vs direct I/O) |
| 14 | Sockets: TCP/UDP, bind/listen/accept, non-blocking I/O | Echo server (blocking + non-blocking versions) |
| 15 | Event-driven I/O: select → poll → epoll, edge vs level triggered | epoll-based multi-client server |
| 16 | Profiling: perf, ftrace, strace, Valgrind, ASan/TSan | Profile all previous projects, write perf reports |

### Compiler & ABI Internals (Week 15–16 overlap)

- vtable layout, virtual dispatch cost measurement
- Calling conventions (System V AMD64 ABI)
- Name mangling, `extern "C"`, linker scripts basics
- LTO (Link-Time Optimization) and its impact on inlining

### Mini-Projects

1. **Multi-client chat server** — epoll-based, non-blocking, with clean resource management (RAII for fds)
2. **Memory profiler** — tracks mmap/munmap calls via LD_PRELOAD, reports allocation hot-spots

### Acceptance Criteria

- [ ] Chat server handles 1000+ concurrent connections
- [ ] perf report identifies and explains top 3 hotspots
- [ ] mmap-based file I/O benchmarked and compared with explanation
- [ ] Design doc: virtual memory layout diagram for your programs

### Resources

- *The Linux Programming Interface* — Michael Kerrisk
- *Understanding the Linux Kernel* — Bovet & Cesati
- `man 7 signal`, `man 2 mmap`, `man 7 epoll`

---

## Phase 3: Concurrency & Atomics (Month 5–6)

**Goal:** Single consolidated concurrency phase. From mutexes to lock-free. This is where Qualcomm/NVIDIA/AMD interviews focus heavily.

### Month 5: Threading Fundamentals & Synchronization

| Week | Topic | Deliverable |
|------|-------|-------------|
| 17 | std::thread, std::jthread, thread lifecycle, join/detach | Thread pool (fixed-size, task queue) |
| 18 | Mutexes: std::mutex, lock_guard, unique_lock, deadlock avoidance | Dining philosophers (multiple solutions) |
| 19 | Condition variables, semaphores, barriers, latches (C++20) | Bounded producer-consumer queue |
| 20 | Thread safety analysis: TSan, race detection, happens-before | Annotate and fix races in a given buggy codebase |

### Month 6: Atomics, Memory Model & Lock-Free

| Week | Topic | Deliverable |
|------|-------|-------------|
| 21 | std::atomic: load/store/exchange/CAS, lock-free property | Atomic counter, spinlock implementation |
| 22 | Memory ordering: seq_cst, acquire/release, relaxed, fences | Programs demonstrating ordering differences |
| 23 | Lock-free SPSC ring buffer | Full implementation + stress test + benchmark |
| 24 | Lock-free MPMC queue (intro), ABA problem, hazard pointers | Design doc for MPMC queue (implement in Phase 5) |

### Mini-Projects

1. **Thread pool with work-stealing** — move-only task type, RAII lifetime, benchmarked
2. **SPSC lock-free ring buffer** — cache-line aligned, templated, stress-tested

### Acceptance Criteria

- [ ] SPSC ring: zero lost items under 10M ops stress test (2 threads, 10 min)
- [ ] Thread pool: handles 100K tasks, benchmark against `std::async`
- [ ] Memory orderings documented with "why" for each choice
- [ ] TSan clean on all concurrency code
- [ ] Design doc: memory ordering diagram for SPSC ring buffer

### Resources

- *C++ Concurrency in Action* — Anthony Williams
- Jeff Preshing's blog on lock-free programming
- CppCon talks on atomics (Herb Sutter, Fedor Pikus)

---

## Phase 4: Hardware Architecture & GPU Compute (Month 7–8)

**Goal:** Understand what happens below the OS. CPU microarchitecture, caches, GPU compute.

### Month 7: CPU Architecture & Cache Effects

| Week | Topic | Deliverable |
|------|-------|-------------|
| 25 | CPU pipeline: fetch, decode, execute, retire; branch prediction | Benchmark: sorted vs unsorted array (branch prediction demo) |
| 26 | Cache hierarchy: L1/L2/L3, cache lines, associativity, MESI protocol | Array traversal benchmark: row-major vs column-major |
| 27 | False sharing, cache-line padding, NUMA topology | Fix false sharing in a multi-threaded counter benchmark |
| 28 | x86 vs ARM: memory models (TSO vs weak), ISA differences | Cross-compile a benchmark to ARM, compare behavior |

### Month 8: GPU Compute & Heterogeneous Systems

| Week | Topic | Deliverable |
|------|-------|-------------|
| 29 | GPU architecture: warps/wavefronts, SIMT, memory hierarchy | Diagram: GPU vs CPU execution model |
| 30 | CUDA basics: kernel launch, threadIdx, blockIdx, shared memory | Matrix multiply in CUDA (naive → shared memory → tiled) |
| 31 | HIP (AMD) / CUDA portability, compute abstraction | Port CUDA matrix multiply to HIP |
| 32 | DMA, PCIe, host↔device transfers, pinned memory, streams | Benchmark: pageable vs pinned memory transfer |

### eBPF (Week 28 overlap)

- Write a simple eBPF program for tracing syscalls or network packets
- Understand how eBPF enables safe kernel-space programmability

### Mini-Projects

1. **Cache-aware matrix multiply** — benchmark naive vs blocked vs SIMD-hinted, analyze with perf
2. **GPU vector add + matrix multiply** — CUDA/HIP, profile with nvprof/rocprof
3. **False sharing eliminator** — tool that detects and fixes false sharing in given code

### Acceptance Criteria

- [ ] Can explain MESI protocol and demonstrate false sharing with numbers
- [ ] CUDA matrix multiply achieves >50% of theoretical peak (for given GPU)
- [ ] Cross-compiled ARM binary runs correctly under QEMU
- [ ] perf stat report with IPC, cache miss %, branch miss % for all benchmarks
- [ ] Design doc: CPU vs GPU execution model comparison

### Resources

- *Computer Systems: A Programmer's Perspective* — Bryant & O'Hallaron
- Intel/ARM architecture manuals
- NVIDIA CUDA Programming Guide
- AMD HIP Programming Guide

---

## Phase 5: Cross-Layer Integration Projects (Month 9–10)

**Goal:** Architect-level projects that span multiple layers. This is your portfolio.

### Project A: High-Performance Packet Processor (Primary)

```
                     ┌──────────────┐
      NIC (raw sock) │  CAPTURE     │ epoll + mmap'd ring buffer
      ───────────────►  THREAD      │
                     └──────┬───────┘
                            │ lock-free SPSC ring
                     ┌──────▼───────┐
                     │  PARSE       │ zero-copy packet parsing
                     │  THREAD(s)   │ thread-pinned, NUMA-aware
                     └──────┬───────┘
                            │ lock-free MPMC queue
                     ┌──────▼───────┐
                     │  PROCESS     │ filtering, stats, forwarding
                     │  THREAD(s)   │ SIMD-optimized where possible
                     └──────┬───────┘
                            │
                     ┌──────▼───────┐
                     │  STATS /     │ atomic counters, per-thread
                     │  OUTPUT      │ cache-line aligned
                     └──────────────┘
```

**Demonstrates:** epoll I/O, lock-free queues, NUMA awareness, cache optimization, RAII resource management, move semantics, templates, profiling.

### Project B: AI-Assisted Performance Profiler

```
     ┌──────────────────────────────────────────┐
     │         TARGET APPLICATION               │
     │  (instrumented with eBPF / perf_events)  │
     └──────────────┬───────────────────────────┘
                    │ telemetry stream
     ┌──────────────▼───────────────────────────┐
     │        COLLECTOR AGENT (C++)             │
     │  • Lock-free ring buffer for events      │
     │  • Zero-copy shared memory transport     │
     │  • Cache-line aligned event structs      │
     └──────────────┬───────────────────────────┘
                    │ batched events
     ┌──────────────▼───────────────────────────┐
     │        ANALYSIS AGENT (Python + C++)     │
     │  • ML model: classify bottleneck type    │
     │    (CPU-bound / memory-bound / IO-bound) │
     │  • Anomaly detection on latency          │
     │  • Suggest optimization (cache, thread,  │
     │    algorithm, data structure)             │
     └──────────────┬───────────────────────────┘
                    │ recommendations
     ┌──────────────▼───────────────────────────┐
     │        DASHBOARD / CLI OUTPUT            │
     │  "Bottleneck: L3 cache miss rate 45%     │
     │   Suggestion: restructure SoA → AoS      │
     │   for hot loop at main.cpp:142"          │
     └──────────────────────────────────────────┘
```

**Demonstrates:** AI integration, cross-layer visibility, C++ ↔ Python interop, eBPF, lock-free transport, systems reasoning.

### Project C: Lock-Free MPMC Queue (Hard Data Structure)

- Full hazard-pointer or epoch-based reclamation
- Templated, allocator-aware, cache-line padded
- Stress tested with 8+ producer/consumer threads
- Benchmarked against `moodycamel::ConcurrentQueue` and `boost::lockfree::queue`

### Deliverables for each project

- [ ] Working code with CI (GitHub Actions: GCC + Clang, ASan + TSan)
- [ ] Design doc (3–5 pages): architecture, memory ordering rationale, trade-offs
- [ ] Benchmark report with graphs
- [ ] Profiling report (perf, cachegrind, or nvprof)
- [ ] README with build instructions and usage

---

## Phase 6: Interview Readiness & Technical Leadership (Month 11–12)

**Goal:** Staff-level readiness. Technical depth + leadership signal.

### Technical Interview Prep (starts Month 3, intensifies here)

**Weekly cadence from Month 3:**

- 2 LeetCode medium/hard (concurrency, systems, bit manipulation)
- 1 system design problem per week
- 1 "explain to me" verbal drill

**Month 11–12 intensive focus areas:**

| Focus Area | Practice |
|-----------|---------|
| C++ deep | "Implement shared_ptr", "What does noexcept affect?", "Explain vtable layout" |
| Concurrency | "Design a thread-safe LRU cache", "Implement read-write lock", "Explain ABA" |
| Systems | "What happens when you call malloc?", "Explain page fault handling", "Design epoll" |
| Architecture | "x86 vs ARM memory model", "Explain cache coherence", "What is false sharing?" |
| GPU | "Explain warp divergence", "CUDA shared memory bank conflicts", "Host↔device transfer optimization" |
| Design | "Design a high-throughput logging system", "Design a lock-free queue for a packet pipeline" |

### Staff-Level Leadership Skills

| Skill | How to demonstrate |
|-------|--------------------|
| **Technical vision** | Write a 2-page RFC for each Phase 5 project before coding |
| **Mentoring** | Publish projects on GitHub with detailed READMEs; write blog posts |
| **Code review** | Review open-source PRs (DPDK, LLVM, Linux kernel) |
| **Cross-team influence** | Document how projects span layers; show systems thinking |
| **Trade-off articulation** | Every design doc must have a "Trade-offs & Alternatives" section |

---

## Phase 7: Positioning & Networking (Ongoing from Month 1)

| Month | Action |
|-------|--------|
| 1–2 | Set up GitHub portfolio repo structure. Start a dev blog. |
| 3–4 | First open-source contribution (DPDK/LLVM/Linux documentation or small bug). |
| 5–6 | LinkedIn: post about concurrency learnings with benchmarks. Connect with target company engineers. |
| 7–8 | Submit a talk proposal to a local meetup or CppCon/CppIndia. |
| 9–10 | Portfolio projects polished with READMEs, benchmarks, design docs. |
| 11–12 | Apply. Reach out to recruiters/referrals. Prepare portfolio walkthrough demo (15 min). |

### Resume Bullet Format (Staff-level)

```
• Designed and implemented a lock-free SPSC ring buffer achieving 
  50M ops/sec with zero data loss under TSan, reducing inter-thread 
  latency by 3.2x vs mutex-based queue (C++20, Linux, perf)

• Built an AI-assisted performance profiler combining eBPF telemetry 
  collection in C++ with ML-based bottleneck classification, 
  identifying cache-bound hotspots with 92% accuracy

• Architected a multi-threaded packet processing pipeline handling 
  1Gbps throughput with NUMA-aware thread pinning and zero-copy 
  buffer management, profiled with perf and VTune
```

---

## 🔑 Key Principles

```
1. MEASURE BEFORE OPTIMIZE
   No claim without numbers. perf, cachegrind, nvprof, benchmarks.

2. LAYERS, NOT SILOS
   Every project must touch ≥2 layers. "I optimized the C++ code" is 
   junior. "I traced the bottleneck from userspace through the scheduler 
   to L3 cache contention" is staff.

3. OWNERSHIP OVER KNOWLEDGE
   Reading about atomics ≠ implementing a correct lock-free queue 
   that passes TSan. Build it, test it, break it, fix it.

4. WRITE IT DOWN
   Staff engineers write design docs, not just code. One design doc 
   per phase minimum.

5. AI IS A TOOL, NOT MAGIC
   Know where AI helps (profiling, optimization search, anomaly 
   detection) and where it doesn't (correctness proofs, UB detection).

6. PORTFOLIO > RESUME
   A GitHub repo with a benchmarked lock-free queue + design doc 
   beats 10 bullet points about "experience with concurrency."
```

---

## 📚 Complete Reading List

| Resource | Phase |
|----------|-------|
| *Effective Modern C++* — Scott Meyers | 1 |
| *C++ Concurrency in Action* — Anthony Williams | 3 |
| *The Linux Programming Interface* — Michael Kerrisk | 2 |
| *Understanding the Linux Kernel* — Bovet & Cesati | 2 |
| *Computer Systems: A Programmer's Perspective* — Bryant & O'Hallaron | 4 |
| Intel 64 and IA-32 Software Developer Manuals | 4 |
| ARM Architecture Reference Manual | 4 |
| NVIDIA CUDA Programming Guide | 4 |
| AMD HIP Programming Guide | 4 |
| Jeff Preshing's lock-free programming blog | 3 |
| C++ Core Guidelines (isocpp.github.io) | All |
| cppreference.com | All |
| Compiler Explorer (godbolt.org) for assembly inspection | All |

```
