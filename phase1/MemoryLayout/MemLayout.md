# Memory Layout

## Why Memory layout?

CPU Memory Bus: **64-bit** CPU reads **8 bytes at a time** from specific addresses. Reading can be slow and fast depending on alignment.

## Sizeof

Return of total size or number of a memory bytes allocated.

## alignof()

Tells alignment requirement for a type. alignment is **always a power of 2**.

In a struct, alignment is largest member in a struct

## offsetof()

Tells byte offset of a member in a struct.

## Optimisation

    Arrange members of struct from highest size to lowest.
    if smaller memebers are put first, then it requires padding.

### Optimised padding

Optimised padding depends on the aligbment of the variable in the complex structs

```cpp

struct Padding{
    char c;     // 1 byte, offset 0
    int a;      // this is a 4 byte alignment, so adds 3 bytes padding, so offset is 4 
    double d;   // now offset is 8, which is alignment of double. therefore no padding.

    // Extra 3bytes of memory is wasted.
};

```

Here alignment of struct Padding will be alignment of largest member in the struct which is double d, i.e 8.

## Cache Lines - Modern performance killer

Cache line is the smallest unit of data transfered between the CPU and Memory.

Modern CPU Cache System:

CPU Registers (1 cycle, ~1KB)
        ↓
    L1 Cache (3-4 cycles, 32KB per core, 64-byte lines)
        ↓
    L2 Cache (10-20 cycles, 256KB per core, 64-byte lines)
        ↓
    L3 Cache (40-75 cycles, 8MB shared, 64-byte lines)
        ↓
Main RAM (200-300 cycles, 16GB+)

Key: Whenever you load 1 byte, the ENTIRE 64-byte cache line is loaded!

## Cache Line Mechanism

```cpp

// Typical cache line: 64 bytes on modern x86-64

// When you access one variable:
int counter = 0;

// The CPU doesn't just load 4 bytes (int size)
// It loads entire 64-byte CACHE LINE containing counter

// On-chip cache (64 bytes loaded):
┌─────────────────────────────────────────┐
│ counter │ (and other variables in line) │ ← 64 bytes total
├───────-─┼──────────────────────────────┤
  4 bytes     60 more bytes

```

### False Sharing in Cache Line

