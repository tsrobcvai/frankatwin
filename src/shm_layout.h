// shm_layout.h
//
// POSIX shared memory layout for panda_control Step 10.
//
// Two endpoints:
//   - Writer of "command" / reader of "state":   Python (daemon or LocalPandaController)
//   - Writer of "state"  / reader of "command":  C++ binary `osc_shm` (1 kHz)
//
// Concurrency model:
//   - Command region: seqlock. Writer (Python @ 10-20 Hz) increments seq to an
//     odd value, writes payload, increments seq to the next even value. Reader
//     (C++ @ 1 kHz) loads seq, reads payload, re-loads seq; retries if the two
//     seqs differ or if seq is odd. We use __atomic_* builtins with explicit
//     memory ordering instead of std::atomic<uint64_t> to keep the on-disk
//     layout trivially mappable from numpy.dtype on the Python side.
//   - State region: lock-free single-producer/single-consumer ring buffer of
//     fixed-size frames. C++ writes head atomically after filling slot N; any
//     reader can walk back from head to get the last K frames.
//
// Layout invariants:
//   - All multi-byte fields are 8-byte aligned.
//   - Explicit pad fields are inserted so that Python `numpy.dtype(..., align=True)`
//     produces a byte-for-byte identical layout. The static_asserts at the bottom
//     of this file pin the offsets so a future change cannot silently desync.
//   - No std::atomic<T>: only uint64_t with __atomic_* accesses.
//
// Memory ordering:
//   - Writers: __ATOMIC_RELEASE on the seq store *after* the payload write.
//   - Readers: __ATOMIC_ACQUIRE on the seq loads *before* and *after* the payload
//     read.
//
// Naming: command_* fields are inputs from policy; state_* are outputs from robot.

#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#define PANDA_SHM_MAGIC          0x50414e44u  // 'PAND' little-endian
#define PANDA_SHM_VERSION        3u           // v3: added state tau_J (measured link-side torque)
#define PANDA_SHM_STATE_FRAMES   1024u        // ring buffer depth
#define PANDA_SHM_DEFAULT_NAME   "/panda_osc" // POSIX shm name (must start with '/')

// ---------------------------------------------------------------------------
// Header (32 B)
// ---------------------------------------------------------------------------
struct PandaShmHeader {
  uint32_t magic;              // PANDA_SHM_MAGIC, written once at creation
  uint32_t version;            // PANDA_SHM_VERSION
  uint32_t state_frames;       // PANDA_SHM_STATE_FRAMES (echoed for safety)
  uint32_t reserved0;          // pad to 8-byte boundary
  uint64_t controller_pid;     // pid of osc_shm process; 0 if not running
  uint64_t state_head;         // monotonically increasing write index
};

// ---------------------------------------------------------------------------
// Command (Python -> C++). Size: 120 B.
// ---------------------------------------------------------------------------
// Seqlock convention:
//   seq even  => stable, payload valid
//   seq odd   => write in progress
// Writer flow:
//   1. s = __atomic_load_n(&seq, RELAXED); store s+1 with RELEASE.   (now odd)
//   2. write payload fields (plain store).
//   3. store s+2 with RELEASE.                                       (now even)
// Reader flow:
//   do {
//     s1 = __atomic_load_n(&seq, ACQUIRE);
//     if (s1 & 1) continue;
//     copy payload;
//     s2 = __atomic_load_n(&seq, ACQUIRE);
//   } while (s1 != s2);
struct PandaShmCommand {
  uint64_t seq;                 // see seqlock convention above
  double   target_pos[3];       // EE target position in base frame (m)
  double   target_quat[4];      // EE target orientation, wxyz, unit norm
  double   kp_pos;              // translational stiffness (N/m)
  double   kp_ori;              // rotational stiffness (Nm/rad)
  double   kd_pos;              // translational damping (Ns/m); 0 = auto 2*sqrt(kp_pos)
  double   kd_ori;              // rotational damping  (Nms/rad); 0 = auto 2*sqrt(kp_ori)
  double   error_delta_pos;     // clip |pose_error_pos| coordinate-wise (m); 0 disables
  double   error_delta_rot;     // clip |pose_error_rot| coordinate-wise (rad); 0 disables
  uint32_t enabled;             // 0 = output zero command torque (hold), 1 = active
  uint32_t reserved0;           // pad
};

// ---------------------------------------------------------------------------
// State frame (C++ -> Python). Size: 384 B (344 payload + 40 pad).
// ---------------------------------------------------------------------------
struct PandaShmStateFrame {
  uint64_t seq;                 // matches the write index (== state_head value when written)
  double   timestamp_s;         // CLOCK_MONOTONIC seconds since shm init
  double   q[7];                // joint positions (rad)
  double   dq[7];               // joint velocities (rad/s)
  double   ee_pos[3];           // EE position in base frame (m)
  double   ee_quat[4];          // EE orientation, wxyz, unit norm
  double   tau[7];              // joint command torques (Nm), post-clamp
  double   ee_linvel[3];        // EE linear velocity, base frame (m/s) = zeroJacobian_v @ dq
  double   ee_angvel[3];        // EE angular velocity, base frame (rad/s) = zeroJacobian_w @ dq
  double   tau_J[7];            // measured link-side joint torque (Nm), incl. gravity (v3+)
  uint64_t reserved0;           // pad to 384 B (cache-line multiple)
  uint64_t reserved1;
  uint64_t reserved2;
  uint64_t reserved3;
  uint64_t reserved4;
};

// ---------------------------------------------------------------------------
// Top-level layout. Size: 32 + 120 + 1024 * 384 = 393368 B (384.15 KiB).
// ---------------------------------------------------------------------------
struct PandaShm {
  PandaShmHeader     header;
  PandaShmCommand    command;
  PandaShmStateFrame states[PANDA_SHM_STATE_FRAMES];
};

// ---------------------------------------------------------------------------
// Layout pinning. If you change any field, update these static_asserts AND
// python/panda_control/shm_layout.py AND tests/test_shm_layout.py.
// ---------------------------------------------------------------------------
#ifdef __cplusplus
static_assert(sizeof(PandaShmHeader) == 32, "PandaShmHeader size drift");
static_assert(sizeof(PandaShmCommand) == 120, "PandaShmCommand size drift");
static_assert(sizeof(PandaShmStateFrame) == 384, "PandaShmStateFrame size drift");
static_assert(sizeof(PandaShm) == 32 + 120 + 1024 * 384, "PandaShm size drift");
static_assert(offsetof(PandaShmHeader, controller_pid) == 16, "header.controller_pid offset drift");
static_assert(offsetof(PandaShmHeader, state_head) == 24, "header.state_head offset drift");
static_assert(offsetof(PandaShmCommand, target_pos) == 8, "command.target_pos offset drift");
static_assert(offsetof(PandaShmCommand, target_quat) == 32, "command.target_quat offset drift");
static_assert(offsetof(PandaShmCommand, kp_pos) == 64, "command.kp_pos offset drift");
static_assert(offsetof(PandaShmCommand, kp_ori) == 72, "command.kp_ori offset drift");
static_assert(offsetof(PandaShmCommand, kd_pos) == 80, "command.kd_pos offset drift");
static_assert(offsetof(PandaShmCommand, kd_ori) == 88, "command.kd_ori offset drift");
static_assert(offsetof(PandaShmCommand, error_delta_pos) == 96, "command.error_delta_pos offset drift");
static_assert(offsetof(PandaShmCommand, error_delta_rot) == 104, "command.error_delta_rot offset drift");
static_assert(offsetof(PandaShmCommand, enabled) == 112, "command.enabled offset drift");
static_assert(offsetof(PandaShmStateFrame, q) == 16, "state.q offset drift");
static_assert(offsetof(PandaShmStateFrame, dq) == 72, "state.dq offset drift");
static_assert(offsetof(PandaShmStateFrame, ee_pos) == 128, "state.ee_pos offset drift");
static_assert(offsetof(PandaShmStateFrame, ee_quat) == 152, "state.ee_quat offset drift");
static_assert(offsetof(PandaShmStateFrame, tau) == 184, "state.tau offset drift");
static_assert(offsetof(PandaShmStateFrame, ee_linvel) == 240, "state.ee_linvel offset drift");
static_assert(offsetof(PandaShmStateFrame, ee_angvel) == 264, "state.ee_angvel offset drift");
static_assert(offsetof(PandaShmStateFrame, tau_J) == 288, "state.tau_J offset drift");
#endif

#ifdef __cplusplus
}  // extern "C"
#endif

// ---------------------------------------------------------------------------
// Atomic accessors (C++ only). Header-only inline helpers.
// ---------------------------------------------------------------------------
#ifdef __cplusplus

namespace panda_shm {

// Writer side: open a seqlock write transaction. Returns the new (odd) seq value.
inline uint64_t cmd_write_begin(PandaShmCommand* cmd) {
  uint64_t s = __atomic_load_n(&cmd->seq, __ATOMIC_RELAXED);
  __atomic_store_n(&cmd->seq, s + 1, __ATOMIC_RELEASE);
  return s + 1;
}

// Writer side: close a seqlock write transaction.
inline void cmd_write_end(PandaShmCommand* cmd, uint64_t odd_seq) {
  __atomic_store_n(&cmd->seq, odd_seq + 1, __ATOMIC_RELEASE);
}

// Reader side: copy command payload into `out`. Spins until a stable snapshot
// is obtained. In practice this completes in 1-2 iterations because the writer
// runs at <=20 Hz and the reader at 1 kHz.
inline void cmd_read(const PandaShmCommand* cmd, PandaShmCommand* out) {
  for (;;) {
    uint64_t s1 = __atomic_load_n(&cmd->seq, __ATOMIC_ACQUIRE);
    if (s1 & 1u) continue;
    *out = *cmd;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    uint64_t s2 = __atomic_load_n(&cmd->seq, __ATOMIC_ACQUIRE);
    if (s1 == s2) {
      out->seq = s1;
      return;
    }
  }
}

// State writer (C++ producer): write into the slot at (head + 1) and then
// publish the new head. Reader uses head to know the latest valid slot.
inline void state_publish(PandaShmHeader* hdr,
                          PandaShmStateFrame* states,
                          const PandaShmStateFrame& frame,
                          uint32_t ring_size) {
  uint64_t head = __atomic_load_n(&hdr->state_head, __ATOMIC_RELAXED);
  uint64_t next = head + 1;
  PandaShmStateFrame& slot = states[next % ring_size];
  slot = frame;
  slot.seq = next;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  __atomic_store_n(&hdr->state_head, next, __ATOMIC_RELEASE);
}

// State reader (Python or NUC daemon): copy the latest valid frame. Returns
// false if the buffer is empty (head == 0).
inline bool state_read_latest(const PandaShmHeader* hdr,
                              const PandaShmStateFrame* states,
                              uint32_t ring_size,
                              PandaShmStateFrame* out) {
  uint64_t head = __atomic_load_n(&hdr->state_head, __ATOMIC_ACQUIRE);
  if (head == 0) return false;
  const PandaShmStateFrame& slot = states[head % ring_size];
  *out = slot;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  uint64_t head2 = __atomic_load_n(&hdr->state_head, __ATOMIC_ACQUIRE);
  // If the writer wrapped the entire buffer between our reads, the frame is
  // suspect; signal failure so the caller can retry or skip.
  return (head2 - head) < ring_size;
}

}  // namespace panda_shm

#endif  // __cplusplus
