// dump_shm_offsets.cpp
//
// Small helper compiled by tests/test_shm_layout.py to print the C++ side's
// sizeof / offsetof for each field in src/shm_layout.h. The Python test then
// cross-references the numbers against the numpy.dtype-derived offsets.
//
// This keeps the layout pinned even if someone reorders or pads a field.

#include "shm_layout.h"

#include <cstddef>
#include <cstdio>

#define PRINT_SIZE(T) std::printf("size:%s:%zu\n", #T, sizeof(T))
#define PRINT_OFFSET(T, M) \
  std::printf("off:%s.%s:%zu\n", #T, #M, offsetof(T, M))

int main() {
  PRINT_SIZE(ShmHeader);
  PRINT_SIZE(ShmCommand);
  PRINT_SIZE(ShmStateFrame);
  PRINT_SIZE(ShmSegment);

  PRINT_OFFSET(ShmHeader, magic);
  PRINT_OFFSET(ShmHeader, version);
  PRINT_OFFSET(ShmHeader, state_frames);
  PRINT_OFFSET(ShmHeader, controller_pid);
  PRINT_OFFSET(ShmHeader, state_head);

  PRINT_OFFSET(ShmCommand, seq);
  PRINT_OFFSET(ShmCommand, target_pos);
  PRINT_OFFSET(ShmCommand, target_quat);
  PRINT_OFFSET(ShmCommand, kp_pos);
  PRINT_OFFSET(ShmCommand, kp_ori);
  PRINT_OFFSET(ShmCommand, kd_pos);
  PRINT_OFFSET(ShmCommand, kd_ori);
  PRINT_OFFSET(ShmCommand, error_delta_pos);
  PRINT_OFFSET(ShmCommand, error_delta_rot);
  PRINT_OFFSET(ShmCommand, enabled);

  PRINT_OFFSET(ShmStateFrame, seq);
  PRINT_OFFSET(ShmStateFrame, timestamp_s);
  PRINT_OFFSET(ShmStateFrame, q);
  PRINT_OFFSET(ShmStateFrame, dq);
  PRINT_OFFSET(ShmStateFrame, ee_pos);
  PRINT_OFFSET(ShmStateFrame, ee_quat);
  PRINT_OFFSET(ShmStateFrame, tau);
  PRINT_OFFSET(ShmStateFrame, ee_linvel);
  PRINT_OFFSET(ShmStateFrame, ee_angvel);
  PRINT_OFFSET(ShmStateFrame, tau_J);

  return 0;
}
