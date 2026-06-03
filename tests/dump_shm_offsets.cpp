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
  PRINT_SIZE(PandaShmHeader);
  PRINT_SIZE(PandaShmCommand);
  PRINT_SIZE(PandaShmStateFrame);
  PRINT_SIZE(PandaShm);

  PRINT_OFFSET(PandaShmHeader, magic);
  PRINT_OFFSET(PandaShmHeader, version);
  PRINT_OFFSET(PandaShmHeader, state_frames);
  PRINT_OFFSET(PandaShmHeader, controller_pid);
  PRINT_OFFSET(PandaShmHeader, state_head);

  PRINT_OFFSET(PandaShmCommand, seq);
  PRINT_OFFSET(PandaShmCommand, target_pos);
  PRINT_OFFSET(PandaShmCommand, target_quat);
  PRINT_OFFSET(PandaShmCommand, kp_pos);
  PRINT_OFFSET(PandaShmCommand, kp_ori);
  PRINT_OFFSET(PandaShmCommand, kd_pos);
  PRINT_OFFSET(PandaShmCommand, kd_ori);
  PRINT_OFFSET(PandaShmCommand, error_delta_pos);
  PRINT_OFFSET(PandaShmCommand, error_delta_rot);
  PRINT_OFFSET(PandaShmCommand, enabled);

  PRINT_OFFSET(PandaShmStateFrame, seq);
  PRINT_OFFSET(PandaShmStateFrame, timestamp_s);
  PRINT_OFFSET(PandaShmStateFrame, q);
  PRINT_OFFSET(PandaShmStateFrame, dq);
  PRINT_OFFSET(PandaShmStateFrame, ee_pos);
  PRINT_OFFSET(PandaShmStateFrame, ee_quat);
  PRINT_OFFSET(PandaShmStateFrame, tau);
  PRINT_OFFSET(PandaShmStateFrame, ee_linvel);
  PRINT_OFFSET(PandaShmStateFrame, ee_angvel);

  return 0;
}
