// fci_lock.h
//
// Advisory whole-robot mutex shared by every frankatwin binary that opens a
// *controlling* libfranka session (osc_shm, move_to).
//
// Why this exists
// ---------------
// The FCI accepts more than one TCP connection to the robot, but only one of
// them may hold the motion/parameter authority. A second client that connects
// while osc_shm is running its 1 kHz control loop connects *fine* and then dies
// on its first parameter command with the unhelpful
//
//   libfranka: Set Joint Impedance command rejected:
//              command not possible in the current mode ("Move")!
//
// which says nothing about the real cause (someone else owns the robot). This
// lock turns that into an explicit, actionable failure *before* we touch
// libfranka at all, and names the process holding the robot.
//
// Mechanism
// ---------
// flock(LOCK_EX | LOCK_NB) on /tmp/frankatwin-fci-<ip>.lock, one lock file per
// robot IP so two robots on one NUC never block each other. flock is tied to
// the open file description, so the kernel releases it when the process exits
// -- including on SIGKILL or a segfault. That is the whole reason for choosing
// flock over a pidfile: a crashed controller can never leave a stale lock that
// needs manual cleanup.
//
// The holder writes "pid=<pid> exe=<name>" into the file after acquiring, so a
// blocked process can report who is in its way.
//
// Scope: this guards *controlling* sessions only. read_current_q,
// read_current_pose and read_load do a single readOnce() and never command the
// robot, so they coexist with a running controller and take no lock.
// gripper_cmd talks to the gripper server on its own TCP port (1338), which is
// independent of the FCI session -- also no lock.
//
// Advisory, not mandatory: it cannot stop a non-frankatwin FCI client
// (franka_ros, a Desk operation). Those still surface as the raw libfranka
// error, which is why move_to.cpp also annotates that exception.

#ifndef FRANKATWIN_FCI_LOCK_H
#define FRANKATWIN_FCI_LOCK_H

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

namespace frankatwin {

// Exit code used by every binary that fails to take the lock. 5 is unused by
// both osc_shm and move_to.
constexpr int kExitRobotBusy = 5;

// One lock file per robot. Non-alphanumeric characters in the IP are folded to
// '_' so a malformed --robot-ip can never escape /tmp.
inline std::string fci_lock_path(const std::string& robot_ip) {
  std::string safe;
  safe.reserve(robot_ip.size());
  for (char c : robot_ip) {
    const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                    (c >= 'A' && c <= 'Z') || c == '.' || c == '-';
    safe.push_back(ok ? c : '_');
  }
  return "/tmp/frankatwin-fci-" + safe + ".lock";
}

// RAII holder for the per-robot FCI lock.
class FciLock {
 public:
  FciLock() = default;
  ~FciLock() { release(); }

  FciLock(const FciLock&) = delete;
  FciLock& operator=(const FciLock&) = delete;

  // Try to take the lock for `robot_ip`, retrying for up to `wait_ms`.
  //
  // The retry window is not cosmetic: the daemon's reset path stops osc_shm and
  // immediately spawns move_to, so the previous holder may still be winding
  // down its libfranka session when we start. Without a short wait, every
  // daemon-driven reset would race and fail.
  //
  // `who` is recorded in the lock file for the benefit of the next process that
  // finds it taken (pass argv[0] or a short binary name).
  // On failure, *holder is set to the lock file's contents when readable.
  bool acquire(const std::string& robot_ip, const std::string& who,
               int wait_ms, std::string* holder) {
    path_ = fci_lock_path(robot_ip);
    // O_CLOEXEC: the lock must not leak into children we spawn later.
    fd_ = ::open(path_.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    if (fd_ < 0) {
      if (holder != nullptr) {
        *holder = "cannot open " + path_ + ": " + std::strerror(errno);
      }
      return false;
    }
    // open()'s mode is masked by umask; force the permissive mode so a lock
    // created by one user does not shut another out of the file entirely.
    (void)::fchmod(fd_, 0666);

    const int kPollMs = 50;
    int waited = 0;
    for (;;) {
      if (::flock(fd_, LOCK_EX | LOCK_NB) == 0) {
        write_holder(who);
        return true;
      }
      if (errno != EWOULDBLOCK) {
        if (holder != nullptr) {
          *holder = std::string("flock failed: ") + std::strerror(errno);
        }
        close_fd();
        return false;
      }
      if (waited >= wait_ms) break;
      struct timespec ts {};
      ts.tv_sec = 0;
      ts.tv_nsec = static_cast<long>(kPollMs) * 1000000L;
      ::nanosleep(&ts, nullptr);
      waited += kPollMs;
    }
    if (holder != nullptr) *holder = read_holder();
    close_fd();
    return false;
  }

  void release() {
    if (fd_ < 0) return;
    // Blank the holder line first: the fd close below drops the lock, and a
    // stale "pid=..." in the file would misname the holder to the next reader.
    if (::ftruncate(fd_, 0) != 0) { /* best effort */ }
    ::flock(fd_, LOCK_UN);
    close_fd();
  }

 private:
  void close_fd() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void write_holder(const std::string& who) {
    if (::ftruncate(fd_, 0) != 0) return;
    char buf[256];
    const int n = std::snprintf(buf, sizeof(buf), "pid=%ld exe=%s\n",
                                static_cast<long>(::getpid()), who.c_str());
    if (n <= 0) return;
    if (::pwrite(fd_, buf, static_cast<size_t>(n), 0) < 0) { /* best effort */ }
  }

  // Contents are diagnostic only -- an empty or truncated read just yields a
  // vaguer message, never a wrong decision (flock already made the decision).
  std::string read_holder() const {
    char buf[256];
    const ssize_t n = ::pread(fd_, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return "unknown process";
    buf[n] = '\0';
    std::string s(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == ' ')) s.pop_back();
    return s.empty() ? "unknown process" : s;
  }

  int fd_{-1};
  std::string path_;
};

}  // namespace frankatwin

#endif  // FRANKATWIN_FCI_LOCK_H
