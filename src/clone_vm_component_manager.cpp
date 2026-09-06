// Copyright 2026 play_launch developers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "play_launch_container/clone_vm_component_manager.hpp"

#include <dlfcn.h>
#include <locale.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <utility>

#include "rmw/rmw.h"

namespace play_launch_container
{

namespace
{

constexpr std::size_t kChildStackSize = 8u * 1024u * 1024u;
constexpr int kStopGraceMillis = 5000;

// ── glibc TLS layout ────────────────────────────────────────────────────────
//
// A clone(CLONE_VM) child shares the address space but must NOT share thread-
// local storage: the parent's TLS carries glibc's per-thread malloc cache, and
// two threads consuming one tcache is a double-free waiting to happen. So each
// child is given a fresh block from `_dl_allocate_tls` and started with
// CLONE_SETTLS.
//
// The block still needs the fixups `start_thread` would normally do, and where
// those fields live is architecture-dependent in a way that is easy to get
// wrong. x86_64 and aarch64 use different TLS *variants*, not merely different
// offsets:
//
//   x86_64  (variant II): the thread pointer IS the struct pthread
//   aarch64 (variant I) : the struct pthread sits BELOW the thread pointer
//                         (1984 bytes below, on glibc 2.35)
//
// An earlier attempt at this manager hardcoded the x86_64 numbers, which on
// aarch64 write past the thread pointer into the thread-local variable block --
// silent corruption that hangs rather than crashes. Both numbers are therefore
// measured once at startup instead: the gap by comparing `pthread_self()`
// against the thread pointer, the tid slot by scanning for our own tid.

std::ptrdiff_t g_pd_from_tp = 0;
int g_tid_offset = -1;
bool g_layout_ready = false;

void * thread_pointer()
{
  void * tp = nullptr;
#if defined(__aarch64__)
  __asm__ volatile("mrs %0, tpidr_el0" : "=r"(tp));
#elif defined(__x86_64__)
  syscall(SYS_arch_prctl, 0x1003 /* ARCH_GET_FS */, &tp);
#else
#error "clone-vm container: no thread-pointer accessor for this architecture"
#endif
  return tp;
}

void discover_tls_layout()
{
  if (g_layout_ready) {
    return;
  }
  g_pd_from_tp =
    reinterpret_cast<char *>(pthread_self()) - reinterpret_cast<char *>(thread_pointer());
  const auto tid = static_cast<pid_t>(syscall(SYS_gettid));
  char * pd = reinterpret_cast<char *>(pthread_self());
  for (int off = 0; off < 2048; off += 4) {
    int value = 0;
    std::memcpy(&value, pd + off, sizeof value);
    if (value == tid) {
      g_tid_offset = off;
      break;
    }
  }
  g_layout_ready = true;
}

void * allocate_child_tls()
{
  static auto allocate =
    reinterpret_cast<void * (*)(void *)>(dlsym(RTLD_DEFAULT, "_dl_allocate_tls"));
  return allocate ? allocate(nullptr) : nullptr;
}

void release_child_tls(void * tls)
{
  static auto deallocate =
    reinterpret_cast<void (*)(void *, bool)>(dlsym(RTLD_DEFAULT, "_dl_deallocate_tls"));
  if (deallocate && tls) {
    deallocate(tls, true);
  }
}

/// What the parent hands a child. Lives in the shared address space and is
/// owned by the parent, which frees it after the child is reaped.
struct ChildBootstrap
{
  rclcpp::Executor * executor;
};

int child_main(void * arg)
{
  auto * boot = static_cast<ChildBootstrap *>(arg);

  // Our own identity in the fresh TLS block. glibc's mutex ownership checks
  // and pthread_create read it, and without this the block still carries
  // whatever tid the allocation inherited.
  if (g_tid_offset >= 0) {
    char * pd = reinterpret_cast<char *>(thread_pointer()) + g_pd_from_tp;
    const auto tid = static_cast<pid_t>(syscall(SYS_gettid));
    std::memcpy(pd + g_tid_offset, &tid, sizeof tid);
  }

  // `_dl_allocate_tls` builds the block but does not do what `start_thread`
  // would: the locale pointer and the per-thread ctype tables are left null,
  // and the first `printf("%f")` or `isalpha()` then dereferences them. Both
  // are reachable from ordinary logging, so this must happen before anything
  // in the node runs.
  uselocale(LC_GLOBAL_LOCALE);
  using ctype_init_fn = void (*)();
  if (auto * init = reinterpret_cast<ctype_init_fn>(dlsym(RTLD_DEFAULT, "__ctype_init"))) {
    init();
  }

  // Without CLONE_SIGHAND the child owns its signal table, so this resets only
  // this child. Equivalent to CLONE_CLEAR_SIGHAND, which is not available on
  // every kernel this has to run on.
  //
  // Deliberately no SIGTERM handler: the parent stops children with
  // `executor->cancel()` and never signals them, so a handler here would only
  // create a second path into the same shutdown.
  struct sigaction reset
  {
  };
  reset.sa_handler = SIG_DFL;
  for (int sig = 1; sig < _NSIG; ++sig) {
    sigaction(sig, &reset, nullptr);
  }

  // THIS IS WHAT MAKES THE WHOLE MODE WORK, and it is not obvious.
  //
  // A core-dumping signal -- SIGSEGV, SIGABRT, SIGBUS, the ones a crashing node
  // actually raises -- is not a per-task event when the task shares its mm. To
  // write the dump the kernel needs the address space to itself, so
  // do_coredump() calls zap_threads() on every other task using that mm. In
  // this container that is the manager and every sibling node: the crash we are
  // here to contain takes down exactly what we are trying to protect.
  //
  // Measured both ways on an AGX Orin: SIGSEGV to one child kills the container
  // and its sibling, and with the two calls below it kills only the child that
  // raised it (child Z, sibling S, parent alive). An undumpable task makes
  // do_coredump() bail before it ever reaches the zap.
  //
  // The cost is that clone children produce no core files. That is the trade:
  // a core per node, or a container that survives one. Note the archived
  // clone-vm design asserts "SIGSEGV kills only this child" with no mention of
  // this -- the claim is true only once these two calls are in place.
  prctl(PR_SET_DUMPABLE, 0, 0, 0, 0);
  struct rlimit no_core
  {
    0, 0
  };
  setrlimit(RLIMIT_CORE, &no_core);

  boot->executor->spin();

  // Returning makes the kernel `_exit()` this child: no atexit handlers, no
  // stdio flush. Both would run against a shared address space the parent is
  // still using.
  return 0;
}

}  // namespace

CloneVmComponentManager::CloneVmComponentManager(
  std::weak_ptr<rclcpp::Executor> executor, std::string node_name,
  const rclcpp::NodeOptions & node_options)
: ObservableComponentManager(std::move(executor), std::move(node_name), node_options)
{
  discover_tls_layout();
  RCLCPP_WARN(
    get_logger(), "clone-vm container mode is EXPERIMENTAL. struct pthread at TP%+td, tid at +%d.",
    g_pd_from_tp, g_tid_offset);
  if (g_tid_offset < 0) {
    RCLCPP_ERROR(
      get_logger(),
      "could not locate the tid slot in struct pthread; clone children will not "
      "have a usable identity and glibc will misbehave in them");
  }
}

CloneVmComponentManager::~CloneVmComponentManager()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & [node_id, child] : children_) {
    stop_child(node_id, child);
  }
  children_.clear();
}

bool CloneVmComponentManager::rmw_is_supported(std::string * reason_out)
{
  const char * id = rmw_get_implementation_identifier();
  const std::string rmw = id ? id : "";

  // Cyclone reads a thread-local from a dlopen'd module on its take path. That
  // goes through a dynamic TLS descriptor, whose resolver walks the calling
  // thread's DTV, and a `_dl_allocate_tls` block is not in a state it accepts:
  // the child segfaults in `_dl_tlsdesc_dynamic` under `dds_take` on the first
  // message. Refuse up front rather than crash on the first subscription.
  if (rmw.find("cyclonedds") != std::string::npos) {
    if (reason_out) {
      *reason_out =
        "rmw_cyclonedds_cpp segfaults in a clone(CLONE_VM) child: _dl_tlsdesc_dynamic "
        "under dds_take, on the first message taken. Use RMW_IMPLEMENTATION="
        "rmw_fastrtps_cpp, or --container-mode isolated.";
    }
    return false;
  }

  // Everything else is unmeasured rather than known-good. Say so and continue:
  // this mode exists to be evaluated, and refusing an untested backend would
  // prevent the evaluation it is for.
  if (rmw.find("fastrtps") == std::string::npos && reason_out) {
    *reason_out = "rmw '" + rmw +
                  "' has not been tested under clone(CLONE_VM); only rmw_fastrtps_cpp has. "
                  "Proceeding anyway.";
  }
  return true;
}

void CloneVmComponentManager::add_node_to_executor(uint64_t node_id)
{
  // One executor per node, as ComponentManagerIsolated does -- the difference
  // is what spins it.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_wrappers_[node_id].get_node_base_interface());

  void * tls = allocate_child_tls();
  if (!tls) {
    RCLCPP_ERROR(get_logger(), "node %lu: _dl_allocate_tls failed", node_id);
    return;
  }
  // CLONE_SETTLS wants the value the thread pointer should take, which is the
  // struct-pthread address adjusted back across the variant gap.
  void * child_tp = reinterpret_cast<char *>(tls) - g_pd_from_tp;

  void * stack = mmap(
    nullptr, kChildStackSize, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1,
    0);
  if (stack == MAP_FAILED) {
    RCLCPP_ERROR(
      get_logger(), "node %lu: child stack mmap failed: %s", node_id, std::strerror(errno));
    release_child_tls(tls);
    return;
  }

  // Leaked on purpose if the clone succeeds: the child dereferences it, and
  // there is no point at which the parent knows the child is past that read.
  // One pointer per node, freed never; the alternative is a use-after-free.
  auto * boot = new ChildBootstrap{executor.get()};

  // No CLONE_THREAD. A separate thread group is the entire point: it gives the
  // child its own PID, its own signal disposition, and therefore a SIGSEGV
  // that kills one node instead of the container.
  const int flags = CLONE_VM | CLONE_FS | CLONE_FILES | CLONE_SETTLS | SIGCHLD;
  const pid_t pid = clone(
    child_main, static_cast<char *>(stack) + kChildStackSize, flags, boot, nullptr, child_tp,
    nullptr);

  if (pid < 0) {
    RCLCPP_ERROR(get_logger(), "node %lu: clone failed: %s", node_id, std::strerror(errno));
    munmap(stack, kChildStackSize);
    release_child_tls(tls);
    delete boot;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    children_[node_id] = Child{pid, executor, stack, kChildStackSize, tls};
  }
  RCLCPP_INFO(get_logger(), "node %lu: spinning in clone-vm child pid %d", node_id, pid);
}

void CloneVmComponentManager::remove_node_from_executor(uint64_t node_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = children_.find(node_id);
  if (it == children_.end()) {
    return;
  }
  stop_child(node_id, it->second);
  children_.erase(it);
}

bool CloneVmComponentManager::stop_child(uint64_t node_id, Child & child)
{
  if (child.pid < 0) {
    return true;
  }

  // cancel() writes the executor's interrupt guard condition -- a pipe write --
  // so spin() returns and child_main returns into the kernel's _exit.
  //
  // A signal must NEVER be used here. The child shares this address space, so
  // killing it while it holds a non-robust mutex inside rclcpp or the DDS stack
  // leaves that mutex locked forever, and the container then parks every thread
  // in futex_wait_queue_me during shutdown. That was measured, not assumed.
  if (child.executor) {
    child.executor->cancel();
  }

  int status = 0;
  bool reaped = false;
  for (int elapsed = 0; elapsed < kStopGraceMillis; elapsed += 100) {
    if (waitpid(child.pid, &status, WNOHANG) == child.pid) {
      reaped = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!reaped) {
    // Do not escalate to a signal, and do not reclaim the stack or the TLS: the
    // child is still running in this address space, and unmapping the stack it
    // is executing on is worse than the leak. Report it and move on.
    RCLCPP_ERROR(
      get_logger(),
      "node %lu: clone-vm child pid %d did not exit after cancel(); leaking its stack and "
      "TLS deliberately -- it is still running in this address space and cannot be "
      "signalled without deadlocking the container",
      node_id, child.pid);
    return false;
  }

  munmap(child.stack, child.stack_size);
  release_child_tls(child.tls);
  child.pid = -1;
  RCLCPP_INFO(get_logger(), "node %lu: clone-vm child exited", node_id);
  return true;
}

}  // namespace play_launch_container
