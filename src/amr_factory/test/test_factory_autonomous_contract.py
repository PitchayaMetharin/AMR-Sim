"""Offline contract probes for the factory's autonomous terminal seams.

These tests compile the production method bodies against small dependency seams.
They do not initialize ROS or assert implementation text.
"""

from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "factory_supervisor_node.cpp").read_text()


def _method(start, end):
    begin = SOURCE.index(start)
    finish = SOURCE.index(end, begin)
    return SOURCE[begin:finish]


def _run_cpp(name, program):
    with tempfile.TemporaryDirectory(prefix="factory-autonomous-") as directory:
        directory = Path(directory)
        binary = directory / name
        compiled = subprocess.run(
            ["g++", "-std=c++17", "-pthread", "-x", "c++", "-", "-o", str(binary)],
            input=program,
            text=True,
            capture_output=True,
        )
        assert compiled.returncode == 0, compiled.stderr
        result = subprocess.run([str(binary)], text=True, capture_output=True)
        assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
        return result.stdout


def _sequence_probe():
    body = _method("  void sequence_loop()", "  bool cancel_home_navigation_goal(")
    body = body.replace(
        "    std::shared_ptr<SequenceGoalHandle> goal;\n",
        "    if (inject_unsafe_before_terminal) {\n"
        "      ready=false;\n"
        "      held_product_=true;\n"
        "      current_product_attached_=true;\n"
        "    }\n"
        "    std::shared_ptr<SequenceGoalHandle> goal;\n",
        1,
    )
    return r'''
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
using namespace std::chrono_literals;
struct Execute { struct Result {
  enum {SUCCESS=0, INVALID_REQUEST=1, DEPENDENCY_UNAVAILABLE=2,
    PREPARATION_FAILED=3, EXECUTION_FAILED=4, CANCELED=5,
    INTERLOCK_FAILED=6, RETAINED_PRODUCT_FAULT=7};
  int outcome=SUCCESS; bool delivered=true; std::string message;
}; };
struct RunSequence { struct Result {
  enum {SUCCESS=0, STOPPED=1, CANCELED=2, JOB_FAILED=3, HOME_FAILED=4,
    INTERLOCK_FAILED=5, INVALID_REQUEST=6, DEPENDENCY_UNAVAILABLE=7};
  int outcome=SUCCESS; unsigned completed_jobs=0, completed_cycles=0;
  std::string message;
}; };
struct SequenceGoalHandle {
  RunSequence::Result terminal; bool canceling=false; int terminal_kind=0;
  bool inject_late_cancel=false;
  bool *terminal_decided=nullptr, *terminal_decided_seen=nullptr;
  bool *sequence_cancel_requested=nullptr, *active_cancel_requested=nullptr;
  bool is_active() {
    if (terminal_decided_seen) *terminal_decided_seen = terminal_decided && *terminal_decided;
    if (inject_late_cancel && terminal_decided && *terminal_decided) {
      if (sequence_cancel_requested) *sequence_cancel_requested=true;
      if (active_cancel_requested) *active_cancel_requested=true;
    }
    return true;
  }
  bool is_canceling() const {return canceling;}
  void succeed(std::shared_ptr<RunSequence::Result> r) {terminal=*r; terminal_kind=1;}
  void canceled(std::shared_ptr<RunSequence::Result> r) {terminal=*r; terminal_kind=2;}
  void abort(std::shared_ptr<RunSequence::Result> r) {terminal=*r; terminal_kind=3;}
};
struct Record {std::string product_id;};
struct Probe {
  std::mutex mutex_;
  std::vector<std::string> sequence_template_{"pickup_a"};
  std::unordered_map<std::string,Record> products_by_station_{{"pickup_a",{"101"}},
    {"pickup_b",{"102"}}};
  std::shared_ptr<SequenceGoalHandle> sequence_goal_=std::make_shared<SequenceGoalHandle>();
  unsigned sequence_index_=0,current_cycle_=0,completed_cycles_=0,completed_jobs_=0;
  unsigned sequence_cycle_limit_=1;
  std::string current_station_id_,current_destination_id_,current_product_id_,active_product_id_,detail_;
  std::string sequence_final_station_="home";
  bool sequence_cancel_requested_=false,active_cancel_requested_=false;
  bool graceful_stop_requested_=false,fault_latched_=false,held_product_=false;
  bool current_product_attached_=false,stopping_=false,home_cancel_confirmation_failed_=false;
  bool home_interlock_failed_=false;
  bool sequence_active_=true,sequence_terminal_decided_=false,ready=true;
  bool cancel_after_first=false,cancel_during_proof=false,fault_after_first=false;
  bool cancel_during_home=false;
  bool attachment_after_proof=false,attachment_after_complete=false;
  bool inject_unsafe_before_terminal=false,stop_after_complete=false;
  std::vector<bool> proof_results;
  std::size_t proof_index=0;
  unsigned execute_calls=0,wait_calls=0;
  int phase_=0,last_outcome_=0,home_calls=0;
  struct Call {Execute::Result result; bool canceled=false;};
  Call scripted;
  Call call_execute_cycle(const std::string&,const std::string&,const std::string&) {
    ++execute_calls;
    return scripted;
  }
  bool is_cancel_requested() {return active_cancel_requested_ || stopping_;}
  bool wait_for_manipulator_ready(std::chrono::seconds) {
    ++wait_calls;
    if (proof_index < proof_results.size()) ready=proof_results[proof_index++];
    const bool proof=ready;
    if (cancel_during_proof && wait_calls == 1U) {
      sequence_cancel_requested_=true;
      active_cancel_requested_=true;
    }
    if (attachment_after_proof && wait_calls == 1U) {
      ready=false;
      held_product_=true;
      current_product_attached_=true;
    }
    return proof;
  }
  bool manipulator_ready_locked() const {return ready;}
  bool safe_empty_stow_locked() const {
    return !fault_latched_ && !held_product_ && !current_product_attached_ &&
      manipulator_ready_locked();
  }
  bool navigate_home() {
    ++home_calls;
    if (cancel_during_home) {
      sequence_cancel_requested_=true;
      active_cancel_requested_=true;
      return false;
    }
    return true;
  }
  void publish_sequence_feedback(unsigned phase,const std::string&) {
    if (phase == 2U && execute_calls == 1U) {
      if (cancel_after_first) {
        sequence_cancel_requested_=true;
        active_cancel_requested_=true;
      }
      if (fault_after_first) fault_latched_=true;
      if (attachment_after_complete) {
        ready=false;
        held_product_=true;
        current_product_attached_=true;
      }
      if (stop_after_complete) graceful_stop_requested_=true;
    }
  }
''' + body + r'''
};
bool expect_sequence(Probe & probe, int outcome, int terminal_kind, unsigned execute_calls,
  unsigned wait_calls, unsigned completed_jobs, unsigned completed_cycles, bool fault_latched,
  bool held_product, bool product_attached, unsigned expected_home_calls=0) {
  auto goal=probe.sequence_goal_;
  probe.sequence_loop();
  return goal->terminal.outcome==outcome && goal->terminal_kind==terminal_kind &&
    probe.execute_calls==execute_calls && probe.wait_calls==wait_calls &&
    goal->terminal.completed_jobs==completed_jobs &&
    goal->terminal.completed_cycles==completed_cycles &&
    probe.fault_latched_==fault_latched && probe.held_product_==held_product &&
    probe.current_product_attached_==product_attached && probe.home_calls==expected_home_calls;
}
int main() {
  Probe success; auto success_goal=success.sequence_goal_; success.sequence_loop();
  if(success.home_calls!=1 || success_goal->terminal.outcome!=RunSequence::Result::SUCCESS ||
     success_goal->terminal_kind!=1 || success_goal->terminal.completed_jobs!=1) return 1;

  Probe failed; auto failed_goal=failed.sequence_goal_;
  failed.scripted.result.outcome=Execute::Result::DEPENDENCY_UNAVAILABLE;
  failed.scripted.result.delivered=false; failed.sequence_loop();
  if(failed.home_calls!=0 || failed_goal->terminal.outcome!=RunSequence::Result::DEPENDENCY_UNAVAILABLE) return 2;

  Probe proof; auto proof_goal=proof.sequence_goal_; proof.ready=false;
  proof.sequence_loop();
  if(proof.home_calls!=0 || !proof.fault_latched_ ||
     proof_goal->terminal.outcome!=RunSequence::Result::INTERLOCK_FAILED) return 3;

  Probe preserved; auto preserved_goal=preserved.sequence_goal_; preserved.ready=false;
  preserved.fault_latched_=true; preserved.sequence_loop();
  if(preserved.home_calls!=0 || !preserved.fault_latched_ ||
     preserved_goal->terminal.outcome!=RunSequence::Result::INTERLOCK_FAILED) return 4;

  Probe retained; auto retained_goal=retained.sequence_goal_;
  retained.scripted.canceled=true;
  retained.scripted.result.outcome=Execute::Result::INTERLOCK_FAILED;
  retained.scripted.result.delivered=false; retained.sequence_loop();
  if(retained.home_calls!=0 || retained_goal->terminal.outcome!=RunSequence::Result::INTERLOCK_FAILED ||
     !retained.fault_latched_) return 5;

  Probe executing_cancel; auto executing_goal=executing_cancel.sequence_goal_;
  executing_cancel.scripted.canceled=true;
  executing_cancel.scripted.result.outcome=Execute::Result::CANCELED;
  executing_cancel.sequence_goal_->canceling=false; executing_cancel.sequence_loop();
  if(executing_goal->terminal.outcome!=RunSequence::Result::CANCELED || executing_goal->terminal_kind!=3) return 6;

  Probe canceling; auto canceling_goal=canceling.sequence_goal_;
  canceling.scripted.canceled=true;
  canceling.scripted.result.outcome=Execute::Result::CANCELED;
  canceling.sequence_goal_->canceling=true; canceling.sequence_loop();
  if(canceling_goal->terminal.outcome!=RunSequence::Result::CANCELED || canceling_goal->terminal_kind!=2) return 7;

  Probe canceled_stale;
  canceled_stale.sequence_final_station_="";
  canceled_stale.scripted.canceled=true;
  canceled_stale.scripted.result.outcome=Execute::Result::CANCELED;
  canceled_stale.attachment_after_proof=true;
  canceled_stale.proof_results={true};
  if(!expect_sequence(canceled_stale,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,true,true)) return 8;
  Probe canceled_stale_canceling;
  canceled_stale_canceling.sequence_final_station_="";
  canceled_stale_canceling.scripted.canceled=true;
  canceled_stale_canceling.scripted.result.outcome=Execute::Result::CANCELED;
  canceled_stale_canceling.attachment_after_proof=true;
  canceled_stale_canceling.proof_results={true};
  canceled_stale_canceling.sequence_goal_->canceling=true;
  if(!expect_sequence(canceled_stale_canceling,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,true,true)) return 9;

  Probe before_executing;
  before_executing.sequence_cancel_requested_=true;
  if(!expect_sequence(before_executing,RunSequence::Result::CANCELED,3,0,1,0,0,false,false,false)) return 10;
  Probe before_canceling;
  before_canceling.sequence_cancel_requested_=true;
  before_canceling.sequence_goal_->canceling=true;
  if(!expect_sequence(before_canceling,RunSequence::Result::CANCELED,2,0,1,0,0,false,false,false)) return 11;

  Probe before_missing;
  before_missing.sequence_cancel_requested_=true;
  before_missing.proof_results={false};
  before_missing.held_product_=true;
  before_missing.current_product_attached_=true;
  before_missing.completed_jobs_=4;
  before_missing.completed_cycles_=2;
  if(!expect_sequence(before_missing,RunSequence::Result::INTERLOCK_FAILED,3,0,1,4,2,true,true,true)) return 12;
  Probe before_missing_canceling;
  before_missing_canceling.sequence_cancel_requested_=true;
  before_missing_canceling.sequence_goal_->canceling=true;
  before_missing_canceling.proof_results={false};
  if(!expect_sequence(before_missing_canceling,RunSequence::Result::INTERLOCK_FAILED,3,0,1,0,0,true,false,false)) return 13;

  Probe before_fault;
  before_fault.sequence_cancel_requested_=true;
  before_fault.fault_latched_=true;
  if(!expect_sequence(before_fault,RunSequence::Result::INTERLOCK_FAILED,3,0,1,0,0,true,false,false)) return 14;
  Probe before_fault_canceling;
  before_fault_canceling.sequence_cancel_requested_=true;
  before_fault_canceling.sequence_goal_->canceling=true;
  before_fault_canceling.fault_latched_=true;
  if(!expect_sequence(before_fault_canceling,RunSequence::Result::INTERLOCK_FAILED,3,0,1,0,0,true,false,false)) return 15;

  Probe between_executing;
  between_executing.sequence_template_={"pickup_a","pickup_b"};
  between_executing.cancel_after_first=true;
  between_executing.proof_results={true,true};
  if(!expect_sequence(between_executing,RunSequence::Result::CANCELED,3,1,2,1,0,false,false,false)) return 16;
  Probe between_canceling;
  between_canceling.sequence_template_={"pickup_a","pickup_b"};
  between_canceling.cancel_after_first=true;
  between_canceling.sequence_goal_->canceling=true;
  between_canceling.proof_results={true,true};
  if(!expect_sequence(between_canceling,RunSequence::Result::CANCELED,2,1,2,1,0,false,false,false)) return 17;
  Probe between_missing;
  between_missing.sequence_template_={"pickup_a","pickup_b"};
  between_missing.cancel_after_first=true;
  between_missing.proof_results={true,false};
  if(!expect_sequence(between_missing,RunSequence::Result::INTERLOCK_FAILED,3,1,2,1,0,true,false,false)) return 18;
  Probe between_fault;
  between_fault.sequence_template_={"pickup_a","pickup_b"};
  between_fault.cancel_after_first=true;
  between_fault.fault_after_first=true;
  between_fault.proof_results={true,true};
  if(!expect_sequence(between_fault,RunSequence::Result::INTERLOCK_FAILED,3,1,2,1,0,true,false,false)) return 19;

  Probe final_executing;
  final_executing.sequence_final_station_="";
  final_executing.cancel_during_proof=true;
  final_executing.proof_results={true};
  if(!expect_sequence(final_executing,RunSequence::Result::CANCELED,3,1,1,1,1,false,false,false)) return 20;
  Probe final_canceling;
  final_canceling.sequence_final_station_="";
  final_canceling.cancel_during_proof=true;
  final_canceling.sequence_goal_->canceling=true;
  final_canceling.proof_results={true};
  if(!expect_sequence(final_canceling,RunSequence::Result::CANCELED,2,1,1,1,1,false,false,false)) return 21;
  Probe final_missing;
  final_missing.sequence_final_station_="";
  final_missing.cancel_during_proof=true;
  final_missing.proof_results={false};
  if(!expect_sequence(final_missing,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,false,false)) return 22;

  Probe home_cancel_executing;
  home_cancel_executing.cancel_during_home=true;
  home_cancel_executing.proof_results={true,true};
  if(!expect_sequence(home_cancel_executing,RunSequence::Result::CANCELED,3,1,2,1,1,false,false,false,1)) return 23;
  Probe home_cancel_canceling;
  home_cancel_canceling.cancel_during_home=true;
  home_cancel_canceling.sequence_goal_->canceling=true;
  home_cancel_canceling.proof_results={true,true};
  if(!expect_sequence(home_cancel_canceling,RunSequence::Result::CANCELED,2,1,2,1,1,false,false,false,1)) return 24;
  Probe home_cancel_missing;
  home_cancel_missing.cancel_during_home=true;
  home_cancel_missing.proof_results={true,false};
  if(!expect_sequence(home_cancel_missing,RunSequence::Result::INTERLOCK_FAILED,3,1,2,1,1,true,false,false,1)) return 25;

  Probe newer_attachment;
  newer_attachment.sequence_final_station_="";
  newer_attachment.attachment_after_proof=true;
  newer_attachment.proof_results={true};
  if(!expect_sequence(newer_attachment,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,true,true)) return 26;

  Probe final_feedback_executing;
  final_feedback_executing.sequence_final_station_="";
  final_feedback_executing.cancel_after_first=true;
  final_feedback_executing.proof_results={true,true};
  if(!expect_sequence(final_feedback_executing,RunSequence::Result::CANCELED,3,1,2,1,1,false,false,false)) return 27;
  Probe final_feedback_canceling;
  final_feedback_canceling.sequence_final_station_="";
  final_feedback_canceling.cancel_after_first=true;
  final_feedback_canceling.sequence_goal_->canceling=true;
  final_feedback_canceling.proof_results={true,true};
  if(!expect_sequence(final_feedback_canceling,RunSequence::Result::CANCELED,2,1,2,1,1,false,false,false)) return 28;
  Probe final_feedback_missing;
  final_feedback_missing.sequence_final_station_="";
  final_feedback_missing.cancel_after_first=true;
  final_feedback_missing.proof_results={true,false};
  if(!expect_sequence(final_feedback_missing,RunSequence::Result::INTERLOCK_FAILED,3,1,2,1,1,true,false,false)) return 29;

  Probe late_cancel;
  late_cancel.sequence_final_station_="";
  late_cancel.sequence_goal_->inject_late_cancel=true;
  late_cancel.sequence_goal_->terminal_decided=&late_cancel.sequence_terminal_decided_;
  late_cancel.sequence_goal_->terminal_decided_seen=&late_cancel.sequence_terminal_decided_;
  late_cancel.sequence_goal_->sequence_cancel_requested=&late_cancel.sequence_cancel_requested_;
  late_cancel.sequence_goal_->active_cancel_requested=&late_cancel.active_cancel_requested_;
  auto late_cancel_goal=late_cancel.sequence_goal_;
  late_cancel.sequence_loop();
  if(late_cancel_goal->terminal.outcome!=RunSequence::Result::SUCCESS ||
     late_cancel_goal->terminal_kind!=1 || !late_cancel.sequence_terminal_decided_) return 30;

  Probe execution_missing;
  execution_missing.sequence_final_station_="";
  execution_missing.scripted.result.outcome=Execute::Result::EXECUTION_FAILED;
  execution_missing.scripted.result.delivered=false;
  execution_missing.proof_results={false};
  if(!expect_sequence(execution_missing,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,false,false)) return 31;

  Probe dependency_missing;
  dependency_missing.sequence_final_station_="";
  dependency_missing.scripted.result.outcome=Execute::Result::DEPENDENCY_UNAVAILABLE;
  dependency_missing.scripted.result.delivered=false;
  dependency_missing.proof_results={false};
  if(!expect_sequence(dependency_missing,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,false,false)) return 32;

  Probe execution_safe;
  execution_safe.sequence_final_station_="";
  execution_safe.scripted.result.outcome=Execute::Result::EXECUTION_FAILED;
  execution_safe.scripted.result.delivered=false;
  execution_safe.proof_results={true};
  if(!expect_sequence(execution_safe,RunSequence::Result::JOB_FAILED,3,1,1,0,0,false,false,false)) return 33;

  Probe dependency_safe;
  dependency_safe.sequence_final_station_="";
  dependency_safe.scripted.result.outcome=Execute::Result::DEPENDENCY_UNAVAILABLE;
  dependency_safe.scripted.result.delivered=false;
  dependency_safe.proof_results={true};
  if(!expect_sequence(dependency_safe,RunSequence::Result::DEPENDENCY_UNAVAILABLE,3,1,1,0,0,false,false,false)) return 34;

  Probe attachment_between_jobs;
  attachment_between_jobs.sequence_template_={"pickup_a","pickup_b"};
  attachment_between_jobs.sequence_final_station_="";
  attachment_between_jobs.attachment_after_complete=true;
  attachment_between_jobs.proof_results={true};
  if(!expect_sequence(attachment_between_jobs,RunSequence::Result::INTERLOCK_FAILED,3,1,2,1,0,true,true,true)) return 35;

  Probe early_cancel_attachment;
  early_cancel_attachment.sequence_final_station_="";
  early_cancel_attachment.sequence_cancel_requested_=true;
  early_cancel_attachment.proof_results={true};
  early_cancel_attachment.attachment_after_proof=true;
  if(!expect_sequence(early_cancel_attachment,RunSequence::Result::INTERLOCK_FAILED,3,0,1,0,0,true,true,true)) return 36;

  Probe delivered_safe;
  delivered_safe.sequence_final_station_="";
  delivered_safe.scripted.result.outcome=Execute::Result::SUCCESS;
  delivered_safe.scripted.result.delivered=false;
  delivered_safe.proof_results={true};
  if(!expect_sequence(delivered_safe,RunSequence::Result::JOB_FAILED,3,1,1,0,0,false,false,false)) return 37;

  Probe delivered_cancel;
  delivered_cancel.sequence_final_station_="";
  delivered_cancel.scripted.canceled=true;
  delivered_cancel.scripted.result.outcome=Execute::Result::SUCCESS;
  delivered_cancel.scripted.result.delivered=false;
  delivered_cancel.proof_results={true};
  if(!expect_sequence(delivered_cancel,RunSequence::Result::JOB_FAILED,3,1,1,0,0,false,false,false)) return 38;

  Probe delivered_missing;
  delivered_missing.sequence_final_station_="";
  delivered_missing.scripted.result.outcome=Execute::Result::SUCCESS;
  delivered_missing.scripted.result.delivered=false;
  delivered_missing.proof_results={false};
  if(!expect_sequence(delivered_missing,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,false,false)) return 39;

  Probe retained_explicit;
  retained_explicit.sequence_final_station_="";
  retained_explicit.scripted.result.outcome=Execute::Result::RETAINED_PRODUCT_FAULT;
  retained_explicit.scripted.result.delivered=false;
  auto retained_explicit_goal=retained_explicit.sequence_goal_;
  retained_explicit.sequence_loop();
  if(retained_explicit_goal->terminal.outcome!=RunSequence::Result::INTERLOCK_FAILED ||
     !retained_explicit.fault_latched_ || !retained_explicit.held_product_ ||
     !retained_explicit.current_product_attached_) return 40;

  Probe final_attachment;
  final_attachment.sequence_final_station_="";
  final_attachment.attachment_after_complete=true;
  if(!expect_sequence(final_attachment,RunSequence::Result::INTERLOCK_FAILED,3,1,1,1,1,true,true,true)) return 41;

  Probe stopped_attachment;
  stopped_attachment.sequence_final_station_="";
  stopped_attachment.attachment_after_complete=true;
  stopped_attachment.stop_after_complete=true;
  if(!expect_sequence(stopped_attachment,RunSequence::Result::INTERLOCK_FAILED,3,1,1,1,1,true,true,true)) return 42;

  Probe generic_terminal_unsafe;
  generic_terminal_unsafe.sequence_final_station_="";
  generic_terminal_unsafe.scripted.result.outcome=Execute::Result::EXECUTION_FAILED;
  generic_terminal_unsafe.scripted.result.delivered=false;
  generic_terminal_unsafe.proof_results={true};
  generic_terminal_unsafe.inject_unsafe_before_terminal=true;
  if(!expect_sequence(generic_terminal_unsafe,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,true,true)) return 43;

  Probe dependency_terminal_unsafe;
  dependency_terminal_unsafe.sequence_final_station_="";
  dependency_terminal_unsafe.scripted.result.outcome=Execute::Result::DEPENDENCY_UNAVAILABLE;
  dependency_terminal_unsafe.scripted.result.delivered=false;
  dependency_terminal_unsafe.proof_results={true};
  dependency_terminal_unsafe.inject_unsafe_before_terminal=true;
  if(!expect_sequence(dependency_terminal_unsafe,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,true,true)) return 44;

  Probe delivered_terminal_unsafe;
  delivered_terminal_unsafe.sequence_final_station_="";
  delivered_terminal_unsafe.scripted.result.outcome=Execute::Result::SUCCESS;
  delivered_terminal_unsafe.scripted.result.delivered=false;
  delivered_terminal_unsafe.proof_results={true};
  delivered_terminal_unsafe.inject_unsafe_before_terminal=true;
  if(!expect_sequence(delivered_terminal_unsafe,RunSequence::Result::INTERLOCK_FAILED,3,1,1,0,0,true,true,true)) return 45;

  std::cout << "R5/R6/R7 sequence, dispatch gate, generic-failure proof, attachment, and action-state probes passed\n";
}
'''


def _transport_cancel_probe():
    body = _method("  void finish_transport_canceled(", "  void finish_transport_failed(")
    return r'''
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
struct Transport { struct Result {
  enum {SUCCESS=0, CANCELED=1, INTERLOCK_FAILED=2};
  bool delivered=true; int outcome=SUCCESS; std::string message;
}; };
struct TransportGoalHandle {
  bool active=true, canceling=false; int terminal_kind=0; Transport::Result terminal;
  bool is_active() const {return active;}
  bool is_canceling() const {return canceling;}
  void canceled(std::shared_ptr<Transport::Result> r) {terminal=*r; terminal_kind=2;}
  void abort(std::shared_ptr<Transport::Result> r) {terminal=*r; terminal_kind=3;}
};
struct Probe {
  std::mutex mutex_; bool held_product_=false; std::string detail_; int last_outcome_=0, phase_=0;
''' + body + r'''
};
int main() {
  Probe executing; auto executing_goal=std::make_shared<TransportGoalHandle>();
  executing_goal->canceling=false; executing.finish_transport_canceled(executing_goal,"trigger cancel");
  if(executing_goal->terminal_kind!=3 || executing_goal->terminal.outcome!=Transport::Result::CANCELED) return 1;
  Probe canceling; auto canceling_goal=std::make_shared<TransportGoalHandle>();
  canceling_goal->canceling=true; canceling.finish_transport_canceled(canceling_goal,"action cancel");
  if(canceling_goal->terminal_kind!=2 || canceling_goal->terminal.outcome!=Transport::Result::CANCELED) return 2;
  std::cout << "ordinary Trigger/EXECUTING abort and CANCELING canceled passed\n";
}
'''


def _sequence_cancel_admission_probe():
    body = _method("  rclcpp_action::CancelResponse handle_sequence_cancel(",
        "  void handle_sequence_accepted(")
    return r'''
#include <iostream>
#include <memory>
#include <mutex>
namespace rclcpp_action { enum class CancelResponse {REJECT, ACCEPT}; }
struct SequenceGoalHandle {
  bool active=true;
  bool is_active() const {return active;}
};
struct Probe {
  std::mutex mutex_;
  std::shared_ptr<SequenceGoalHandle> sequence_goal_;
  bool sequence_terminal_decided_=false;
  bool sequence_cancel_requested_=false,active_cancel_requested_=false;
''' + body + r'''
};
int main() {
  Probe probe; auto goal=std::make_shared<SequenceGoalHandle>(); probe.sequence_goal_=goal;
  probe.sequence_terminal_decided_=true;
  if(probe.handle_sequence_cancel(goal)!=rclcpp_action::CancelResponse::REJECT ||
     probe.sequence_cancel_requested_ || probe.active_cancel_requested_) return 1;
  probe.sequence_terminal_decided_=false;
  if(probe.handle_sequence_cancel(goal)!=rclcpp_action::CancelResponse::ACCEPT ||
     !probe.sequence_cancel_requested_ || !probe.active_cancel_requested_) return 2;
  std::cout << "terminal cancellation admission closes atomically\n";
}
'''


def _home_cancel_probe():
    body = _method("  void execute_home(", "  void cancel_current_manipulation()")
    return r'''
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
using namespace std::chrono_literals;
struct NavigateStation { struct Result {
  enum {SUCCESS=0, CANCELED=1, NAVIGATION_FAILED=2, INTERLOCK_FAILED=3};
  int outcome=SUCCESS; std::string message;
}; struct Feedback {int phase=0; std::string current_station_id;}; };
struct HomeGoalHandle {
  bool active=true, canceling=false; int terminal_kind=0; NavigateStation::Result terminal;
  bool is_active() const {return active;}
  bool is_canceling() const {return canceling;}
  void publish_feedback(std::shared_ptr<NavigateStation::Feedback>) {}
  void succeed(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=1;}
  void canceled(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=2;}
  void abort(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=3;}
};
struct Probe {
  std::mutex mutex_; bool home_cancel_confirmation_failed_=false, home_interlock_failed_=false;
  bool home_cancel_requested_=true, fault_latched_=false;
  bool sequence_cancel_requested_=false, stopping_=false;
  bool held_product_=false, current_product_attached_=false, ready=true;
  unsigned wait_calls=0;
  bool home_active_=true; int phase_=0, last_outcome_=0; std::shared_ptr<HomeGoalHandle> home_goal_;
  bool navigate_home() {return false;}
  bool wait_for_manipulator_ready(std::chrono::seconds) {++wait_calls; return ready;}
  bool manipulator_ready_locked() const {return ready;}
  bool safe_empty_stow_locked() const {
    return !fault_latched_ && !held_product_ && !current_product_attached_ &&
      manipulator_ready_locked();
  }
''' + body + r'''
};
int main() {
  Probe executing; auto executing_goal=std::make_shared<HomeGoalHandle>(); executing_goal->canceling=false;
  executing.home_goal_=executing_goal; executing.execute_home(executing_goal);
  if(executing_goal->terminal_kind!=3 || executing_goal->terminal.outcome!=NavigateStation::Result::CANCELED ||
     executing.wait_calls!=1 || executing.fault_latched_) return 1;
  Probe canceling; auto canceling_goal=std::make_shared<HomeGoalHandle>(); canceling_goal->canceling=true;
  canceling.home_goal_=canceling_goal; canceling.execute_home(canceling_goal);
  if(canceling_goal->terminal_kind!=2 || canceling_goal->terminal.outcome!=NavigateStation::Result::CANCELED ||
     canceling.wait_calls!=1 || canceling.fault_latched_) return 2;
  Probe unsafe; auto unsafe_goal=std::make_shared<HomeGoalHandle>(); unsafe.ready=false;
  unsafe.home_goal_=unsafe_goal; unsafe.execute_home(unsafe_goal);
  if(unsafe_goal->terminal_kind!=3 || unsafe_goal->terminal.outcome!=NavigateStation::Result::INTERLOCK_FAILED ||
     unsafe.wait_calls!=1 || !unsafe.fault_latched_) return 3;
  Probe attached; auto attached_goal=std::make_shared<HomeGoalHandle>();
  attached.held_product_=true; attached.current_product_attached_=true;
  attached.home_goal_=attached_goal; attached.execute_home(attached_goal);
  if(attached_goal->terminal_kind!=3 || attached_goal->terminal.outcome!=NavigateStation::Result::INTERLOCK_FAILED ||
     attached.wait_calls!=1 || !attached.fault_latched_) return 4;
  std::cout << "home EXECUTING/CANCELING proof controls and unsafe attachment fault passed\n";
}
'''


def _home_fault_probe():
    body = _method("  void execute_home(", "  void cancel_current_manipulation()")
    return r'''
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
using namespace std::chrono_literals;
struct NavigateStation { struct Result {
  enum {SUCCESS=0, CANCELED=1, NAVIGATION_FAILED=2, INTERLOCK_FAILED=3};
  int outcome=SUCCESS; std::string message;
}; struct Feedback {int phase=0; std::string current_station_id;}; };
struct HomeGoalHandle {
  bool active=true; bool canceling=false; int terminal_kind=0; NavigateStation::Result terminal;
  bool is_active() const {return active;}
  bool is_canceling() const {return canceling;}
  void publish_feedback(std::shared_ptr<NavigateStation::Feedback>) {}
  void succeed(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=1;}
  void canceled(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=2;}
  void abort(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=3;}
};
struct Probe {
  std::mutex mutex_; bool home_cancel_confirmation_failed_=false, home_interlock_failed_=false;
  bool home_cancel_requested_=false, sequence_cancel_requested_=false, stopping_=false;
  bool home_active_=true, fault_latched_=false;
  bool held_product_=false, current_product_attached_=false, ready=true;
  int phase_=0, last_outcome_=0; std::shared_ptr<HomeGoalHandle> home_goal_;
  bool navigate_home() {return false;}
  bool wait_for_manipulator_ready(std::chrono::seconds) {return ready;}
  bool manipulator_ready_locked() const {return ready;}
  bool safe_empty_stow_locked() const {
    return !fault_latched_ && !held_product_ && !current_product_attached_ &&
      manipulator_ready_locked();
  }
''' + body + r'''
};
int main() {
  Probe interlock; auto interlock_goal=std::make_shared<HomeGoalHandle>();
  interlock.home_goal_=interlock_goal; interlock.home_interlock_failed_=true;
  interlock.execute_home(interlock_goal);
  if(!interlock.fault_latched_ ||
     interlock_goal->terminal.outcome!=NavigateStation::Result::INTERLOCK_FAILED) return 1;

  Probe cancellation; auto cancellation_goal=std::make_shared<HomeGoalHandle>();
  cancellation.home_goal_=cancellation_goal; cancellation.home_cancel_confirmation_failed_=true;
  cancellation.execute_home(cancellation_goal);
  if(!cancellation.fault_latched_ ||
     cancellation_goal->terminal.outcome!=NavigateStation::Result::INTERLOCK_FAILED) return 2;
  std::cout << "standalone home failures latch fault\n";
}
'''


def _cycle_wait_probe():
    body = _method("  ExecuteCall call_execute_cycle(", "  void sequence_loop()")
    return r'''
#include <chrono>
#include <future>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <mutex>
#include <string>
using namespace std::chrono_literals;
namespace rclcpp { inline bool ok_value=true; inline bool ok() {return ok_value;} }
namespace rclcpp_action { enum class ResultCode {SUCCEEDED, ABORTED, CANCELED}; }
struct Execute {
  struct Goal {std::string pickup_station_id, destination_station_id;};
  struct Result {enum {SUCCESS=0, DEPENDENCY_UNAVAILABLE=1, EXECUTION_FAILED=2, CANCELED=3,
    INTERLOCK_FAILED=4}; int outcome=SUCCESS; bool delivered=false; std::string product_id, message;};
};
struct ExecuteGoalHandle {};
namespace rclcpp_action {
template<class Action>
struct Client {
  struct SendGoalOptions {
    std::function<void(std::shared_ptr<ExecuteGoalHandle>)> goal_response_callback;
  };
};
}
struct WrappedResult {rclcpp_action::ResultCode code=rclcpp_action::ResultCode::SUCCEEDED;
  std::shared_ptr<Execute::Result> result;};
struct SendFuture {
  std::shared_ptr<ExecuteGoalHandle> handle=std::make_shared<ExecuteGoalHandle>();
  template<class Rep,class Period> std::future_status wait_for(std::chrono::duration<Rep,Period>)
    {return std::future_status::ready;}
  std::shared_ptr<ExecuteGoalHandle> get() {return handle;}
};
struct ResultFuture {
  bool *cancel_sent=nullptr; WrappedResult wrapped;
  template<class Rep,class Period> std::future_status wait_for(std::chrono::duration<Rep,Period> duration) {
    if (cancel_sent && *cancel_sent) return std::future_status::timeout;
    return std::future_status::ready;
  }
  WrappedResult get() {return wrapped;}
};
struct Client {
  using Options=rclcpp_action::Client<Execute>::SendGoalOptions;
  ResultFuture result_future; int send_calls=0;
  bool accept_immediately=true;
  std::function<void(std::shared_ptr<ExecuteGoalHandle>)> response_callback;
  bool *cancel_sent=nullptr, *downstream_live=nullptr;
  enum class WaitMutation {NONE, STALE, ATTACHMENT, PRIOR_FAULT};
  bool server_available=true; WaitMutation wait_mutation=WaitMutation::NONE;
  bool *cancel_during_wait=nullptr, *active_cancel_requested=nullptr;
  bool *ready=nullptr, *held_product=nullptr, *current_product_attached=nullptr;
  bool *fault_latched=nullptr;
  bool wait_for_action_server(std::chrono::seconds) {
    if (cancel_during_wait && *cancel_during_wait && active_cancel_requested)
      *active_cancel_requested=true;
    if (wait_mutation == WaitMutation::STALE && ready) *ready=false;
    if (wait_mutation == WaitMutation::ATTACHMENT) {
      if (ready) *ready=false;
      if (held_product) *held_product=true;
      if (current_product_attached) *current_product_attached=true;
    }
    if (wait_mutation == WaitMutation::PRIOR_FAULT && fault_latched) *fault_latched=true;
    return server_available;
  }
  SendFuture async_send_goal(Execute::Goal, Options options) {
    ++send_calls;
    response_callback=std::move(options.goal_response_callback);
    if (downstream_live) *downstream_live=true;
    if (accept_immediately && response_callback) response_callback(std::make_shared<ExecuteGoalHandle>());
    return {};
  }
  ResultFuture async_get_result(std::shared_ptr<ExecuteGoalHandle>) {
    auto future=result_future; future.cancel_sent=cancel_sent; return future;
  }
};
struct Probe {
  using ExecuteCall=struct {Execute::Result result; bool accepted=false; bool canceled=false;};
  std::mutex mutex_; std::shared_ptr<Client> execute_client_=std::make_shared<Client>();
  std::shared_ptr<ExecuteGoalHandle> current_execute_goal_; bool active_cancel_requested_=false;
  bool sequence_cancel_requested_=false, stopping_=false, fault_latched_=false;
  bool held_product_=false, current_product_attached_=false, ready=true;
  bool cancel_during_wait=false, cancel_after_dispatch=false, cancel_sent=false;
  bool downstream_live=false, cancel_while_live=false; int cancel_checks=0;
  Probe() {
    execute_client_->cancel_during_wait=&cancel_during_wait;
    execute_client_->active_cancel_requested=&active_cancel_requested_;
    execute_client_->ready=&ready;
    execute_client_->held_product=&held_product_;
    execute_client_->current_product_attached=&current_product_attached_;
    execute_client_->fault_latched=&fault_latched_;
    execute_client_->cancel_sent=&cancel_sent;
    execute_client_->downstream_live=&downstream_live;
  }
  bool manipulator_ready_locked() const {return ready;}
  bool is_cancel_requested() {
    ++cancel_checks; if(cancel_during_wait && cancel_checks >= 2) active_cancel_requested_=true;
    if(cancel_after_dispatch && execute_client_->send_calls > 0) active_cancel_requested_=true;
    return active_cancel_requested_ || stopping_;
  }
  void cancel_current_manipulation(std::shared_ptr<ExecuteGoalHandle>) {
    cancel_sent=true; cancel_while_live=downstream_live;
  }
''' + body + r'''
};
int main() {
  Probe before; before.active_cancel_requested_=true;
  auto early=before.call_execute_cycle("p","d","101");
  if(!early.canceled || early.accepted || early.result.outcome!=Execute::Result::CANCELED ||
     before.execute_client_->send_calls!=0) return 1;
  Probe timeout; timeout.execute_client_->server_available=false;
  auto unavailable=timeout.call_execute_cycle("p","d","101");
  if(unavailable.canceled || unavailable.accepted ||
     unavailable.result.outcome!=Execute::Result::DEPENDENCY_UNAVAILABLE ||
     timeout.fault_latched_ || timeout.execute_client_->send_calls!=0) return 2;

  Probe during; during.cancel_during_wait=true;
  during.execute_client_->server_available=false;
  auto late=during.call_execute_cycle("p","d","101");
  if(!late.canceled || late.accepted || during.cancel_sent || during.execute_client_->send_calls!=0 ||
     late.result.outcome!=Execute::Result::CANCELED || during.fault_latched_) return 3;

  const auto expect_timeout_interlock = [](Client::WaitMutation mutation, bool expected_ready,
      bool expected_held, bool expected_attached) {
    Probe probe;
    probe.execute_client_->server_available=false;
    probe.execute_client_->wait_mutation=mutation;
    const auto result=probe.call_execute_cycle("p","d","101");
    return !result.canceled && !result.accepted && !result.result.delivered &&
      result.result.outcome==Execute::Result::INTERLOCK_FAILED &&
      probe.execute_client_->send_calls==0 && probe.fault_latched_ &&
      probe.ready==expected_ready && probe.held_product_==expected_held &&
      probe.current_product_attached_==expected_attached;
  };
  if(!expect_timeout_interlock(Client::WaitMutation::STALE,false,false,false) ||
     !expect_timeout_interlock(Client::WaitMutation::ATTACHMENT,false,true,true) ||
     !expect_timeout_interlock(Client::WaitMutation::PRIOR_FAULT,true,false,false)) return 4;

  Probe downstream; downstream.cancel_after_dispatch=true;
  auto after=downstream.call_execute_cycle("p","d","101");
  if(!after.canceled || !after.accepted || !downstream.cancel_sent ||
     downstream.execute_client_->send_calls!=1 || after.result.outcome!=Execute::Result::INTERLOCK_FAILED) return 5;

  Probe late_acceptance;
  late_acceptance.execute_client_->accept_immediately=false;
  rclcpp::ok_value=false;
  auto unresolved=late_acceptance.call_execute_cycle("p","d","101");
  rclcpp::ok_value=true;
  if(unresolved.canceled || unresolved.accepted ||
     unresolved.result.outcome!=Execute::Result::INTERLOCK_FAILED ||
     !late_acceptance.downstream_live || late_acceptance.cancel_sent) return 6;
  late_acceptance.execute_client_->response_callback(std::make_shared<ExecuteGoalHandle>());
  if(!late_acceptance.cancel_sent || !late_acceptance.cancel_while_live) return 7;
  std::cout << "cancel-before-dispatch, cancellation-during-wait, and downstream handshake passed\n";
}
'''


def _cycle_final_interlock_probe():
    body = _method("  ExecuteCall call_execute_cycle(", "  void sequence_loop()")
    body = body.replace("std::mutex", "TrackingMutex")
    body = body.replace(
        "struct PendingGoal {\n      TrackingMutex mutex;",
        "struct PendingGoal {\n      std::mutex mutex;",
    )
    body = body.replace(
        "std::lock_guard<TrackingMutex> lock(pending->mutex)",
        "std::lock_guard<std::mutex> lock(pending->mutex)",
    )
    body = body.replace(
        "std::unique_lock<TrackingMutex> pending_lock(pending->mutex)",
        "std::unique_lock<std::mutex> pending_lock(pending->mutex)",
    )
    return r'''
#include <chrono>
#include <cstdlib>
#include <future>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
using namespace std::chrono_literals;
namespace rclcpp { inline bool ok_value=true; inline bool ok() {return ok_value;} }
namespace rclcpp_action { enum class ResultCode {SUCCEEDED, ABORTED, CANCELED}; }
struct TrackingMutex {
  bool held=false; std::thread::id owner;
  void lock() {
    if (held) std::abort();
    held=true; owner=std::this_thread::get_id();
  }
  void unlock() {
    if (!held || owner != std::this_thread::get_id()) std::abort();
    held=false; owner=std::thread::id{};
  }
  bool is_owned_by_current_thread() const {
    return held && owner == std::this_thread::get_id();
  }
};
struct Execute {
  struct Goal {std::string pickup_station_id, destination_station_id;};
  struct Result {
    enum {SUCCESS=0, INVALID_REQUEST=1, DEPENDENCY_UNAVAILABLE=2,
      PREPARATION_FAILED=3, EXECUTION_FAILED=4, CANCELED=5,
      INTERLOCK_FAILED=6, RETAINED_PRODUCT_FAULT=7};
    int outcome=SUCCESS; bool delivered=false; std::string product_id, message;
  };
};
struct ExecuteGoalHandle {};
namespace rclcpp_action {
template<class Action>
struct Client {
  struct SendGoalOptions {
    std::function<void(std::shared_ptr<ExecuteGoalHandle>)> goal_response_callback;
  };
};
}
struct WrappedResult {
  rclcpp_action::ResultCode code=rclcpp_action::ResultCode::SUCCEEDED;
  std::shared_ptr<Execute::Result> result;
};
struct SendFuture {
  std::shared_ptr<ExecuteGoalHandle> handle=std::make_shared<ExecuteGoalHandle>();
  template<class Rep,class Period>
  std::future_status wait_for(std::chrono::duration<Rep,Period>) {return std::future_status::ready;}
  std::shared_ptr<ExecuteGoalHandle> get() {return handle;}
};
struct ResultFuture {
  WrappedResult wrapped;
  template<class Rep,class Period>
  std::future_status wait_for(std::chrono::duration<Rep,Period>)
    {return std::future_status::ready;}
  WrappedResult get() {return wrapped;}
};
struct Client {
  using Options=rclcpp_action::Client<Execute>::SendGoalOptions;
  TrackingMutex *guard=nullptr;
  int send_calls=0;
  bool dispatch_inside_guard=false;
  std::shared_ptr<Execute::Result> result=std::make_shared<Execute::Result>();
  enum class WaitMutation {NONE, ACTIVE_CANCEL, STALE_ATTACHMENT};
  bool server_available=true;
  WaitMutation wait_mutation=WaitMutation::NONE;
  bool *active_cancel_requested=nullptr;
  bool *ready=nullptr;
  bool *held_product=nullptr;
  bool *current_product_attached=nullptr;
  Client() {result->delivered=true;}
  bool wait_for_action_server(std::chrono::seconds) {
    if (wait_mutation == WaitMutation::ACTIVE_CANCEL && active_cancel_requested)
      *active_cancel_requested=true;
    if (wait_mutation == WaitMutation::STALE_ATTACHMENT) {
      if (ready) *ready=false;
      if (held_product) *held_product=true;
      if (current_product_attached) *current_product_attached=true;
    }
    return server_available;
  }
  SendFuture async_send_goal(Execute::Goal, Options options) {
    dispatch_inside_guard = guard && guard->is_owned_by_current_thread();
    ++send_calls;
    if (options.goal_response_callback) options.goal_response_callback(std::make_shared<ExecuteGoalHandle>());
    return {};
  }
  ResultFuture async_get_result(std::shared_ptr<ExecuteGoalHandle>) {
    ResultFuture future;
    future.wrapped.result=result;
    return future;
  }
};
struct Probe {
  using ExecuteCall=struct {Execute::Result result; bool accepted=false; bool canceled=false;};
  TrackingMutex mutex_;
  std::shared_ptr<Client> execute_client_=std::make_shared<Client>();
  std::shared_ptr<ExecuteGoalHandle> current_execute_goal_;
  bool sequence_cancel_requested_=false, active_cancel_requested_=false, stopping_=false;
  bool fault_latched_=false, held_product_=false, current_product_attached_=false;
  bool ready=true;
  unsigned completed_jobs_=0, completed_cycles_=0, home_calls=0;
  int phase_=0;
  Probe() {
    execute_client_->guard=&mutex_;
    execute_client_->active_cancel_requested=&active_cancel_requested_;
    execute_client_->ready=&ready;
    execute_client_->held_product=&held_product_;
    execute_client_->current_product_attached=&current_product_attached_;
  }
  bool is_cancel_requested() {
    if (mutex_.is_owned_by_current_thread()) std::abort();
    return active_cancel_requested_ || stopping_;
  }
  bool manipulator_ready_locked() const {return ready;}
  void cancel_current_manipulation(std::shared_ptr<ExecuteGoalHandle>) {}
  ExecuteCall dispatch_sequence_job(bool inject_unsafe, bool final_job=false) {
    {
      std::lock_guard<TrackingMutex> lock(mutex_);
      if (fault_latched_ || held_product_ || current_product_attached_ ||
        !manipulator_ready_locked()) std::abort();
      phase_=1;  // PREPARING: the production loop has released its precheck lock.
    }
    if (inject_unsafe) {
      std::lock_guard<TrackingMutex> lock(mutex_);
      ready=false; held_product_=true; current_product_attached_=true;
    }
    auto call=call_execute_cycle("pickup_a","dispatch","101");
    std::lock_guard<TrackingMutex> lock(mutex_);
    if (call.result.outcome==Execute::Result::INTERLOCK_FAILED) {
      fault_latched_=true;
    } else if (call.result.outcome==Execute::Result::SUCCESS && call.result.delivered) {
      ++completed_jobs_;
      if (final_job) ++completed_cycles_;
    }
    return call;
  }
''' + body + r'''
};
int main() {
  Probe before_first;
  auto first=before_first.dispatch_sequence_job(true);
  if(before_first.execute_client_->send_calls!=0 ||
     first.result.outcome!=Execute::Result::INTERLOCK_FAILED || first.result.delivered ||
     first.result.product_id!="101" || first.result.message.empty() ||
     !before_first.fault_latched_ || !before_first.held_product_ ||
     !before_first.current_product_attached_ || before_first.completed_jobs_!=0 ||
     before_first.completed_cycles_!=0 || before_first.home_calls!=0) return 1;

  Probe before_later;
  auto completed=before_later.dispatch_sequence_job(false);
  auto later=before_later.dispatch_sequence_job(true);
  if(completed.result.outcome!=Execute::Result::SUCCESS ||
     before_later.execute_client_->send_calls!=1 || later.result.outcome!=Execute::Result::INTERLOCK_FAILED ||
     before_later.completed_jobs_!=1 || before_later.completed_cycles_!=0 ||
     before_later.home_calls!=0 || !before_later.fault_latched_ ||
     !before_later.held_product_ || !before_later.current_product_attached_) return 2;

  Probe stale; stale.ready=false;
  auto stale_result=stale.call_execute_cycle("pickup_a","dispatch","101");
  if(stale.execute_client_->send_calls!=0 ||
     stale_result.result.outcome!=Execute::Result::INTERLOCK_FAILED ||
     !stale.fault_latched_ || stale_result.result.product_id!="101") return 3;

  Probe retained; retained.held_product_=true;
  auto retained_result=retained.call_execute_cycle("pickup_a","dispatch","101");
  if(retained.execute_client_->send_calls!=0 ||
     retained_result.result.outcome!=Execute::Result::INTERLOCK_FAILED ||
     !retained.fault_latched_ || !retained.held_product_ || retained.current_product_attached_) return 4;

  Probe attached; attached.current_product_attached_=true;
  auto attached_result=attached.call_execute_cycle("pickup_a","dispatch","101");
  if(attached.execute_client_->send_calls!=0 ||
     attached_result.result.outcome!=Execute::Result::INTERLOCK_FAILED ||
     !attached.fault_latched_ || !attached.current_product_attached_ || attached.held_product_) return 5;

  Probe prior_fault; prior_fault.fault_latched_=true;
  auto prior_result=prior_fault.call_execute_cycle("pickup_a","dispatch","101");
  if(prior_fault.execute_client_->send_calls!=0 ||
     prior_result.result.outcome!=Execute::Result::INTERLOCK_FAILED || !prior_fault.fault_latched_) return 6;

  Probe timeout_safe;
  timeout_safe.execute_client_->server_available=false;
  auto timeout_result=timeout_safe.call_execute_cycle("pickup_a","dispatch","101");
  if(timeout_safe.execute_client_->send_calls!=0 || timeout_result.canceled ||
     timeout_result.accepted || timeout_result.result.outcome!=Execute::Result::DEPENDENCY_UNAVAILABLE ||
     timeout_result.result.delivered || timeout_safe.fault_latched_ ||
     !timeout_safe.ready || timeout_safe.held_product_ ||
     timeout_safe.current_product_attached_) return 7;

  Probe timeout_cancel;
  timeout_cancel.execute_client_->server_available=false;
  timeout_cancel.execute_client_->wait_mutation=Client::WaitMutation::ACTIVE_CANCEL;
  auto timeout_canceled=timeout_cancel.call_execute_cycle("pickup_a","dispatch","101");
  if(timeout_cancel.execute_client_->send_calls!=0 || !timeout_canceled.canceled ||
     timeout_canceled.accepted || timeout_canceled.result.outcome!=Execute::Result::CANCELED ||
     timeout_canceled.result.delivered || timeout_cancel.fault_latched_ ||
     !timeout_cancel.active_cancel_requested_ || !timeout_cancel.ready ||
     timeout_cancel.held_product_ || timeout_cancel.current_product_attached_) return 8;

  Probe timeout_unsafe;
  timeout_unsafe.execute_client_->server_available=false;
  timeout_unsafe.execute_client_->wait_mutation=Client::WaitMutation::STALE_ATTACHMENT;
  auto timeout_interlock=timeout_unsafe.call_execute_cycle("pickup_a","dispatch","101");
  if(timeout_unsafe.execute_client_->send_calls!=0 || timeout_interlock.canceled ||
     timeout_interlock.accepted || timeout_interlock.result.outcome!=Execute::Result::INTERLOCK_FAILED ||
     timeout_interlock.result.delivered || !timeout_unsafe.fault_latched_ ||
     timeout_unsafe.ready || !timeout_unsafe.held_product_ ||
     !timeout_unsafe.current_product_attached_ ||
     timeout_interlock.result.product_id!="101" || timeout_interlock.result.message.empty()) return 9;

  const auto expect_unsafe_cancel = [](bool stopping, bool stale, bool held,
      bool attached, bool prior_fault) {
    Probe probe;
    probe.active_cancel_requested_ = !stopping;
    probe.stopping_ = stopping;
    probe.ready = !stale;
    probe.held_product_ = held;
    probe.current_product_attached_ = attached;
    probe.fault_latched_ = prior_fault;
    const auto result=probe.call_execute_cycle("pickup_a","dispatch","101");
    return probe.execute_client_->send_calls==0 &&
      result.result.outcome==Execute::Result::INTERLOCK_FAILED &&
      !result.canceled && !result.result.delivered &&
      result.result.product_id=="101" && !result.result.message.empty() &&
      probe.fault_latched_ && probe.held_product_==held &&
      probe.current_product_attached_==attached;
  };
  const bool active_unsafe =
    expect_unsafe_cancel(false,true,false,false,false) &&
    expect_unsafe_cancel(false,false,true,false,false) &&
    expect_unsafe_cancel(false,false,false,true,false) &&
    expect_unsafe_cancel(false,false,false,false,true);
  const bool stopping_unsafe =
    expect_unsafe_cancel(true,true,false,false,false) &&
    expect_unsafe_cancel(true,false,true,false,false) &&
    expect_unsafe_cancel(true,false,false,true,false) &&
    expect_unsafe_cancel(true,false,false,false,true);
  if(!active_unsafe || !stopping_unsafe) return 10;

  Probe simultaneous; simultaneous.ready=false; simultaneous.held_product_=true;
  simultaneous.current_product_attached_=true; simultaneous.sequence_cancel_requested_=true;
  auto simultaneous_result=simultaneous.call_execute_cycle("pickup_a","dispatch","101");
  if(simultaneous.execute_client_->send_calls!=0 ||
     simultaneous_result.result.outcome!=Execute::Result::INTERLOCK_FAILED ||
     !simultaneous.fault_latched_ || !simultaneous.held_product_ ||
     !simultaneous.current_product_attached_) return 11;

  Probe sequence_cancel; sequence_cancel.sequence_cancel_requested_=true;
  auto sequence_cancel_result=sequence_cancel.call_execute_cycle("pickup_a","dispatch","101");
  if(sequence_cancel.execute_client_->send_calls!=0 ||
     sequence_cancel_result.result.outcome!=Execute::Result::CANCELED ||
     !sequence_cancel_result.canceled || sequence_cancel.fault_latched_) return 12;

  Probe active_cancel; active_cancel.active_cancel_requested_=true;
  auto active_cancel_result=active_cancel.call_execute_cycle("pickup_a","dispatch","101");
  if(active_cancel.execute_client_->send_calls!=0 ||
     active_cancel_result.result.outcome!=Execute::Result::CANCELED ||
     !active_cancel_result.canceled || active_cancel.fault_latched_) return 13;

  Probe stopping; stopping.stopping_=true;
  auto stopping_result=stopping.call_execute_cycle("pickup_a","dispatch","101");
  if(stopping.execute_client_->send_calls!=0 ||
     stopping_result.result.outcome!=Execute::Result::CANCELED ||
     !stopping_result.canceled || stopping.fault_latched_) return 14;

  Probe positive;
  auto positive_result=positive.dispatch_sequence_job(false,true);
  if(positive_result.result.outcome!=Execute::Result::SUCCESS ||
     positive.execute_client_->send_calls!=1 || positive.completed_jobs_!=1 ||
     positive.completed_cycles_!=1 || positive.home_calls!=0 ||
     !positive.execute_client_->dispatch_inside_guard) return 15;

  std::cout << "final interlock authorization, retained evidence, cancellation priority, and atomic send passed\n";
}
'''


def _home_dispatch_probe():
    body = _method("  bool navigate_home()", "  void execute_home(")
    send_and_unlock = "    navigation_client_->async_send_goal(goal, options);\n    lock.unlock();"
    assert send_and_unlock in body
    pre_correction_body = body.replace(
        send_and_unlock,
        "    lock.unlock();\n    navigation_client_->async_send_goal(goal, options);",
        1,
    )
    body = body.replace("std::mutex", "TrackingMutex")
    pre_correction_body = pre_correction_body.replace("std::mutex", "TrackingMutex")
    for source_name in ("body", "pre_correction_body"):
        source = locals()[source_name]
        source = source.replace(
            "struct PendingHomeGoal {\n      TrackingMutex mutex;",
            "struct PendingHomeGoal {\n      std::mutex mutex;",
        )
        source = source.replace(
            "std::lock_guard<TrackingMutex> pending_lock(pending->mutex)",
            "std::lock_guard<std::mutex> pending_lock(pending->mutex)",
        )
        source = source.replace(
            "std::lock_guard<TrackingMutex> abandoned_lock(pending->mutex)",
            "std::lock_guard<std::mutex> abandoned_lock(pending->mutex)",
        )
        source = source.replace(
            "std::unique_lock<TrackingMutex> pending_lock(pending->mutex)",
            "std::unique_lock<std::mutex> pending_lock(pending->mutex)",
        )
        if source_name == "body":
            body = source
        else:
            pre_correction_body = source
    return r'''
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
using namespace std::chrono_literals;
namespace rclcpp { inline bool ok_value=true; inline bool ok() {return ok_value;} }
namespace rclcpp_action {
enum class ResultCode {SUCCEEDED, ABORTED, CANCELED};
namespace exceptions { struct UnknownGoalHandleError : std::exception {}; }
}
#define RCLCPP_WARN(...) do {} while(false)
struct TrackingMutex {
  bool held=false;
  std::thread::id owner;
  void lock() {
    if (held) std::abort();
    held=true;
    owner=std::this_thread::get_id();
  }
  void unlock() {
    if (!held || owner != std::this_thread::get_id()) std::abort();
    held=false;
    owner=std::thread::id{};
  }
  bool is_owned_by_current_thread() const {
    return held && owner == std::this_thread::get_id();
  }
};
struct NavigateToPose {
  struct Goal {
    struct Header {std::string frame_id; int stamp=0;};
    struct Pose {struct Position {double x=0.0,y=0.0;};
      struct Orientation {double z=0.0,w=1.0;}; Position position; Orientation orientation;};
    struct PoseStamped {Header header; Pose pose;}; PoseStamped pose;
  };
};
struct GoalHandle {};
namespace rclcpp_action {
template<class Action> using ClientGoalHandle=::GoalHandle;
template<class Action>
struct Client {
  struct SendGoalOptions {
    std::function<void(std::shared_ptr<GoalHandle>)> goal_response_callback;
  };
};
}
struct AcceptedFuture {
  std::shared_ptr<GoalHandle> handle=std::make_shared<GoalHandle>();
  template<class Rep,class Period> std::future_status wait_for(std::chrono::duration<Rep,Period>)
    {return std::future_status::ready;}
  std::shared_ptr<GoalHandle> get() {return handle;}
};
struct WrappedResult {rclcpp_action::ResultCode code=rclcpp_action::ResultCode::SUCCEEDED;};
struct ResultFuture {
  template<class Rep,class Period> std::future_status wait_for(std::chrono::duration<Rep,Period>)
    {return std::future_status::ready;}
  WrappedResult get() {return {};}
};
struct Client {
  using Options=rclcpp_action::Client<NavigateToPose>::SendGoalOptions;
  TrackingMutex *guard=nullptr; bool dispatch_inside_guard=false;
  int dispatch_calls=0;
  bool accept_immediately=true, cancel_sent=false;
  std::function<void(std::shared_ptr<GoalHandle>)> response_callback;
  bool wait_for_action_server(std::chrono::seconds) {return true;}
  AcceptedFuture async_send_goal(NavigateToPose::Goal, Options options) {
    dispatch_inside_guard = guard && guard->is_owned_by_current_thread();
    ++dispatch_calls;
    response_callback=std::move(options.goal_response_callback);
    if (accept_immediately && response_callback) response_callback(std::make_shared<GoalHandle>());
    return {};
  }
  void async_cancel_goal(std::shared_ptr<GoalHandle>) {cancel_sent=true;}
  ResultFuture async_get_result(std::shared_ptr<GoalHandle>) {return {};}
};
struct StationRecord {std::array<double,3> approach{1.0,2.0,0.4};};
struct CurrentProbe {
  TrackingMutex mutex_; std::shared_ptr<Client> navigation_client_=std::make_shared<Client>();
  std::unordered_map<std::string,StationRecord> stations_{{"home",{}}};
  bool home_cancel_confirmation_failed_=false, home_interlock_failed_=false;
  bool home_cancel_requested_=false, sequence_cancel_requested_=false, stopping_=false;
  bool fault_latched_=false, ready=true;
  bool manipulator_ready_locked() const {return ready;}
  bool held_product_=false, current_product_attached_=false;
  bool safe_empty_stow_locked() const {
    return !fault_latched_ && !held_product_ && !current_product_attached_ &&
      manipulator_ready_locked();
  }
  int now() const {return 0;}
  CurrentProbe() {navigation_client_->guard=&mutex_;}
  template<class Goal, class Result>
  bool cancel_home_navigation_goal(const std::shared_ptr<Goal>&, Result&) {return false;}
''' + body + r'''
};
struct PreCorrectionProbe {
  TrackingMutex mutex_; std::shared_ptr<Client> navigation_client_=std::make_shared<Client>();
  std::unordered_map<std::string,StationRecord> stations_{{"home",{}}};
  bool home_cancel_confirmation_failed_=false, home_interlock_failed_=false;
  bool home_cancel_requested_=false, sequence_cancel_requested_=false, stopping_=false;
  bool fault_latched_=false, ready=true;
  bool manipulator_ready_locked() const {return ready;}
  bool held_product_=false, current_product_attached_=false;
  bool safe_empty_stow_locked() const {
    return !fault_latched_ && !held_product_ && !current_product_attached_ &&
      manipulator_ready_locked();
  }
  int now() const {return 0;}
  PreCorrectionProbe() {navigation_client_->guard=&mutex_;}
  template<class Goal, class Result>
  bool cancel_home_navigation_goal(const std::shared_ptr<Goal>&, Result&) {return false;}
''' + pre_correction_body + r'''
};
int main() {
  CurrentProbe expired; expired.ready=false;
  if(expired.navigate_home() || expired.navigation_client_->dispatch_calls!=0 ||
     !expired.home_interlock_failed_) return 1;
  CurrentProbe fresh;
  if(!fresh.navigate_home() || fresh.navigation_client_->dispatch_calls!=1 ||
     fresh.home_interlock_failed_ || !fresh.navigation_client_->dispatch_inside_guard) return 2;
  PreCorrectionProbe pre_correction;
  if(!pre_correction.navigate_home() || pre_correction.navigation_client_->dispatch_calls!=1 ||
     pre_correction.navigation_client_->dispatch_inside_guard) return 3;
  CurrentProbe late;
  late.navigation_client_->accept_immediately=false;
  rclcpp::ok_value=false;
  if(late.navigate_home() || !late.home_interlock_failed_ ||
     late.navigation_client_->dispatch_calls!=1) return 4;
  rclcpp::ok_value=true;
  late.navigation_client_->response_callback(std::make_shared<GoalHandle>());
  if(!late.navigation_client_->cancel_sent) return 5;
  std::cout << "fresh proof dispatch, lock-owner instrumentation, and late acceptance cancellation passed\n";
}
'''


def _transport_execution_probe():
    body = _method("  void execute_job(", "  ExecuteCall call_execute_cycle(")
    return r'''
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
using namespace std::chrono_literals;
struct Execute { struct Result {
  enum {SUCCESS=0, DEPENDENCY_UNAVAILABLE=1, PREPARATION_FAILED=2, CANCELED=3,
    INTERLOCK_FAILED=4, RETAINED_PRODUCT_FAULT=5};
  int outcome=SUCCESS; bool delivered=true; std::string message;
}; };
struct Transport { struct Result {
  enum {SUCCESS=0, PLACE_FAILED=1, DEPENDENCY_UNAVAILABLE=2, PICK_FAILED=3,
    INTERLOCK_FAILED=4, CANCELED=5};
  int outcome=SUCCESS; bool delivered=true; std::string message;
}; struct Feedback {int phase=0;}; };
struct TransportGoalHandle {
  bool active=true, canceling=false; int terminal_kind=0; Transport::Result terminal;
  bool is_active() const {return active;}
  bool is_canceling() const {return canceling;}
  void succeed(std::shared_ptr<Transport::Result> r) {terminal=*r; terminal_kind=1;}
  void canceled(std::shared_ptr<Transport::Result> r) {terminal=*r; terminal_kind=2;}
  void abort(std::shared_ptr<Transport::Result> r) {terminal=*r; terminal_kind=3;}
};
using TransportGoal = TransportGoalHandle;
struct ExecuteCall {Execute::Result result; bool accepted=false, canceled=false;};
struct QueuedJob {std::shared_ptr<TransportGoalHandle> goal; std::string pickup_station,destination_station,product_id;};
struct Probe {
  std::mutex mutex_;
  bool held_product_=false, current_product_attached_=false, fault_latched_=false, ready=true;
  bool active_cancel_requested_=false, stopping_=false, active_transport_terminal_decided_=false;
  bool inject_cancel_after_proof=false, inject_unsafe_after_proof=false;
  int finish_kind=0, finish_outcome=0, wait_calls=0, last_outcome_=0, phase_=0, completed_jobs_=0;
  std::string detail_;
  ExecuteCall scripted;
  void publish_transport_feedback(const std::shared_ptr<TransportGoalHandle>&,uint8_t,const std::string&,bool) {}
  ExecuteCall call_execute_cycle(const std::string&,const std::string&,const std::string&) {return scripted;}
  bool is_cancel_requested() {return active_cancel_requested_ || stopping_;}
  bool wait_for_manipulator_ready(std::chrono::seconds) {
    ++wait_calls;
    const bool proof=ready;
    if (inject_cancel_after_proof) active_cancel_requested_=true;
    if (inject_unsafe_after_proof) {
      ready=false;
      held_product_=true;
      current_product_attached_=true;
    }
    return proof;
  }
  bool manipulator_ready_locked() const {return ready;}
  bool safe_empty_stow_locked() const {
    return !fault_latched_ && !held_product_ && !current_product_attached_ &&
      manipulator_ready_locked();
  }
  void finish_transport_canceled(const std::shared_ptr<TransportGoalHandle>& goal,const std::string&) {
    finish_kind=2; finish_outcome=Transport::Result::CANCELED;
    auto result=std::make_shared<Transport::Result>(); result->outcome=finish_outcome;
    if(goal && goal->is_active()) goal->canceled(result);
  }
  void finish_transport_failed(const std::shared_ptr<TransportGoalHandle>& goal,uint8_t outcome,const std::string&) {
    finish_kind=3; finish_outcome=outcome; if(outcome==Transport::Result::INTERLOCK_FAILED) fault_latched_=true;
    auto result=std::make_shared<Transport::Result>(); result->outcome=outcome;
    if(goal && goal->is_active()) goal->abort(result);
  }
''' + body + r'''
};
int main() {
  QueuedJob job; job.goal=std::make_shared<TransportGoalHandle>();
  Probe retained; retained.scripted.canceled=true;
  retained.scripted.result.outcome=Execute::Result::INTERLOCK_FAILED;
  retained.execute_job(job);
  if(job.goal->terminal_kind!=3 || job.goal->terminal.outcome!=Transport::Result::INTERLOCK_FAILED ||
     !retained.fault_latched_ || retained.wait_calls!=0) return 1;
  Probe unsafe_cancel; unsafe_cancel.ready=false; unsafe_cancel.scripted.canceled=true;
  unsafe_cancel.scripted.result.outcome=Execute::Result::CANCELED; unsafe_cancel.execute_job(job);
  if(job.goal->terminal_kind!=3 || job.goal->terminal.outcome!=Transport::Result::INTERLOCK_FAILED ||
     !unsafe_cancel.fault_latched_ || unsafe_cancel.wait_calls!=1) return 2;
  Probe safe_cancel; safe_cancel.scripted.canceled=true;
  job.goal->canceling=true;
  safe_cancel.scripted.result.outcome=Execute::Result::CANCELED; safe_cancel.execute_job(job);
  if(job.goal->terminal_kind!=2 || job.goal->terminal.outcome!=Transport::Result::CANCELED ||
     safe_cancel.wait_calls!=1) return 3;

  QueuedJob late_cancel_job; late_cancel_job.goal=std::make_shared<TransportGoalHandle>();
  Probe late_cancel; late_cancel.inject_cancel_after_proof=true;
  late_cancel.scripted.result.outcome=Execute::Result::SUCCESS;
  late_cancel.scripted.result.delivered=true;
  late_cancel.execute_job(late_cancel_job);
  if(late_cancel_job.goal->terminal_kind!=3 ||
     late_cancel_job.goal->terminal.outcome!=Transport::Result::CANCELED ||
     late_cancel.completed_jobs_!=0) return 4;

  QueuedJob stale_success_job; stale_success_job.goal=std::make_shared<TransportGoalHandle>();
  Probe stale_success; stale_success.inject_unsafe_after_proof=true;
  stale_success.scripted.result.outcome=Execute::Result::SUCCESS;
  stale_success.scripted.result.delivered=true;
  stale_success.execute_job(stale_success_job);
  if(stale_success_job.goal->terminal.outcome!=Transport::Result::INTERLOCK_FAILED ||
     !stale_success.fault_latched_ || stale_success.completed_jobs_!=0 ||
     !stale_success.held_product_ || !stale_success.current_product_attached_) return 5;

  QueuedJob stale_cancel_job; stale_cancel_job.goal=std::make_shared<TransportGoalHandle>();
  Probe stale_cancel; stale_cancel.inject_unsafe_after_proof=true;
  stale_cancel.scripted.canceled=true;
  stale_cancel.scripted.result.outcome=Execute::Result::CANCELED;
  stale_cancel.execute_job(stale_cancel_job);
  if(stale_cancel_job.goal->terminal.outcome!=Transport::Result::INTERLOCK_FAILED ||
     !stale_cancel.fault_latched_ || stale_cancel_job.goal->terminal_kind!=3) return 6;

  QueuedJob generic_failure_job; generic_failure_job.goal=std::make_shared<TransportGoalHandle>();
  Probe generic_failure; generic_failure.scripted.result.outcome=Execute::Result::DEPENDENCY_UNAVAILABLE;
  generic_failure.scripted.result.delivered=false;
  generic_failure.execute_job(generic_failure_job);
  if(generic_failure_job.goal->terminal.outcome!=Transport::Result::DEPENDENCY_UNAVAILABLE ||
     generic_failure.wait_calls!=1 || generic_failure.fault_latched_) return 7;

  QueuedJob generic_missing_job; generic_missing_job.goal=std::make_shared<TransportGoalHandle>();
  Probe generic_missing; generic_missing.ready=false;
  generic_missing.scripted.result.outcome=Execute::Result::DEPENDENCY_UNAVAILABLE;
  generic_missing.scripted.result.delivered=false;
  generic_missing.execute_job(generic_missing_job);
  if(generic_missing_job.goal->terminal.outcome!=Transport::Result::INTERLOCK_FAILED ||
     generic_missing.wait_calls!=1 || !generic_missing.fault_latched_) return 8;

  QueuedJob delivered_safe_job; delivered_safe_job.goal=std::make_shared<TransportGoalHandle>();
  Probe delivered_safe; delivered_safe.scripted.result.outcome=Execute::Result::SUCCESS;
  delivered_safe.scripted.result.delivered=false;
  delivered_safe.execute_job(delivered_safe_job);
  if(delivered_safe_job.goal->terminal.outcome!=Transport::Result::PLACE_FAILED ||
     delivered_safe.wait_calls!=1 || delivered_safe.fault_latched_) return 9;

  QueuedJob delivered_missing_job; delivered_missing_job.goal=std::make_shared<TransportGoalHandle>();
  Probe delivered_missing; delivered_missing.ready=false;
  delivered_missing.scripted.result.outcome=Execute::Result::SUCCESS;
  delivered_missing.scripted.result.delivered=false;
  delivered_missing.execute_job(delivered_missing_job);
  if(delivered_missing_job.goal->terminal.outcome!=Transport::Result::INTERLOCK_FAILED ||
     delivered_missing.wait_calls!=1 || !delivered_missing.fault_latched_) return 10;

  QueuedJob delivered_cancel_job; delivered_cancel_job.goal=std::make_shared<TransportGoalHandle>();
  Probe delivered_cancel; delivered_cancel.scripted.canceled=true;
  delivered_cancel.scripted.result.outcome=Execute::Result::SUCCESS;
  delivered_cancel.scripted.result.delivered=false;
  delivered_cancel.execute_job(delivered_cancel_job);
  if(delivered_cancel_job.goal->terminal.outcome!=Transport::Result::PLACE_FAILED ||
     delivered_cancel.wait_calls!=1 || delivered_cancel.fault_latched_) return 11;
  std::cout << "transport typed failure priority and safe cancel proof passed\n";
}
'''


def _home_terminal_probe():
    body = _method("  void execute_home(", "  void cancel_current_manipulation(")
    return r'''
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
using namespace std::chrono_literals;
struct NavigateStation { struct Result {
  enum {SUCCESS=0, CANCELED=1, NAVIGATION_FAILED=2, INTERLOCK_FAILED=3};
  int outcome=SUCCESS; std::string message;
}; struct Feedback {int phase=0; std::string current_station_id;}; };
struct HomeGoalHandle {
  bool active=true, canceling=false; int terminal_kind=0; NavigateStation::Result terminal;
  bool is_active() const {return active;}
  bool is_canceling() const {return canceling;}
  void publish_feedback(std::shared_ptr<NavigateStation::Feedback>) {}
  void succeed(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=1;}
  void canceled(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=2;}
  void abort(std::shared_ptr<NavigateStation::Result> r) {terminal=*r; terminal_kind=3;}
};
struct Probe {
  std::mutex mutex_;
  bool home_cancel_confirmation_failed_=false, home_interlock_failed_=false;
  bool home_cancel_requested_=false, sequence_cancel_requested_=false, stopping_=false;
  bool fault_latched_=false, held_product_=false, current_product_attached_=false, ready=true;
  bool home_active_=true, navigate_success=true, inject_cancel_on_navigate=false;
  bool inject_unsafe_after_proof=false;
  unsigned wait_calls=0; int phase_=0, last_outcome_=0;
  std::shared_ptr<HomeGoalHandle> home_goal_;
  bool navigate_home() {
    if (inject_cancel_on_navigate) home_cancel_requested_=true;
    return navigate_success;
  }
  bool wait_for_manipulator_ready(std::chrono::seconds) {
    ++wait_calls;
    const bool proof=ready;
    if (inject_unsafe_after_proof) {
      ready=false;
      held_product_=true;
      current_product_attached_=true;
    }
    return proof;
  }
  bool manipulator_ready_locked() const {return ready;}
  bool safe_empty_stow_locked() const {
    return !fault_latched_ && !held_product_ && !current_product_attached_ &&
      manipulator_ready_locked();
  }
''' + body + r'''
};
int main() {
  Probe success; auto success_goal=std::make_shared<HomeGoalHandle>();
  success.home_goal_=success_goal; success.execute_home(success_goal);
  if(success_goal->terminal_kind!=1 || success_goal->terminal.outcome!=NavigateStation::Result::SUCCESS ||
     success.wait_calls!=1 || success.fault_latched_) return 1;

  Probe unsafe; auto unsafe_goal=std::make_shared<HomeGoalHandle>();
  unsafe.home_goal_=unsafe_goal; unsafe.inject_unsafe_after_proof=true;
  unsafe.execute_home(unsafe_goal);
  if(unsafe_goal->terminal_kind!=3 || unsafe_goal->terminal.outcome!=NavigateStation::Result::INTERLOCK_FAILED ||
     unsafe.wait_calls!=1 || !unsafe.fault_latched_ || !unsafe.held_product_) return 2;

  Probe failed_safe; auto failed_safe_goal=std::make_shared<HomeGoalHandle>();
  failed_safe.home_goal_=failed_safe_goal; failed_safe.navigate_success=false;
  failed_safe.execute_home(failed_safe_goal);
  if(failed_safe_goal->terminal_kind!=3 ||
     failed_safe_goal->terminal.outcome!=NavigateStation::Result::NAVIGATION_FAILED ||
     failed_safe.wait_calls!=1 || failed_safe.fault_latched_) return 5;

  Probe failed_missing; auto failed_missing_goal=std::make_shared<HomeGoalHandle>();
  failed_missing.home_goal_=failed_missing_goal; failed_missing.navigate_success=false;
  failed_missing.ready=false; failed_missing.execute_home(failed_missing_goal);
  if(failed_missing_goal->terminal_kind!=3 ||
     failed_missing_goal->terminal.outcome!=NavigateStation::Result::INTERLOCK_FAILED ||
     failed_missing.wait_calls!=1 || !failed_missing.fault_latched_) return 6;

  Probe failed_retained; auto failed_retained_goal=std::make_shared<HomeGoalHandle>();
  failed_retained.home_goal_=failed_retained_goal; failed_retained.navigate_success=false;
  failed_retained.held_product_=true; failed_retained.current_product_attached_=true;
  failed_retained.execute_home(failed_retained_goal);
  if(failed_retained_goal->terminal_kind!=3 ||
     failed_retained_goal->terminal.outcome!=NavigateStation::Result::INTERLOCK_FAILED ||
     failed_retained.wait_calls!=1 || !failed_retained.fault_latched_ ||
     !failed_retained.held_product_ || !failed_retained.current_product_attached_) return 7;

  Probe failed_prior_fault; auto failed_prior_fault_goal=std::make_shared<HomeGoalHandle>();
  failed_prior_fault.home_goal_=failed_prior_fault_goal; failed_prior_fault.navigate_success=false;
  failed_prior_fault.fault_latched_=true; failed_prior_fault.execute_home(failed_prior_fault_goal);
  if(failed_prior_fault_goal->terminal_kind!=3 ||
     failed_prior_fault_goal->terminal.outcome!=NavigateStation::Result::INTERLOCK_FAILED ||
     failed_prior_fault.wait_calls!=1 || !failed_prior_fault.fault_latched_) return 8;

  Probe late_executing; auto late_executing_goal=std::make_shared<HomeGoalHandle>();
  late_executing.home_goal_=late_executing_goal; late_executing.inject_cancel_on_navigate=true;
  late_executing.execute_home(late_executing_goal);
  if(late_executing_goal->terminal_kind!=3 ||
     late_executing_goal->terminal.outcome!=NavigateStation::Result::CANCELED ||
     late_executing.wait_calls!=1 || late_executing.fault_latched_) return 3;

  Probe late_canceling; auto late_canceling_goal=std::make_shared<HomeGoalHandle>();
  late_canceling.home_goal_=late_canceling_goal; late_canceling.inject_cancel_on_navigate=true;
  late_canceling_goal->canceling=true; late_canceling.execute_home(late_canceling_goal);
  if(late_canceling_goal->terminal_kind!=2 ||
     late_canceling_goal->terminal.outcome!=NavigateStation::Result::CANCELED ||
     late_canceling.wait_calls!=1 || late_canceling.fault_latched_) return 4;

  std::cout << "home commit reproof and late cancellation passed\n";
}
'''


def _retained_status_probe():
    body = _method(
        "  void update_manipulator_status_locked(",
        "  void load_registry()",
    )
    return r'''
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
struct ManipulatorStatus {
  enum {STOWED_EMPTY=1, OTHER=2};
  std::uint64_t source_boot_id=0, sequence=0;
  bool valid=false, product_attached=false, base_motion_allowed=false;
  int state=OTHER; std::string product_id;
};
struct Probe {
  ManipulatorStatus manipulator_status_;
  bool have_manipulator_status_=false, held_product_=false;
  bool current_product_attached_=false, fault_latched_=false;
  std::chrono::steady_clock::time_point manipulator_received_{};
''' + body + r'''
};
int main() {
  Probe attached;
  ManipulatorStatus product;
  product.source_boot_id=7; product.sequence=1; product.valid=true;
  product.product_attached=true; product.base_motion_allowed=false; product.state=ManipulatorStatus::OTHER;
  attached.update_manipulator_status_locked(product);
  if(!attached.held_product_ || !attached.current_product_attached_) return 1;

  ManipulatorStatus invalid_empty=product;
  invalid_empty.sequence=2; invalid_empty.product_attached=false;
  invalid_empty.valid=false; invalid_empty.product_id.clear();
  attached.update_manipulator_status_locked(invalid_empty);
  if(!attached.held_product_ || attached.current_product_attached_) return 2;

  attached.fault_latched_=true;
  ManipulatorStatus fault_empty=invalid_empty;
  fault_empty.sequence=3; fault_empty.valid=true; fault_empty.state=ManipulatorStatus::STOWED_EMPTY;
  fault_empty.base_motion_allowed=true;
  attached.update_manipulator_status_locked(fault_empty);
  if(!attached.held_product_ || attached.current_product_attached_) return 3;

  attached.fault_latched_=false;
  fault_empty.sequence=4;
  attached.update_manipulator_status_locked(fault_empty);
  if(attached.held_product_ || attached.current_product_attached_) return 4;

  ManipulatorStatus stale=product;
  stale.sequence=4; stale.product_attached=true;
  attached.update_manipulator_status_locked(stale);
  if(attached.held_product_ || attached.current_product_attached_) return 5;
  std::cout << "retained evidence is monotonic until valid empty stow\n";
}
'''


def _transport_cancel_admission_probe():
    body = _method(
        "  rclcpp_action::CancelResponse handle_transport_cancel(",
        "  void handle_transport_accepted(",
    )
    return r'''
#include <algorithm>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
namespace rclcpp_action { enum class CancelResponse {REJECT, ACCEPT}; }
struct Transport { struct Result { enum {CANCELED=1}; bool delivered=false; int outcome=CANCELED; std::string message; }; };
struct Goal {
  bool active=true, canceling=true; int terminal_kind=0; Transport::Result terminal;
  bool is_active() const {return active;}
  bool is_canceling() const {return canceling;}
  void canceled(std::shared_ptr<Transport::Result> r) {terminal=*r; terminal_kind=2;}
  void abort(std::shared_ptr<Transport::Result> r) {terminal=*r; terminal_kind=3;}
};
using TransportGoalHandle=Goal;
struct QueuedJob {std::shared_ptr<Goal> goal; std::string pickup_station,destination_station,product_id;};
struct Probe {
  std::mutex mutex_; std::shared_ptr<Goal> active_goal_; std::deque<QueuedJob> queue_;
  bool active_cancel_requested_=false, active_transport_terminal_decided_=false;
  int cancel_calls=0; int last_outcome_=0;
  void cancel_current_manipulation() {++cancel_calls;}
''' + body + r'''
};
int main() {
  Probe closed; auto closed_goal=std::make_shared<Goal>(); closed.active_goal_=closed_goal;
  closed.active_transport_terminal_decided_=true;
  if(closed.handle_transport_cancel(closed_goal)!=rclcpp_action::CancelResponse::REJECT ||
     closed.active_cancel_requested_ || closed.cancel_calls!=0) return 1;
  Probe open; auto open_goal=std::make_shared<Goal>(); open.active_goal_=open_goal;
  if(open.handle_transport_cancel(open_goal)!=rclcpp_action::CancelResponse::ACCEPT ||
     !open.active_cancel_requested_ || open.cancel_calls!=1) return 2;
  std::cout << "transport cancellation admission closes at terminal commit\n";
}
'''


def test_sequence_terminal_and_home_guards_use_extracted_production_method():
    output = _run_cpp("sequence_probe", _sequence_probe())
    assert "R5/R6/R7 sequence" in output


def test_sequence_cancel_admission_closes_at_terminal_commit():
    output = _run_cpp("sequence_cancel_admission_probe", _sequence_cancel_admission_probe())
    assert "admission closes atomically" in output


def test_transport_canceled_terminal_respects_public_action_state():
    output = _run_cpp("transport_cancel_probe", _transport_cancel_probe())
    assert "EXECUTING abort" in output


def test_home_canceled_terminal_respects_public_action_state():
    output = _run_cpp("home_cancel_probe", _home_cancel_probe())
    assert "proof controls" in output


def test_standalone_home_failures_latch_factory_fault():
    output = _run_cpp("home_fault_probe", _home_fault_probe())
    assert "latch fault" in output


def test_cancel_before_and_during_downstream_wait_are_typed():
    output = _run_cpp("cycle_wait_probe", _cycle_wait_probe())
    assert "cancel-before-dispatch" in output


def test_final_child_dispatch_rechecks_interlock_and_send_is_atomic():
    output = _run_cpp("cycle_final_interlock_probe", _cycle_final_interlock_probe())
    assert "atomic send" in output


def test_home_dispatch_rechecks_fresh_empty_stow_before_send():
    output = _run_cpp("home_dispatch_probe", _home_dispatch_probe())
    assert "lock-owner instrumentation" in output


def test_transport_typed_failure_precedes_cancellation_and_requires_proof():
    output = _run_cpp("transport_execution_probe", _transport_execution_probe())
    assert "typed failure priority" in output


def test_standalone_home_commit_reproofs_and_handles_late_cancel():
    output = _run_cpp("home_terminal_probe", _home_terminal_probe())
    assert "home commit reproof" in output


def test_retained_product_evidence_survives_invalid_or_faulted_empty_status():
    output = _run_cpp("retained_status_probe", _retained_status_probe())
    assert "retained evidence" in output


def test_transport_cancel_admission_closes_after_atomic_terminal_decision():
    output = _run_cpp("transport_cancel_admission_probe", _transport_cancel_admission_probe())
    assert "admission closes" in output
