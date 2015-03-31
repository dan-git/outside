#ifndef __COUNTING_FSM__H__
#define __COUNTING_FSM__H__

#include "FBFSM/FBFSM.h"

class CountingFSM
{
 public:
  /**
   * An FSM that has two states.  It switches between the states every limit counts.
   */
  CountingFSM(int limit);
  void update();

 private:
  void setupFSM();

  void on_enter_state1();
  int on_update_state1();
  void on_exit_state1();

  void on_enter_state2();
  int on_update_state2();

  FBFSM m_fsm;

  int m_state1;
  int m_state2;

  int m_limit;
  int m_counter;
};

#endif  // __COUNTING_FSM__H__
