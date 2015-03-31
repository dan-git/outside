#include <boost/bind.hpp>

#include "CountingFSM.h"

CountingFSM::CountingFSM(int limit) : m_limit(limit), m_counter(0)
{
  setupFSM();

  // Start in state 1.
  m_fsm.set_state(m_state1);
}

void CountingFSM::update()
{
  m_fsm.update();
}

void CountingFSM::setupFSM()
{
  m_state1 = m_fsm.add_state();  // 0
  m_state2 = m_fsm.add_state();  // 1

  m_fsm.set_entry_function(m_state1, boost::bind(&CountingFSM::on_enter_state1, this));
  m_fsm.set_update_function(m_state1, boost::bind(&CountingFSM::on_update_state1, this));
  m_fsm.set_exit_function(m_state1, boost::bind(&CountingFSM::on_exit_state1, this));

  m_fsm.set_entry_function(m_state2, boost::bind(&CountingFSM::on_enter_state2, this));
  m_fsm.set_update_function(m_state2, boost::bind(&CountingFSM::on_update_state2, this));
}

void CountingFSM::on_enter_state1()
{
  ROS_INFO("Entering state1.");
  m_counter = 0;
}

int CountingFSM::on_update_state1()
{
  m_counter++;
  ROS_INFO("Counter = %d.", m_counter);
  if (m_counter < m_limit)
  {
    // The return from this function is the next state.  If we have not reached the limit, the return should be
    // state1 again.
    return m_state1;
  }
  // We have reached the limit so we should change states.
  return m_state2;
}

void CountingFSM::on_exit_state1()
{
  ROS_INFO("Leaving state1");
}

void CountingFSM::on_enter_state2()
{
  ROS_INFO("Entering state2.");
  m_counter = 0;
}

int CountingFSM::on_update_state2()
{
  m_counter++;
  ROS_INFO("Counter = %d.", m_counter);
  if (m_counter < m_limit)
  {
    // The return from this function is the next state.  If we have not reached the limit, the return should be
    // state2 again.
    return m_state2;
  }
  // We have reached the limit so we should change states.
  return m_state1;
}

int main(int argc, char** argv)
{
  CountingFSM fsm(3);
  for (int i = 0; i < 20; i++)
  {
    ROS_INFO("i = %d", i);
    fsm.update();
  }
}
