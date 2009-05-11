
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * $Id$
 */

#ifndef MESSAGE_H
#define MESSAGE_H

#include "Global.hh"
#include "RefCnt.hh"
#include "RefCountable.hh"
#include "EventQueue.hh"

class Message;
typedef RefCnt<Message> MsgPtr;

class Message : public RefCountable {
public:
  // Constructors
  Message() : RefCountable() { m_time = g_eventQueue_ptr->getTime();  m_LastEnqueueTime = g_eventQueue_ptr->getTime(); m_DelayedCycles = 0;}

  // Destructor
  virtual ~Message() { }

  // Public Methods
  virtual Message* clone() const = 0;
  virtual void destroy() = 0;
  virtual void print(ostream& out) const = 0;

  void setDelayedCycles(const int& cycles) { m_DelayedCycles = cycles; }
  const int& getDelayedCycles() const {return m_DelayedCycles;}
  int& getDelayedCycles() {return m_DelayedCycles;}
  void setLastEnqueueTime(const Time& time) { m_LastEnqueueTime = time; }
  const Time& getLastEnqueueTime() const {return m_LastEnqueueTime;}
  Time& getLastEnqueueTime() {return m_LastEnqueueTime;}

  const Time& getTime() const { return m_time; }
  void setTime(const Time& new_time) { m_time = new_time; }
private:
  // Private Methods

  // Data Members (m_ prefix)
  Time m_time;
  Time m_LastEnqueueTime; // my last enqueue time
  int m_DelayedCycles; // my delayed cycles

};

// Output operator declaration
ostream& operator<<(ostream& out, const Message& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Message& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //MESSAGE_H
