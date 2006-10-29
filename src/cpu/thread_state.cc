/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 *
 * Authors: Kevin Lim
 */

#include "base/output.hh"
#include "cpu/profile.hh"
#include "cpu/thread_state.hh"
#include "sim/serialize.hh"

#if FULL_SYSTEM
#include "cpu/quiesce_event.hh"
#include "kern/kernel_stats.hh"
#endif

#if FULL_SYSTEM
ThreadState::ThreadState(int _cpuId, int _tid)
    : cpuId(_cpuId), tid(_tid), lastActivate(0), lastSuspend(0),
      profile(NULL), profileNode(NULL), profilePC(0), quiesceEvent(NULL),
      microPC(0), nextMicroPC(1), funcExeInst(0), storeCondFailures(0)
#else
ThreadState::ThreadState(int _cpuId, int _tid, Process *_process,
                         short _asid, MemObject *mem)
    : cpuId(_cpuId), tid(_tid), lastActivate(0), lastSuspend(0),
      process(_process), asid(_asid),
      microPC(0), nextMicroPC(1), funcExeInst(0), storeCondFailures(0)
#endif
{
    numInst = 0;
    numLoad = 0;
}

void
ThreadState::serialize(std::ostream &os)
{
    SERIALIZE_ENUM(_status);
    // thread_num and cpu_id are deterministic from the config
    SERIALIZE_SCALAR(funcExeInst);
    SERIALIZE_SCALAR(inst);
    SERIALIZE_SCALAR(microPC);
    SERIALIZE_SCALAR(nextMicroPC);

#if FULL_SYSTEM
    Tick quiesceEndTick = 0;
    if (quiesceEvent->scheduled())
        quiesceEndTick = quiesceEvent->when();
    SERIALIZE_SCALAR(quiesceEndTick);
    if (kernelStats)
        kernelStats->serialize(os);
#endif
}

void
ThreadState::unserialize(Checkpoint *cp, const std::string &section)
{

    UNSERIALIZE_ENUM(_status);
    // thread_num and cpu_id are deterministic from the config
    UNSERIALIZE_SCALAR(funcExeInst);
    UNSERIALIZE_SCALAR(inst);
    UNSERIALIZE_SCALAR(microPC);
    UNSERIALIZE_SCALAR(nextMicroPC);

#if FULL_SYSTEM
    Tick quiesceEndTick;
    UNSERIALIZE_SCALAR(quiesceEndTick);
    if (quiesceEndTick)
        quiesceEvent->schedule(quiesceEndTick);
    if (kernelStats)
        kernelStats->unserialize(cp, section);
#endif
}

#if FULL_SYSTEM

void
ThreadState::profileClear()
{
    if (profile)
        profile->clear();
}

void
ThreadState::profileSample()
{
    if (profile)
        profile->sample(profileNode, profilePC);
}

#endif
