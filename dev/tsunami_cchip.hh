/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

/* @file
 * Turbolaser system bus node (processor, memory, or IO)
 */

#ifndef __TSUNAMI_CCHIP_HH__
#define __TSUNAMI_CCHIP_HH__

#include "mem/functional_mem/mmap_device.hh"
#include "dev/tsunami.hh"

/*
 * Tsunami CChip
 */
class TsunamiCChip : public MmapDevice
{
  public:

  protected:
    Tsunami *tsunami;
    uint64_t dim[Tsunami::Max_CPUs];
    uint64_t dir[Tsunami::Max_CPUs];
    bool dirInterrupting[Tsunami::Max_CPUs];
    uint64_t drir;

  public:
    TsunamiCChip(const std::string &name, Tsunami *t,
               Addr addr, Addr mask, MemoryController *mmu);

    virtual Fault read(MemReqPtr req, uint8_t *data);
    virtual Fault write(MemReqPtr req, const uint8_t *data);

    void postDRIR(uint64_t bitvector);
    void clearDRIR(uint64_t bitvector);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    uint64_t misc;
    bool RTCInterrupting;
};

#endif // __TSUNAMI_CCHIP_HH__
