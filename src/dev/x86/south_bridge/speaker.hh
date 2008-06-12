/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#ifndef __DEV_X86_SOUTH_BRIDGE_SPEAKER_HH__
#define __DEV_X86_SOUTH_BRIDGE_SPEAKER_HH__

#include "arch/x86/x86_traits.hh"
#include "base/range.hh"
#include "dev/x86/south_bridge/sub_device.hh"

namespace X86ISA
{

class I8254;

class Speaker : public SubDevice
{
  protected:
    BitUnion8(SpeakerControl)
        Bitfield<0> gate;
        Bitfield<1> speaker;
        Bitfield<5> timer;
    EndBitUnion(SpeakerControl)

    SpeakerControl controlVal;

    I8254 * timer;

  public:

    Speaker(I8254 * _timer) : timer(_timer)
    {}
    Speaker(I8254 * _timer, Tick _latency) :
        SubDevice(_latency), timer(_timer)
    {}
    Speaker(I8254 * _timer, Addr start, Addr size, Tick _latency) :
        SubDevice(start, size, _latency), timer(_timer)
    {}

    Tick read(PacketPtr pkt);

    Tick write(PacketPtr pkt);
};

}; // namespace X86ISA

#endif //__DEV_X86_SOUTH_BRIDGE_SPEAKER_HH__
