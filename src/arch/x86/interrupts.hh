/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#ifndef __ARCH_X86_INTERRUPTS_HH__
#define __ARCH_X86_INTERRUPTS_HH__

#include "arch/x86/apicregs.hh"
#include "arch/x86/faults.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"
#include "dev/io_device.hh"
#include "dev/x86/intdev.hh"
#include "params/X86LocalApic.hh"
#include "sim/eventq.hh"

class ThreadContext;

namespace X86ISA
{

class Interrupts : public BasicPioDevice, IntDev
{
  protected:
    // Storage for the APIC registers
    uint32_t regs[NUM_APIC_REGS];

    /*
     * Timing related stuff.
     */
    Tick latency;
    Tick clock;

    class ApicTimerEvent : public Event
    {
      public:
        ApicTimerEvent() : Event()
        {}

        void process()
        {
            warn("Local APIC timer event doesn't do anything!\n");
        }
    };

    ApicTimerEvent apicTimerEvent;

    /*
     * IRR and ISR maintenance.
     */
    uint8_t IRRV;
    uint8_t ISRV;

    int
    findRegArrayMSB(ApicRegIndex base)
    {
        int offset = 7;
        do {
            if (regs[base + offset] != 0) {
                return offset * 32 + findMsbSet(regs[base + offset]);
            }
        } while (offset--);
        return 0;
    }

    void
    updateIRRV()
    {
        IRRV = findRegArrayMSB(APIC_INTERRUPT_REQUEST_BASE);
    }

    void
    updateISRV()
    {
        ISRV = findRegArrayMSB(APIC_IN_SERVICE_BASE);
    }

    void
    setRegArrayBit(ApicRegIndex base, uint8_t vector)
    {
        regs[base + (vector % 32)] |= (1 << (vector >> 5));
    }

    void
    clearRegArrayBit(ApicRegIndex base, uint8_t vector)
    {
        regs[base + (vector % 32)] &= ~(1 << (vector >> 5));
    }

    bool
    getRegArrayBit(ApicRegIndex base, uint8_t vector)
    {
        return bits(regs[base + (vector % 32)], vector >> 5);
    }

  public:
    /*
     * Params stuff.
     */
    typedef X86LocalApicParams Params;

    void setClock(Tick newClock)
    {
        clock = newClock;
    }

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    /*
     * Functions to interact with the interrupt port from IntDev.
     */
    Tick read(PacketPtr pkt);
    Tick write(PacketPtr pkt);
    Tick recvMessage(PacketPtr pkt);

    void addressRanges(AddrRangeList &range_list)
    {
        range_list.clear();
        range_list.push_back(RangeEx(x86LocalAPICAddress(0, 0),
                                     x86LocalAPICAddress(0, 0) + PageBytes));
    }

    void getIntAddrRange(AddrRangeList &range_list)
    {
        range_list.clear();
        range_list.push_back(RangeEx(x86InterruptAddress(0, 0),
                    x86InterruptAddress(0, 0) + PhysAddrAPICRangeSize));
    }

    Port *getPort(const std::string &if_name, int idx = -1)
    {
        if (if_name == "int_port")
            return intPort;
        return BasicPioDevice::getPort(if_name, idx);
    }

    /*
     * Functions to access and manipulate the APIC's registers.
     */

    uint32_t readReg(ApicRegIndex miscReg);
    void setReg(ApicRegIndex reg, uint32_t val);
    void setRegNoEffect(ApicRegIndex reg, uint32_t val)
    {
        regs[reg] = val;
    }

    /*
     * Constructor.
     */

    Interrupts(Params * p) : BasicPioDevice(p), IntDev(this),
                             latency(p->pio_latency), clock(0)
    {
        pioSize = PageBytes;
        memset(regs, 0, sizeof(regs));
        //Set the local apic DFR to the flat model.
        regs[APIC_DESTINATION_FORMAT] = (uint32_t)(-1);
        ISRV = 0;
        IRRV = 0;
    }

    /*
     * Functions for retrieving interrupts for the CPU to handle.
     */

    bool check_interrupts(ThreadContext * tc) const;
    Fault getInterrupt(ThreadContext * tc);
    void updateIntrInfo(ThreadContext * tc);

    /*
     * Serialization.
     */

    void serialize(std::ostream & os)
    {
        panic("Interrupts::serialize unimplemented!\n");
    }

    void unserialize(Checkpoint * cp, const std::string & section)
    {
        panic("Interrupts::unserialize unimplemented!\n");
    }

    /*
     * Old functions needed for compatability but which will be phased out
     * eventually.
     */
    void post(int int_num, int index)
    {
        panic("Interrupts::post unimplemented!\n");
    }

    void clear(int int_num, int index)
    {
        panic("Interrupts::clear unimplemented!\n");
    }

    void clear_all()
    {
        panic("Interrupts::clear_all unimplemented!\n");
    }
};

};

#endif // __ARCH_X86_INTERRUPTS_HH__
