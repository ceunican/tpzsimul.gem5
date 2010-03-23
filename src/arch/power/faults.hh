/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2009 The University of Edinburgh
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
 *          Timothy M. Jones
 */

#ifndef __ARCH_POWER_FAULTS_HH__
#define __ARCH_POWER_FAULTS_HH__

#include "sim/faults.hh"

namespace PowerISA
{

class PowerFault : public FaultBase
{
  protected:
    FaultName _name;

    PowerFault(FaultName name)
        : _name(name)
    {
    }

    FaultName
    name() const
    {
        return _name;
    }
};


class UnimplementedOpcodeFault : public PowerFault
{
  public:
    UnimplementedOpcodeFault()
        : PowerFault("Unimplemented Opcode")
    {
    }
};


class MachineCheckFault : public PowerFault
{
  public:
    MachineCheckFault()
        : PowerFault("Machine Check")
    {
    }
};


class AlignmentFault : public PowerFault
{
  public:
    AlignmentFault()
        : PowerFault("Alignment")
    {
    }

    bool
    isAlignmentFault() const
    {
        return true;
    }
};


static inline Fault
genMachineCheckFault()
{
    return new MachineCheckFault();
}

} // PowerISA namespace

#endif // __ARCH_POWER_FAULTS_HH__
