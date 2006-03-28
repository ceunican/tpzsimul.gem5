/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "dev/io_device.hh"
#include "sim/builder.hh"

void
PioPort::SendEvent::process()
{
    if (port->sendTiming(packet) == Success)
        return;

    port->transmitList.push_back(&packet);
}

PioDevice::PioDevice(const std::string &name, Platform *p)
    : SimObject(name), platform(p)
{
    pioPort = new PioPort(this);
}


bool
PioDevice::recvTiming(Packet &pkt)
{
    device->recvAtomic(pkt);
    sendTiming(pkt, pkt.responseTime-pkt.requestTime);
    return Success;
}

PioDevice::~PioDevice()
{
    if (pioPort)
        delete pioInterface;
}

void
DmaPort::sendDma(Packet &pkt)
{
    device->platform->system->memoryMode()
    {
        case MemAtomic:
    }
}

DmaDevice::DmaDevice(const std::string &name, Platform *p)
    : PioDevice(name, p)
{
    dmaPort = new dmaPort(this);
}

void
DmaPort::dmaAction(Memory::Command cmd, DmaPort port, Addr addr, int size,
                     Event *event, uint8_t *data = NULL)
{

    assert(event);

    int prevSize = 0;
    Packet basePkt;
    Request baseReq;

    basePkt.flags = 0;
    basePkt.coherence = NULL;
    basePkt.senderState = NULL;
    basePkt.src = 0;
    basePkt.dest = 0;
    basePkt.cmd = cmd;
    basePkt.result = Unknown;
    basePkt.request = NULL;
    baseReq.nicReq = true;
    baseReq.time = curTick;

    completionEvent = event;

    for (ChunkGenerator gen(addr, size, peerBlockSize());
         !gen.done(); gen.next()) {
            Packet *pkt = new Packet(basePkt);
            Request *req = new Request(baseReq);
            pkt->addr = gen.addr();
            pkt->size = gen.size();
            pkt->req = req;
            pkt->req->paddr = pkt->addr;
            pkt->req->size = pkt->size;
            // Increment the data pointer on a write
            pkt->data = data ? data + prevSize : NULL ;
            prevSize = pkt->size;

            sendDma(*pkt);
    }
}


void
DmaPort::sendDma(Packet &pkt)
{
   // some kind of selction between access methods
   // more work is going to have to be done to make
   // switching actually work
   MemState state = device->platform->system->memState;

   if (state == Timing) {
       if (sendTiming(pkt) == Failure)
           transmitList.push_back(&packet);
   } else if (state == Atomic) {
       sendAtomic(pkt);
       completionEvent->schedule(pkt.responseTime - pkt.requestTime);
       completionEvent == NULL;
   } else if (state == Functional) {
       sendFunctional(pkt);
       // Is this correct???
       completionEvent->schedule(pkt.responseTime - pkt.requestTime);
       completionEvent == NULL;
   } else
       panic("Unknown memory command state.");

}

DmaDevice::~DmaDevice()
{
    if (dmaInterface)
        delete dmaInterface;
}


