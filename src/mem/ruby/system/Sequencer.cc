
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

#include "mem/ruby/libruby.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/protocol/Protocol.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/system/CacheMemory.hh"
#include "mem/protocol/CacheMsg.hh"
#include "mem/ruby/recorder/Tracer.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/protocol/Protocol.hh"
#include "mem/gems_common/Map.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "cpu/rubytest/RubyTester.hh"

#include "params/RubySequencer.hh"

Sequencer *
RubySequencerParams::create()
{
    return new Sequencer(this);
}
 
Sequencer::Sequencer(const Params *p)
    : RubyPort(p), deadlockCheckEvent(this)
{
    m_store_waiting_on_load_cycles = 0;
    m_store_waiting_on_store_cycles = 0;
    m_load_waiting_on_store_cycles = 0;
    m_load_waiting_on_load_cycles = 0;
    
    m_outstanding_count = 0;

    m_max_outstanding_requests = 0;
    m_deadlock_threshold = 0;
    m_instCache_ptr = NULL;
    m_dataCache_ptr = NULL;

    m_instCache_ptr = p->icache;
    m_dataCache_ptr = p->dcache;
    m_max_outstanding_requests = p->max_outstanding_requests;
    m_deadlock_threshold = p->deadlock_threshold;
    m_usingRubyTester = p->using_ruby_tester;

    assert(m_max_outstanding_requests > 0);
    assert(m_deadlock_threshold > 0);
    assert(m_instCache_ptr != NULL);
    assert(m_dataCache_ptr != NULL);
}

Sequencer::~Sequencer() {

}

void Sequencer::wakeup() {
  // Check for deadlock of any of the requests
  Time current_time = g_eventQueue_ptr->getTime();

  // Check across all outstanding requests
  int total_outstanding = 0;

  Vector<Address> keys = m_readRequestTable.keys();
  for (int i=0; i<keys.size(); i++) {
    SequencerRequest* request = m_readRequestTable.lookup(keys[i]);
    if (current_time - request->issue_time >= m_deadlock_threshold) {
      WARN_MSG("Possible Deadlock detected");
      WARN_EXPR(request);
      WARN_EXPR(m_version);
      WARN_EXPR(request->ruby_request.paddr);
      WARN_EXPR(keys.size());
      WARN_EXPR(current_time);
      WARN_EXPR(request->issue_time);
      WARN_EXPR(current_time - request->issue_time);
      ERROR_MSG("Aborting");
    }
  }

  keys = m_writeRequestTable.keys();
  for (int i=0; i<keys.size(); i++) {
    SequencerRequest* request = m_writeRequestTable.lookup(keys[i]);
    if (current_time - request->issue_time >= m_deadlock_threshold) {
      WARN_MSG("Possible Deadlock detected");
      WARN_EXPR(request);
      WARN_EXPR(m_version);
      WARN_EXPR(current_time);
      WARN_EXPR(request->issue_time);
      WARN_EXPR(current_time - request->issue_time);
      WARN_EXPR(keys.size());
      ERROR_MSG("Aborting");
    }
  }
  total_outstanding += m_writeRequestTable.size() + m_readRequestTable.size();

  assert(m_outstanding_count == total_outstanding);

  if (m_outstanding_count > 0) { // If there are still outstanding requests, keep checking
    schedule(deadlockCheckEvent, 
             (m_deadlock_threshold * g_eventQueue_ptr->getClock()) + curTick);
  }
}

void Sequencer::printStats(ostream & out) const {
  out << "Sequencer: " << m_name << endl;
  out << "  store_waiting_on_load_cycles: " << m_store_waiting_on_load_cycles << endl;
  out << "  store_waiting_on_store_cycles: " << m_store_waiting_on_store_cycles << endl;
  out << "  load_waiting_on_load_cycles: " << m_load_waiting_on_load_cycles << endl;
  out << "  load_waiting_on_store_cycles: " << m_load_waiting_on_store_cycles << endl;
}

void Sequencer::printProgress(ostream& out) const{
  /*
  int total_demand = 0;
  out << "Sequencer Stats Version " << m_version << endl;
  out << "Current time = " << g_eventQueue_ptr->getTime() << endl;
  out << "---------------" << endl;
  out << "outstanding requests" << endl;

  Vector<Address> rkeys = m_readRequestTable.keys();
  int read_size = rkeys.size();
  out << "proc " << m_version << " Read Requests = " << read_size << endl;
  // print the request table
  for(int i=0; i < read_size; ++i){
    SequencerRequest * request = m_readRequestTable.lookup(rkeys[i]);
    out << "\tRequest[ " << i << " ] = " << request->type << " Address " << rkeys[i]  << " Posted " << request->issue_time << " PF " << PrefetchBit_No << endl;
    total_demand++;
  }

  Vector<Address> wkeys = m_writeRequestTable.keys();
  int write_size = wkeys.size();
  out << "proc " << m_version << " Write Requests = " << write_size << endl;
  // print the request table
  for(int i=0; i < write_size; ++i){
      CacheMsg & request = m_writeRequestTable.lookup(wkeys[i]);
      out << "\tRequest[ " << i << " ] = " << request.getType() << " Address " << wkeys[i]  << " Posted " << request.getTime() << " PF " << request.getPrefetch() << endl;
      if( request.getPrefetch() == PrefetchBit_No ){
        total_demand++;
      }
  }

  out << endl;

  out << "Total Number Outstanding: " << m_outstanding_count << endl;
  out << "Total Number Demand     : " << total_demand << endl;
  out << "Total Number Prefetches : " << m_outstanding_count - total_demand << endl;
  out << endl;
  out << endl;
  */
}

void Sequencer::printConfig(ostream& out) const {
  out << "Seqeuncer config: " << m_name << endl;
  out << "  controller: " << m_controller->getName() << endl;
  out << "  version: " << m_version << endl;
  out << "  max_outstanding_requests: " << m_max_outstanding_requests << endl;
  out << "  deadlock_threshold: " << m_deadlock_threshold << endl;
}

// Insert the request on the correct request table.  Return true if
// the entry was already present.
bool Sequencer::insertRequest(SequencerRequest* request) {
  int total_outstanding = m_writeRequestTable.size() + m_readRequestTable.size();

  assert(m_outstanding_count == total_outstanding);

  // See if we should schedule a deadlock check
  if (deadlockCheckEvent.scheduled() == false) {
    schedule(deadlockCheckEvent, m_deadlock_threshold + curTick);
  }

  Address line_addr(request->ruby_request.paddr);
  line_addr.makeLineAddress();
  if ((request->ruby_request.type == RubyRequestType_ST) ||
      (request->ruby_request.type == RubyRequestType_RMW_Read) ||  
      (request->ruby_request.type == RubyRequestType_RMW_Write) ||  
      (request->ruby_request.type == RubyRequestType_Locked_Read) ||  
      (request->ruby_request.type == RubyRequestType_Locked_Write)) {
    if (m_writeRequestTable.exist(line_addr)) {
      m_writeRequestTable.lookup(line_addr) = request;
      //      return true;
      assert(0); // drh5: isn't this an error?  do you lose the initial request?
    }
    m_writeRequestTable.allocate(line_addr);
    m_writeRequestTable.lookup(line_addr) = request;
    m_outstanding_count++;
  } else {
    if (m_readRequestTable.exist(line_addr)) {
      m_readRequestTable.lookup(line_addr) = request;
      //      return true;
      assert(0); // drh5: isn't this an error?  do you lose the initial request?
    }
    m_readRequestTable.allocate(line_addr);
    m_readRequestTable.lookup(line_addr) = request;
    m_outstanding_count++;
  }

  g_system_ptr->getProfiler()->sequencerRequests(m_outstanding_count);

  total_outstanding = m_writeRequestTable.size() + m_readRequestTable.size();
  assert(m_outstanding_count == total_outstanding);

  return false;
}

void Sequencer::removeRequest(SequencerRequest* srequest) {

  assert(m_outstanding_count == m_writeRequestTable.size() + m_readRequestTable.size());

  const RubyRequest & ruby_request = srequest->ruby_request;
  Address line_addr(ruby_request.paddr);
  line_addr.makeLineAddress();
  if ((ruby_request.type == RubyRequestType_ST) ||
      (ruby_request.type == RubyRequestType_RMW_Read) ||  
      (ruby_request.type == RubyRequestType_RMW_Write) ||  
      (ruby_request.type == RubyRequestType_Locked_Read) ||
      (ruby_request.type == RubyRequestType_Locked_Write)) {
    m_writeRequestTable.deallocate(line_addr);
  } else {
    m_readRequestTable.deallocate(line_addr);
  }
  m_outstanding_count--;

  assert(m_outstanding_count == m_writeRequestTable.size() + m_readRequestTable.size());
}

void Sequencer::writeCallback(const Address& address, DataBlock& data) {

  assert(address == line_address(address));
  assert(m_writeRequestTable.exist(line_address(address)));

  SequencerRequest* request = m_writeRequestTable.lookup(address);

  removeRequest(request);

  assert((request->ruby_request.type == RubyRequestType_ST) ||
         (request->ruby_request.type == RubyRequestType_RMW_Read) ||  
         (request->ruby_request.type == RubyRequestType_RMW_Write) ||  
         (request->ruby_request.type == RubyRequestType_Locked_Read) ||
         (request->ruby_request.type == RubyRequestType_Locked_Write));

  if (request->ruby_request.type == RubyRequestType_Locked_Read) {
    m_dataCache_ptr->setLocked(address, m_version);
  }
  else if (request->ruby_request.type == RubyRequestType_RMW_Read) {
    m_controller->blockOnQueue(address, m_mandatory_q_ptr);
  }
  else if (request->ruby_request.type == RubyRequestType_RMW_Write) {
    m_controller->unblock(address);
  }

  hitCallback(request, data);
}

void Sequencer::readCallback(const Address& address, DataBlock& data) {

  assert(address == line_address(address));
  assert(m_readRequestTable.exist(line_address(address)));

  SequencerRequest* request = m_readRequestTable.lookup(address);
  removeRequest(request);

  assert((request->ruby_request.type == RubyRequestType_LD) ||
	 (request->ruby_request.type == RubyRequestType_RMW_Read) ||
         (request->ruby_request.type == RubyRequestType_IFETCH));

  hitCallback(request, data);
}

void Sequencer::hitCallback(SequencerRequest* srequest, DataBlock& data) {
  const RubyRequest & ruby_request = srequest->ruby_request;
  Address request_address(ruby_request.paddr);
  Address request_line_address(ruby_request.paddr);
  request_line_address.makeLineAddress();
  RubyRequestType type = ruby_request.type;
  Time issued_time = srequest->issue_time;

  // Set this cache entry to the most recently used
  if (type == RubyRequestType_IFETCH) {
    if (m_instCache_ptr->isTagPresent(request_line_address) )
      m_instCache_ptr->setMRU(request_line_address);
  } else {
    if (m_dataCache_ptr->isTagPresent(request_line_address) )
      m_dataCache_ptr->setMRU(request_line_address);
  }

  assert(g_eventQueue_ptr->getTime() >= issued_time);
  Time miss_latency = g_eventQueue_ptr->getTime() - issued_time;

  // Profile the miss latency for all non-zero demand misses
  if (miss_latency != 0) {
    g_system_ptr->getProfiler()->missLatency(miss_latency, type);

    if (Debug::getProtocolTrace()) {
      g_system_ptr->getProfiler()->profileTransition("Seq", m_version, Address(ruby_request.paddr),
                                                     "", "Done", "", int_to_string(miss_latency)+" cycles");
    }
  }
  /*
  if (request.getPrefetch() == PrefetchBit_Yes) {
    return; // Ignore the prefetch
  }
  */

  // update the data
  if (ruby_request.data != NULL) {
    if ((type == RubyRequestType_LD) ||
        (type == RubyRequestType_IFETCH) ||
        (type == RubyRequestType_RMW_Read)) {
      memcpy(ruby_request.data, data.getData(request_address.getOffset(), ruby_request.len), ruby_request.len);
    } else {
      data.setData(ruby_request.data, request_address.getOffset(), ruby_request.len);
    }
  }

  //
  // If using the RubyTester, update the RubyTester sender state's subBlock 
  // with the recieved data.  The tester will later access this state.
  // Note: RubyPort will access it's sender state before the RubyTester.
  //
  if (m_usingRubyTester) {
      RubyTester::SenderState* testerSenderState;
      testerSenderState = safe_cast<RubyTester::SenderState*>( \
          safe_cast<RubyPort::SenderState*>(ruby_request.pkt->senderState)->saved);
      testerSenderState->subBlock->mergeFrom(data);
  }

  ruby_hit_callback(ruby_request.pkt);
  delete srequest;
}

// Returns true if the sequencer already has a load or store outstanding
RequestStatus Sequencer::getRequestStatus(const RubyRequest& request) {
  bool is_outstanding_store = m_writeRequestTable.exist(line_address(Address(request.paddr)));
  bool is_outstanding_load = m_readRequestTable.exist(line_address(Address(request.paddr)));
  if ( is_outstanding_store ) {
    if ((request.type == RubyRequestType_LD) ||
        (request.type == RubyRequestType_IFETCH) ||
        (request.type == RubyRequestType_RMW_Read)) {
      m_store_waiting_on_load_cycles++;
    } else {
      m_store_waiting_on_store_cycles++;
    }
    return RequestStatus_Aliased;
  } else if ( is_outstanding_load ) {
    if ((request.type == RubyRequestType_ST) ||
        (request.type == RubyRequestType_RMW_Write) ) {
      m_load_waiting_on_store_cycles++;
    } else {
      m_load_waiting_on_load_cycles++;
    }
    return RequestStatus_Aliased;
  }

  if (m_outstanding_count >= m_max_outstanding_requests) {
    return RequestStatus_BufferFull;
  }
  
  return RequestStatus_Ready;
}

bool Sequencer::empty() const {
  return (m_writeRequestTable.size() == 0) && (m_readRequestTable.size() == 0);
}


RequestStatus Sequencer::makeRequest(const RubyRequest & request)
{
  assert(Address(request.paddr).getOffset() + request.len <= 
         RubySystem::getBlockSizeBytes());
  RequestStatus status = getRequestStatus(request);
  if (status == RequestStatus_Ready) {
    SequencerRequest *srequest = new SequencerRequest(request, 
                                                  g_eventQueue_ptr->getTime());
    bool found = insertRequest(srequest);
    if (!found) {
      if (request.type == RubyRequestType_Locked_Write) {
        // NOTE: it is OK to check the locked flag here as the mandatory queue will be checked first
        // ensuring that nothing comes between checking the flag and servicing the store
        if (!m_dataCache_ptr->isLocked(line_address(Address(request.paddr)), m_version)) {
          return RequestStatus_LlscFailed;
        }
        else {
          m_dataCache_ptr->clearLocked(line_address(Address(request.paddr)));
        }
      }
      issueRequest(request);

      // TODO: issue hardware prefetches here
      return RequestStatus_Issued;
    }
    else {
        panic("Sequencer::makeRequest should never be called if the request"\
              "is already outstanding\n");
        return RequestStatus_NULL;
    }
  } else {
    return status;
  }
}

void Sequencer::issueRequest(const RubyRequest& request) {

  // TODO: get rid of CacheMsg, CacheRequestType, and AccessModeTYpe, & have SLICC use RubyRequest and subtypes natively
  CacheRequestType ctype;
  switch(request.type) {
  case RubyRequestType_IFETCH:
    ctype = CacheRequestType_IFETCH;
    break;
  case RubyRequestType_LD:
    ctype = CacheRequestType_LD;
    break;
  case RubyRequestType_ST:
    ctype = CacheRequestType_ST;
    break;
  case RubyRequestType_Locked_Read:
  case RubyRequestType_Locked_Write:
    ctype = CacheRequestType_ATOMIC;
    break;
  case RubyRequestType_RMW_Read:
    ctype = CacheRequestType_ATOMIC;
    break;
  case RubyRequestType_RMW_Write:
    ctype = CacheRequestType_ATOMIC;
    break;
  default:
    assert(0);
  }
  AccessModeType amtype;
  switch(request.access_mode){
  case RubyAccessMode_User:
    amtype = AccessModeType_UserMode;
    break;
  case RubyAccessMode_Supervisor:
    amtype = AccessModeType_SupervisorMode;
    break;
  case RubyAccessMode_Device:
    amtype = AccessModeType_UserMode;
    break;
  default:
    assert(0);
  }
  Address line_addr(request.paddr);
  line_addr.makeLineAddress();
  CacheMsg msg(line_addr, Address(request.paddr), ctype, Address(request.pc), amtype, request.len, PrefetchBit_No, request.proc_id);

  if (Debug::getProtocolTrace()) {
    g_system_ptr->getProfiler()->profileTransition("Seq", m_version, Address(request.paddr),
                                                   "", "Begin", "", RubyRequestType_to_string(request.type));
  }

  if (g_system_ptr->getTracer()->traceEnabled()) {
    g_system_ptr->getTracer()->traceRequest(this, line_addr, Address(request.pc),
                                            request.type, g_eventQueue_ptr->getTime());
  }

  Time latency = 0;  // initialzed to an null value

  if (request.type == RubyRequestType_IFETCH)
    latency = m_instCache_ptr->getLatency();
  else
    latency = m_dataCache_ptr->getLatency();

  // Send the message to the cache controller
  assert(latency > 0);

  assert(m_mandatory_q_ptr != NULL);
  m_mandatory_q_ptr->enqueue(msg, latency);
}
/*
bool Sequencer::tryCacheAccess(const Address& addr, CacheRequestType type,
                               AccessModeType access_mode,
                               int size, DataBlock*& data_ptr) {
  if (type == CacheRequestType_IFETCH) {
    return m_instCache_ptr->tryCacheAccess(line_address(addr), type, data_ptr);
  } else {
    return m_dataCache_ptr->tryCacheAccess(line_address(addr), type, data_ptr);
  }
}
*/

void Sequencer::print(ostream& out) const {
  out << "[Sequencer: " << m_version
      << ", outstanding requests: " << m_outstanding_count;

  out << ", read request table: " << m_readRequestTable
      << ", write request table: " << m_writeRequestTable;
  out << "]";
}

// this can be called from setState whenever coherence permissions are upgraded
// when invoked, coherence violations will be checked for the given block
void Sequencer::checkCoherence(const Address& addr) {
#ifdef CHECK_COHERENCE
  g_system_ptr->checkGlobalCoherenceInvariant(addr);
#endif
}

