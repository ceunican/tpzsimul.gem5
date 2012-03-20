/*
 * Copyright (c) 2012 The University of Cantabria (Spain)
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

#include <cassert>
#include <numeric>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/protocol/TopologyType.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/simple/SimpleLink.hh"
#include "mem/ruby/network/simple/Throttle.hh"
#include "mem/ruby/network/topaz/TopazNetwork.hh"
#include "mem/ruby/network/topaz/TopazSwitch.hh"
#include "mem/ruby/network/topaz/TopazSwitchFlow.hh"
#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/Topology.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/system/System.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

// Helper for Ruby-Topaz Mapping
// Example for 8 L1s, 32 L2s and 8 memories.
//
// 0 -------- 7 8 -------- 39 40 -------- 47      -> node number received
// L1_0 -- L1_7 L2_0 -- L2_31 Mem_0 -- Mem_7      -> MachineID returned

extern inline
MachineID _nodeNumber_to_MachineID(SwitchID node) {
    for (MachineType m = MachineType_FIRST; m < MachineType_NUM; ++m) {
        int num_machines = MachineType_base_count(m);
        if (node < num_machines) {
            MachineID mid = {m, node};
            return mid;
        }
        else {
            node = node - num_machines;
        }
    }
    MachineType merror = MachineType_FIRST;
    MachineID error = {merror,node};
    return error;
}

TopazNetwork::TopazNetwork(const Params *p)
    : Network(p)
{
    m_buffer_size = p->buffer_size;
    m_endpoint_bandwidth = p->endpoint_bandwidth;
    m_adaptive_routing = p->adaptive_routing;

    // Note: the parent Network Object constructor is called before the
    // TopazNetwork child constructor.  Therefore, the member variables
    // used below should already be initialized.

    m_endpoint_switches.resize(m_nodes);

    m_in_use.resize(m_virtual_networks);
    m_ordered.resize(m_virtual_networks);
    for (int i = 0; i < m_virtual_networks; i++) {
        m_in_use[i] = false;
        m_ordered[i] = false;
    }
    m_number_messages=0;
    m_number_ordered_messages=0;
    m_number_topaz_ordered_messages=0;
    m_number_topaz_messages=0;
    m_totalNetMsg=0;
    m_totalTopazMsg=0;
    m_forward_mapping = new SwitchID[m_nodes];
    m_reverse_mapping.resize(m_nodes);

    // Allocate to and from queues
    m_toNetQueues.resize(m_nodes);
    m_fromNetQueues.resize(m_nodes);
    for (int node = 0; node < m_nodes; node++) {
       m_toNetQueues[node].resize(m_virtual_networks);
       m_fromNetQueues[node].resize(m_virtual_networks);
       for (int j = 0; j < m_virtual_networks; j++) {
            m_toNetQueues[node][j] = new
                    MessageBuffer(csprintf("toNet node %d j %d", node, j));
            m_toNetQueues[node][j]->toNet();
            m_fromNetQueues[node][j] = new
                    MessageBuffer(csprintf("fromNet node %d j %d", node, j));
             m_fromNetQueues[node][j]->fromNet();
        }
    }
    m_flitSize=p->topaz_flit_size;
    m_processorClockRatio=p->topaz_clock_ratio;
    m_simulName = (p->topaz_network).c_str();
    m_topazInitFile = (p->topaz_init_file).c_str();
    m_topaz_adaptive_interface_threshold = p->topaz_adaptive_interface_threshold;
}

void
TopazNetwork::init()
{
    Network::init();

    // The topology pointer should have already been initialized in
    // the parent class network constructor.
    assert(m_topology_ptr != NULL);
    int number_of_switches = m_topology_ptr->numSwitches();
    for (int i = 0; i < number_of_switches; i++) {
        m_switch_ptr_vector.push_back(new TopazSwitch(i, this));
    }

    // false because this isn't a reconfiguration
    m_topology_ptr->createLinks(this, false);
    //
    //  TOPAZ INITIALIZATION AND INSTALLERS
    //
    //
    m_permanentDisable=false;
    //TOPAZ initialization file is here?
    //(otherwise segment.fault)
    ifstream own(m_topazInitFile);
    if( ! own.good() ){
        cerr << "<TOPAZ>  Can't open topaz init file :: " << m_topazInitFile << endl;
        cerr<<" That file must have the route to Router, Network & Simulation SGML"<< endl;
        cerr<<" you can declare it with --topaz-init-file"<<endl;
        cerr<<"</TOPAZ>"<<endl;
        assert(0);
    }
    own.close();
    TPZString initString ;
    initString = TPZString("TPZSimul -q -s ") + m_simulName +
                 TPZString (" -t EMPTY -d 100000 -v 4 -F ")+m_topazInitFile;
    m_firstTrigger=~0;
    //Run each virtual network in a separate phisical netwokr (aka DASH)
    //or just rely on virtual netowkrs
    //the network should be conveived to manage separetelly all
    //traffic if we want to avoid end-to-end deadlock
    //Most CMP oriented TOPAZ networks has such capability
    for (int i = 0; i < m_virtual_networks; i++) {
       TPZSIMULATOR()->createSimulation(initString);
       //Simulated networks go from 1,2... (Watch out! is +1 than slicc's vnets)
       if (TPZSIMULATOR()->getSimulation(1)->needToUnify()) {
          m_unify=1;
          break;
       }
       else {
          m_unify=m_virtual_networks;
       }
    }
    if (m_flitSize  == 0) { //Not specified by ruby con
       m_flitSize=TPZSIMULATOR()->getSimulation(1)->getFlitSize();
       cerr<<"Warning:topaz_flit was not specified in in Ruby.py options.";
       cerr<<" It will be get from SGML specification."<<endl;
    }


    if (m_flitSize == 0) {
       cerr<<"<TOPAZ> FLITSIZE must be specified in simulation sgml file as";
       cerr<<" <LinkWidth id=[whatever bytes]" << endl;
       cerr<<"Closing simulation </TOPAZ>" << endl;
       assert(0);
    }

    for (unsigned i = 1; i <= m_unify; i++)
      TPZSIMULATOR()->getSimulation(i)->setPacketLength(
                                         getMessageSizeTopaz(MessageSizeType_Data));
    //Clock ratio
    if (m_processorClockRatio == 0 ) {
       m_processorClockRatio=int(TPZSIMULATOR()->
                                 getSimulation(1)->getNetworkClockRatioSGML());
    }
    //Network cant be faster than memory
    assert(m_processorClockRatio>=1);

    this->enableTopaz();
    //Grab rubys seed and re-set it
    //(Topaz may internally redefine seed if it is defined in SGM
    srandom(g_system_ptr->getRandomSeed());
}

int TopazNetwork::getMessageSizeTopaz(MessageSizeType size_type) const {
    //Padding last flit
    return (int) ceil((double) RubySystem::getNetwork()-> MessageSizeType_to_int(
        size_type)/m_flitSize);
}


bool TopazNetwork::useGemsNetwork(int vnet) {
    bool useGems = false;
    // During warmup we use GEMS' network
    if (inWarmup()) useGems = true;
    // Messages in order must go all through the same network. If this is an
    // ordered virtual network, send the message through the network where you
    // find any message.
    if (isVNetOrdered(vnet)) {
        if (numberOfOrderedMessages() > 0) {
            assert(numberOfTopazOrderedMessages() == 0);
            useGems = true;
        }
        else if (numberOfTopazOrderedMessages() > 0) {
            useGems = false;
        }
    }
    const int messages_in_ruby = numberOfMessages();
    if (m_number_topaz_messages < 0) {
        static bool warn_topaz_count_is_wrong = true;
        if (warn_topaz_count_is_wrong) {
            DPRINTF(RubyNetwork,"Topaz is not counting messages correctly,");
            DPRINTF(RubyNetwork," ignoring g_MAX_MESSAGES_THROUGH_GEMS.");
            DPRINTF(RubyNetwork,"Please, fix topaz if you want to use this parameter.");
            warn_topaz_count_is_wrong = false;
        }
        useGems = false;
    }
    // If the total number of messages is small
    //(less than m_topaz_adaptive_interface_threshold), we can use SimpleNetwor
    if ((messages_in_ruby + m_number_topaz_messages) <
                             m_topaz_adaptive_interface_threshold)
        useGems = true;
    else
        useGems = false;
    return useGems;
}

void TopazNetwork::enableTopaz(){
    if( m_permanentDisable ){
        cout<<"TOPAZ PERMANETLY DISABLED"<<endl;
        return;
    }
    cout<<endl<<"<TOPAZ> +++++++ Topaz Enabled! +++++++ </TOPAZ>"<<endl;
    m_in_warmup = false;
    return;
}

void TopazNetwork::disableTopaz(){
    cout<<endl<<"<TOPAZ> ******* Topaz DISABLED! ******* </TOPAZ>"<<endl;
    m_in_warmup = true;
    return;
}

MessageBuffer*
TopazNetwork::getToSimNetQueue(NodeID id, bool ordered, int network_num)
{
  checkNetworkAllocation(id, ordered, network_num);
  return m_toNetQueues[id][network_num];
}

MessageBuffer*
TopazNetwork::getFromSimNetQueue(NodeID id, bool ordered, int network_num)
{
  checkNetworkAllocation(id, ordered, network_num);
  return m_fromNetQueues[id][network_num];
}

void
TopazNetwork::reset()
{
    for (int node = 0; node < m_nodes; node++) {
        for (int j = 0; j < m_virtual_networks; j++) {
            m_toNetQueues[node][j]->clear();
            m_fromNetQueues[node][j]->clear();
        }
    }

    for(int i = 0; i < m_switch_ptr_vector.size(); i++){
        m_switch_ptr_vector[i]->clearBuffers();
    }
}

TopazNetwork::~TopazNetwork()
{
    for (int i = 0; i < m_nodes; i++) {
        deletePointers(m_toNetQueues[i]);
        deletePointers(m_fromNetQueues[i]);
    }
    deletePointers(m_switch_ptr_vector);
    deletePointers(m_buffers_to_free);
    // delete m_topology_ptr;
}

// From a switch to an endpoint node
void
TopazNetwork::makeOutLink(SwitchID src, NodeID dest, BasicLink* link,
                           LinkDirection direction,
                           const NetDest& routing_table_entry,
                           bool isReconfiguration)
{
    assert(dest < m_nodes);
    assert(src < m_switch_ptr_vector.size());
    assert(m_switch_ptr_vector[src] != NULL);

    if (isReconfiguration) {
        m_switch_ptr_vector[src]->reconfigureOutPort(routing_table_entry);
        return;
    }

    SimpleExtLink *simple_link = safe_cast<SimpleExtLink*>(link);

    m_switch_ptr_vector[src]->addOutPort(m_fromNetQueues[dest],
                                         routing_table_entry,
                                         simple_link->m_latency,
                                         simple_link->m_bw_multiplier);

    m_endpoint_switches[dest] = m_switch_ptr_vector[src];
}

// From an endpoint node to a switch
void
TopazNetwork::makeInLink(NodeID src, SwitchID dest, BasicLink* link,
                          LinkDirection direction,
                          const NetDest& routing_table_entry,
                          bool isReconfiguration)
{
    assert(src < m_nodes);
    if (isReconfiguration) {
        // do nothing
        return;
    }

    m_switch_ptr_vector[dest]->addInPort(m_toNetQueues[src]);
}

// From a switch to a switch
void
TopazNetwork::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                LinkDirection direction,
                                const NetDest& routing_table_entry,
                                bool isReconfiguration)
{
    if (isReconfiguration) {
        m_switch_ptr_vector[src]->reconfigureOutPort(routing_table_entry);
        return;
    }

    // Create a set of new MessageBuffers
    std::vector<MessageBuffer*> queues;
    for (int i = 0; i < m_virtual_networks; i++) {
        // allocate a buffer
        MessageBuffer* buffer_ptr = new MessageBuffer;
        buffer_ptr->setOrdering(true);
        if (m_buffer_size > 0) {
            buffer_ptr->resize(m_buffer_size);
        }
        queues.push_back(buffer_ptr);
        // remember to deallocate it
        m_buffers_to_free.push_back(buffer_ptr);
    }
    // Connect it to the two switches
    SimpleIntLink *simple_link = safe_cast<SimpleIntLink*>(link);

    m_switch_ptr_vector[dest]->addInPort(queues);
    m_switch_ptr_vector[src]->addOutPort(queues, routing_table_entry,
                                         simple_link->m_latency,
                                         simple_link->m_bw_multiplier);
}

void
TopazNetwork::checkNetworkAllocation(NodeID id, bool ordered, int network_num)
{
    assert(id < m_nodes);
    assert(network_num < m_virtual_networks);

    if (ordered) {
        m_ordered[network_num] = true;
    }
    m_in_use[network_num] = true;
}

MessageBuffer*
TopazNetwork::getToNetQueue(NodeID id, bool ordered, int network_num,
                             std::string vnet_type)
{
    checkNetworkAllocation(id, ordered, network_num);
    return m_toNetQueues[id][network_num];
}

MessageBuffer*
TopazNetwork::getFromNetQueue(NodeID id, bool ordered, int network_num,
                               std::string vnet_type)
{
    checkNetworkAllocation(id, ordered, network_num);
    return m_fromNetQueues[id][network_num];
}

const std::vector<Throttle*>*
TopazNetwork::getThrottles(NodeID id) const
{
    assert(id >= 0);
    assert(id < m_nodes);
    assert(m_endpoint_switches[id] != NULL);
    return m_endpoint_switches[id]->getThrottles();
}

void
TopazNetwork::printStats(ostream& out) const
{
    out<<"<TOPAZ>"<<endl;
    out<<"Ratio Processor Clock/Network Clock = "<<m_processorClockRatio<<endl;
    out<<"Flit size in bytes                  = "<<m_flitSize<<" bytes"<<endl;
    for(unsigned currentVnet=1;currentVnet<=m_unify;currentVnet++){
        TPZSIMULATOR()->getSimulation(currentVnet)->writeSimulationStatus(out);
        TPZSIMULATOR()->getSimulation(currentVnet)->
                        getNetwork()->writeComponentStatus(out);
    }
    out << endl;
    TopazNetwork* net=static_cast<TopazNetwork*>(g_system_ptr->getNetwork());
    long int Topaz_messages=net->getTotalTopazMsg();
    int percent = 100*Topaz_messages/(Topaz_messages+net->getTotalMsg()+1);
    out << "TOPAZ NETWORK USAGE" << endl;
    out << "Usage TOPAZ network: " << percent << "%" << endl;
    out << "Total TOPAZ messages: " << Topaz_messages << endl;
    out << "Total Number of messages: " << Topaz_messages+net->getTotalMsg() << endl;
    out << endl;
    out <<"</TOPAZ>"<<endl;
    out << endl;
    out << "SimpleNetwork Stats: The traffic you see here should be null"<< endl;
    out << "if your are not using the adaptive-interfaz)"<<endl;
    out << "-------------" << endl;
    out << endl;
    //
    // Determine total counts before printing out each switch's stats
    //
    std::vector<uint64> total_msg_counts;
    total_msg_counts.resize(MessageSizeType_NUM);
    for (MessageSizeType type = MessageSizeType_FIRST;
         type < MessageSizeType_NUM;
         ++type) {
        total_msg_counts[type] = 0;
    }

    for (int i = 0; i < m_switch_ptr_vector.size(); i++) {
        const std::vector<Throttle*>* throttles =
            m_switch_ptr_vector[i]->getThrottles();

        for (int p = 0; p < throttles->size(); p++) {

            const std::vector<std::vector<int> >& message_counts =
                ((*throttles)[p])->getCounters();

            for (MessageSizeType type = MessageSizeType_FIRST;
                 type < MessageSizeType_NUM;
                 ++type) {

                const std::vector<int> &mct = message_counts[type];
                int sum = accumulate(mct.begin(), mct.end(), 0);
                total_msg_counts[type] += uint64(sum);
            }
        }
    }
    uint64 total_msgs = 0;
    uint64 total_bytes = 0;
    for (MessageSizeType type = MessageSizeType_FIRST;
         type < MessageSizeType_NUM;
         ++type) {

        if (total_msg_counts[type] > 0) {
            out << "total_msg_count_" << type << ": " << total_msg_counts[type]
                << " " << total_msg_counts[type] *
                uint64(RubySystem::getNetwork()->MessageSizeType_to_int(type))
                << endl;

            total_msgs += total_msg_counts[type];

            total_bytes += total_msg_counts[type] *
                uint64(RubySystem::getNetwork()->MessageSizeType_to_int(type));

        }
    }

    out << "total_msgs: " << total_msgs
        << " total_bytes: " << total_bytes << endl;

    out << endl;
    for (int i = 0; i < m_switch_ptr_vector.size(); i++) {
        m_switch_ptr_vector[i]->printStats(out);
    }
    m_topology_ptr->printStats(out);
}

void
TopazNetwork::clearStats()
{
    for (int i = 0; i < m_switch_ptr_vector.size(); i++) {
        m_switch_ptr_vector[i]->clearStats();
    }
    m_topology_ptr->clearStats();
}

void
TopazNetwork::printConfig(ostream& out) const
{
    out << endl;
    out << "Network Configuration" << endl;
    out << "---------------------" << endl;
    out << "network: SIMPLE_NETWORK" << endl;
    out << "topology: " << m_topology_ptr->getName() << endl;
    out << endl;

    for (int i = 0; i < m_virtual_networks; i++) {
        out << "virtual_net_" << i << ": ";
        if (m_in_use[i]) {
            out << "active, ";
            if (m_ordered[i]) {
                out << "ordered" << endl;
            } else {
                out << "unordered" << endl;
            }
        } else {
            out << "inactive" << endl;
        }
    }
    out << endl;

    for(int i = 0; i < m_switch_ptr_vector.size(); i++) {
        m_switch_ptr_vector[i]->printConfig(out);
    }

    m_topology_ptr->printConfig(out);
}

void
TopazNetwork::print(ostream& out) const
{
    out << "[TopazNetwork]";
}

void TopazNetwork::increaseNumMsg(int num){
    m_number_messages+=num;
}

void TopazNetwork::increaseNumTopazMsg(int num){
    m_number_topaz_messages+=num;
    m_totalTopazMsg+=num;
}

void TopazNetwork::decreaseNumMsg(int vnet){
    m_number_messages--;
    assert(m_number_messages >= 0);
    if (m_ordered[vnet]){
        if ( numberOfOrderedMessages() > 0 ){
            assert(!(numberOfTopazOrderedMessages()>0));
            m_number_ordered_messages--;
        }
    }
}

void TopazNetwork::decreaseNumTopazMsg (int vnet){
    m_number_topaz_messages--;
    assert(m_number_messages >= 0);
    if ( m_ordered[vnet] ){
        assert(!(numberOfOrderedMessages()>0));
        m_number_topaz_ordered_messages--;
    }
}

void TopazNetwork::increaseNumOrderedMsg(int num){
    m_number_ordered_messages+=num;
}

void TopazNetwork::increaseNumTopazOrderedMsg(int num){
    m_number_topaz_ordered_messages+=num;
}

void TopazNetwork::setTopazMapping (SwitchID ext_node, SwitchID int_node) {
    int_node -= 2*m_nodes;
    MachineID machine = _nodeNumber_to_MachineID(ext_node);
    m_forward_mapping[ext_node] = int_node;
    m_reverse_mapping[int_node].add(machine);
}

TopazNetwork *
TopazNetworkParams::create()
{
    return new TopazNetwork(this);
}
