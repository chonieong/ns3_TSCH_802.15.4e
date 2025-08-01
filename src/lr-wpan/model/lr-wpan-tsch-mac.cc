/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors:
 *  Gary Pei <guangyu.pei@boeing.com>
 *  kwong yin <kwong-sang.yin@boeing.com>
 *  Tom Henderson <thomas.r.henderson@boeing.com>
 *  Sascha Alexander Jopen <jopen@cs.uni-bonn.de>
 *  Erwan Livolant <erwan.livolant@inria.fr>
 *  Luis Pacheco <luisbelem@gmail.com>
 *  Peishuo Li <pressthunder@gmail.com>
 *  Peter Kourzanov <peter.kourzanov@gmail.com>
 */

#include "lr-wpan-tsch-mac.h"
#include "lr-wpan-csmaca.h"
#include "lr-wpan-mac-trailer.h"
#include <ns3/simulator.h>
#include <ns3/log.h>
#include <ns3/uinteger.h>
#include <ns3/node.h>
#include <ns3/packet.h>
#include <ns3/random-variable-stream.h>
#include <ns3/double.h>
#include <ns3/lr-wpan-net-device.h> //change by ding
#include "lr-wpan-tsch-net-device.h" //change by ding
#include <ns3/lr-wpan-tsch-helper.h> //change by ding
#include <ns3/ping6.h> 
#include <ns3/applications-module.h> 
#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/applications-module.h>
#include "ns3/application.h"

NS_LOG_COMPONENT_DEFINE ("LrWpanTschMac");

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT                                   \
  std::clog << "[address " << m_shortAddress << "] ";

namespace ns3 {

int not_send_count=0;
int zero_queue_size=0;
bool change_size=false;
int new_size=0;
int num_node=0;
int nrnodes = -1;

int mac_packetlist[99] = {0};  // add by ieong
int mac_Totalpacket = 0;
int forset = 0;
int forset2 = 0;
int forset3 = 0;
int ping6_forset = 0;
std::vector<std::vector<int> > mac_adjacency_matrix;
//int DCFL[99][16][2] = {0};
int channellist = 15;
std::vector<std::vector<int> > consecutive_matrix(99, std::vector<int>(16, 0));
int Isconsecutive = 0;
int packetorack = 0;
int firstforconsceutive = 1;
int consecutiveongoing = 0;
int consecutive_mode = 0;  // to control consecutive or not
std::vector<Ptr<Packet> > consecutive_txPkt(20);
//Ptr<Packet> consective_txPkt = 0;
std::vector<uint32_t> consecutive_txLinkSequence(20);
//uint32_t consective_txLinkSequence = 0;
Mac16Address consecutive_addr;
uint32_t consecutive_macTxID;
uint32_t consecutive_macRxID;
//int consecutive_cancel_timeslot = 1;
int slotframe_size_mac = 0;
////
std::vector<uint16_t> consecutive_Link_erasing(20);
uint16_t consecutive_Link_erasing_temp = 0;
int consecutive_Link_erasing_number = 0;
////
uint64_t restore = -1;
uint64_t unlock = -1;
uint64_t ASN_change = -1;
std::vector<uint64_t> ping6_ASN(10);
int ping6_ASN_count = 0;
std::vector<uint64_t> ping6(10);
std::vector<int> ping6_SeqNum(50);
std::vector<Mac16Address> ping6_DstAddr(50);
std::vector<Mac16Address> ping6_SrcAddr(50);

int TimeslotLength_adaptive = 0;
int ping6_count = 0;
int ping6_total = 0;
int ping6_header = 0;
int ping6_header_round = 0;
int ping6_ChangeTimeslotLength = 0;
////

uint8_t next_packet_size = 0;
int next_packet_size_currentTimeslot = 0;
int previous_packet_size_currentTimeslot = -1;

std::vector<uint16_t> node_TimeslotLength(500,10000);

int max_pktsize = 0;
////
//changeable parameter
//uint16_t pktsize_mac = 60;
//uint16_t TimeslotLength_mac = 3664 + 1000000 * ( 9 + 6 + pktsize_mac ) * 2 /62500 + 224;
uint16_t pktsize_mac;
uint16_t TimeslotLength_mac;
//uint16_t TimeslotLength_mac = 10000; // default
//double generate_interval = 0.025;
double generate_interval;
//int tsforcheck = 0;
int current_TX;
int current_RX;
int current_Offset_list[16];
int current_Offset_totalcount = 0;
int current_Offset_count = 0;
int current_Timeslot;
int current_Offset;

int dev_count = 0;
//int pktlist[33] = {0};                                          // need change for different nrnodes
std::vector<int> pktlist(33,1);
LrWpanTschMac *dev[33];  // to count processing which device    // need change for different nrnodes
//int zero_queue_count[33] = {0};
std::vector<int> zero_queue_count(33,0);
std::vector<int> zero_shared_queue_count(33,0);
//std::vector<uint64_t> node_ASN(33,-1);
std::vector<std::vector<int> > TimeslotLengthForAllocate(33, std::vector<int>(33, 0));   // need change for different nrnodes and timeslot

//std::vector<std::vector<double> > EnqueueTime_SeqNum(33, std::vector<double>(99, 0));   // need change for different nrnodes and timeslot
std::vector<std::vector<std::vector<double> > > EnqueueTime_SeqNum(33, std::vector<std::vector<double> >(1000, std::vector<double>(2,-1)));  // need change for different nrnodes and timeslot
std::vector<int> EnqueueTime_SeqNum_count(33,0);
std::vector<int> queue(33,0);
std::vector<int> AddedLink_node(33,0);  // to count link added base on queue 

//std::vector<std::vector<std::vector<int> > > scheduling_table(99, std::vector<std::vector<int> >(15, std::vector<int>(2,0)));  // scheduling_table[timeslot][offset][TX/RX]
std::vector<std::vector<std::vector<std::vector<int> > > > scheduling_table(500, std::vector<std::vector<std::vector<int> > >(15, std::vector<std::vector<int> >(10, std::vector<int>(2,-1))));  // scheduling_table[timeslot][offset][dedicated/shared][TX/RX]
std::vector<std::vector<std::vector<std::vector<int> > > > scheduling_table_temp(500, std::vector<std::vector<std::vector<int> > >(15, std::vector<std::vector<int> >(10, std::vector<int>(2,-1))));  // scheduling_table[timeslot][offset][dedicated/shared][TX/RX]

std::vector<int> scheduling_table_link(500,0);     // scheduling_table_link[timeslot]
std::vector<int> scheduling_table_link_temp(500,0);     // scheduling_table_link[timeslot]
std::vector<std::vector<int> > scheduling_table_SharedLink(500, std::vector<int>(15, 1)); // scheduling_table_SharedLink[timeslot][offset]
std::vector<std::vector<int> > scheduling_table_SharedLink_temp(500, std::vector<int>(15, 1)); // scheduling_table_SharedLink[timeslot][offset]

bool FindInQueue = 0;
std::vector<int> link_mac;
int max_link_mac;
std::vector<std::vector<std::vector<int> > > DCFL_mac;
int TimeslotForDCFL_mac;
std::vector<std::vector<int> > DCFL_TimeslotLength_mac(500, std::vector<int>(16, 0));

int TimeslotLengthForAllocate_control = 0;
uint64_t current_ASN = 0;
//std::vector<int> totaltime(1000,0);
int totaltime_count = 0;
int usetime_node_TimeslotLength = 0;
int usetime_plan3[500] = {0};
int usetime_plan4[500] = {0};
uint16_t max = 0;
double TotalDelay = 0;
double TotalDelay_count = 0;
bool Delay_calculation = 0;

bool add_link_control = 0;
bool delete_link_control = 0;
uint16_t add_link_count = 0;
uint16_t delete_link_count = 0;
std::vector<std::vector<int> > add_link(99, std::vector<int>(4, 0));
std::vector<std::vector<int> > delete_link(99, std::vector<int>(4, 0));
bool adding_slotframe_size_mac = 0;
bool deleting_slotframe_size_mac = 0;

int AddOrDelete = 0;
int plan = 1; // ( timeslot length )  // 1 orgin // 2 uniform // 3 adaptive 
bool SLA = 1;      // SLA + UPA = ASAP
bool UPA = 1;      // SLA + UPA = ASAP
std::vector<int> UPA_queue(33,0); 
std::vector<int> UPA_addslot(33,0); 
std::vector<bool> UPA_processing(33,0); 
std::vector<Mac16Address> UPA_addr(33); 
bool w3 = 0;
bool w4 = 0;   // TATL = w4+w5+plan3
bool w5 = 0;
bool e_TSCH_Orch = 0;
bool e_TSCH_Orch_processing = 0;
//int e_TSCH_Orch_Addlink_Count = 0;
std::vector<int> e_TSCH_Orch_Addlink_Count(33,0); 
std::vector<int> e_TSCH_Orch_AddedLink(33,1); 
std::vector<int> e_TSCH_Orch_next_slotframe_queue(33,1); 
bool changing_ASN = 0;
int incasn_slotframe_size = 0;
int adaptive_slotframe_size = 0;
int adaptive_slotframe_index = 1;
bool new_slotframe = 0;  // for making ts=0, else start at ts=1
//bool directly_change = 0;
std::vector<int> directly_change_add(99,0);
std::vector<int> directly_change_delete(99,0);

int SharedLink_count = 0;
int before_SharedLink_count = 0;
int DedicatedLink_count = 0;
int before_DedicatedLink_count = 0;
std::vector<std::vector<std::vector<int> > > SharedLink_Compress(99, std::vector<std::vector<int> >(2, std::vector<int>(4,0)));

std::vector<int> PacketSeqNum(99,0);

std::vector<int> TimeslotSkipForNewAdd(99,0);
//std::vector<int> TimeslotSkipForTwoDelete(20,0);
std::vector<std::vector<int> > TimeslotSkipForTwoDelete(99, std::vector<int>(2, 0));

int TimeslotSkipForNewAdd_count = 0;
int TimeslotSkipForTwoDelete_count = 0;

ApplicationContainer Ping6apps;
uint16_t ping6_TimeslotLength = 10000;
int last_packetsize = 0;
int max_ping6_pktsize_mac = 0;
bool ping6_incasn_control = 0;

NS_OBJECT_ENSURE_REGISTERED (LrWpanTschMac);

const uint32_t LrWpanTschMac::aMinMPDUOverhead = 9; // Table 85

TypeId
LrWpanTschMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LrWpanTschMac")
    .SetParent<Object> ()
    .AddTraceSource ("MacTxEnqueue",
                     "Trace source indicating a packet has was enqueued in the transaction queue",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macTxEnqueueTrace))
    .AddTraceSource ("MacTxDequeue",
                     "Trace source indicating a packet has was dequeued from the transaction queue",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macTxDequeueTrace))
    .AddTraceSource ("MacTx",
                     "Trace source indicating a packet has arrived for transmission by this device",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macTxTrace))
    .AddTraceSource ("MacMaxRetries",
                     "Trace source indicating the maximum number of retries has been reached",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macMaxRetries))
    .AddTraceSource ("MacTxOk",
                     "Trace source indicating a packet has been successfully sent",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macTxOkTrace))
    .AddTraceSource ("MacTxDrop",
                     "Trace source indicating a packet has been dropped during transmission",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macTxDropTrace))
    .AddTraceSource ("MacPromiscRx",
                     "A packet has been received by this device, has been passed up from the physical layer "
                     "and is being forwarded up the local protocol stack.  This is a promiscuous trace,",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macPromiscRxTrace))
    .AddTraceSource ("MacRx",
                     "A packet has been received by this device, has been passed up from the physical layer "
                     "and is being forwarded up the local protocol stack.  This is a non-promiscuous trace,",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macRxTrace))
    .AddTraceSource ("MacRxDrop",
                     "Trace source indicating a packet was received, but dropped before being forwarded up the stack",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macRxDropTrace))
    .AddTraceSource ("Sniffer",
                     "Trace source simulating a non-promiscuous packet sniffer attached to the device",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_snifferTrace))
    .AddTraceSource ("PromiscSniffer",
                     "Trace source simulating a promiscuous packet sniffer attached to the device",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_promiscSnifferTrace))
    .AddTraceSource ("MacState",
                     "The state of LrWpan Mac",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macStateLogger))
    .AddTraceSource ("MacSentPkt",
                     "Trace source reporting some information about the sent packet",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_sentPktTrace))

      //Timeslot type tracing
    .AddTraceSource ("MacEmptyBuffer",
                     "Device has no packet at its buffer",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macRxEmptyBufferTrace))
    .AddTraceSource ("MacRxDataTxAck",
                     "Device receives a data packet and sends an ACK",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macRxDataTxAckTrace))
    .AddTraceSource ("MacTxData",
                     "Device sends a data packet",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macTxDataTrace))
    .AddTraceSource ("MacRxData",
                     "Device receives a data packet",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macRxDataTrace))
    .AddTraceSource ("MacTxDataRxAck",
                     "Device sends a data packet and receives an ACK",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macTxDataRxAckTrace))
    .AddTraceSource ("MacSleep",
                     "The timeslot is not assigned for the device",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macSleepTrace))
    .AddTraceSource ("MacIdle",
                     "Device listens for a packet but does not receive one",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macIdleTrace))
    .AddTraceSource ("MacChannelBusy",
                     "Device performs a CCA and the channel is busy",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macChannelBusyTrace))
    .AddTraceSource ("MacWaitAck",
                     "Device sends a data packet, listens for an ACK but does not receive one",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macWaitAckTrace))
    .AddTraceSource ("MacLinkInformation",
                     "Received power and bias power, Channel, Rx and Tx Node ID.",
                     MakeTraceSourceAccessor (&LrWpanTschMac::m_macLinkInformation))
      ;
  return tid;
}

LrWpanTschMac::LrWpanTschMac ()
{
  // First set the state to a known value, call ChangeMacState to fire trace source.
  m_lrWpanMacState = TSCH_MAC_IDLE;
  ChangeMacState (TSCH_MAC_IDLE);
  m_lrWpanMacStatePending = TSCH_MAC_IDLE;

  //LrWpanCsmaCa csmatest(); //change by ding   // ppp
  //csmatest->GetNB();  //ppp

  m_macPanId = 0;
  m_associationStatus = ASSOCIATED;
  m_selfExt = Mac64Address::Allocate ();
  m_shortAddress = Mac16Address::Allocate();
  m_macPromiscuousMode = false;
  m_macMaxFrameRetries = 5;
  //for csmaca
  m_numCsmacaRetry = 0; //change by ding
  m_retransmission = 0; //change by ding
  m_txPkt = 0;
  m_txLinkSequence = 0;

  Ptr<UniformRandomVariable> uniformVar = CreateObject<UniformRandomVariable> ();
  uniformVar->SetAttribute ("Min", DoubleValue (0.0));
  uniformVar->SetAttribute ("Max", DoubleValue (255.0));
  m_macDsn = SequenceNumber8 (uniformVar->GetValue ());

  m_macCCAEnabled = true;
  m_macHoppingEnabled = true;
  m_sharedLink = false;
  m_emptySlot = true;
  m_newSlot = true;
  m_random = CreateObject<UniformRandomVariable> ();

  
  
  ResetMacTschPibAttributes();
  ResetMacTimeslotTemplate();
  // can change the value to limit the number of frequency band ( channel) //ieong
  //SetDefaultHoppingSequence(16);
  SetDefaultHoppingSequence(channellist); //change to 15 since channel 11 have error

  ping6_ASN[0] = -1; // add by ieong
  node_TimeslotLength[0] = 10000; // use default for ts = 0 // since no packet traffic for ts =0
  std::cout<< " Plan : " << plan << std::endl;
  std::cout<< " w5 : " << w5 << std::endl;

  //AddedLink_node[1] = 1; // need cancel
}

LrWpanTschMac::~LrWpanTschMac ()
{
}

void
LrWpanTschMac::DoInitialize ()
{
  m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);
  Object::DoInitialize ();
}

void
LrWpanTschMac::DoDispose ()
{
  if (m_csmaCa != 0)//retransmi //change by ding
    {
      m_csmaCa->Dispose ();
      m_csmaCa = 0;
    }
  m_txPkt = 0;
  m_txLinkSequence = 0;

  for (uint32_t i = 0; i < m_txQueueAllLink.size (); i++)
    {
      for (uint32_t j = 0; j < m_txQueueAllLink[i]->txQueuePerLink.size(); j++)
        {
          m_txQueueAllLink[i]->txQueuePerLink[j]->txQPkt = 0;
          delete m_txQueueAllLink[i]->txQueuePerLink[j];
        }
      m_txQueueAllLink[i]->txQueuePerLink.clear();
      delete m_txQueueAllLink[i];
    }
  m_txQueueAllLink.clear ();

  m_phy = 0;
  m_mcpsDataIndicationCallback = MakeNullCallback< void, McpsDataIndicationParams, Ptr<Packet> > ();
  m_mcpsDataConfirmCallback = MakeNullCallback< void, McpsDataConfirmParams > ();

  m_mlmeSetSlotframeConfirmCallback = MakeNullCallback< void, MlmeSetSlotframeConfirmParams > ();
  m_mlmeTschModeConfirmCallback = MakeNullCallback< void, MlmeTschModeConfirmParams > ();
  m_mlmeSetLinkConfirmCallback = MakeNullCallback< void, MlmeSetLinkConfirmParams > ();

  Object::DoDispose ();
}

void
LrWpanTschMac::SetShortAddress (Mac16Address address)
{
  //NS_LOG_FUNCTION (this << address);
  m_shortAddress = address;
}

void
LrWpanTschMac::SetExtendedAddress (Mac64Address address)
{
  //NS_LOG_FUNCTION (this << address);
  m_selfExt = address;
}

Mac16Address
LrWpanTschMac::GetShortAddress () const
{
  NS_LOG_FUNCTION (this);
  return m_shortAddress;
}

Mac64Address
LrWpanTschMac::GetExtendedAddress () const
{
  NS_LOG_FUNCTION (this);
  return m_selfExt;
}

//maybe can change here by ding
void
LrWpanTschMac::McpsDataRequest (TschMcpsDataRequestParams params, Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this << p);
  Ptr<Packet> temp_packet = p;

  /*if( double(Now().GetSeconds()) >= ( 5+generate_interval) && double(Now().GetSeconds()) < ( 5+generate_interval+0.01)){

    for( int i=1; i<dev_count; i++){
      if( dev[i] == this ){
        pktlist[i]++;
        std::cout<< " Adding pktlist of i : " << i << std::endl;
        break;
      }
    }
  }*/

  // for break the packet generate from ping6 app. It should not generate from here // ieong
  Ptr<Ping6> ping6App = DynamicCast<Ping6>(Ping6apps.Get(0));
  //std::cout<< " ping6 packet size : " << int(ping6App->GetSize()) << std::endl;
  int ping6_interval = 0.1 * (dev_count-1) * 10;
  if(ping6_interval <= 0)
    ping6_interval = 1;
  

  //if (double(Now().GetSeconds()) == int(Now().GetSeconds()) && int(p->GetSize()) == int(ping6App->GetSize()) + 43)
  if (int(float(Now().GetSeconds()*10)) % ping6_interval == 0 && int(float(Now().GetSeconds()*100))%10 == 0 && int(p->GetSize()) == int(ping6App->GetSize()) + 43)
  {
    std::cout << " breaking the ping6 packet "<< int(p->GetSize()) << std::endl;
    return;
  }
  else{
    last_packetsize = int(p->GetSize());
  }


  if(w5){
    NS_LOG_DEBUG("Making packet size : " << p->GetSize() << " in ASN " << m_macTschPIBAttributes.m_macASN 
            << " and slotframe " << m_macTschPIBAttributes.m_macASN/adaptive_slotframe_size);
  }
  else{
    NS_LOG_DEBUG("Making packet size : " << p->GetSize() << " in ASN " << m_macTschPIBAttributes.m_macASN 
            << " and slotframe " << m_macTschPIBAttributes.m_macASN/slotframe_size_mac);
  }

  if (SLA)
  {
    if (int(p->GetSize()) > max_ping6_pktsize_mac)
    { // 74+43 = 117        // 69+48 = 117
      max_ping6_pktsize_mac = int(p->GetSize());
      for (int i = 0; i <= nrnodes; i++)
      {
        Ptr<Ping6> ping6App1 = DynamicCast<Ping6>(Ping6apps.Get(i));
        ping6App1->SetSize(max_ping6_pktsize_mac);
        if (i == 0)
          std::cout << "change from random packet to ping6App : " << ping6App1->GetSize() << std::endl;
      }
    }
  }

  
  //std::cout << " m_macDsn.GetValue () : " << int(m_macDsn.GetValue ()) << std::endl;

  /*
  if( int(p->GetSize()) > pktsize_mac+10 && forset2 == 0){
    double time = double(Now().GetSeconds()) - 5;
    std::cout << " time : " << time << std::endl;
    
    calculate_ping6_ASN(time);
    forset2 = 1;

  }*/

  McpsDataConfirmParams confirmParams;
  confirmParams.m_msduHandle = params.m_msduHandle;

  // TODO: We need a drop trace for the case that the packet is too large or the request parameters are maleformed.
  //       The current tx drop trace is not suitable, because packets dropped using this trace carry the mac header
  //       and footer, while packets being dropped here do not have them.

  LrWpanMacHeader macHdr (LrWpanMacHeader::LRWPAN_MAC_DATA, m_macDsn.GetValue ());   // add 3 bytes here // ieong
  m_macDsn++;

  //macHdr.SetCountPing6();    // for count ping6


/*
  std::cout << "macHdr.GetSrcPanId() : " << macHdr.GetSrcPanId() << std::endl;
  std::cout << "macHdr.GetDstPanId() : " << macHdr.GetDstPanId() << std::endl;
  std::cout << "macHdr.GetKeyIdSrc64() : " << macHdr.GetKeyIdSrc64() << std::endl;
  std::cout << "macHdr.GetKeyIdSrc32() : " << macHdr.GetKeyIdSrc32() << std::endl;
  std::cout << "macHdr.GetShortSrcAddr() : " << macHdr.GetShortSrcAddr() << std::endl;
  std::cout << "macHdr.GetExtSrcAddr() : " << macHdr.GetExtSrcAddr() << std::endl;
  std::cout << "macHdr.GetShortDstAddr() : " << macHdr.GetShortDstAddr() << std::endl;
  std::cout << "macHdr.GetExtDstAddr() : " << macHdr.GetExtDstAddr() << std::endl;
*/
  if (p->GetSize () > LrWpanPhy::aMaxPhyPacketSize - aMinMPDUOverhead) // ieong // aMaxMACPayloadSize = aMaxPhyPacketSize - aMinMPDUOverhead // table 85
    {
      // Note, this is just testing maximum theoretical frame size per the spec
      // The frame could still be too large once headers are put on
      // in which case the phy will reject it instead
      NS_LOG_ERROR (this << " packet too big: " << p->GetSize ());
      confirmParams.m_status = IEEE_802_15_4_FRAME_TOO_LONG;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }


  macHdr.SetFrameVer(2);
  if ((params.m_srcAddrMode == NO_PANID_ADDR)
      && (params.m_dstAddrMode == NO_PANID_ADDR))
    {
      NS_LOG_ERROR (this << " Can not send packet with no Address field" );
      confirmParams.m_status = IEEE_802_15_4_INVALID_ADDRESS;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }

  macHdr.SetNoPanIdComp ();

  switch (params.m_srcAddrMode)
    {
    case NO_PANID_ADDR:
      macHdr.SetSrcAddrMode (params.m_srcAddrMode);
      break;
    case ADDR_MODE_RESERVED:
      macHdr.SetSrcAddrMode (params.m_srcAddrMode);
      break;
    case SHORT_ADDR:
      macHdr.SetSrcAddrMode (params.m_srcAddrMode);
      macHdr.SetSrcAddrFields (GetPanId (), GetShortAddress ());
      break;
    case EXT_ADDR:
      macHdr.SetSrcAddrMode (params.m_srcAddrMode);
      macHdr.SetSrcAddrFields (GetPanId (), GetExtendedAddress ());
      break;
    default:
      NS_LOG_ERROR (this << " Can not send packet with incorrect Source Address mode = " << params.m_srcAddrMode);
      confirmParams.m_status = IEEE_802_15_4_INVALID_ADDRESS;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }

  if (params.m_SecurityLevel == 0)
    {
      macHdr.SetSecDisable ();
    }
  else
    {
      //TODO
      //add some security level by ding
    }

  if (params.m_frameControlOptions.m_PanIdSupressed)
    {
      macHdr.SetPanIdComp();
    }
  else
    {
      macHdr.SetNoPanIdComp();
    }

  if (params.m_frameControlOptions.IesIncluded)
    {
      macHdr.SetIEField();
      //TODO: Insert IEs
    }
  else 
    {
      macHdr.SetNoIEField();
    }

  if (params.m_frameControlOptions.SeqNSupressed)
    {
      macHdr.SetSeqNumSup();
    }
  else
    {
      macHdr.SetNoSeqNumSup();
      macHdr.SetSeqNum(m_macDsn.GetValue());
    }

  if (params.m_sendMultipurpose)
    {
      //TODO
    }
  else if (params.m_frameControlOptions.SeqNSupressed || params.m_frameControlOptions.IesIncluded)
    {
      //macHdr.SetFrameVer(2);
    }
  else
    {
      //macHdr.SetFrameVer(1);
    }

  if (params.m_ACK_TX)
    {
      macHdr.SetAckReq ();
      //std::cout << " need ack " << std::endl;
    }
  else
    {
      macHdr.SetNoAckReq ();    // broadcast
      //std::cout << " no need ack " << std::endl;

    }

  if (params.m_GTSTX)
    {
      //TODO
    }
  else if (params.m_IndirectTx)
    {
      //TODO: indirect tx, overrrided by gts
    }

  if ((macHdr.GetFrameVer() == 1 || macHdr.GetFrameVer() == 0) && params.m_dstAddrMode == 0)
    {
      //TODO: frame directed to PAN coordinator, with the the pan id as src pan id
      //maybe can change here by ding
    }

  if ((macHdr.GetFrameVer() == 1 || macHdr.GetFrameVer() == 0) && params.m_srcAddrMode == 0)
    {
      //TODO: frame originated from PAN coordinator, with the the pan id as dst pan id
      //maybe can change here by ding 
    }

  if (macHdr.GetFrameVer() == 2)
    {
      //TODO: can be broadcast
      //maybe can change here by ding
    }


  macHdr.SetDstAddrMode (params.m_dstAddrMode);   // add 4 bytes here // ieong
  // TODO: Add field for EXT_ADDR destination address (and use it here).
  //maybe can change here by ding



  macHdr.SetDstAddrFields (params.m_dstPanId, params.m_dstAddr);
  macHdr.SetSecDisable ();

  p->AddHeader (macHdr); // 7 bytes
  //std::cout << "after add header p : " << int(p->GetSize()) << std::endl;


  LrWpanMacTrailer macTrailer;
  // Calculate FCS if the global attribute ChecksumEnable is set.
  if (Node::ChecksumEnabled ())
    {
      macTrailer.EnableFcs (true);
      macTrailer.SetFcs (p);
    }
  p->AddTrailer (macTrailer);  // 2 bytes
  m_macTxEnqueueTrace (p);

  TxQueueRequestElement *txQElement = new TxQueueRequestElement;
  txQElement->txQMsduHandle = params.m_msduHandle;
  txQElement->txQPkt = p;
  txQElement->txRequestNB = 0;
  txQElement->txRequestCW = 0;
  //txQElement->txRequestCW = (uint8_t)m_random->GetInteger (0, 4);;

  Mac16Address dstAddr = macHdr.GetShortDstAddr ();
///////////////////////
if(Delay_calculation){
  for( int i=1; i<dev_count; i++){
    if( dev[i] == this ){
      EnqueueTime_SeqNum[i][EnqueueTime_SeqNum_count[i]][0] = double(Now().GetSeconds());
      EnqueueTime_SeqNum[i][EnqueueTime_SeqNum_count[i]][1] = double(macHdr.GetSeqNum());
      EnqueueTime_SeqNum_count[i]++;
      //std:: cout << " EnqueueTime_SeqNum_count[i] : " << EnqueueTime_SeqNum_count[i] << " i : " << i << std::endl;
    }
  }
}
  
//////////////////////

  bool flag_findLinkQueue = false;
  if (m_txQueueAllLink.size () == 0){
    flag_findLinkQueue = false;
    }
  else{
      for (uint32_t i = 0; i < m_txQueueAllLink.size (); i++) {
          if (m_txQueueAllLink[i]->txDstAddr == dstAddr){

              m_txQueueAllLink[i]->txQueuePerLink.push_back(txQElement);
              NS_LOG_DEBUG("Enqueuing packet with SeqNum = " << (int)macHdr.GetSeqNum()
                           << " in existed link queue with link sequence = " << i
                           << " and queue size = " << m_txQueueAllLink[i]->txQueuePerLink.size());
                          
              /*
              //std::cout << "p : " << int(p->GetSize()) << std::endl;;
              //std::cout << "pktsize_mac : " << pktsize_mac << std::endl;
              if( int(p->GetSize()) > pktsize_mac+10  && forset3 == 0){
                ping6_SeqNum[ping6_total] = (int)macHdr.GetSeqNum();
                ping6_DstAddr[ping6_total] = macHdr.GetShortDstAddr();
                ping6_SrcAddr[ping6_total] = macHdr.GetShortSrcAddr();
                ping6_total++;
                forset3 = 1;

                //std::cout<< " macHdr.GetExtDstAddr() : " << macHdr.GetExtDstAddr() << std::endl;
                //std::cout<< " macHdr.GetExtSrcAddr() : " << macHdr.GetExtSrcAddr() << std::endl;
                //std::cout<< " macHdr.GetShortDstAddr() : " << macHdr.GetShortDstAddr() << std::endl;
                //std::cout<< " macHdr.GetShortSrcAddr() : " << macHdr.GetShortSrcAddr() << std::endl;

                    
                std::cout<< " Adding ping6 packet : " << std::endl;
                for ( int j=0; j<ping6_total; j++){
                  std::cout << " ping6_SeqNum : " << ping6_SeqNum[j] 
                            << " ping6_DstAddr : " << ping6_DstAddr[j]
                            << " ping6_SrcAddr : " << ping6_SrcAddr[j]
                            << std::endl;
                  
                }

              }*/
              flag_findLinkQueue = true;
              break;
            }
          }
       }
  if (!flag_findLinkQueue)
     SetTxLinkQueue (txQElement, dstAddr, macHdr.GetSeqNum());

  /*
  NS_LOG_DEBUG("Printing Queue link");
  Ptr<Packet> QueuePacket = Create<Packet>(0);
  for (std::deque<TxQueueLinkElement *>::iterator i = m_txQueueAllLink.begin(); i != m_txQueueAllLink.end(); i++)
  {
    QueuePacket = (*i)->txQueuePerLink.front()->txQPkt->Copy();
    std::cout << "In Find TX TxPacket : " << QueuePacket->GetSize();
    LrWpanMacHeader macHdr;
    QueuePacket->PeekHeader (macHdr);
    std::cout << " with seqnum = " << (int)macHdr.GetSeqNum() << std::endl;
    
  }
  std::cout<<std::endl;*/

}

void
LrWpanTschMac::SetTxLinkQueue (TxQueueRequestElement * newRequestElement, Mac16Address newDstAddr, uint8_t newSeqNum)
{

  NS_LOG_FUNCTION (this);

  TxQueueLinkElement *txQueueLinkElement = new TxQueueLinkElement;

  txQueueLinkElement->txQueuePerLink.push_back (newRequestElement);

  NS_LOG_DEBUG("Enqueuing packet with SeqNum = " << (int)newSeqNum
               << " in queue with link sequence = " << m_txQueueAllLink.size());

  txQueueLinkElement->txDstAddr = newDstAddr;
  txQueueLinkElement->txLinkBE = m_macTschPIBAttributes.macMinBE;

  m_txQueueAllLink.push_back (txQueueLinkElement);
}

void //change by ding
LrWpanTschMac::SetCsmaCa (Ptr<LrWpanCsmaCa> csmaCa)
{
  m_csmaCa = csmaCa;
}

void
LrWpanTschMac::SetPhy (Ptr<LrWpanPhy> phy)
{
  m_phy = phy;
}

Ptr<LrWpanPhy>
LrWpanTschMac::GetPhy (void)
{
  return m_phy;
}

void
LrWpanTschMac::SetMcpsDataIndicationCallback (McpsDataIndicationCallback c)
{
  m_mcpsDataIndicationCallback = c;
}

void
LrWpanTschMac::SetMcpsDataConfirmCallback (McpsDataConfirmCallback c)
{
  m_mcpsDataConfirmCallback = c;
}

void 
LrWpanTschMac::SetMlmeSetSlotframeConfirmCallback (MlmeSetSlotframeConfirmCallback c)
{
  m_mlmeSetSlotframeConfirmCallback = c;
}
void 
LrWpanTschMac::SetMlmeTschModeConfirmCallback (MlmeTschModeConfirmCallback c)
{
  m_mlmeTschModeConfirmCallback = c;
}
void 
LrWpanTschMac::SetMlmeSetLinkConfirmCallback (MlmeSetLinkConfirmCallback c)
{
  m_mlmeSetLinkConfirmCallback = c;
}

/*
void 
SetMlmeKeepAliveConfirmCallback (MlmeKeepAliveConfirmCallback c)
{
  m_mlmeKeepAliveConfirmCallback = c;
}*/

void
LrWpanTschMac::PdDataIndication (uint32_t psduLength, Ptr<Packet> p, uint8_t lqi)
{
  NS_ASSERT (m_lrWpanMacState == TSCH_MAC_ACK_PENDING  || TSCH_MAC_ACK_PENDING_END || TSCH_MAC_RX || TSCH_PKT_WAIT_END);

  NS_LOG_FUNCTION (this << psduLength << p << (int)lqi);

  bool acceptFrame;

  // from sec 7.5.6.2 Reception and rejection, Std802.15.4-2006
  // level 1 filtering, test FCS field and reject if frame fails
  // level 2 filtering if promiscuous mode pass frame to higher layer otherwise perform level 3 filtering
  // level 3 filtering accept frame
  // if Frame type and version is not reserved, and
  // if there is a dstPanId then dstPanId=m_macPanId or broadcastPanI, and
  // if there is a shortDstAddr then shortDstAddr =shortMacAddr or broadcastAddr, and
  // if beacon frame then srcPanId = m_macPanId
  // if only srcAddr field in Data or Command frame,accept frame if srcPanId=m_macPanId

  Ptr<Packet> originalPkt = p->Copy (); // because we will strip headers

  m_promiscSnifferTrace (originalPkt);

  m_macPromiscRxTrace (originalPkt);
  // XXX no rejection tracing (to macRxDropTrace) being performed below

  LrWpanMacTrailer receivedMacTrailer;
  p->RemoveTrailer (receivedMacTrailer);
  if (Node::ChecksumEnabled ())
    {
      receivedMacTrailer.EnableFcs (true);
    }

  // level 1 filtering
  if (!receivedMacTrailer.CheckFcs (p))
    {
      m_macRxDropTrace (originalPkt);
      NS_LOG_DEBUG("FCS check fail");
    }
  else
    {
      LrWpanMacHeader receivedMacHdr;
      p->RemoveHeader (receivedMacHdr);

      McpsDataIndicationParams params;        
      if (receivedMacHdr.IsSeqNumSup())
          {
            params.m_dsn = 0;
          }
        else
          {
            params.m_dsn = receivedMacHdr.GetSeqNum ();
          }
      params.m_mpduLinkQuality = lqi;
      params.m_srcPanId = receivedMacHdr.GetSrcPanId ();
      params.m_srcAddrMode = receivedMacHdr.GetSrcAddrMode ();
      // TODO: Add field for EXT_ADDR source address.
      if (params.m_srcAddrMode == SHORT_ADDR)
        {
          params.m_srcAddr = receivedMacHdr.GetShortSrcAddr ();
        }
      params.m_dstPanId = receivedMacHdr.GetDstPanId ();
      params.m_dstAddrMode = receivedMacHdr.GetDstAddrMode ();
      // TODO: Add field for EXT_ADDR destination address.
      if (params.m_dstAddrMode == SHORT_ADDR)
        {
          params.m_dstAddr = receivedMacHdr.GetShortDstAddr ();
        }

      NS_LOG_DEBUG ("Packet from " << params.m_srcAddr << " to " << params.m_dstAddr << " with seqnum = " << (int)receivedMacHdr.GetSeqNum());


      if (m_macPromiscuousMode)
        {
          //level 2 filtering
          if (!m_mcpsDataIndicationCallback.IsNull ())
            {
              NS_LOG_DEBUG ("promiscuous mode, forwarding up");
              m_mcpsDataIndicationCallback (params, p);
            }
          else
            {
              NS_LOG_ERROR (this << " Data Indication Callback not initialised");
            }
        }
      else
        {
          //level 3 frame filtering
          acceptFrame = (receivedMacHdr.GetType () != LrWpanMacHeader::LRWPAN_MAC_RESERVED);
          if (acceptFrame)
            {
              acceptFrame = (receivedMacHdr.GetFrameVer () == 2);
            }

            if (acceptFrame && receivedMacHdr.GetFrameVer () == 2 &&
                (
                  (receivedMacHdr.GetDstAddrMode() == 0 && receivedMacHdr.GetSrcAddrMode() == 0 && receivedMacHdr.IsPanIdComp()) ||
                  (receivedMacHdr.GetDstAddrMode() > 0 && receivedMacHdr.GetSrcAddrMode() == 0 && !receivedMacHdr.IsPanIdComp()) ||
                  (receivedMacHdr.GetDstAddrMode() > 0 && receivedMacHdr.GetSrcAddrMode() > 0 && !receivedMacHdr.IsPanIdComp())
                ))
              {
                acceptFrame = receivedMacHdr.GetDstPanId () == m_macPanId
                || receivedMacHdr.GetDstPanId () == 0xffff;
              }


          if (acceptFrame
              && (receivedMacHdr.GetDstAddrMode () == 2))
            {
              acceptFrame = receivedMacHdr.GetShortDstAddr () == m_shortAddress
                || receivedMacHdr.GetShortDstAddr () == Mac16Address ("ff:ff");      // check for broadcast addrs
            }

          if (acceptFrame
              && (receivedMacHdr.GetDstAddrMode () == 3))
            {
              acceptFrame = (receivedMacHdr.GetExtDstAddr () == m_selfExt);
            }


          if (acceptFrame
              && (receivedMacHdr.GetType () == LrWpanMacHeader::LRWPAN_MAC_BEACON))
            {
              if (m_macPanId == 0xffff)
                {
                  acceptFrame = true;
                }
              else
                {
                  acceptFrame = receivedMacHdr.GetSrcPanId () == m_macPanId;NS_LOG_DEBUG (acceptFrame << "-5");
                }
            }

          if (acceptFrame)
            {
              //if(receivedMacHdr.IsData ())   // ieong // only count if it is data for totalrx
                m_macRxTrace (originalPkt);
              
              if (receivedMacHdr.IsAcknowledgment () && (m_lrWpanMacState == TSCH_MAC_ACK_PENDING || m_lrWpanMacState == TSCH_MAC_ACK_PENDING_END))
                {
              
                  LrWpanMacHeader macHdr;
                  m_txPkt->PeekHeader (macHdr);

                  m_macTxDataRxAckTrace(m_latestPacketSize);

                  // receivedMacHdr.IsSeqNumSup() mean no sequence number for =1
                  if (receivedMacHdr.IsSeqNumSup() || (receivedMacHdr.GetSeqNum () == macHdr.GetSeqNum ()))   
                    {
                      m_macTxOkTrace (m_txPkt);
                      // If it is an ACK with the expected sequence number, finish the transmission
                      // and notify the upper layer.
                      if (!m_mcpsDataConfirmCallback.IsNull ())
                        {
                          TxQueueRequestElement *txQElement = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front ();
                          McpsDataConfirmParams confirmParams;
                          confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                          confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                          m_mcpsDataConfirmCallback (confirmParams);
                        }
                      RemoveTxQueueElement ();

                      



                      //std::cout << "receivedMacHdr.GetSeqNum () : " << (int)receivedMacHdr.GetSeqNum () << std::endl;
                      //std::cout << "macHdr.GetSeqNum () : " << (int)macHdr.GetSeqNum () << std::endl;

                      NS_LOG_DEBUG ("ACK successfully received " << (int)receivedMacHdr.GetSeqNum ());
                      


                      // add by ieong // for consectuive of TX
                      if( Isconsecutive == 0 && !UPA){   // consecutive is done here   // orgin
                        m_setMacState.Cancel ();
                        m_setMacState = Simulator::ScheduleNow (&LrWpanTschMac::SetLrWpanMacState, this, TSCH_MAC_IDLE);
                        consecutiveongoing = 0;
                      }
                      else if( Isconsecutive != 0 ) //since consective is not zero, keep process data 
                      {
                        std::cout << " In Tx, Isconsecutive =  " << Isconsecutive << std::endl;
                        firstforconsceutive = 0;
                        Isconsecutive--;
                        std::cout << " change consecutive : " << Isconsecutive << std::endl;

                        ChangeMacState(TSCH_MAC_IDLE);

                        m_emptySlot = true;
                        m_txPkt = FindTxPacketInEmptySlot(consecutive_addr);
                        std::cout << "consecutive Get m_txPkt : " << m_txPkt->GetSize() << std::endl;
                        
                        Time time2wait = MicroSeconds(def_MacTimeslotTemplate.m_macTsRxTx);
                        Simulator::Schedule (time2wait,&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_SENDING); // change mac to sending from idle for TX
                        m_lrWpanMacStatePending = TSCH_MAC_SENDING;
                        Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);

                        std::cout << " in process tx " << std::endl;

                        return;

                      }
                      if (UPA)
                      {
                        for (int i = 0; i < dev_count; i++)
                        {
                          if (this == dev[i])
                          {
                            if (UPA_processing[i])
                            {
                              if (UPA_queue[i] != 0)
                              {

                                ChangeMacState(TSCH_MAC_IDLE);
                                m_emptySlot = true;
                                current_TX = i;
                                m_txPkt = FindTxPacketInEmptySlot(UPA_addr[i]);
                                std::cout << "UPA Get m_txPkt : " << m_txPkt->GetSize() << std::endl;
                                Time time2wait = MicroSeconds(def_MacTimeslotTemplate.m_macTsRxTx);
                                Simulator::Schedule(time2wait, &LrWpanTschMac::SetLrWpanMacState, this, TSCH_MAC_SENDING); // change mac to sending from idle for TX
                                m_lrWpanMacStatePending = TSCH_MAC_SENDING;
                                Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState, this, TSCH_MAC_IDLE);
                                std::cout << " In Tx of UPA for node " << i << " and queue " << UPA_queue[i] << std::endl;
                                UPA_queue[i]--;
                                return;
                              }
                              else
                              { // done UPA
                                std::cout << " UPA is done " << std::endl;
                                UPA_processing[i] = 0;
                                m_setMacState.Cancel();
                                m_setMacState = Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState, this, TSCH_MAC_IDLE);
                              }
                            }
                          }
                        }
                      }

                      //TODO: check if it is a nack
                    }
                  else
                    {

                      //std::cout << "receivedMacHdr.GetSeqNum () : " << (int)receivedMacHdr.GetSeqNum () << std::endl;
                      //std::cout << "macHdr.GetSeqNum () : " << (int)macHdr.GetSeqNum () << std::endl;


                      NS_LOG_DEBUG ("ACK received with wrong seq num" << m_selfExt);
                      HandleTxFailure ();
                    }

                    if (m_lrWpanMacState == TSCH_MAC_ACK_PENDING_END)
                      {

                        //NS_LOG_DEBUG ("come1");
                        ChangeMacState(TSCH_MAC_IDLE);
                      }
                    else
                      {
                        //NS_LOG_DEBUG ("come2");
                        Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);
                      }
                }
              else if (receivedMacHdr.IsData () && !m_mcpsDataIndicationCallback.IsNull() && (m_lrWpanMacState == TSCH_MAC_RX || m_lrWpanMacState == TSCH_PKT_WAIT_END))
                {
                  if(w5){
                    current_Timeslot = m_macTschPIBAttributes.m_macASN % adaptive_slotframe_size;
                  }
                  else{
                    current_Timeslot = m_macTschPIBAttributes.m_macASN % slotframe_size_mac;
                  }
                  std::cout << " current_Timeslot : " << current_Timeslot << std::endl;
                  
                  //////////////////
                  // for end to end delay
                  if(Delay_calculation){
                    double time = 0;
                    for ( int i=1; i<dev_count; i++){
                      for( int j=0; j<EnqueueTime_SeqNum_count[i]; j++){
                        if( EnqueueTime_SeqNum[i][j][1] != -1 ){
                          if( EnqueueTime_SeqNum[i][j][1] == double(receivedMacHdr.GetSeqNum()) ){
                            time = EnqueueTime_SeqNum[i][j][0];
                            EnqueueTime_SeqNum[i][j][0] = -1;
                            EnqueueTime_SeqNum[i][j][1] = -1;
                            NS_LOG_DEBUG( " for packet at node " << i);
                            break;
                          }
                          else{
                            break;
                          }
                        }
                      
                      }
                      if( time != 0 )
                        break;
                    }
                    if( time == 0){
                      //NS_FATAL_ERROR(" Error ");
                    }
                    else{
                      NS_LOG_DEBUG( " current time = " << double(Now().GetSeconds()) << 
                                  " packet time = " << time << " Delay = " << double(Now().GetSeconds()) - time );

                      time = double(Now().GetSeconds()) - time;
                      TotalDelay+=time;
                      TotalDelay_count++;
                      NS_LOG_DEBUG( " TotalDelay = " << TotalDelay << " TotalDelay_count = " << TotalDelay_count << " Avg Delay = " << TotalDelay/TotalDelay_count );

                    }
                  
                  }
                  

                  /////////////////
                  
                  std::cout<< " receivedMacHdr.GetNextPacket() : " << int(receivedMacHdr.GetNextPacket()) << std::endl;

                  if(w4){

                    if( receivedMacHdr.GetNextPacket() != 0){
                      if(receivedMacHdr.GetNextPacket() > 128 ){
                        receivedMacHdr.SetNextPacket(receivedMacHdr.GetNextPacket() - uint8_t(128));
                        std::cout<< " Confirme first bit is 1 in receive, process add. packet size is " << int(receivedMacHdr.GetNextPacket()) << std::endl;
                        AddOrDelete = 1;
                      }
                      else{
                        std::cout<< " Confirme first bit is 0 in receive, process keep. packet size is " << int(receivedMacHdr.GetNextPacket()) << std::endl;
                        AddOrDelete = 0;
                      }
                    }
                    else{
                      if( AddedLink_node[current_TX] >= 1 )
                      {
                        std::cout << " header is 0 in receive. process delete " << std::endl;
                        AddOrDelete = 2;
                      }
                      else{
                        AddOrDelete = 0;
                        std::cout << " Not find next packet. Current state is "<< AddedLink_node[current_TX] << " and shared link. Do nothing. " << std::endl;
                      }
                    }

                  }
                  
                  /////////// change slotframe size
                  if( this == dev[0] && AddOrDelete == 1 && ( w3||w4)){
                    
                    for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){
                      if( (int)receivedMacHdr.GetSeqNum() == PacketSeqNum[int(i->macTxID)] && int(i->macTimeslot) == current_Timeslot){
                        current_TX = int(i->macTxID); 
                        break;
                      }
                    }

                    for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++)
                    {
                      if( int(i->macTimeslot) == current_Timeslot && int(i->macTxID) == current_TX)
                      {

                        // to prevent add 2 times in 1 period
                        for (int m = 0; m < add_link_count; m++){
                          if( add_link[m][2] == int(i->macTxID) && add_link[m][3] == int(i->macRxID) ){
                            std::cout << " Cannot add 2 times in 1 period for link " << int(i->macTxID) << "->" << int(i->macRxID) << std::endl;
                            break;
                          }
                        }

                        add_link[add_link_count][0] = int(i->macTimeslot);
                        add_link[add_link_count][1] = int(i->macChannelOffset);
                        add_link[add_link_count][2] = int(i->macTxID);
                        add_link[add_link_count][3] = int(i->macRxID);
                        add_link_control = 1;
                        add_link_count++;
                        NS_LOG_DEBUG("Will add Link " << int(i->macTxID) << "->" << int(i->macRxID) <<
                                     " for Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) << " to next Timeslot" );
                        NS_LOG_DEBUG("Number of add link is " << add_link_count );
                        break;
                      }
                   }

                  }
                  else if( this == dev[0] && AddOrDelete == 2 && (w3||w4)){
                    for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++)
                    {

                      if( int(i->macTimeslot) == current_Timeslot && int(i->macTxID) == current_TX)
                      {
                        bool b=0;
                        // to prevent add 2 times in 1 period
                        for (int m = 0; m < delete_link_count; m++)
                        {
                          if (delete_link[m][2] == int(i->macTxID) && delete_link[m][3] == int(i->macRxID))
                          {
                            std::cout << " Cannot delete 2 times in 1 period for link " << int(i->macTxID) << "->" << int(i->macRxID) << std::endl;
                            TimeslotSkipForTwoDelete[TimeslotSkipForTwoDelete_count][0] = int(i->macTxID);
                            TimeslotSkipForTwoDelete[TimeslotSkipForTwoDelete_count][1] = int(i->macRxID);
                            TimeslotSkipForTwoDelete_count++;
                            b = 1;
                            break;
                          }
                        }

                        if( b != 1 ){
                          delete_link[delete_link_count][0] = int(i->macTimeslot);
                          delete_link[delete_link_count][1] = int(i->macChannelOffset);
                          delete_link[delete_link_count][2] = int(i->macTxID);
                          delete_link[delete_link_count][3] = int(i->macRxID);
                          delete_link_control = 1;
                          delete_link_count++;
                          NS_LOG_DEBUG("Will delete Link " << int(i->macTxID) << "->" << int(i->macRxID) << 
                                       " for Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) );
                          NS_LOG_DEBUG("Number of delete link is " << delete_link_count );
                          break;
                        }
                        
                      }
                    }
                  }
                  AddOrDelete = 0;

                  ///////////









                  
                  if( receivedMacHdr.GetNextPacket() != 0){

                    next_packet_size_currentTimeslot = current_Timeslot;

                    if (previous_packet_size_currentTimeslot != next_packet_size_currentTimeslot)
                    {

                      if (AddedLink_node[current_TX] != 0)
                      {

                        TimeslotLength_mac = 3664 + (1000000 * (10 + 6 + int(receivedMacHdr.GetNextPacket())) * 2 / 62500) + 300;
                        node_TimeslotLength[next_packet_size_currentTimeslot] = TimeslotLength_mac;

                        // std::cout << " current_Offset_list[current_Offset_count] : " << current_Offset_list[current_Offset_count]
                        //           << " and current_Offset_count : " << current_Offset_count << std::endl;
                        // DCFL_TimeslotLength_mac[current_Timeslot][current_Offset_list[current_Offset_count]] = TimeslotLength_mac;
                        DCFL_TimeslotLength_mac[current_Timeslot][current_Offset] = TimeslotLength_mac;
                        NS_LOG_DEBUG(" Will change at next slotframe for Timeslot Length : " << TimeslotLength_mac
                                                                                                << " at timeslot " << next_packet_size_currentTimeslot);
                      }
                    
                    }
                    else{
                      TimeslotLength_mac = 3664 + (1000000 * ( 10 + 6 + int(receivedMacHdr.GetNextPacket())) *2 /62500) + 300;
                      if( node_TimeslotLength[next_packet_size_currentTimeslot] < TimeslotLength_mac ){
                        //std::cout << " current_Offset_list[current_Offset_count] : " << current_Offset_list[current_Offset_count] 
                        //          << " and current_Offset_count : " << current_Offset_count << std::endl;                        
                        NS_LOG_DEBUG(" Changing biggest one for TimeslotLength_mac : " << TimeslotLength_mac << " from " << node_TimeslotLength[next_packet_size_currentTimeslot]
                                     << " at timeslot " << next_packet_size_currentTimeslot );
                        node_TimeslotLength[next_packet_size_currentTimeslot] = TimeslotLength_mac;
                        //DCFL_TimeslotLength_mac[current_Timeslot][current_Offset_list[current_Offset_count]] = TimeslotLength_mac;
                        DCFL_TimeslotLength_mac[current_Timeslot][current_Offset] = TimeslotLength_mac;

                      }
                      else{
                        //std::cout << " current_Offset_list[current_Offset_count] : " << current_Offset_list[current_Offset_count] 
                        //          << " and current_Offset_count : " << current_Offset_count << std::endl;                        
                        //DCFL_TimeslotLength_mac[current_Timeslot][current_Offset_list[current_Offset_count]] = TimeslotLength_mac;
                        DCFL_TimeslotLength_mac[current_Timeslot][current_Offset] = TimeslotLength_mac;
                        NS_LOG_DEBUG(" Less than current TimeslotLength_mac : " << TimeslotLength_mac 
                                     << ", keeping the previous one of TimeslotLength_mac : " << node_TimeslotLength[next_packet_size_currentTimeslot]
                                     << " at timeslot " << next_packet_size_currentTimeslot );
                      }
                    }
                    
                    previous_packet_size_currentTimeslot = next_packet_size_currentTimeslot;
                    current_Offset_count++;
                    
                  
                  }
                  else 
                  {
                    node_TimeslotLength[next_packet_size_currentTimeslot] = 10000;
                    previous_packet_size_currentTimeslot = next_packet_size_currentTimeslot;
                    current_Offset_count++;
                    NS_LOG_DEBUG( " Not find in received header, Will change back at next slotframe for default Timeslot Length : 10000 " 
                                 << " at timeslot " << current_Timeslot);

                  
                  }

                  if (UPA && this == dev[0]) // after received and the queue is not zero. Will process UPA
                  {
                    if (!UPA_processing[current_TX]) // no process again. Only for start
                    {
                      if (UPA_queue[current_TX] != 0)
                      {
                        UPA_addslot[current_TX] = UPA_queue[current_TX] % 6 == 0 ? UPA_queue[current_TX] / 6 : UPA_queue[current_TX] / 6 + 1;
                        if (UPA_addslot[current_TX] > 4)
                          UPA_addslot[current_TX] = 4;

                        UPA_processing[current_TX] = 1;
                        // UPA_queue[current_TX] = 0;
                        std::cout << " received packet for UPA_queue is not zero. Starting UPA for TX " << current_TX << " and queue " << UPA_queue[current_TX] << std::endl;
                      }
                    }
                  }

                  // If it is a data frame, push it up the stack.
                  NS_LOG_DEBUG ("Packet successfully received from " << params.m_srcAddr);
                  m_mcpsDataIndicationCallback (params, p);
                  m_latestPacketSize = originalPkt->GetSize();
                  //TODO: check the src MAC address
                  if (receivedMacHdr.IsAckReq ())
                    {
                      NS_LOG_DEBUG("Sending ack for a data packet.");
                      Simulator::Schedule(MicroSeconds(def_MacTimeslotTemplate.m_macTsTxAckDelay),&LrWpanTschMac::SendAck,this,
                                          receivedMacHdr.GetSeqNum(),receivedMacHdr.IsSeqNumSup());
                      m_lrWpanMacStatePending = TSCH_MAC_SENDING;
                      Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);
                    }
                  else
                    {
                      m_macRxDataTrace(m_latestPacketSize);
                    }

                if (m_lrWpanMacState == TSCH_PKT_WAIT_END)
                  {
                    ChangeMacState(TSCH_MAC_IDLE);
                  }
                else
                  {
                    Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);
                  }

                }
              else
                {
                  //TODO: packet not expected
                  NS_LOG_DEBUG("Packet not expected pkt type = " << receivedMacHdr.GetType());
                  if (receivedMacHdr.IsData ())
                    {
                      m_macRxDataTrace(p->GetSize());
                    }
                }
            }
          else
            {
              
              m_macRxDropTrace (originalPkt);
              NS_LOG_DEBUG("Filter fail");
            }
        }
    }
}

void
LrWpanTschMac::SendAck (uint8_t seqno, bool seqnumsup)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (m_lrWpanMacState == TSCH_MAC_IDLE);

  // Generate a corresponding ACK Frame.
  LrWpanMacHeader macHdr;
  macHdr.SetType(LrWpanMacHeader::LRWPAN_MAC_ACKNOWLEDGMENT);
  macHdr.SetFrameVer (2);
  if (!seqnumsup)
    {
      macHdr.SetNoSeqNumSup();
      macHdr.SetSeqNum(seqno);
    }
  else
    {
      macHdr.SetSeqNumSup();
    }
/*
  LrWpanMacTrailer macTrailer1;
  Ptr<Packet> ackPacket1 = Create<Packet> (0);
  ackPacket1->AddHeader (macHdr);

  if (Node::ChecksumEnabled ())
    {
      macTrailer1.EnableFcs (true);
      macTrailer1.SetFcs (ackPacket1);
    }
  //ackPacket1->AddTrailer (macTrailer1);

  NS_LOG_DEBUG("Testing ack with size = " << ackPacket1->GetSize() );
*/

  macHdr.SetNoPanIdComp();
  macHdr.SetDstAddrMode(0);
  macHdr.SetSrcAddrMode(0);

  macHdr.SetIEField();
  macHdr.NewAckIE(7); //TODO: timing!
  macHdr.EndNoPayloadIE();

  LrWpanMacTrailer macTrailer;
  Ptr<Packet> ackPacket = Create<Packet> (0);
  ackPacket->AddHeader (macHdr);

  // Calculate FCS if the global attribute ChecksumEnable is set.
  if (Node::ChecksumEnabled ())
    {
      macTrailer.EnableFcs (true);
      macTrailer.SetFcs (ackPacket);
    }
  ackPacket->AddTrailer (macTrailer);

  // Enqueue the ACK packet for further processing
  // when the transmitter is activated.

  m_txPkt = ackPacket;

  NS_LOG_DEBUG("Sending ack with size = " << m_txPkt->GetSize() << " " << m_txPkt->GetSerializedSize());
  // Switch transceiver to TX mode. Proceed sending the Ack on confirm.
  SetLrWpanMacState (TSCH_MAC_SENDING);
}

void
LrWpanTschMac::RemoveTxQueueElement ()
{
  NS_LOG_FUNCTION (this);

  TxQueueRequestElement *txQElement = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front();
  Ptr<const Packet> p = txQElement->txQPkt;
  //m_numCsmacaRetry += m_csmaCa->LrWpanCsmaCa::GetNB () + 1; //need to change net-device m_csmaca->SetMac (m_mac); or m_mac->SetCsmaCa (m_csmaca); by ding
  // ppp
  //uint8_t test;
  //test=m_csmaCa->LrWpanCsmaCa::GetNB();
  //m_csmaCa->GetMacMinBE();
  //m_numCsmacaRetry +=8;
  //NS_LOG_UNCOND("csmacaretry ");

  Ptr<Packet> pkt = p->Copy ();
  LrWpanMacHeader hdr;
  pkt->RemoveHeader (hdr);
  if (hdr.GetShortDstAddr () != Mac16Address ("ff:ff"))
    {
      if (txQElement->txRequestNB == m_macMaxFrameRetries)
        {
          NS_LOG_DEBUG ("Maximum retry reached, delete one request in the queue with link position = "<< m_txLinkSequence);
          m_macMaxRetries(p);
        }
      else
        {
          //NS_LOG_UNCOND("sent pkt trace");//change by ding
          m_sentPktTrace (p, m_retransmission + 1, m_numCsmacaRetry); //change by ding
          //m_sentPktTrace (p, txQElement->txRequestNB + 1);
        }
    }

  txQElement->txQPkt = 0;
  delete txQElement;

  m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.pop_front ();

  if (m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size() == 0){
      m_txQueueAllLink.erase(m_txQueueAllLink.begin() + m_txLinkSequence);
      NS_LOG_DEBUG ("Delete queue with link position = "<< m_txLinkSequence);
    }
  m_txLinkSequence = 0;
  m_txPkt = 0;
  m_numCsmacaRetry = 0; //change by ding
  m_macTxDequeueTrace (p);
}

void
LrWpanTschMac::PdDataConfirm (LrWpanPhyEnumeration status)
{
  NS_ASSERT (m_lrWpanMacState == TSCH_MAC_SENDING);

  NS_LOG_FUNCTION (this << status << m_txQueueAllLink.size ());

  LrWpanMacHeader macHdr;
  m_txPkt->PeekHeader (macHdr);

  if (status == IEEE_802_15_4_PHY_SUCCESS)
    {
      if (!macHdr.IsAcknowledgment ())
        {
          // We have just send a regular data packet, check if we have to wait
          // for an ACK.
          packetorack = 0;   // use in check it it packet or ack
          NS_LOG_DEBUG("Packet transmission successful");
          for( int i=0; i<dev_count; i++){
            if(this == dev[i]){
              current_TX = i;
              std::cout << "for transmit packet current_TX : " << current_TX << std::endl;
            }
          }
          if (macHdr.IsAckReq ())
            {
              Simulator::Schedule (MicroSeconds(def_MacTimeslotTemplate.m_macTsRxAckDelay),&LrWpanTschMac::WaitAck,this);
              m_lrWpanMacStatePending = TSCH_MAC_ACK_PENDING;
            }
          else
            {
              m_macTxOkTrace (m_txPkt);
              m_macTxDataTrace(m_latestPacketSize);
              // remove the copy of the packet that was just sent
              if (!m_mcpsDataConfirmCallback.IsNull ())
                {
                  McpsDataConfirmParams confirmParams;
                  NS_ASSERT_MSG (m_txQueueAllLink.size () > 0, "TxQsize = 0");
                  TxQueueRequestElement *txQElement = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front ();
                  confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                  confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                  m_mcpsDataConfirmCallback (confirmParams);
                }
              RemoveTxQueueElement ();
            }
        }
      else
        {
          packetorack = 1;
          NS_LOG_DEBUG("ACK transmission sussesfull");
          m_macRxDataTxAckTrace(m_latestPacketSize);
          // We have send an ACK. Clear the packet buffer.
          m_txPkt = 0;
        }
    }
  else if (status == IEEE_802_15_4_PHY_UNSPECIFIED)
    {

      if (!macHdr.IsAcknowledgment ())
        {
          NS_LOG_DEBUG("Unable to send packet");
          if ((Now().GetSeconds() - m_lastTransmission.GetSeconds()) == 0.0)
            {
              m_macRxDataTrace(m_latestPacketSize);
            }
          m_latestPacketSize = m_txPkt->GetSize();
          NS_ASSERT_MSG (m_txQueueAllLink.size () > 0, "TxQsize = 0");
          TxQueueRequestElement *txQElement = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front ();
          m_macTxDropTrace (txQElement->txQPkt);
          if (!m_mcpsDataConfirmCallback.IsNull ())
            {
              McpsDataConfirmParams confirmParams;
              confirmParams.m_msduHandle = txQElement->txQMsduHandle;
              confirmParams.m_status = IEEE_802_15_4_FRAME_TOO_LONG;
              m_mcpsDataConfirmCallback (confirmParams);
            }
          RemoveTxQueueElement ();
        }
      else
        {
          NS_LOG_ERROR ("Unable to send ACK");
          if ((Now().GetSeconds() - m_lastTransmission.GetSeconds()) == 0.0)
            {
              m_macRxDataTrace(m_latestPacketSize);
            }
          else
            {
              m_macRxDataTxAckTrace(m_latestPacketSize);
            }
        }
    }
  else
    {
      // Something went really wrong. The PHY is not in the correct state for
      // data transmission.
      NS_FATAL_ERROR ("Transmission attempt failed with PHY status " << status);
    }

  // add by ieong // for the consecutive of RX
  // after ACK transmit successful, it will change mac idle and turn off for the RX node.
  if( ((Isconsecutive == 0 && packetorack == 1) || packetorack == 0 ) && !UPA ){   // it is RX procees of consecutive and no need to do anything for packetorarck = 0 ( packet )

    m_setMacState.Cancel ();
    m_setMacState = Simulator::ScheduleNow (&LrWpanTschMac::SetLrWpanMacState, this, TSCH_MAC_IDLE);
  }
  else if( Isconsecutive != 0 && packetorack == 1){   // keep it RX on   

    std::cout << " In Rx, Isconsecutive =  " << Isconsecutive << std::endl;

    Time time2wait = MicroSeconds(def_MacTimeslotTemplate.m_macTsRxTx);
    Simulator::Schedule (time2wait,&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_RX);
    m_lrWpanMacStatePending = TSCH_MAC_RX;
    Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);

  }
  if(UPA){
    if(!macHdr.IsAcknowledgment ()){   // data
      m_setMacState.Cancel ();
      m_setMacState = Simulator::ScheduleNow (&LrWpanTschMac::SetLrWpanMacState, this, TSCH_MAC_IDLE);
    }
    else{  // ack

      for ( int i=0; i<dev_count; i++){
        if(UPA_processing[i] && UPA_queue[i] != 0){
          std::cout << " In Rx of UPA for node " << i << " and queue " << UPA_queue[i] << std::endl;

          Time time2wait = MicroSeconds(def_MacTimeslotTemplate.m_macTsRxTx);
          Simulator::Schedule (time2wait,&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_RX);
          m_lrWpanMacStatePending = TSCH_MAC_RX;
          Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);
        }
      }
      
    }
  }
  
}

void
LrWpanTschMac::PlmeCcaConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_FUNCTION (this << status);
  // Direct this call through the csmaCa object
  //m_csmaCa->PlmeCcaConfirm (status); //change by ding   //ppp
  
  if (status != IEEE_802_15_4_PHY_IDLE)
    {
      NS_LOG_DEBUG("CCA failure");
      m_macChannelBusyTrace(0);
      SetLrWpanMacState (TSCH_CHANNEL_ACCESS_FAILURE);
    }
  else
    {
      NS_LOG_DEBUG("CCA successfull");
      SetLrWpanMacState (TSCH_CHANNEL_IDLE);
    }
}

void
LrWpanTschMac::PlmeEdConfirm (LrWpanPhyEnumeration status, uint8_t energyLevel)
{
  NS_LOG_FUNCTION (this << status << energyLevel);

}

void
LrWpanTschMac::PlmeGetAttributeConfirm (LrWpanPhyEnumeration status,
                                    LrWpanPibAttributeIdentifier id,
                                    LrWpanPhyPibAttributes* attribute)
{
  NS_LOG_FUNCTION (this << status << id << attribute);
}

void
LrWpanTschMac::PlmeSetTRXStateConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_FUNCTION (this << status);
  
  if (status == IEEE_802_15_4_PHY_FORCE_TRX_OFF)
      m_setMacState.Cancel ();

  else if (m_lrWpanMacState == TSCH_MAC_SENDING && (status == IEEE_802_15_4_PHY_TX_ON || status == IEEE_802_15_4_PHY_SUCCESS
                                               || status == IEEE_802_15_4_PHY_TRX_SWITCHING || status == IEEE_802_15_4_PHY_TRX_START))
    {
      if (status == IEEE_802_15_4_PHY_TX_ON || status == IEEE_802_15_4_PHY_SUCCESS)
      {
        std::cout << "m_txPkt : " << m_txPkt->GetSize() << std::endl;
        NS_ASSERT (m_txPkt);

        // Start sending if we are in state SENDING and the PHY transmitter was enabled.
        m_promiscSnifferTrace (m_txPkt);
        m_snifferTrace (m_txPkt);
        //m_macTxTrace (m_txPkt);            // it will count ack in tree topology . not count ack first  // add by ieong
        //m_macTxTrace (m_txPkt);            
        m_lastTransmission = Now();

        LrWpanMacHeader macHdr;
        m_txPkt->PeekHeader(macHdr);
        
        if (macHdr.IsData())
          {
            m_macTxTrace (m_txPkt);          // only count if it is data  // add by ieong
            m_latestPacketSize = m_txPkt->GetSize();
          }

        m_phy->PdDataRequest (m_txPkt->GetSize (), m_txPkt);
      }
    }

  else if (m_lrWpanMacState == TSCH_MAC_CCA && (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
      // Start the CSMA algorithm as soon as the receiver is enabled.
      //NS_LOG_DEBUG("With CSMA/CA");    //ppp
      //m_csmaCa->Start (); //change by ding //start csmaca? //ppp
      m_phy->PlmeCcaRequest();
    }
  else if (m_lrWpanMacState == TSCH_MAC_RX && (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
      NS_LOG_DEBUG("Wait for a packet for " << def_MacTimeslotTemplate.m_macTsRxWait);
      Simulator::Schedule(MicroSeconds(def_MacTimeslotTemplate.m_macTsRxWait),&LrWpanTschMac::RxWaitDone,this);
    }
  else if (m_lrWpanMacState == TSCH_MAC_IDLE)
    {
      //strange bug here suddenly can't run in lr-wapn-tsch-test --nrnodes=1 and 2 by ding
      // NS_ASSERT (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS || status == IEEE_802_15_4_PHY_TRX_OFF
                //  || status == IEEE_802_15_4_PHY_TRX_SWITCHING || status == IEEE_802_15_4_PHY_TRX_START);
      // Do nothing special when going idle.
    }
  else if (m_lrWpanMacState == TSCH_PKT_WAIT_END)
    {

      
      if (m_macWaitDone == Now())
        {
           
          //didnt receive any packet
          m_macIdleTrace(0);
          ChangeMacState (TSCH_MAC_IDLE);
        }
      NS_ASSERT (status == IEEE_802_15_4_PHY_TRX_OFF || status == IEEE_802_15_4_PHY_SUCCESS);
      
      NS_LOG_DEBUG("End of waiting for a packet at " << Now().GetSeconds());

      if(w5){
        current_Timeslot = m_macTschPIBAttributes.m_macASN % adaptive_slotframe_size;
      }
      else{
        current_Timeslot = m_macTschPIBAttributes.m_macASN % slotframe_size_mac;
      }
      //if( this == dev[0] && AddOrDelete == 2 ){
      if( this == dev[0] && AddedLink_node[current_TX]>=1 && (w3||w4)){
        
        std::cout << "current_TX : " << current_TX << std::endl;
        
        for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++)
        {

          if( int(i->macTimeslot) == current_Timeslot )
          {

            if(delete_link_count!=0){ // to prevent add 2 times in 1 period
              if( delete_link[delete_link_count-1][2] == int(i->macTxID) && delete_link[delete_link_count-1][3] == int(i->macRxID) ){
                break;
              }
            }

            delete_link[delete_link_count][0] = int(i->macTimeslot);
            delete_link[delete_link_count][1] = int(i->macChannelOffset);
            delete_link[delete_link_count][2] = int(i->macTxID);
            delete_link[delete_link_count][3] = int(i->macRxID);
            delete_link_control = 1;
            delete_link_count++;
            NS_LOG_DEBUG("Will delete Link " << int(i->macTxID) << "->" << int(i->macRxID) << " for Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) );
            NS_LOG_DEBUG("Number of delete link is " << delete_link_count );
            break;
          }

        }
        AddOrDelete = 0;

      }
      
      
    }
  else if (m_lrWpanMacState == TSCH_MAC_ACK_PENDING_END)
    {
      if (m_macWaitDone == Now())
        {
          //didnt receive any packet
          m_macWaitAckTrace(m_latestPacketSize);
          NS_LOG_DEBUG("No ack received.");
          HandleTxFailure ();
          ChangeMacState (TSCH_MAC_IDLE);
        }

      NS_ASSERT (status == IEEE_802_15_4_PHY_TRX_OFF || status == IEEE_802_15_4_PHY_SUCCESS);
      NS_LOG_DEBUG("End of waiting for an ack at " << Now().GetSeconds());
    }
  else if (m_lrWpanMacState == TSCH_MAC_ACK_PENDING)
    {
      //strange bug here suddenly can't run in lr-wapn-tsch-test --nrnodes=6 by ding
      // NS_ASSERT (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS);
    }
  else
    {
      // TODO: What to do when we receive an error?
      // If we want to transmit a packet, but switching the transceiver on results
      // in an error, we have to recover somehow (and start sending again).
      //maybe can change here by ding
      //strange bug here suddenly can't run in lr-wapn-tsch-test --nrnodes=1 and 2 by ding
      // NS_FATAL_ERROR ("Error changing transceiver state");
    }
}

void
LrWpanTschMac::PlmeSetAttributeConfirm (LrWpanPhyEnumeration status,
                                    LrWpanPibAttributeIdentifier id)
{
  NS_LOG_FUNCTION (this << status << id);
}

void
LrWpanTschMac::SetLrWpanMacState (LrWpanTschMacState macState)
{
  NS_LOG_FUNCTION (this << " mac state = " << macState);

  McpsDataConfirmParams confirmParams;

  if (macState == TSCH_MAC_IDLE)
    {
      ChangeMacState (TSCH_MAC_IDLE);
      if (m_lrWpanMacStatePending == TSCH_MAC_IDLE){

        //std::cout << " Making phy TRX OFF " << std::endl;  //  change to idle
        m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);
        
      }/*
      else if( m_lrWpanMacStatePending == TSCH_MAC_ACK_PENDING && consecutiveongoing == 1){
        ChangeMacState (TSCH_MAC_ACK_PENDING);
        m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
      }
      else if( m_lrWpanMacStatePending == TSCH_MAC_ACK_PENDING && consecutiveongoing == 1){

      }*/
      else
        {
          m_lrWpanMacStatePending = TSCH_MAC_IDLE;
          if (m_newSlot && firstforconsceutive == 1)   // add by ieong // add consecutiveongoing for make it don't IEEE_802_15_4_PHY_TRX_START
           {
              m_newSlot = false;
              m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_START);
          }
              else
            m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_SWITCHING);
        }
    }
  else if (macState == TSCH_MAC_ACK_PENDING)
    {
      ChangeMacState (TSCH_MAC_ACK_PENDING);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
    }
  else if (macState == TSCH_MAC_CCA || macState == TSCH_MAC_RX)
    {


      //std::cout << " Come in TSCH_MAC_CCA/TSCH_MAC_RX  and m_macWaitDone : " << m_macWaitDone << std::endl;
      //std::cout<< " m_lrWpanMacState : " << m_lrWpanMacState << std::endl;
      //std::cout<< " macState : " << m_lrWpanMacState << std::endl;
      // delete

      NS_ASSERT (m_lrWpanMacState == TSCH_MAC_IDLE || m_lrWpanMacState == TSCH_MAC_ACK_PENDING);

      ChangeMacState (macState);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
    }
  else if (macState == TSCH_PKT_WAIT_END || macState == TSCH_MAC_ACK_PENDING_END)
    {

      //std::cout << " Come in first and m_macWaitDone : " << m_macWaitDone << std::endl;
      //delete

      NS_ASSERT (m_lrWpanMacState == TSCH_MAC_RX || m_lrWpanMacState == TSCH_MAC_ACK_PENDING);

      ChangeMacState (macState);
      m_macWaitDone = Now();

      //std::cout << " Come in and m_macWaitDone : " << m_macWaitDone << std::endl;
      //delete

      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);
    }
  else if (m_lrWpanMacState == TSCH_MAC_CCA && macState == TSCH_CHANNEL_IDLE)
    {
      // Channel is idle, set transmitter to TX_ON
      NS_LOG_DEBUG (this << " channel idle, set TX_ON");
      ChangeMacState (TSCH_MAC_SENDING);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TX_ON);
    }
  else if (m_lrWpanMacState == TSCH_MAC_CCA && macState == TSCH_CHANNEL_ACCESS_FAILURE)
    {
      NS_ASSERT (m_txPkt);

      // cannot find a clear channel, drop the current packet.
      NS_LOG_DEBUG ( this << " cannot find clear channel");
      confirmParams.m_msduHandle = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front()->txQMsduHandle;
      confirmParams.m_status = IEEE_802_15_4_CHANNEL_ACCESS_FAILURE;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      // remove the copy of the packet that was just sent
      ChangeMacState (TSCH_MAC_IDLE);

      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);


    }
  else if (m_lrWpanMacState == TSCH_MAC_IDLE && macState == TSCH_MAC_SENDING)
    {
      // sending without cca
      ChangeMacState (TSCH_MAC_SENDING);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TX_ON);
    }

}

LrWpanAssociationStatus
LrWpanTschMac::GetAssociationStatus (void) const
{
  return m_associationStatus;
}

void
LrWpanTschMac::SetAssociationStatus (LrWpanAssociationStatus status)
{
  m_associationStatus = status;
}

uint16_t
LrWpanTschMac::GetPanId (void) const
{
  return m_macPanId;
}

void
LrWpanTschMac::SetPanId (uint16_t PanId)
{
  m_macPanId = PanId;
}

void
LrWpanTschMac::ChangeMacState (LrWpanTschMacState newState)
{
  NS_LOG_LOGIC (this << " change lrwpan mac state from "
                     << m_lrWpanMacState << " to "
                     << newState);
  m_macStateLogger (m_lrWpanMacState, newState);
  m_lrWpanMacState = newState;
}

//change by ding
uint64_t
LrWpanTschMac::GetMacAckWaitDuration (void) const
{
  return m_csmaCa->GetUnitBackoffPeriod () + m_phy->aTurnaroundTime + m_phy->GetPhySHRDuration ()
         + ceil (6 * m_phy->GetPhySymbolsPerOctet ());
}

uint8_t
LrWpanTschMac::GetMacMaxFrameRetries (void) const
{
  return m_macMaxFrameRetries;
}

void
LrWpanTschMac::SetMacMaxFrameRetries (uint8_t retries)
{
  m_macMaxFrameRetries = retries;
}

/*
TSCH method
*/

//call by AddSlotframe in lr-wpan-tsch-helper.cc
//MLME-SET-SLOTFRAME.request primitive is used to add, delete, or modify a slotframe at the MAC sublayer
void
LrWpanTschMac::MlmeSetSlotframeRequest (MlmeSetSlotframeRequestParams params)
{
  NS_LOG_DEBUG(this);
  MlmeSetSlotframeConfirmParams confirmParams;
  bool foundsf = false;
  num_node=params.size;//use in time adjust //change by ding
  slotframe_size_mac = params.size; // add by ieong // get slotframe size
  incasn_slotframe_size = params.size;
  adaptive_slotframe_size = params.size;
  switch(params.Operation) {
    case MlmeSlotframeOperation_ADD: //add
      confirmParams.slotframeHandle = params.slotframeHandle;
      if (params.slotframeHandle < 0 && params.slotframeHandle > 255) {
        confirmParams.Status = MlmeSetSlotframeConfirmStatus_INVALID_PARAMETER;
      } else {
        MacPibSlotframeAttributes entry;
        entry.slotframeHandle = params.slotframeHandle;
        entry.size = params.size;
        m_macSlotframeTable.push_back(entry);
        confirmParams.Status = MlmeSetSlotframeConfirmStatus_SUCCESS;
      }
      nrnodes++;
      

      break;
    case MlmeSlotframeOperation_DELETE: //delete
      confirmParams.slotframeHandle = params.slotframeHandle;
      
      for (std::list<MacPibSlotframeAttributes>::iterator i = m_macSlotframeTable.begin();i != m_macSlotframeTable.end();i++)
        {
          if (i->slotframeHandle == params.slotframeHandle) {
            foundsf = true;
            confirmParams.Status = MlmeSetSlotframeConfirmStatus_SUCCESS;
            if (currentLink.active && currentLink.slotframeHandle == params.slotframeHandle)
              {
                //wait
              }
            else
              {
                //delete
              }
          }
        }

      if (!foundsf) 
        {
          confirmParams.Status = MlmeSetSlotframeConfirmStatus_SLOTFRAME_NOT_FOUND;
        }

      break;
    case MlmeSlotframeOperation_MODIFY: //modify
      confirmParams.slotframeHandle = params.slotframeHandle;
      for (std::list<MacPibSlotframeAttributes>::iterator i = m_macSlotframeTable.begin();i != m_macSlotframeTable.end();i++)
        {
          if (i->slotframeHandle == params.slotframeHandle) {
            foundsf = true;
            confirmParams.Status = MlmeSetSlotframeConfirmStatus_SUCCESS;
            i->size = params.size;
          }
        }

      if (!foundsf) 
        {
          confirmParams.Status = MlmeSetSlotframeConfirmStatus_SLOTFRAME_NOT_FOUND;
        }

      break;
    default:
      break;
  }

  if ((!m_mlmeSetSlotframeConfirmCallback.IsNull ()))
    {
      m_mlmeSetSlotframeConfirmCallback (confirmParams);
    }
  else
    {
      NS_LOG_ERROR ("m_mlmeSetSlotframeConfirmCallback not initialized");
    }
}

void
LrWpanTschMac::MlmeSetLinkRequest (MlmeSetLinkRequestParams params)
{
  NS_LOG_DEBUG(this);
  MlmeSetLinkConfirmParams confirmParams;
  confirmParams.linkHandle = params.linkHandle;
  confirmParams.slotframeHandle = params.slotframeHandle;
  MacPibLinkAttributes entry;
  bool  foundlink = false;
  //std::list<MacPibLinkAttributes>::iterator it;

  int t = int(params.Timeslot);
  int c = int(params.ChannelOffset);
  int TxID = int(params.TxID);
  int RxID = int(params.RxID);

  switch(params.Operation) {
    case MlmeSetLinkRequestOperation_ADD_LINK:
        if ( params.linkOptions[0] && params.linkOptions[2] == 0 ) {

          //if( c > scheduling_table_link[t] ){
          //  scheduling_table_link[t]++;
          //}

          std::cout << " TX : " << TxID << " RX : " << RxID << std::endl;
          scheduling_table[t][scheduling_table_link[t]][0][0] = TxID;
          scheduling_table[t][scheduling_table_link[t]][0][1] = RxID;
          scheduling_table_link[t]++;
          std::cout << " set dedicated link " << scheduling_table[t][scheduling_table_link[t]-1][0][0]
                    << "->" << scheduling_table[t][scheduling_table_link[t]-1][0][1] << std::endl;
          AddedLink_node[TxID]++;

          if (UPA)
          {
            if (TxID != 0)
            {
              UPA_addr[TxID] = params.nodeAddr;
            }
          }
        }
        else if (params.linkOptions[0] && params.linkOptions[2] ) {

          //if( c > scheduling_table_link[t] ){
          //  scheduling_table_link[t]++;
          //}

          if(scheduling_table_SharedLink[t][c]) // first in for shared link
          {
            scheduling_table_link[t]++;
          }

          std::cout << " TX : " << TxID << " RX : " << RxID << std::endl;
          scheduling_table[t][c][scheduling_table_SharedLink[t][c]][0] = TxID;
          scheduling_table[t][c][scheduling_table_SharedLink[t][c]][1] = RxID;
          //scheduling_table_link[t]++;
          scheduling_table_SharedLink[t][c]++;
          SharedLink_count++;
          std::cout << " set shared link " << scheduling_table[t][scheduling_table_link[t]][scheduling_table_SharedLink[t][params.ChannelOffset]-1][0]
                    << "->" << scheduling_table[t][scheduling_table_link[t]][scheduling_table_SharedLink[t][params.ChannelOffset]-1][1] << std::endl;

        }

        
        
      
      
    
      entry.macLinkHandle = params.linkHandle;
      entry.macLinkOptions = params.linkOptions; //b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.
      entry.macLinkType = params.linkType;
      entry.slotframeHandle = params.slotframeHandle;
      entry.macNodeAddr = params.nodeAddr; //not using Mac16_Address because 0xffff means the link can be used for frames destined for the boradcast address
      entry.macTimeslot = params.Timeslot; //refer to 5.1.1.5
      entry.macChannelOffset = params.ChannelOffset; //refer to 5.1.1.5.3
      entry.macLinkFadingBias = params.linkFadingBias;
      entry.macTxID = params.TxID;
      entry.macRxID = params.RxID;

      m_macLinkTable.push_back(entry);
      confirmParams.Status = MlmeSetLinkConfirmStatus_SUCCESS;
      //std::cout << " m_macLinkTable.size() : " << m_macLinkTable.size() << std::endl;

      break;
    case MlmeSetLinkRequestOperation_DELETE_LINK:    
      for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++)
        {
          if (i->slotframeHandle == params.slotframeHandle && i->macLinkHandle == params.linkHandle) {
            foundlink = true;
            confirmParams.Status = MlmeSetLinkConfirmStatus_SUCCESS;
            if (currentLink.active && currentLink.slotframeHandle == params.slotframeHandle && currentLink.linkHandle == params.linkHandle)
              {
                m_waitingLink = true;
                m_waitingLinkParams = params;
              }
            else
              {
                m_waitingLink = false;
                m_macLinkTable.erase(i);
              }
            break;
          }
        }

      if (!foundlink) 
        {
          confirmParams.Status = MlmeSetLinkConfirmStatus_UNKNOWN_LINK;
        }
      break;
    case MlmeSetLinkRequestOperation_MODIFY_LINK:
      for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++)
        {
          if (i->slotframeHandle == params.slotframeHandle && i->macLinkHandle == params.linkHandle) {
            foundlink = true;
            confirmParams.Status = MlmeSetLinkConfirmStatus_SUCCESS;
            if (currentLink.active && currentLink.slotframeHandle == params.slotframeHandle && currentLink.linkHandle == params.linkHandle)
              {
                m_waitingLink = true;
                m_waitingLinkParams = params;
              }
            else
              {
                NS_LOG_DEBUG("TSCH modifying link");
                m_waitingLink = false;
                i->macLinkOptions = params.linkOptions; //b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.
                i->macLinkType = params.linkType;
                i->macNodeAddr = params.nodeAddr; //not using Mac16_Address, 0xffff means the link can be used for frames destined for the broadcast address
                i->macTimeslot = params.Timeslot; //refer to 5.1.1.5
                i->macChannelOffset = params.ChannelOffset; //refer to 5.1.1.5.3
                i->macLinkFadingBias = params.linkFadingBias;
                i->macTxID = params.TxID;
                i->macRxID = params.RxID;
              }
            break;
          }
        }

      if (!foundlink) 
        {
          confirmParams.Status = MlmeSetLinkConfirmStatus_UNKNOWN_LINK;
        }
      break;
    default:
      break;
  }

  if ((!m_mlmeSetLinkConfirmCallback.IsNull ()) && !m_waitingLink)
    {
      m_mlmeSetLinkConfirmCallback (confirmParams);
    }
  else if(!m_waitingLink)
    {
      NS_LOG_ERROR ("m_mlmeSetLinkConfirmCallback not initialized");
    }
}

void
LrWpanTschMac::MlmeTschModeRequest (MlmeTschModeRequestParams params)
{
  NS_LOG_FUNCTION(this);
  MlmeTschModeConfirmParams confirmParams;
  confirmParams.TSCHMode = params.TSCHMode;

  switch(params.TSCHMode) {
    case MlmeTschMode_ON:

      /*TODO: Check if it is synced
      if (its not synced) {
        confirmParams.Status = LrWpanMlmeTschModeConfirmStatus_NO_SYNC; //no sync
      }*/

      m_waitingLink = false;
      SetLrWpanMacState(TSCH_MAC_IDLE);
      //schedule asn incrementation
      Simulator::ScheduleNow (&LrWpanTschMac::IncAsn,this);

      confirmParams.Status = LrWpanMlmeTschModeConfirmStatus_SUCCESS; //success
      break;
    case MlmeTschMode_OFF:
      Simulator::Stop();
      confirmParams.Status = LrWpanMlmeTschModeConfirmStatus_SUCCESS;
      break;
    default:
      break;
  }

  if ((!m_mlmeTschModeConfirmCallback.IsNull ()))
    {
      m_mlmeTschModeConfirmCallback (confirmParams);
    }
  else
    {
      NS_LOG_ERROR ("m_mlmeTschModeConfirmCallback not initialized");
    }
}

void
LrWpanTschMac::IncAsn()
{

  NS_LOG_FUNCTION (this);
  m_newSlot = 1;
  
  m_macTschPIBAttributes.m_macASN++;
  //std::cout<< " ping6 m_size : " << Ping6::GetSize() << std::endl;
  // 分返n個ASN
  if( m_macTschPIBAttributes.m_macASN == 0){
    dev[dev_count] = this;
    dev_count++;
    if( dev[0] == this ){
      Initial();
      if( nrnodes < 6 )  // 
        w5 = 0;
    }
  
  }
  if (SLA)
  {
    int sec = (double(Now().GetSeconds()) * 10);   // 0.0
    int sec1 = (double(Now().GetSeconds()) * 100); // 0.00
    if (this == dev[0] && (sec % 10) == 9 && (sec1 % 10) == 9 && ping6_incasn_control == 0)
    {
      Ptr<Ping6> ping6App1 = DynamicCast<Ping6>(Ping6apps.Get(0));
      ping6_TimeslotLength = 3664 + (1000000 * (6 + 10 + int(ping6App1->GetSize())) * 2 / 62500) + 300;
      std::cout << "Using size : " << int(ping6App1->GetSize()) << " length : " <<  ping6_TimeslotLength << std::endl;
      std::cout << " reset ping6  " << std::endl;
      for (int i = 0; i < dev_count; i++)
      {
        Ptr<Ping6> ping6App = DynamicCast<Ping6>(Ping6apps.Get(i));
        //if(last_packetsize > 74)
        //  last_packetsize = 74;
        ping6App->SetSize(last_packetsize);
        if( i==0 )
          std::cout << " ping6App : " << ping6App->GetSize() << std::endl;
      }
      max_ping6_pktsize_mac = last_packetsize;
      ping6_incasn_control = 1;  // to prevent run two times in .99
    }
    if(this == dev[0] && (sec % 10) == 0){
      ping6_incasn_control = 0;
    }
  }

  /*
  if( double(Now().GetSeconds()) > 10 && double(Now().GetSeconds()) < 11)
  {
    for (int i = 0; i < dev_count; i++)
    {
      Ptr<Ping6> ping6App = DynamicCast<Ping6>(Ping6apps.Get(i));
      ping6App->SetSize(14);
      std::cout << " ping6App : " << ping6App->GetSize() << std::endl;
    }
  }*/


  /*
  for( int i=0;i<dev_count; i++){
    if( dev[i]== this ){
      node_ASN[i]++;
      //NS_LOG_DEBUG ( " ASN " << node_ASN[i] << " for node " << i );
      //NS_LOG_DEBUG ( " m_macASN " << m_macASN );

      if( node_ASN[i] != m_macTschPIBAttributes.m_macASN ){
        NS_FATAL_ERROR( " ERROR ASN " );
      }
      break;
    } 
  }
  */
  
  if( current_ASN != m_macTschPIBAttributes.m_macASN ){

    /*if( current_Offset_count == current_Offset_totalcount-1 ){
      current_Offset_count = 0;
      current_Offset_totalcount = 0;
    }*/
    current_Offset_count = 0;
    current_Offset_totalcount = 0;
  }
  current_ASN = m_macTschPIBAttributes.m_macASN;

  if( this == dev[0] && m_macTschPIBAttributes.m_macASN == 0)
  {
    print_scheduling_table();
    //adaptive_slotframe_size = slotframe_size_mac;
    //incasn_slotframe_size = slotframe_size_mac;
  }    

  if( this == dev[0] && m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 && w4){
    previous_packet_size_currentTimeslot = -1;
    for( int i=0; i<TimeslotSkipForNewAdd_count; i++)
      TimeslotSkipForNewAdd[i] = 0;
    TimeslotSkipForNewAdd_count = 0;
    for( int i=0; i<dev_count; i++){
      PacketSeqNum[i] = 0;
    }

    //clearbackoffwindow();

    SharedLink_cal();
    if( DedicatedLink_count != 0 || SharedLink_count != 0){
      int aa = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
      slotframe_size_mac =  1 + aa + DedicatedLink_count;
    }
    
    print_scheduling_table();


  }

  



  //////////////   change slotframe size
  if( (add_link_control && m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 )|| adding_slotframe_size_mac ){

    std::cout<< " process add " << std::endl;
    add_process();

  }
  if( (delete_link_control && m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 )|| deleting_slotframe_size_mac ){

    std::cout<< " process delete " << std::endl;
    delete_process();

  }

  if( (m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 && ( add_link_count != 0 || delete_link_count != 0 ) && w4) || (changing_ASN)){

    // shared link compress
    int a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;

    for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){
      for( int m=0; m<a; m++){
        for ( int j=0; j<=1; j++){
          if( int(i->macTxID) == SharedLink_Compress[m][j][2] && int(i->macRxID) == SharedLink_Compress[m][j][3] && (SharedLink_Compress[m][j][2] != 0 || SharedLink_Compress[m][j][3] != 0)){
            if( int(i->macTimeslot) != SharedLink_Compress[m][j][0] ){
              std::cout << "for shared link " << SharedLink_Compress[m][j][2] << "->" << SharedLink_Compress[m][j][3] << ", " << int(i->macTimeslot) << " change to "
                        << SharedLink_Compress[m][j][0] << std::endl;
              i->macTimeslot = uint16_t(SharedLink_Compress[m][j][0]);
            }
          }
        }
      }
      
    }
  }

  /*if(m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 && add_link_count == 0 && delete_link_count == 0 && w5){

    for (std::list<MacPibSlotframeAttributes>::iterator s = m_macSlotframeTable.begin();s != m_macSlotframeTable.end();s++)
    {
      int a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
      slotframe_size_mac =  1 + a + DedicatedLink_count;
      s->size = uint16_t(adaptive_slotframe_size);

      NS_LOG_DEBUG( " Change slotframe size to " << int(s->size));
      std::cout << " ASN from " << m_macTschPIBAttributes.m_macASN;
      while( m_macTschPIBAttributes.m_macASN%s->size != 0 ){
        m_macTschPIBAttributes.m_macASN++;
      }
      std::cout << " change to " << m_macTschPIBAttributes.m_macASN << " to make ts = 0 "<<std::endl;
    }

  }*/
  if( this == dev[0] && m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 && w4 && m_macTschPIBAttributes.m_macASN != 0){
    if( w5 && m_macTschPIBAttributes.m_macASN != 0)
      adaptive_slotframe();

    if(w5){
      incasn_slotframe_size = adaptive_slotframe_size;
      changing_ASN = 1;
    }
    else{
      incasn_slotframe_size = slotframe_size_mac;
    }  
    
    std::cout << " incasn_slotframe_size : " << incasn_slotframe_size << std::endl;
  }


  if( changing_ASN ){
    for (std::list<MacPibSlotframeAttributes>::iterator s = m_macSlotframeTable.begin();s != m_macSlotframeTable.end();s++)
    {
      int a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
      slotframe_size_mac =  1 + a + DedicatedLink_count;
      s->size = uint16_t(adaptive_slotframe_size);

      NS_LOG_DEBUG( " Change slotframe size to " << int(s->size));
      std::cout << " ASN from " << m_macTschPIBAttributes.m_macASN;
      if(new_slotframe){
        while( m_macTschPIBAttributes.m_macASN % int(s->size) != 0 ){
          //std::cout << " 111 " << std::endl;
          m_macTschPIBAttributes.m_macASN++;
        }
        std::cout << " change to " << m_macTschPIBAttributes.m_macASN << " to make ts = 0 "<<std::endl;
      }
      else{
        while( m_macTschPIBAttributes.m_macASN % int(s->size) != 1 ){
          //std::cout << " 222 " << std::endl;
          m_macTschPIBAttributes.m_macASN++;
        }
        std::cout << " change to " << m_macTschPIBAttributes.m_macASN << " to make ts = 1 "<<std::endl;
      }
      
    }

    
  }

  if( (m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 && w4 ) || changing_ASN )
  {
    
    update_scheduling();

  }






  if( (this == dev[dev_count-1] &&  m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 && w4) || (changing_ASN && this == dev[dev_count-1]) ){         // renews
    for( int k=0; k<add_link_count; k++){
      add_link[k][0] = 0;
      add_link[k][1] = 0;
      add_link[k][2] = 0;
      add_link[k][3] = 0;
      directly_change_add[k] = 0;
    }
    add_link_control = 0;
    add_link_count = 0;
    adding_slotframe_size_mac = 0;

    for( int k=0; k<delete_link_count; k++){
      delete_link[k][0] = 0;
      delete_link[k][1] = 0;
      delete_link[k][2] = 0;
      delete_link[k][3] = 0;
      directly_change_delete[k] = 0;
    }
    delete_link_control = 0;
    delete_link_count = 0;
    deleting_slotframe_size_mac = 0;
    
    changing_ASN = 0;
    new_slotframe = 0;
    

  }

  if( e_TSCH_Orch && m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 && m_macTschPIBAttributes.m_macASN != 0 && this == dev[0] ){

    slotframe_size_mac = 0;
    for (int i = 0; i < dev_count; i++)
    {
      slotframe_size_mac += e_TSCH_Orch_next_slotframe_queue[i];
    }
    incasn_slotframe_size = slotframe_size_mac;

    for (int i = 0; i < dev_count; i++)
    {
      std::cout << " i : " << i << " e_TSCH_Orch_next_slotframe_queue[i] : " << e_TSCH_Orch_next_slotframe_queue[i]
                << " e_TSCH_Orch_AddedLink[i] : " << e_TSCH_Orch_AddedLink[i] << std::endl;
    }

    std::cout << " process e_TSCH_Orch " << std::endl;
    e_TSCH_Orch_process();

    for (int i = 0; i < dev_count; i++)
    {
      e_TSCH_Orch_next_slotframe_queue[i] = 1; // renews
    }
  }

  if( UPA && m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0 && m_macTschPIBAttributes.m_macASN != 0 && this == dev[0] ){
    for( int i=0; i<dev_count; i++ ){
      UPA_queue[i] = 0;
      UPA_addslot[i] = 0;
    }
  }


  //////////////

  //dev++;
  //dev %= (nrnodes+1+1);
  
  
  

  if( m_macTschPIBAttributes.m_macASN%slotframe_size_mac == 0 && TimeslotLengthForAllocate_control == 0 && plan != 1){
    
    
    


    /*
    std::cout << " Scheduling Table : " << std::endl;

    for( int i=0; i<TimeslotForDCFL_mac; i++){
      std::cout.width(8);
      std::cout<< i << "";
    }
    std::cout << std::endl;

    std::ostringstream oss;
    for( int j=0; j<max_link_mac; j++){
      for( int i=0; i<TimeslotForDCFL_mac; i++){
        if( DCFL_mac[i][j][0]!=0 || DCFL_mac[i][j][1]!=0){
          oss.str("");
          oss << DCFL_mac[i][j][0] << "->" << DCFL_mac[i][j][1];
          std::cout.width(8);
          std::cout<< oss.str() << "";
        }
        else{
          oss.str("");
          std::cout.width(8);
          std::cout<< oss.str() << "";
        }

      }
    std::cout << std::endl;
    }


    std::cout << " Timeslot Length : " << std::endl;

    for( int i=0; i<TimeslotForDCFL_mac; i++){
      std::cout.width(8);
      std::cout<< i << "";
    }
    std::cout << std::endl;

    std::ostringstream oss1;
    for( int j=0; j<max_link_mac; j++){
      for( int i=0; i<TimeslotForDCFL_mac; i++){
        if( DCFL_mac[i][j][0]!=0 || DCFL_mac[i][j][1]!=0){
          oss1.str("");
          oss1 << DCFL_TimeslotLength_mac[i][j];
          std::cout.width(8);
          std::cout<< oss1.str() << "";
        }
        else{
          oss1.str("");
          std::cout.width(8);
          std::cout<< oss1.str() << "";
        }
      }
      std::cout << std::endl;
    }
    */


    if( plan == 4 ){
      TimeslotLength_allocate();
      TimeslotLengthForAllocate_control = 1;
    }
    

/*
    for( int i=0;i<TimeslotForDCFL_mac; i++){
      totaltime[totaltime_count] += TimeslotLengthForAllocate[0][i];
      //totaltime[totaltime_count] += node_TimeslotLength[i];
    
    }
    totaltime_count++;


    for( int i=0; i<totaltime_count; i++){
      NS_LOG_DEBUG("count : " << i << " total time : " << totaltime[i] );
    }
*/

  }
  else if( m_macTschPIBAttributes.m_macASN%slotframe_size_mac != 0 && TimeslotLengthForAllocate_control == 1 ){
    TimeslotLengthForAllocate_control = 0;
  }

  

  // uniform timeslot length
  /*
  if( ping6_ChangeTimeslotLength == 1 ){
    def_MacTimeslotTemplate.m_macTsTimeslotLength = TimeslotLength_mac;
    NS_LOG_DEBUG("Changing maximum packet size TimeslotLength : " << def_MacTimeslotTemplate.m_macTsTimeslotLength << " at ASN " << m_macTschPIBAttributes.m_macASN);
  }*/


  // 1. default
  if( plan == 1){
    def_MacTimeslotTemplate.m_macTsTimeslotLength = 10000;   // default 
  }



  // 2. uniform timeslot length
  ///////////////////////////
  if( plan == 2){

    if( m_macTschPIBAttributes.m_macASN%slotframe_size_mac == 0 ){   // ts = 0;
      max = 0;
      for ( int i=1; i<slotframe_size_mac; i++){
        if( node_TimeslotLength[i] > max ){
          max = node_TimeslotLength[i];
          //std::cout<< " max : " << max << std::endl;
        }
      }
      //for ( int i=0; i<TimeslotForDCFL_mac; i++){
        //node_TimeslotLength[i] = max;
      //}

      def_MacTimeslotTemplate.m_macTsTimeslotLength = 10000;   // use 10000 in advertisement
    }
    else{
      def_MacTimeslotTemplate.m_macTsTimeslotLength = max;
    }

  }

  /////////////////////////
  // 3.
  if( plan == 3 ){
    if(!w4)
        def_MacTimeslotTemplate.m_macTsTimeslotLength = node_TimeslotLength[m_macTschPIBAttributes.m_macASN%incasn_slotframe_size];
    if(w4){

        if( (this == dev[0] && m_macTschPIBAttributes.m_macASN%incasn_slotframe_size == 0) || ( this == dev[0] && changing_ASN) ){         // timeslot shift

          for ( int i=0; i<slotframe_size_mac; i++){
            std::cout<< " i : " << i <<" node_TimeslotLength[i] : " << node_TimeslotLength[i] << std::endl;
          }
          std::cout<<std::endl;
          for( int k=0; k<TimeslotSkipForNewAdd_count; k++){
            std::cout<<"TimeslotSkipForNewAdd[k] : " << TimeslotSkipForNewAdd[k] << std::endl;
          }






          std::cout << " delete shift " << std::endl;
          //bool b=0;
          for( int k=0; k<delete_link_count; k++){
            int bias = 0;
            for ( int g=0; g<add_link_count; g++){
              if( delete_link[k][0] > add_link[g][0] ){
                bias++;
              }
            }
            for ( int g=0; g<k; g++){
              if( delete_link[k][0] > delete_link[g][0] ){
                bias--;
              }
            }
            std::cout << " doing delete_link[k][0] : " << delete_link[k][0] << " bias : " << bias << std::endl;

            for( int m=delete_link[k][0]+1+bias; m<slotframe_size_mac; m++){
              //b = 0;
              //for( int n=0; n<TimeslotSkipForNewAdd_count; n++){
              //  if( m-1 == TimeslotSkipForNewAdd[n] )
              //    b = 1;
              //}
              //if( b==1)
              //  continue;
              node_TimeslotLength[m-1] = node_TimeslotLength[m];
              node_TimeslotLength[m] = 10000;
            }

            for ( int i=0; i<slotframe_size_mac; i++){
              std::cout<< " i : " << i <<" node_TimeslotLength[i] : " << node_TimeslotLength[i] << std::endl;
            }

          }
          for( int n=0; n<TimeslotSkipForNewAdd_count; n++){
            if( node_TimeslotLength[TimeslotSkipForNewAdd[n]] != 10000)
            {
              std::cout<<"TimeslotSkipForNewAdd[n] : " << TimeslotSkipForNewAdd[n] << std::endl;
              std::cout << " KKK" << std::endl;
            }
          }

          //for ( int i=0; i<slotframe_size_mac; i++){
          //  std::cout<< " i : " << i <<" node_TimeslotLength[i] : " << node_TimeslotLength[i] << std::endl;
          //}


          //for( int k=0; k<TimeslotSkipForTwoDelete_count; k++){
          //  std::cout<<"TimeslotSkipForTwoDelete[k] : " << TimeslotSkipForTwoDelete[k] << std::endl;
          //}
          /*
          bool b=0;
          for( int j=2; j<99; j++){
            for ( int i=2; i<99-j; i++){
              //std::cout<<"node_TimeslotLength[i] : " << node_TimeslotLength[i] << std::endl;
              b = 0;
              //if(node_TimeslotLength[i] != 10000 && node_TimeslotLength[i-1] == 10000 ){
              if(node_TimeslotLength[i-1] == 10000 ){
                for( int k=0; k<TimeslotSkipForNewAdd_count; k++){
                  if( TimeslotSkipForNewAdd[k] == i-1 )
                  {
                    b = 1;
                    break;
                  }  
                }
                //for( int k=0; k<TimeslotSkipForTwoDelete_count; k++){
                //  if( TimeslotSkipForTwoDelete[k] == i-1 )
                //  {
                //    b = 1;
                //    break;
                //  }  
                //}


                if( b==1 )
                  continue;
                node_TimeslotLength[i-1] = node_TimeslotLength[i];
                node_TimeslotLength[i] = 10000; 
                //for( int k=0; k<TimeslotSkipForNewAdd_count; k++){
                //  if( TimeslotSkipForNewAdd[k] == i )
                //  {
                //    TimeslotSkipForNewAdd[k] = i-1;
                    //break;
                //  }
                //}


              }
            }
          }

          for ( int i=0; i<slotframe_size_mac; i++){
            std::cout<< " i : " << i <<" node_TimeslotLength[i] : " << node_TimeslotLength[i] << std::endl;
          }*/

          for( int i=0; i<TimeslotSkipForTwoDelete_count; i++)
          {
            TimeslotSkipForTwoDelete[i][0] = 0;
            TimeslotSkipForTwoDelete[i][1] = 0;
          }  
          TimeslotSkipForTwoDelete_count = 0;


        }
        


        if( int(m_macTschPIBAttributes.m_macASN%incasn_slotframe_size) > 0 && int(m_macTschPIBAttributes.m_macASN%incasn_slotframe_size) <= DedicatedLink_count){
            def_MacTimeslotTemplate.m_macTsTimeslotLength = node_TimeslotLength[m_macTschPIBAttributes.m_macASN%incasn_slotframe_size];   // only for dedicated link
        }
        else{
            def_MacTimeslotTemplate.m_macTsTimeslotLength = 10000;
        }
    }


  }
  /*
  if(m_macTschPIBAttributes.m_macASN%slotframe_size_mac == 0 && this == dev[0] && m_macTschPIBAttributes.m_macASN!=0 && plan != 1){

    std::cout<< " plan3 Use time " << usetime_node_TimeslotLength << " for slotframe " << m_macTschPIBAttributes.m_macASN/slotframe_size_mac - 1 << std::endl;
    usetime_plan3[m_macTschPIBAttributes.m_macASN/slotframe_size_mac - 1] = usetime_node_TimeslotLength;
    usetime_node_TimeslotLength = 0;

  }
  if(this == dev[0] && plan != 1)
    usetime_node_TimeslotLength += node_TimeslotLength[m_macTschPIBAttributes.m_macASN%slotframe_size_mac];

  
  if(m_macTschPIBAttributes.m_macASN/slotframe_size_mac == 101 && this == dev[0] && plan != 1){

    for( int i=0; i<101; i++){
      std::cout<< " usetime_plan3 : " << usetime_plan3[i] << " usetime_plan4 : " << usetime_plan4[i] << " for slotframe " << i;
      if(usetime_plan3[i]>usetime_plan4[i])
        std::cout<< " 4 win " << std::endl;
      else
        std::cout<< " 3 win " << std::endl;
    }

  }*/

  /////////////////////////
  // 4.
  if( plan == 4 ){

    //if( m_macTschPIBAttributes.m_macASN%slotframe_size_mac == 0 || m_macTschPIBAttributes.m_macASN%slotframe_size_mac == 1 ){
    for ( int i=0; i<dev_count; i++ ){
      if( dev[i] == this ) {
        if( TimeslotLengthForAllocate[i][m_macTschPIBAttributes.m_macASN%slotframe_size_mac] == 0)  // no data traffic? 
        {
          def_MacTimeslotTemplate.m_macTsTimeslotLength = 10000;

        }
        else
          def_MacTimeslotTemplate.m_macTsTimeslotLength = TimeslotLengthForAllocate[i][m_macTschPIBAttributes.m_macASN%slotframe_size_mac];
      }
    }
  }

  //}
  if(SLA){
    if(ping6_TimeslotLength != 0)
      def_MacTimeslotTemplate.m_macTsTimeslotLength = ping6_TimeslotLength;
  }

  /////////////////////////


  NS_LOG_DEBUG("Changing random packet size TimeslotLength : " << def_MacTimeslotTemplate.m_macTsTimeslotLength 
              << " at ASN " << m_macTschPIBAttributes.m_macASN
              << " at ts " << m_macTschPIBAttributes.m_macASN%incasn_slotframe_size);
  

  Simulator::Schedule (MicroSeconds(def_MacTimeslotTemplate.m_macTsTimeslotLength),&LrWpanTschMac::IncAsn,this);
  currentLink.active = false;
  forset3 = 0; // add by ieong

  for (int i = 0; i < dev_count; i++)
  {
    if (this == dev[i])
    {
      if(UPA_processing[i]){
        NS_LOG_DEBUG(" breaking scheduling timeslot for UPA of TX ");
        return;
      }
    }
  }

  if(this==dev[0]){
    for(int i=0; i<dev_count; i++){
      if(UPA_processing[i]){
        NS_LOG_DEBUG(" breaking scheduling timeslot for UPA of RX ");
        return;
      }
    }
  }



  if (m_lrWpanMacState == TSCH_MAC_ACK_PENDING_END && consecutiveongoing == 0) // not impect to consceutive
  {
    // In last timeslot an ACK was expected, the PHY received something(BUSY_RX)
    // but it didn't succeed
    NS_LOG_DEBUG("A packet was received, but not the ack");
    m_macRxDataTxAckTrace(m_latestPacketSize);
    HandleTxFailure();
    Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState, this, TSCH_MAC_IDLE);
  }

  if (m_lrWpanMacState == TSCH_PKT_WAIT_END && consecutiveongoing == 0) // not impect to consceutive
  {
    // In last timeslot an packet was expected but wasn't received
    NS_LOG_DEBUG("A packet was received, but not the expected one");
    m_macRxDataTrace(m_latestPacketSize);
    Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState, this, TSCH_MAC_IDLE);
  }

  if (m_waitingLink)
    {
      Simulator::ScheduleNow(&LrWpanTschMac::MlmeSetLinkRequest,this,m_waitingLinkParams);
    }

  if ( consecutive_mode == 1){
    if( consecutive_Link_erasing_number != 0 && int(m_macTschPIBAttributes.m_macASN) == slotframe_size_mac ){
      NS_LOG_DEBUG("slotframe_size_mac : " << slotframe_size_mac );
      for ( int k=0; k<consecutive_Link_erasing_number; k++){
        for (std::list<MacPibLinkAttributes>::iterator it = m_macLinkTable.begin();it != m_macLinkTable.end();it++) {
          if( it->macTimeslot == consecutive_Link_erasing[k]){
            m_macLinkTable.erase(it);  // cancel link
            NS_LOG_DEBUG("Link erasing for timeslot : " << it->macTimeslot );
            break;
          }
        }
      }
      PrintingTimeslotTable();

      for (std::list<MacPibSlotframeAttributes>::iterator it = m_macSlotframeTable.begin();it != m_macSlotframeTable.end();it++) // cancel 1 slotframe in each devs
      {
        it->size -= consecutive_Link_erasing_number; 
        NS_LOG_DEBUG("slotframe : " << it->size);
        NS_LOG_DEBUG("consecutive_Link_erasing_number : " << consecutive_Link_erasing_number);
        std::cout << " Since slotframe size " << slotframe_size_mac;
        slotframe_size_mac = it->size;
      } 
      consecutive_Link_erasing_number = 0;
      //consecutive_cancel_timeslot = 0; 

      std::cout << " changed to slotframe size " << slotframe_size_mac << ", ASN " << m_macTschPIBAttributes.m_macASN;
      while( m_macTschPIBAttributes.m_macASN % slotframe_size_mac != 0){
        m_macTschPIBAttributes.m_macASN++;
      }
      std::cout << " change to ASN "  <<  m_macTschPIBAttributes.m_macASN << " to make start from ts = 0 " << std::endl;

    }

    



    for (std::list<MacPibSlotframeAttributes>::iterator it = m_macSlotframeTable.begin();it != m_macSlotframeTable.end();it++) // cancel slotframe in each devs
    {
      if( it->size != slotframe_size_mac )
      {
        it->size = slotframe_size_mac;
        NS_LOG_DEBUG("slotframe : " << it->size);
        while( m_macTschPIBAttributes.m_macASN % slotframe_size_mac != 0){  // make restart from ts=0
        m_macTschPIBAttributes.m_macASN++;
        }
      }
    
    }
  }
  
  
  /*
  // for process the ping6 packet
  // set next 2 slotframe will change to default timeslot length
  if( m_macTschPIBAttributes.m_macASN == ping6_ASN[ping6_ASN_count]-1){   // actually change at next slot, so need - 1
    def_MacTimeslotTemplate.m_macTsTimeslotLength = 10000;
    restore = m_macTschPIBAttributes.m_macASN + 2 * slotframe_size_mac;    
    NS_LOG_DEBUG("Changing default TimeslotLength : " << def_MacTimeslotTemplate.m_macTsTimeslotLength );

  }
  else if( m_macTschPIBAttributes.m_macASN == ping6_ASN[ping6_ASN_count] ){  // for next ping6 packet
    ping6_ASN_count++;
    NS_LOG_DEBUG("ping6_ASN_count : " << ping6_ASN_count );

  } 
  else if(  m_macTschPIBAttributes.m_macASN == restore-1 ) // after next 2 slotframe will go back to adaptive timeslot length
  {
    def_MacTimeslotTemplate.m_macTsTimeslotLength = TimeslotLength_mac;
    NS_LOG_DEBUG("def_MacTimeslotTemplate.m_macTsTimeslotLength : " << def_MacTimeslotTemplate.m_macTsTimeslotLength );

  }*/


  /*
  if( ping6_ChangeTimeslotLength == 1 ){   // want to process at last timeslot to make next slotfrmae change timeslot length
    uint16_t ts = m_macTschPIBAttributes.m_macASN % slotframe_size_mac;
    if ( ts != slotframe_size_mac-1 ){
      ASN_change = m_macTschPIBAttributes.m_macASN + slotframe_size_mac - ts - 1;  
    }
    else if ( ts == slotframe_size_mac-1 ){
      ASN_change = m_macTschPIBAttributes.m_macASN;
    }
    NS_LOG_DEBUG(" Plan to change at ASN " << ASN_change);
    
  }
  
  if( m_macTschPIBAttributes.m_macASN == ASN_change ){
    def_MacTimeslotTemplate.m_macTsTimeslotLength = 10000;
    restore = m_macTschPIBAttributes.m_macASN + 2 * slotframe_size_mac;    
    unlock = m_macTschPIBAttributes.m_macASN + slotframe_size_mac + 1; 
    NS_LOG_DEBUG("Changing default TimeslotLength : " << def_MacTimeslotTemplate.m_macTsTimeslotLength << " at ASN " << m_macTschPIBAttributes.m_macASN);
  }
  else if ( m_macTschPIBAttributes.m_macASN == unlock ){ // unlock at ts=0 of the last slotframe of default timeslot length
    ping6_header_round = 0;
    ping6_header = 0;
    //NS_LOG_DEBUG("unlock at ASN " << m_macTschPIBAttributes.m_macASN);
  }
  else if ( m_macTschPIBAttributes.m_macASN == restore ){
    def_MacTimeslotTemplate.m_macTsTimeslotLength = TimeslotLength_mac;
    NS_LOG_DEBUG("Changing back for packet size TimeslotLength : " << def_MacTimeslotTemplate.m_macTsTimeslotLength << " at ASN " << m_macTschPIBAttributes.m_macASN);
  }
  */


 
 
  for (std::list<MacPibSlotframeAttributes>::iterator it = m_macSlotframeTable.begin();it != m_macSlotframeTable.end();it++) 
    { 
      Simulator::ScheduleNow(&LrWpanTschMac::ScheduleTimeslot,this,it->slotframeHandle,it->size);
      // To display timeslot size  // ieong
      //std::printf("size %d\n",it->size); //change by ding
    }
}

void
LrWpanTschMac::SetMacCCAEnabled(bool cca)
{
  m_macCCAEnabled = cca;
}

bool
LrWpanTschMac::GetMacCCAEnables()
{
  return m_macCCAEnabled;
}

//could be changed here by ding
// Schedule the channle hopping of the slotframe with given handle and size
//宣告函式開始，設定輸入的參數為handle和size，其中handle代表時隙表的某個時間，size代表時隙表的大小。
//size = 35 default seems like not 35?
//size is increase while more node exist
void
LrWpanTschMac::ScheduleTimeslot(uint8_t handle, uint16_t size)
{
  // std::cout<<"num of node "<<num_node<<std::endl;
  //change by ding
  if(!change_size)
  {
    new_size=size;
  }
  if(change_size)
  {
    size=new_size;
    // std::cout<<"size after change"<<size<<std::endl;
  }
  // size=20; //change by ding
  // printf("in schedule timeslot %d\n",size);
  if(not_send_count>=10*num_node && size<2*num_node) //50
  {
    // std::cout<<"Increase"<<std::endl;
    // // std::cout<<"not send count before "<<not_send_count<<std::endl;
    // std::cout<<"size before "<<size<<std::endl;
    // size=new_size+1;
    // new_size=size;
    // // LrWpanTschHelper::ModSlotframe(netdev,0,size+1);
    // not_send_count=0;
    // change_size=true;
    // std::cout<<"size after "<<size<<std::endl;
    // // std::cout<<"not send count after "<<not_send_count<<std::endl;
    // NS_LOG_DEBUG("Increase slot frame size, size after change "<<size);
  }
  if (zero_queue_size>=10*num_node && size>num_node) //50
  {
    // std::cout<<"Decrease"<<std::endl;
    // std::cout<<"size before "<<size<<std::endl;
    // size=new_size-1;
    // new_size=size;
    // zero_queue_size=0;
    // change_size=true;
    // NS_LOG_DEBUG("Decrease slot frame size, size after change "<<size);
    // std::cout<<"size after "<<size<<std::endl;
  }
  
  uint16_t ts = m_macTschPIBAttributes.m_macASN%size;
  //std::cout << " size : " << int(size) << std::endl;
  //計算當前的時隙 (ts)，用當前的ASN除以size的餘數
  bool myts = false;
  m_currentReceivedPower = 0;
  NS_LOG_DEBUG("Timeslot " << m_macTschPIBAttributes.m_macASN << " ts = " << (int)ts << " Queue size = " << m_txQueueAllLink.size());

  //add by ieong 
  ///////////
  
  // unit timeslot length
  /*
    if( ping6_ChangeTimeslotLength == 1 )
      ping6_ChangeTimeslotLength = 0;
    if( ts == 0 ){
    max_pktsize = 0;
    ping6_header = 0;
  }
  else if ( ts== slotframe_size_mac - 1 )  // want to send changed header at last timeslot to modify timeslot length of next slotframe 
  {
    ping6_header = 1;  

  }*/
  ///////////
  // different timeslot length
  


  if(ts>=20) //change by ding
  {
  //   // SetCsmaCa (m_csmaCa);
  //   NS_LOG_UNCOND("in ts = 9");
  //   // m_csmaCa->Start ();
  //   Ptr<LrWpanNetDevice> dev0 = CreateObject<LrWpanNetDevice> ();
  //   Ptr<Packet> p0 = Create<Packet> (50);
  //   // McpsDataRequestParams params;
  //   TschMcpsDataRequestParams params;
  //   params.m_srcAddrMode = SHORT_ADDR;
  //   params.m_dstAddrMode = SHORT_ADDR;
  //   params.m_dstPanId = 0;
  //   params.m_dstAddr = Mac16Address ("00:00");
  //   params.m_msduHandle = 0;
  //   // params.m_txOptions = TX_OPTION_ACK;
    // McpsDataRequest (params, p0);
    // dev0->GetMac ()->McpsDataRequest (params, p0);
    // SetMacCCAEnabled(true);
    // m_sharedLink = true;
    
    // return;
    
    // csmacamode=true;
  }else
  {
    // csmacamode=false;
  }

  //add by ieong
  
  if( ts == 0 && forset == 0){
    //generate_packet();
    //PrintingTimeslotTable();
    //calculate_ping6_ASN();
    if( consecutive_mode == 1 )
      ConfirmConsecutiveLink();
    forset++;
  }
  

  
  for (std::list<MacPibLinkAttributes>::iterator it = m_macLinkTable.begin();it != m_macLinkTable.end();it++) {
    if (it->slotframeHandle == handle && it->macTimeslot == ts) {

      

      // macTimeslot 對返add link的timeslot
      //找到了符合的link,將myts設為true，並儲存該link的資訊到currentLink中
      myts = true;
      currentLink.slotframeHandle = handle;
      currentLink.linkHandle = it->macLinkHandle;
      currentLink.active = true;
      //std::cout << " first : " << firstforconsceutive << std::endl;
      //std::cout << " Isconsecutive : " << Isconsecutive << std::endl;
      //std::cout << " consecutiveongoing : " << consecutiveongoing << std::endl;
      //std::cout << " matrix : " << consecutive_matrix[it->macTimeslot][it->macChannelOffset] << std::endl;
      if ( consecutive_mode == 1){

        if( consecutive_matrix[it->macTimeslot][it->macChannelOffset] != 0 && firstforconsceutive == 0) // action for the timeslot already in consecutive
        {
          if( consecutive_macTxID == it->macTxID && consecutive_macRxID == it->macRxID )   // for consecutive的最後剛好下一個timeslot的其他link都是consecutive, 會跳到firstforconsceutive = 1
          {
            if( consecutiveongoing != 0 && Isconsecutive != 0 ){
              //std::cout << " The consecutive is on going " << std::endl;
              NS_LOG_DEBUG("The consecutive is on going");
              continue;
            }
            else if( consecutiveongoing != 0 && Isconsecutive == 0 ){
              //std::cout << " The last consecutive is on going" << std::endl;
              NS_LOG_DEBUG("The last consecutive is on going");
              continue;
            }
            else if( consecutiveongoing == 0 && Isconsecutive == 0 ){
              //std::cout << " The packet already sent or received in consecutive. It can turning off the radio " << std::endl;
              NS_LOG_DEBUG("The packet already sent or received in consecutive, turning off the radio");
              Simulator::ScheduleNow (&LrWpanPhy::PlmeSetTRXStateRequest,m_phy,IEEE_802_15_4_PHY_TRX_OFF);
              m_macSleepTrace (0);
              //consecutive_cancel_timeslot = 1;
              if( consecutive_Link_erasing_temp == 0){
                consecutive_Link_erasing[consecutive_Link_erasing_number] = it->macTimeslot;
                consecutive_Link_erasing_temp = consecutive_Link_erasing[consecutive_Link_erasing_number];
                consecutive_Link_erasing_number++;

              }
              else if( consecutive_Link_erasing_temp != it->macTimeslot){
                consecutive_Link_erasing[consecutive_Link_erasing_number] = it->macTimeslot;
                consecutive_Link_erasing_temp = consecutive_Link_erasing[consecutive_Link_erasing_number];
                consecutive_Link_erasing_number++;

              }

              NS_LOG_DEBUG("consecutive_Link_erasing[consecutive_Link_erasing_number] : " << consecutive_Link_erasing[consecutive_Link_erasing_number]);
              NS_LOG_DEBUG("consecutive_Link_erasing_number : " << consecutive_Link_erasing_number);
              continue;
            }
            else if ( consecutiveongoing == 0 && Isconsecutive != 0 ){
              //std::cout << " Error in consecutive " << std::endl;
              NS_LOG_DEBUG("Error in consecutive");
            }

          }
          else{
            NS_LOG_DEBUG("Other consecutive is coming");
          }
        }
        firstforconsceutive = 1; // each time is first unless consecutive is on going

      }
      


      NS_LOG_DEBUG("Link found at timeslot " << (int)ts);

      

      if (m_macHoppingEnabled)
        {
          
          //Get next channel
          m_currentChannel = def_MacChannelHopping.m_macHoppingSequenceList[
          (m_macTschPIBAttributes.m_macASN+it->macChannelOffset) % def_MacChannelHopping.m_macHoppingSequenceLength
          ];
          
          /*
          int channel_add = 0;
          for( int i=0; i<int(m_macTschPIBAttributes.m_macASN%slotframe_size_mac); i++ ){
            channel_add+=(link_mac[i]-1);
          }

          m_currentChannel = def_MacChannelHopping.m_macHoppingSequenceList[
          int((m_macTschPIBAttributes.m_macASN+it->macChannelOffset+channel_add) % def_MacChannelHopping.m_macHoppingSequenceLength)];   // add by ieong // ASN+link[timeslot-1]-1 % channellength
          */





          consecutive_macTxID = it->macTxID;
          consecutive_macRxID = it->macRxID;

          //Change channel
          NS_LOG_DEBUG("TSCH Changing to channel " << (int)m_currentChannel);
          LrWpanPhyPibAttributes *phyattr = new LrWpanPhyPibAttributes();
          phyattr->phyCurrentChannel = m_currentChannel;
          if (it->macLinkFadingBias != NULL){
              phyattr->phyLinkFadingBias = it->macLinkFadingBias[m_currentChannel-11] ;
          } else {
              phyattr->phyLinkFadingBias = 1;
	  }
	  NS_LOG_DEBUG (this << "setting for channel " << (int)m_currentChannel 
                << " fading bias: " <<
		phyattr->phyLinkFadingBias);
	  m_currentFadingBias = 10 * log10(phyattr->phyLinkFadingBias);
          Simulator::ScheduleNow (&LrWpanPhy::PlmeSetAttributeRequest,m_phy,phyCurrentChannel,phyattr);
        }

      


      if (it->macLinkOptions[0]) {
        //transmit
        if (it->macLinkOptions[2]) {
            m_sharedLink = true;
            NS_LOG_DEBUG("Be careful! Shared Link is Coming!");
        }
        else {
            m_sharedLink = false;
          }
        //if there is packets to be send and it is to the same addr as the link
        NS_LOG_DEBUG("Queue contained link size = " << m_txQueueAllLink.size());
        //當前時階的連接上有多少個待發送的數據包
        //change here by ding
        //if m_txQueueAllLink.size()==0 then count++
        if (m_txQueueAllLink.size()==0)
        {
          zero_queue_size++;
          NS_LOG_DEBUG("queue size is 0, total zero queue size "<<zero_queue_size);
        }


        //ping6_header = 0;
        /*if( m_macTschPIBAttributes.m_macASN%slotframe_size_mac == 0 ){
          for( int i=0; i<slotframe_size_mac;i++){
            next_packet_size[i] = 0;
            //node_TimeslotLength[i] = 10000;
          }
          next_packet_size_currentTimeslot = 0;
        }*/
        m_emptySlot = true;
        //if(it->macTimeslot == tsforcheck )  // same timeslot for current and previous
        //  next_packet_size_currentTimeslot--;  // next_packet_size_currentTimeslot need to same with previous
        next_packet_size_currentTimeslot = it->macTimeslot;
        current_TX = int(it->macTxID);
        current_RX = int(it->macRxID);
        current_Offset_list[current_Offset_totalcount] = int(it->macChannelOffset);
        current_Offset_totalcount++;
        current_Timeslot = int(it->macTimeslot);
        current_Offset = int(it->macChannelOffset);

        queue[int(it->macTxID)] = m_txQueueAllLink.size();
        m_txPkt = FindTxPacketInEmptySlot(it->macNodeAddr);
        if(SLA){
          uint16_t L = 3664 + (1000000 * (6 + int(m_txPkt->GetSize())) * 2 / 62500) + 300;
          if( L > ping6_TimeslotLength ){
            std::cout << " current packet timeslot length " << L << " is bigger than ping6_TimeslotLength " << ping6_TimeslotLength << std::endl;
            std::cout << " It is breaking and wiil not transmit. " << std::endl;
            break;
          }
        }



        //next_packet_size_currentTimeslot++;

        //tsforcheck = it->macTimeslot;

        NS_LOG_DEBUG ( "TX : " << current_TX );
        NS_LOG_DEBUG ( "Channel Offset : " << current_Offset );
        NS_LOG_DEBUG ( "Channel Offset Count : " << current_Offset_count );
        NS_LOG_DEBUG ( "Channel Offset totalCount : " << current_Offset_totalcount );

        NS_LOG_DEBUG ( "Timeslot : " << current_Timeslot );



        std::cout << " Get size of m_txPkt : " << m_txPkt->GetSize() << std::endl;
        //std::cout << " next_packet_size_currentTimeslot : " << next_packet_size_currentTimeslot << std::endl;

        if (!m_emptySlot) {
              
          //If an empty slot is found
              LrWpanMacHeader macHdr;
              m_txPkt->PeekHeader (macHdr);
              NS_LOG_DEBUG("Start timeslot transmiting procedure, seqnum = " << (int)macHdr.GetSeqNum());
              PacketSeqNum[int(it->macTxID)] = (int)macHdr.GetSeqNum();
              std::cout << " PacketSeqNum[int(it->macTxID)] : " << PacketSeqNum[int(it->macTxID)] << std::endl;
              //std::cout<< " macHdr.GetShortDstAddr() : " << macHdr.GetShortDstAddr() << std::endl;
              //std::cout<< " ping6_DstAddr[ping6_count] : " << ping6_DstAddr[ping6_count] << std::endl;
              //std::cout<< " macHdr.GetShortSrcAddr() : " << macHdr.GetShortSrcAddr() << std::endl;
              //std::cout<< " ping6_SrcAddr[ping6_count] " << ping6_SrcAddr[ping6_count] << std::endl;

              /*
              for ( int i=0; i<ping6_total; i++){

                if( macHdr.GetShortDstAddr() == ping6_DstAddr[i] && macHdr.GetShortSrcAddr() == ping6_SrcAddr[i] ){
                  if( (int)macHdr.GetSeqNum() == int(ping6_SeqNum[i] - 1) ){  // previous one packet call next slotframe change to default timeslot length

                    std::cout << " Find SeqNum : " << (int)macHdr.GetSeqNum() << std::endl;
                  
                    LrWpanMacHeader TempmacHdr;
                    LrWpanMacTrailer TempmacTrailer;

                    m_txPkt->RemoveHeader(TempmacHdr);
                    m_txPkt->RemoveTrailer(TempmacTrailer);

                    TempmacHdr.SetNextPacket(1);   
                    m_txPkt->AddHeader(TempmacHdr);

                    LrWpanMacTrailer macTrailer;
                    if (Node::ChecksumEnabled ())
                    {
                      macTrailer.EnableFcs (true);
                      macTrailer.SetFcs (m_txPkt);
                    }
                    m_txPkt->AddTrailer (macTrailer);  
                    //std::cout << "After m_txPkt : " << m_txPkt->GetSize() << std::endl;

                    //ping6_SeqNum[i] = 0;
                    //ping6_DstAddr[i] = 0;
                    //ping6_SrcAddr[i] = 0;

                    //ping6_count++;
                  }
                }
              }
              */


              ////////////////// unit timeslot length
              //if( ping6_header == 1 && ping6_header_round == 0){
              /*if( ping6_header == 1 ){

                LrWpanMacHeader TempmacHdr;
                LrWpanMacTrailer TempmacTrailer;

                m_txPkt->RemoveHeader(TempmacHdr);
                m_txPkt->RemoveTrailer(TempmacTrailer);

                TempmacHdr.SetNextPacket(max_pktsize);   // change ping6 header
                m_txPkt->AddHeader(TempmacHdr);

                LrWpanMacTrailer macTrailer;
                if (Node::ChecksumEnabled ())
                {
                  macTrailer.EnableFcs (true);
                  macTrailer.SetFcs (m_txPkt);
                }
                m_txPkt->AddTrailer (macTrailer);  
                std::cout<< " ping6 header changed " << std::endl;
                //ping6_header_round = 1;  // for control each slotframe only need 1 to call changing for next slotframe

              }*/
              ///////////////////
              
              LrWpanMacHeader TempmacHdr;
              LrWpanMacTrailer TempmacTrailer;

              m_txPkt->RemoveHeader(TempmacHdr);
              m_txPkt->RemoveTrailer(TempmacTrailer);

              TempmacHdr.SetNextPacket(next_packet_size);   // set header 
              if ( int(TempmacHdr.GetNextPacket()) != 0)
                std::cout << " Adding header : " << int(TempmacHdr.GetNextPacket()) << std::endl;
              else
                std::cout << " Header is 0. Not find next packet." << std::endl;
              m_txPkt->AddHeader(TempmacHdr);

              LrWpanMacTrailer macTrailer;
              if (Node::ChecksumEnabled ())
              {
                macTrailer.EnableFcs (true);
                macTrailer.SetFcs (m_txPkt);
              }
              m_txPkt->AddTrailer (macTrailer);  


              //std::cout<< " ping6 header changed " << std::endl;
              //ping6_header_round = 1;  // for control each slotframe only need 1 to call changing for next slotframe
              
              /////////////////////



              if ( consecutive_mode == 1){
                if( consecutive_matrix[it->macTimeslot][it->macChannelOffset] != 0 )  // set consecutive in the first timeslot of consecutive
                { 
                
                  int k = it->macTimeslot;
                  Isconsecutive = 0;
                  while(consecutive_matrix[k][it->macChannelOffset] != 0){
                    Isconsecutive++;
                    k++;
                  }
                  Isconsecutive--; // 3 consecutive 2 jump
                  //firstforconsceutive = 0;  // set it is not the first for consceutive for after timeslot
                  consecutiveongoing = 1;   // set it is consceutive ongoing
                  consecutive_addr = it->macNodeAddr;
                  std::cout << "Start consecutive in Timeslot " << it->macTimeslot << " and Channeloffset "<< it->macChannelOffset
                            << " for consecutive = " << Isconsecutive << std::endl;

                  m_macTxID = it->macTxID;
                  m_macRxID = it->macRxID;

                }
              }

          
              if(m_macCCAEnabled)
                {
                  Time time2wait = MicroSeconds(def_MacTimeslotTemplate.m_macTsCCAOffset);
                  Simulator::Schedule(time2wait, &LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_CCA);
                  m_lrWpanMacStatePending = TSCH_MAC_CCA;
                  Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);
                }
              else
                {
                  Time time2wait = MicroSeconds(def_MacTimeslotTemplate.m_macTsTxOffset);
                  Simulator::Schedule (time2wait,&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_SENDING);
                  m_lrWpanMacStatePending = TSCH_MAC_SENDING;
                  Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);
                }
                break;
        } else {
          //If an empty slot is not found
          m_macRxEmptyBufferTrace(0);
          not_send_count++;
          NS_LOG_DEBUG("Not sending, empty queue, total not send count "<<not_send_count);
          // std::cout<<"not send count "<<not_send_count<<std::endl;
          //m_macSleepTrace (0);
         }
      } else if (it->macLinkOptions[1]) {
        //receive
        NS_LOG_DEBUG("Start timeslot receiving procedure");
        Time time2wait = MicroSeconds(def_MacTimeslotTemplate.m_macTsRxOffset);
        Simulator::Schedule (time2wait,&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_RX);
        m_lrWpanMacStatePending = TSCH_MAC_RX;
        Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);
        }
      break;
    }
  }

  //If not involved in the current timeslot turn off the radio
  if (!myts)
    { 
      NS_LOG_DEBUG("No link in this timeslot, turning off the radio");
      Simulator::ScheduleNow (&LrWpanPhy::PlmeSetTRXStateRequest,m_phy,IEEE_802_15_4_PHY_TRX_OFF);

      m_macSleepTrace (0);
    }
}

void
LrWpanTschMac::WaitAck ()
{
  NS_LOG_FUNCTION(this);

  Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_ACK_PENDING);
  Simulator::Schedule(MicroSeconds(def_MacTimeslotTemplate.m_macTsAckWait),&LrWpanTschMac::AckWaitDone,this);

  uint32_t m_tempID = m_macTxID;
  m_macTxID = m_macRxID;
  m_macRxID = m_tempID;

}

void
LrWpanTschMac::AckWaitDone () {
  if (m_lrWpanMacState == TSCH_MAC_ACK_PENDING)
    {
      Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_ACK_PENDING_END);
    }
  else if (m_lrWpanMacState == TSCH_MAC_IDLE)
    {
      NS_LOG_DEBUG("ACK already received");
    }
  else
    {
      NS_LOG_DEBUG("Should never be here");
    }
}

void
LrWpanTschMac::RxWaitDone () {
  if (m_lrWpanMacState == TSCH_MAC_RX)
    {
      Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_PKT_WAIT_END);
    }
  else if (m_lrWpanMacState == TSCH_MAC_IDLE)
    {
      NS_LOG_DEBUG("Packet already received");
    }
  else
    {
      NS_LOG_DEBUG("Should never be here");
    }
}

void
LrWpanTschMac::ResetMacTschPibAttributes ()
{
  LrwpanMacTschPibAttributes def_MacTsch;
  
  def_MacTsch.macMinBE = 1;
  def_MacTsch.macMaxBE = 7;
  def_MacTsch.m_macDisconnectTime = 255;
  def_MacTsch.m_macJoinPriority = 1;
  def_MacTsch.m_macASN = -1;
  def_MacTsch.m_macNoHLBuffers = false;

  m_macTschPIBAttributes  = def_MacTsch;
}

//maybe can change here by ding
void
LrWpanTschMac::SetDefaultHoppingSequence(uint16_t sequenceLength)
{
  LrWpanMacChannelHopping chtmpl;

  chtmpl.m_macHoppingSequenceID = 0;

  //TODO: get those parameters from the PHY layer
  //and correctly calculate others
  chtmpl.m_macChannelPage = 0;
  chtmpl.m_macNumberOfChannels = 16;
  chtmpl.m_macPhyConfiguration = 8;
  chtmpl.m_macExtendedBitmap.clear();
  chtmpl.m_macHoppingSequenceList.clear();
  chtmpl.m_macHoppingSequenceList.resize(sequenceLength);

  //The SHUFFLE array was previously calculated according to the standard
  //(an LFSR of 9 taps with x^9+x^5+x^0 and seed 255)
  //here only the 30 first positions are populated

  std::vector<uint8_t> SHUFFLE(sequenceLength);
  uint16_t LFSR_OUTPUT[30] = {0x0FF,0x1FE,0x0DF,0x1BE,0x05F,0x0BE,0x17C,0x1DB,0x095,0x12A,
                              0x177,0x1CD,0x0B9,0x172,0x1C7,0x0AD,0x15A,0x197,0x00D,0x01A,0x034,0x068,0x0D0,0x1A0,0x063,0x0C6,0x18C,0x03B,0x076,0x0EC};

  //Currently all channels are used, but the list should be
  //obtained from the PHY
  //The monotonically increasing array currently increases until 26 and then
  //all subsequent positions are filled with 26

  //uint8_t start_ch = 11;  //defaulit set length = 16 , so there are 11~26. //ieong
  uint8_t start_ch = 12; //change to start at ch 12. ch11 will occur CCA failure in no reason? so there are 12~26

  for (unsigned char i = 0;i<sequenceLength;i++)
    {
       chtmpl.m_macHoppingSequenceList[i] = start_ch;
      if (start_ch < 26)
        {
          start_ch++;
        }
      SHUFFLE[i] = LFSR_OUTPUT[i] % sequenceLength;
    }

  for (unsigned char i = 0;i<sequenceLength;i++)
    {
      uint8_t aux = chtmpl.m_macHoppingSequenceList[i];
      chtmpl.m_macHoppingSequenceList[i] = chtmpl.m_macHoppingSequenceList[SHUFFLE[i]];
      chtmpl.m_macHoppingSequenceList[SHUFFLE[i]] = aux;
    }

  /*
  uint8_t uniq_ch = 15;
  for (unsigned char i = 0;i<sequenceLength;i++)
    {
       chtmpl.m_macHoppingSequenceList[i] = uniq_ch;
    }
*/

  chtmpl.m_macHoppingSequenceLength = sequenceLength;

  chtmpl.m_macCurrentHop = chtmpl.m_macHoppingSequenceList.begin();
  chtmpl.m_hopDwellTime = 0;

  def_MacChannelHopping = chtmpl;
  // ieong
  //NS_LOG_UNCOND("the length of the list of channel hopping : " << sequenceLength );

  //NS_LOG_UNCOND("Print the list of channel hopping : ");
  //for (unsigned char i = 0;i<def_MacChannelHopping.m_macHoppingSequenceLength;i++){
  //  NS_LOG_UNCOND( (int)def_MacChannelHopping.m_macHoppingSequenceList[i] );
  //}
  
}

void
LrWpanTschMac::PrintChannelHoppingList(std::ostream &os)
{
  os << "Channel hopping list for device " << m_shortAddress << std::endl;
  for (unsigned char i = 0; i < def_MacChannelHopping.m_macHoppingSequenceLength ; i++)
    {
      os << "CH[" << (int)i << "] = " << (int)def_MacChannelHopping.m_macHoppingSequenceList[i] << std::endl;
    }
}

void
LrWpanTschMac::SetHoppingSequence(std::vector<uint8_t> sequence, uint8_t id)
{
  LrWpanMacChannelHopping chtmpl;

  chtmpl.m_macHoppingSequenceID = id;
  chtmpl.m_macChannelPage = 0;
  chtmpl.m_macNumberOfChannels = 16;
  chtmpl.m_macPhyConfiguration = 8;
  chtmpl.m_macExtendedBitmap.clear();
  chtmpl.m_macHoppingSequenceList = sequence;
  chtmpl.m_macHoppingSequenceLength = sequence.size();
  chtmpl.m_macCurrentHop = chtmpl.m_macHoppingSequenceList.begin();
  chtmpl.m_hopDwellTime = 0;

  def_MacChannelHopping = chtmpl;
}

void
LrWpanTschMac::ResetMacTimeslotTemplate ()
{
  LrWpanMacTimeslotTemplate timeslottemplate;
  timeslottemplate.m_macTimeslotTemplateId = 0;
  timeslottemplate.m_macTsCCAOffset = 1800;
  timeslottemplate.m_macTsCCA = 128;
  timeslottemplate.m_macTsTxOffset = 2120;
  timeslottemplate.m_macTsRxOffset = 1120;
  timeslottemplate.m_macTsRxAckDelay = 800;
  timeslottemplate.m_macTsTxAckDelay = 1000;
  timeslottemplate.m_macTsRxWait = 2200;
  timeslottemplate.m_macTsAckWait = 400;
  timeslottemplate.m_macTsRxTx = 192; 
  timeslottemplate.m_macTsMaxAck = 2400;
  timeslottemplate.m_macTsMaxTx = 4256;
  timeslottemplate.m_macTsTimeslotLength = 10000;
  //timeslottemplate.m_macTsTimeslotLength = TimeslotLength_mac;
  



  //timeslottemplate.m_macTsCCAOffset = 3680;
  //timeslottemplate.m_macTsTxOffset = 4000;
  //timeslottemplate.m_macTsRxOffset = 2700;
  //timeslottemplate.m_macTsRxAckDelay = 4106;
  //timeslottemplate.m_macTsTxAckDelay = 4606;
  //  timeslottemplate.m_macTsRxWait = 2600;
  //  timeslottemplate.m_macTsAckWait = 1000;
  //  timeslottemplate.m_macTsTimeslotLength = 15000;

  def_MacTimeslotTemplate = timeslottemplate;
}

void
LrWpanTschMac::GetPhylinkInformation (double m_receivedPower)
{
  m_currentReceivedPower = m_receivedPower;
  m_macLinkInformation(m_macRxID, m_macTxID, m_currentChannel, m_currentReceivedPower, m_currentFadingBias);
}

Ptr<Packet>
LrWpanTschMac::FindTxPacketInEmptySlot (Mac16Address dstAddr)
{
   NS_LOG_FUNCTION(this);
   Ptr<Packet> TxPacket = Create<Packet> (0);
   m_txLinkSequence = 0;
   FindInQueue = 0;
   for (std::deque<TxQueueLinkElement*>::iterator i = m_txQueueAllLink.begin();i != m_txQueueAllLink.end();i++) {
       if ((*i)->txDstAddr == dstAddr){
           if(m_sharedLink)
            NS_LOG_DEBUG( " (*i)->txQueuePerLink.front()->txRequestCW : " << int((*i)->txQueuePerLink.front()->txRequestCW ));
           if (m_sharedLink  &&  ((*i)->txQueuePerLink.front()->txRequestCW != 0)){
             (*i)->txQueuePerLink.front()->txRequestCW = (*i)->txQueuePerLink.front()->txRequestCW - 1;
              NS_LOG_DEBUG("Find but cannot transmit packet in queue with link position:"<< m_txLinkSequence);
              FindInQueue = 1;
             }
           else{

                TxPacket = (*i)->txQueuePerLink.front()->txQPkt->Copy ();
                std::cout << "In Find TX TxPacket : " << TxPacket->GetSize() << std::endl;
                m_emptySlot = false;
                break;
             }
         }
       m_txLinkSequence++;
     }

   if (!m_emptySlot){
      NS_LOG_DEBUG("Find Tx packet in queue with link position = " << m_txLinkSequence <<" with queue size = "
                    << m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size());

      /*for ( int i=0; i<dev_count ; i++){
        NS_LOG_DEBUG( " pktlist for dev " << i << " : " << pktlist[i] );
      }*/

    }
   else{
       m_txLinkSequence = 0;
       NS_LOG_DEBUG("Fail to find Tx packet in queues. Empty slot confirmed");
       if(!FindInQueue){
        zero_queue_count[current_TX]++;
        NS_LOG_DEBUG("queue size is 0, total zero queue count "<< zero_queue_count[current_TX] );
       }
       else{
        zero_shared_queue_count[current_TX]++;
        NS_LOG_DEBUG("shared queue size is 0, total zero queue count "<< zero_shared_queue_count[current_TX] );
       }
        
        

       //NS_LOG_DEBUG(" with queue size = "<< m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size())
       //node_TimeslotLength[next_packet_size_currentTimeslot] = 10000;

      if( AddedLink_node[current_TX]>1 && w3){
        AddOrDelete = 2;
      }

    }

  node_TimeslotLength[next_packet_size_currentTimeslot] = 10000;   // retrun default first and set if have next packet for receive header after
  DCFL_TimeslotLength_mac[current_Timeslot][current_Offset] = 10000;
  next_packet_size = 0;

  // add by ieong
  if (!m_emptySlot){
    NS_LOG_DEBUG(" Link queue size = "<< m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size());

    if( int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) > 1 && 
        int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) > AddedLink_node[current_TX]*2 && w4)
    
    //if( int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) > AddedLink_node[current_TX]*2 && w4)
    {
      std::cout<< " come w4 for add first bit " << std::endl;
      if( add_link_count == 0 ){
        std::cout << " first bit is 1, added 128" << std::endl;
        next_packet_size += uint8_t(128);
      }
      else{
        for( int i=0; i<add_link_count; i++ ){
          if(add_link[i][2] == current_TX)   // // to prevent add 2 times in 1 period
          {
            std::cout<< add_link[i][2] << " added before " << std::endl;
            break;
          } 
          
          if(i == add_link_count-1)  // add if no addded
          {
            std::cout << " first bit is 1, added 128 " << std::endl;
            next_packet_size += uint8_t(128);
          }  
        }
      }
    }
    
    /*if( int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) > 1 && 
        int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) > AddedLink_node[current_TX]*2 && w4)
    {
      next_packet_size += uint8_t(128);
    }*/
   

      
    if(w4){
      std::cout<< " come w4 for find next packet size " << std::endl;

      TxQueueRequestElement *txQElement = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front();
      TxQueueRequestElement *txQElement1 = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.back();

      if( txQElement != txQElement1 ){  // check only one data or not
        int next_packet = 0;
        if( AddedLink_node[current_TX] == 0 ){
          next_packet = 1;  // next one
        }
        else 
        {
          next_packet = AddedLink_node[current_TX];
          std::cout << " AddedLink_node[current_TX] : " << AddedLink_node[current_TX] << std::endl;
        }
        
        TxQueueRequestElement *txQElement_array[next_packet+1]; 
        int k = 0;
        while ( k!= next_packet+1 ){
          //NS_LOG_DEBUG(" with queue size = "<< m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size());
          txQElement_array[k] = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front(); 
          m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.pop_front ();
          //NS_LOG_DEBUG(" after pop with queue size = "<< m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size());

          std::cout << " int(txQElement_array[k]->txQPkt->GetSize()) : " << int(txQElement_array[k]->txQPkt->GetSize()) << std::endl;
          txQElement1 = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.back();
          if( txQElement_array[k] == txQElement1 && k != next_packet){   // can't find next one
            std::cout<< " not find next pkt in findtx " << std::endl;
            break;
          }

          k++;
        }
        if( k== next_packet+1){  // if run all -> find next packet
          k--;
          next_packet_size += uint8_t(txQElement_array[k]->txQPkt->GetSize());
          if( next_packet_size > 128 )
            std::cout << " next packet size is " << int(next_packet_size)-128 << "+128" << std::endl;
          else
            std::cout << " next packet size is " << int(next_packet_size) << std::endl;
        }
        while( k>=0 ){
          m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.push_front(txQElement_array[k]);  // push the front back
          txQElement_array[k] = NULL;
          k--;
        }
        txQElement = NULL;
        txQElement1 = NULL;
      }
    }

    


    if( int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) > AddedLink_node[current_TX]*2 && w3)
    {
      AddOrDelete = 1;

    }
    else if( int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) < AddedLink_node[current_TX]*2 && AddedLink_node[current_TX]>1 && w3){
      AddOrDelete = 2;
    }

    if( (plan == 2 || plan == 3) && (!w4) ){

      std::cout<< " come w1 " << std::endl;

      TxQueueRequestElement *txQElement = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front();
      TxQueueRequestElement *txQElement1 = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.back();

      if( txQElement != txQElement1 ){  // check only one data or not

        TxQueueRequestElement *txQElement_array[pktlist[current_TX]+1]; 
        int k = 0;
        while ( k!= pktlist[current_TX]+1 ){
          //NS_LOG_DEBUG(" with queue size = "<< m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size());
          txQElement_array[k] = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front(); 
          m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.pop_front ();
          //NS_LOG_DEBUG(" after pop with queue size = "<< m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size());

          std::cout << " int(txQElement_array[k]->txQPkt->GetSize()) : " << int(txQElement_array[k]->txQPkt->GetSize()) << std::endl;
          txQElement1 = m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.back();
          if( txQElement_array[k] == txQElement1 ){   // can't find next one
            std::cout<< " not find next pkt in findtx " << std::endl;
            break;
          }

          k++;
        }
        if( k== pktlist[current_TX]+1){  // if run all -> find next packet
          k--;
          next_packet_size += uint8_t(txQElement_array[k]->txQPkt->GetSize());
          std::cout << " next packet size is " << int(next_packet_size) << std::endl;

        }
        while( k>=0 ){
          m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.push_front(txQElement_array[k]);  // push the front back
          txQElement_array[k] = NULL;
          k--;
        }

        txQElement = NULL;
        txQElement1 = NULL;

      }
    }

    if(e_TSCH_Orch){
      if( e_TSCH_Orch_next_slotframe_queue[current_TX] == 1 ){   // if not, before link have process
        e_TSCH_Orch_next_slotframe_queue[current_TX] = int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) - e_TSCH_Orch_AddedLink[current_TX];
        if( e_TSCH_Orch_next_slotframe_queue[current_TX] < 1)
          e_TSCH_Orch_next_slotframe_queue[current_TX] = 1;
      }
    }

    if (UPA)
    { // find queue for each sending node
      if (!UPA_processing[current_TX])  // after processing, should not update the queue
      {
        if (int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) > 1)
        {
          UPA_queue[current_TX] = int(m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.size()) - 1; // 1 for current transmit
        }
      }
    }
  }
  


   return TxPacket;
}

void
LrWpanTschMac::HandleTxFailure ()
{
  if (m_sharedLink){
      NS_LOG_DEBUG("Shared Link Failure!");
      if (m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front()->txRequestNB > 0
          && m_txQueueAllLink[m_txLinkSequence]->txLinkBE < m_macTschPIBAttributes.macMaxBE){
            //m_txQueueAllLink[m_txLinkSequence]->txLinkBE++;  // orgin  // exp2 too long 
            m_txQueueAllLink[m_txLinkSequence]->txLinkBE = 1;
        }

      uint8_t txBE = m_txQueueAllLink[m_txLinkSequence]->txLinkBE;
      NS_LOG_DEBUG("Backoff exponent for this shared link is:"<< (int)txBE);

      uint8_t upperBound = (uint8_t) pow (2, txBE) - 1;
      m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front()->txRequestCW  = (uint8_t)m_random->GetInteger (0, upperBound);
      NS_LOG_DEBUG("Backoff timeslots for this request in the shared link is:"
                   << (int)m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front()->txRequestCW);

    }

  m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front()->txRequestNB++;
  NS_LOG_DEBUG ("Increment Retries for the top packet in the queue with link position = "<< m_txLinkSequence);

  if (m_txQueueAllLink[m_txLinkSequence]->txQueuePerLink.front()->txRequestNB == m_macMaxFrameRetries){
      RemoveTxQueueElement();
    }
}



void
LrWpanTschMac::generate_packet()   // add by ieong
{

  m_random = CreateObject<UniformRandomVariable> ();
  int random_generate_packet = 0;


  for( int i=1; i<num_node; i++){
    random_generate_packet = (uint8_t)m_random->GetInteger (0, 2);
    mac_packetlist[i] = 1 + random_generate_packet;
    mac_Totalpacket += mac_packetlist[i];
    std::cout<< mac_packetlist[i]<< " ";
  }
  std::cout << std::endl;
  std::cout<< "mac_Totalpacket : "<< mac_Totalpacket << std::endl;
  
  //scheduling();


}



void
LrWpanTschMac::Sendmatrixtomac (std::vector<std::vector<int> >& matrix)  //add by ieong
{

  mac_adjacency_matrix = matrix;
  
  std::cout<< "In Sendmatrixtomac :" << std::endl;
  for ( int i=0; i<static_cast<int>(mac_adjacency_matrix[0].size()); i++){
    for( int j=0; j<static_cast<int>(mac_adjacency_matrix[0].size()); j++){
        std::cout<< mac_adjacency_matrix[i][j] << " ";

    }
    std::cout<<std::endl;
  }

  


}



void
LrWpanTschMac::scheduling()   // add by ieong
{

  int timeslot = 0;
  std::vector<int> running(99);
  //int running[99] = {0};
  std::vector<int> link(99);
  //int link[99] = {0};
  std::vector<std::vector<std::vector<int> > > DCFL(99, std::vector<std::vector<int> >(16, std::vector<int>(2)));
  //int DCFL[99][16][2] = {0};
  std::vector<int> childofsink(99);
  //int childofsink[99] = {0};
  int k = 0, k1 = 0;
  int sinkhavedata = 0;
  for (int i=1; i<num_node; i++){
    if(mac_adjacency_matrix[0][i] == 1)
    {
      childofsink[k] = i;
      k++;
    }
  }
  std::cout<< "k : " << k << std::endl;


  while(mac_packetlist[0] != mac_Totalpacket){

    
    sinkhavedata = 0;

    for (int i=0; i<num_node; i++)
      running[i] = 0;

    for( int i=num_node; i>=1; i--){
        
      k1 = 0;

      if(running[i] == 1 || mac_packetlist[i] == 0)
        continue;

      if( sinkhavedata == 0 ){
        for( int m=0; m<k ; m++){
          if( running[childofsink[m]] == 1 || mac_packetlist[childofsink[m]] == 0)
          {
            k1++;
          }  
        }

      }
      

      if( k1 == k - 1){
        for( int m=0; m<=k ; m++){
          
          if( running[childofsink[m]] == 0 && mac_packetlist[childofsink[m]] != 0){
            DCFL[timeslot][link[timeslot]][0] = childofsink[m];
            DCFL[timeslot][link[timeslot]][1] = 0;
            link[timeslot]++;

            mac_packetlist[childofsink[m]]--;
            mac_packetlist[0]++;

            running[childofsink[m]] = 1;
            running[0] = 1;

            sinkhavedata = 1;

            break;

          }
        }
      }
        

      for(int j=i-1; j>=0; j--){
          
        if( mac_adjacency_matrix[i][j] == 1 && running[j] != 1){

          DCFL[timeslot][link[timeslot]][0] = i;
          DCFL[timeslot][link[timeslot]][1] = j;
          link[timeslot]++;

          mac_packetlist[i]--;
          mac_packetlist[j]++;

          running[i] = 1;
          running[j] = 1;

          break;

        }


      }
    }
    //std::cout<< "packet of root in timeslot " << timeslot <<" is "<<mac_packetlist[0]<< std::endl;
    for( int x=0; x<num_node ;x++){
      std::cout<< mac_packetlist[x] << " ";
    }
    std::cout<<std::endl;
    timeslot ++;
  }    

  
  for( int i=0; i<timeslot ;i++){

    std::cout<< " In timeslot : " << i << std::endl;
    std::cout<< "link[timeslot] : "<< link[i]<<std::endl;
    for( int j=0; j<link[i]; j++){

      std::cout<< DCFL[i][j][0] << " to " << DCFL[i][j][1] << std::endl;

    }

  }

  //ChangingTimeslotTable(DCFL, timeslot, link );

}



/*
// add by ieong
void
LrWpanTschMac::ChangingTimeslotTable(std::vector<std::vector<std::vector<int> > >& DCFL, int timeslot, std::vector<int> link)
{

  int num_link = 1;

  for(std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin()+1;i != m_macLinkTable.end();i++)  
  {
    m_macLinkTable.erase(i);  // empty the LinkTable except the first advertise link
  }


  for( int i=0; i<timeslot ;i++){

    std::cout<< " In timeslot : " << i << std::endl;
    std::cout<< "link[timeslot] : "<< link[i]<<std::endl;
    for( int j=0; j<link[i]; j++){

      std::cout<< DCFL[i][j][0] << " to " << DCFL[i][j][1] << std::endl;

      MacPibLinkAttributes entry;

      entry.macLinkHandle = num_link;
      num_link++;
      entry.macLinkOptions.reset();
      entry.macLinkOptions.set(0,1);
      entry.macLinkType = MlmeSetLinkRequestlinkType_NORMAL;
      entry.slotframeHandle = 0;

      entry.macNodeAddr = Mac16Address::ConvertFrom(devs.Get(dstPos)->GetAddress());

      entry.macTimeslot = i+1;
      entry.macChannelOffset = j;
      entry.linkFadingBias = FadingBias[DCFL[i][j][1]][DCFL[i][j][0]];
      entry.macTxID = DCFL[i][j][0];
      entry.macRxID = DCFL[i][j][1];

      m_macLinkTable.push_back(entry);


    }

  }

  



  for(std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin()+1;i != m_macLinkTable.end();i++)  
  {


  }

} 

*/

// add by ieong
void
LrWpanTschMac::PrintingTimeslotTable()
{
  /*
  int max_offset = 0;
  for(std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++)
  {
    if(i->Channeloffset > max_offset)
      max_offset = i->Channeloffset;
  }*/

  std::cout<< "PrintingTimeslotTable : " << std::endl;
  for(std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++)
  {

    //std::cout << " 111 " << std::endl;
    if( int(i->macTxID) == 0 && int(i->macRxID) == 0 ){
      std::cout<< "In Timeslot " << i->macTimeslot << ", " << i->macTxID << "->" << "all" 
      << " with channel offset " << i->macChannelOffset << std::endl;
    }
    else{
      std::cout<< "In Timeslot " << i->macTimeslot << ", " << i->macTxID << "->" << i->macRxID 
      << " with channel offset " << i->macChannelOffset << std::endl;
    }

  
    

    /*
    std::cout<< "macLinkHandle : " << i->macLinkHandle << std::endl;
    std::cout<< "slotframeHandle : " << (int)i->slotframeHandle << std::endl;
    std::cout<< "macLinkOptions : " << i->macLinkOptions << std::endl;
    std::cout<< "macLinkType : " << i->macLinkType << std::endl;
    std::cout<< "macNodeAddr : " << i->macNodeAddr << std::endl;
    std::cout<< "macTimeslot : " << i->macTimeslot << std::endl;
    std::cout<< "macChannelOffset : " << i->macChannelOffset << std::endl;
    std::cout<< "macLinkFadingBias : " << i->macLinkFadingBias << std::endl;
    std::cout<< "macTxID : " << i->macTxID << std::endl;
    std::cout<< "macRxID : " << i->macRxID << std::endl;
    */


    /*
    i->macLinkOptions = params.linkOptions; //b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.
    i->macLinkType = params.linkType;
    i->macNodeAddr = params.nodeAddr; //not using Mac16_Address, 0xffff means the link can be used for frames destined for the broadcast address
    i->macTimeslot = params.Timeslot; //refer to 5.1.1.5
    i->macChannelOffset = params.ChannelOffset; //refer to 5.1.1.5.3
    i->macLinkFadingBias = params.linkFadingBias;
    i->macTxID = params.TxID;
    i->macRxID = params.RxID;


    entry.macLinkHandle = params.linkHandle;
    entry.macLinkOptions = params.linkOptions; //b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.
    entry.macLinkType = params.linkType;
    entry.slotframeHandle = params.slotframeHandle;
    entry.macNodeAddr = params.nodeAddr; //not using Mac16_Address because 0xffff means the link can be used for frames destined for the boradcast address
    entry.macTimeslot = params.Timeslot; //refer to 5.1.1.5
    entry.macChannelOffset = params.ChannelOffset; //refer to 5.1.1.5.3
    entry.macLinkFadingBias = params.linkFadingBias;
    entry.macTxID = params.TxID;
    entry.macRxID = params.RxID;
    */
  }


  int b = 0;
  int k = 0;
  for( std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end(); i++,k++ ){
    if( i->macChannelOffset > b )
      b = i->macChannelOffset;
    if( k<=9){
      std::cout.width(8);
      std::cout << "ts=" << k;
    }
    else{
      std::cout.width(7);
      std::cout << "ts=" << k;
    }

  }
  std::cout << std::endl;
  
  std::ostringstream oss;
  for( int j=0; j<=b; j++){
    for( std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){
      if( i->macChannelOffset == j ){
        if( int(i->macTxID) == 0 && int(i->macRxID) == 0)
        {
          oss.str("");
          oss << " " << i->macTxID << "->" << "all";
          std::cout.width(9);
          std::cout<< oss.str();
        }
        else{
          oss.str("");
          oss << " " << i->macTxID << "->" << i->macRxID;
          std::cout.width(9);
          std::cout<< oss.str();
        }

      }
      else{
        oss.str("");
        std::cout.width(9);
        std::cout<< " " << oss.str();
      }
    }
    std::cout << std::endl;

  }
  std::cout << std::endl;

  


} 

// add by ieong
void
LrWpanTschMac::ConfirmConsecutiveLink()
{
  int b1 = 0;
  for( std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){
    if( i->macChannelOffset > b1 )
      b1 = i->macChannelOffset;
  }
  int consecutive = 0;
  int first = 1;
  std::list<MacPibLinkAttributes>::iterator prev;
  for( int j=0; j<=b1; j++){
    consecutive = 0;
    //end = 0;
    first = 1;
    for( std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){
      
      if( i->macChannelOffset == j ){
        if( first ){
          prev = i;
          first = 0;
          std::cout << " first in " << std::endl;
          continue;
        }
        if( prev->macTxID == i->macTxID && prev->macRxID == i->macRxID && prev->macTimeslot == i->macTimeslot - 1 ){ 

          //end = i->macTimeslot;  
          consecutive++;

          if( i->macTimeslot != m_macLinkTable.size()-1 ){
            prev = i;
            continue;
          }
            
        }
        while ( consecutive != 0 ){
          consecutive_matrix[i->macTimeslot-consecutive][j] = 1;
          consecutive--;
          if( consecutive == 0)
            consecutive_matrix[i->macTimeslot][j] = 1;

        }
        prev = i;
      }
    }
  }

}

// add by ieong
void
LrWpanTschMac::ConsecutiveHandle()
{
  
  /*
  m_txPkt = FindTxPacketInEmptySlot(it->macNodeAddr);

  Time time2wait = MicroSeconds(def_MacTimeslotTemplate.m_macTsRxTx);
  Simulator::Schedule (time2wait,&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_SENDING);
  m_lrWpanMacStatePending = TSCH_MAC_SENDING;
  Simulator::ScheduleNow(&LrWpanTschMac::SetLrWpanMacState,this,TSCH_MAC_IDLE);
  */


}

void
LrWpanTschMac::calculate_ping6_ASN(double TimeForGenerate)
{
  double packet_generate = 0;
  double ping6_interval = 0.1* nrnodes;

  for ( int i=0; i<99; i++ ){
    // base on ping6 interval to calculation how much packet have generated 
    // start at 1 since ping6 start at 6s // 6-5 = 1 for generate packet for 1 second
    packet_generate = ( TimeForGenerate + i * ping6_interval ) / generate_interval - 1 + i;  
    if( packet_generate > int(packet_generate)){
      packet_generate = int(packet_generate) + 1;
      std::cout << "packet_generate : " << packet_generate << std::endl;
    }
    if(packet_generate < 0 )
      packet_generate = 1;

    ping6_ASN[i] = packet_generate * slotframe_size_mac;
    std::cout<< "ping6_ASN[i] : " << ping6_ASN[i] << " slotframe_size_mac : " 
    << slotframe_size_mac << " i : " << i  << " packet_generate : " << packet_generate <<std::endl;

    if ( ping6_ASN[i] * TimeslotLength_mac / 1000000 >= 25 ){
      std::cout << "stop at " << ping6_ASN[i] * TimeslotLength_mac / 1000000 << std::endl;
      ping6_ASN[i] = 0;
      break;
    }

  }

}

void 
LrWpanTschMac::SendConfigurationToMac(double TimeslotLength, int pktsize, double interval, std::vector<std::vector<std::vector<int> > >& DCFL,
                                      int TimeslotForDCFL, std::vector<int>& link, ApplicationContainer apps)
{

  TimeslotLength_mac = TimeslotLength;
  def_MacTimeslotTemplate.m_macTsTimeslotLength = TimeslotLength_mac;
  pktsize_mac = pktsize;
  generate_interval = interval;
  DCFL_mac = DCFL;
  TimeslotForDCFL_mac = TimeslotForDCFL;
  link_mac = link;
  max_link_mac = link_mac[0];
  for ( int i=1; i<16; i++){
    if( link_mac[i] > max_link_mac)
      max_link_mac = link_mac[i];
  }
  Ping6apps = apps;




  /*
  std::cout<< " TimeslotLength_mac : " << TimeslotLength_mac
           << " pktsize_mac : " << pktsize_mac
           << " generate_interval : " << generate_interval
           << std::endl;

  std::cout << " Scheduling Table : " << std::endl;

  for( int i=0; i<TimeslotForDCFL_mac; i++){
    
    std::cout.width(8);
    std::cout<< i+1 << "";

  }
  std::cout << std::endl;
  

  std::ostringstream oss;
  for( int j=0; j<16; j++){
    for( int i=0; i<TimeslotForDCFL_mac; i++){
      if( link_mac[i] > j){
        oss.str("");
        oss << DCFL_mac[i][j][0] << "->" << DCFL_mac[i][j][1];
        std::cout.width(8);
        std::cout<< oss.str() << "";
      }
      else{
        oss.str("");
        std::cout.width(8);
        std::cout<< oss.str() << "";
      }

    }
    std::cout << std::endl;
  }
  */

  for( int j=0; j<max_link_mac; j++){
    for( int i=0; i<TimeslotForDCFL_mac; i++){
      if( link_mac[i] > j){
        if( DCFL_mac[i][j][0] == 0 && DCFL_mac[i][j][1] == 0 )
          continue;
        DCFL_TimeslotLength_mac[i][j] = 10000;
      }
    }
  }

  


}


void 
LrWpanTschMac::TimeslotLength_allocate()
{

  std::vector<int> max_DCFL_TimeslotLength_mac(TimeslotForDCFL_mac,0);
  std::vector<int> min_DCFL_TimeslotLength_mac(TimeslotForDCFL_mac,10000);
  std::vector<int> sumofmax_DCFL_TimeslotLength_mac(TimeslotForDCFL_mac,0);
  std::vector<std::vector<int> > sumlength_node(nrnodes+1, std::vector<int>(TimeslotForDCFL_mac, 0));


  //uint16_t maxlength[TimeslotForDCFL_mac] = {0};
  
  // clear
  for( int i=0; i<nrnodes+1; i++){
    for(int j=0; j<TimeslotForDCFL_mac; j++){
      TimeslotLengthForAllocate[i][j] = 0;
    }
  }


  // basic for link to node of timeslot length
  std::cout << "for basic " << std::endl;
  for( int i=0; i<TimeslotForDCFL_mac; i++){
    for( int j=0; j<link_mac[i]; j++){
      if( DCFL_mac[i][j][0] == 0 && DCFL_mac[i][j][1] == 0 )
        continue;
      TimeslotLengthForAllocate[DCFL_mac[i][j][0]][i] = DCFL_TimeslotLength_mac[i][j];
      TimeslotLengthForAllocate[DCFL_mac[i][j][1]][i] = DCFL_TimeslotLength_mac[i][j];
      //std::cout << " DCFL_TimeslotLength_mac[i][j] : " << DCFL_TimeslotLength_mac[i][j] << std::endl;

      if( DCFL_TimeslotLength_mac[i][j] > max_DCFL_TimeslotLength_mac[i] )
        max_DCFL_TimeslotLength_mac[i] = DCFL_TimeslotLength_mac[i][j];
      if( DCFL_TimeslotLength_mac[i][j] < min_DCFL_TimeslotLength_mac[i] )
        min_DCFL_TimeslotLength_mac[i] = DCFL_TimeslotLength_mac[i][j];

      
      
    }
    if( i==0 ){
      sumofmax_DCFL_TimeslotLength_mac[i] = max_DCFL_TimeslotLength_mac[i];
    }
    else{
      sumofmax_DCFL_TimeslotLength_mac[i] = (sumofmax_DCFL_TimeslotLength_mac[i-1] + max_DCFL_TimeslotLength_mac[i]);
      
    }
    //std::cout << " max_DCFL_TimeslotLength_mac[i] : " << max_DCFL_TimeslotLength_mac[i] << std::endl;
  }


  for( int i=0; i<nrnodes+1; i++){

    std::cout<< i << " ";

    for(int j=0; j<TimeslotForDCFL_mac; j++){
      std::cout.width(8);
      std::cout<< TimeslotLengthForAllocate[i][j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;







  for( int i=0; i<TimeslotForDCFL_mac; i++){

     // for timeslot length 0 (no link nodes in current timeslot and next timeslot will follow max timeslot length)
    for( int j=0; j<nrnodes+1; j++){
      if( TimeslotLengthForAllocate[j][i] == 0 ){
        TimeslotLengthForAllocate[j][i] = min_DCFL_TimeslotLength_mac[i];
      }
    }

    if( i>0 ){
      // for conflict problem     
      for( int j=0; j<link_mac[i]; j++){
        if( DCFL_mac[i][j][0] == 0 && DCFL_mac[i][j][1] == 0 )
          continue;
        int k = i-1;
        int sum1 = 0;
        int sum2 = 0;
        while(k>=0){
          sum1+=TimeslotLengthForAllocate[DCFL_mac[i][j][0]][k];
          sum2+=TimeslotLengthForAllocate[DCFL_mac[i][j][1]][k];
          k--; 
          //std::cout << " sum1 :" << sum1 
          //        << " sum2 :" << sum2 
          //        << " at i : " << i << " j : " << j << std::endl;

        }
        if( sum1 > sum2 ){
          TimeslotLengthForAllocate[DCFL_mac[i][j][1]][i-1] += (sum1 - sum2);
          sumlength_node[DCFL_mac[i][j][1]][i-1] += (sum1 - sum2);
        }
        else{
          TimeslotLengthForAllocate[DCFL_mac[i][j][0]][i-1] += (sum2 - sum1);
          sumlength_node[DCFL_mac[i][j][0]][i-1] += (sum2 - sum1);

        }
      
      }
      // for channel problem
      for( int j=0; j<link_mac[i]; j++){
        if( DCFL_mac[i][j][0] == 0 && DCFL_mac[i][j][1] == 0 )
          continue;
        int k = i-1;
        int sum1 = 0;
        int sum2 = 0;
        int sum3 = 0;
        while(k>=0){
          sum1+=TimeslotLengthForAllocate[DCFL_mac[i][j][0]][k];
          sum2+=TimeslotLengthForAllocate[DCFL_mac[i][j][1]][k];
          sum3+=TimeslotLengthForAllocate[DCFL_mac[i-1][j+1][0]][k];
          k--; 
          //std::cout << " sum1 :" << sum1 
          //        << " sum2 :" << sum2 
          //        << " at i : " << i << " j : " << j << std::endl;

        }
        if( sum1 < sum3 ){
          TimeslotLengthForAllocate[DCFL_mac[i][j][0]][i-1] += (sum3 - sum1);
          sumlength_node[DCFL_mac[i][j][0]][i-1] += (sum3 - sum1);

        }
        if( sum2 < sum3 ){
          TimeslotLengthForAllocate[DCFL_mac[i][j][1]][i-1] += (sum3 - sum2);
          sumlength_node[DCFL_mac[i][j][1]][i-1] += (sum3 - sum2);

        }
      }


    }

    int max = 0;
    for( int j=0; j<nrnodes+1; j++){
      if( i==0 ){
        sumlength_node[j][i] =  TimeslotLengthForAllocate[j][i];
      }
      else{
        sumlength_node[j][i] = (sumlength_node[j][i-1]+TimeslotLengthForAllocate[j][i]);
      }
      if( sumlength_node[j][i] > max ){
        max = sumlength_node[j][i];
      }
    }

    // for sync
    if( link_mac[i] == 1 || i == TimeslotForDCFL_mac-1 ){
      for( int j=0; j<nrnodes+1; j++){
        if(  sumlength_node[j][i] < max ){
          TimeslotLengthForAllocate[j][i] += ( max - sumlength_node[j][i] );
          sumlength_node[j][i] = max;
        }
      }
    }

  }


// print
  std::cout.width(2);
  std::cout<< "N" << " ";
  for(int j=0; j<TimeslotForDCFL_mac; j++){
    std::cout.width(8);
    std::cout<< j << " ";
    std::cout.width(8);
    std::cout<<  "sum" << " ";

  }
  std::cout << std::endl;

  std::cout.width(2);
  std::cout<< "X" << " ";
  for(int j=0; j<TimeslotForDCFL_mac; j++){
    std::cout.width(8);
    std::cout<< max_DCFL_TimeslotLength_mac[j] << " ";
    std::cout.width(8);
      std::cout<< sumofmax_DCFL_TimeslotLength_mac[j] <<" ";
  }
  std::cout << std::endl;


  for( int i=0; i<nrnodes+1; i++){
    std::cout.width(2);
    std::cout<< i << " ";

    for(int j=0; j<TimeslotForDCFL_mac; j++){
      std::cout.width(8);
      std::cout<< TimeslotLengthForAllocate[i][j] << " ";
      std::cout.width(8);
      std::cout<< sumlength_node[i][j] << " ";

    }
    std::cout << std::endl;
  }

  std::cout<< " Use time " << sumlength_node[0][TimeslotForDCFL_mac-1] << " for slotframe " << m_macTschPIBAttributes.m_macASN/slotframe_size_mac << std::endl;
  usetime_plan4[m_macTschPIBAttributes.m_macASN/slotframe_size_mac] = sumlength_node[0][TimeslotForDCFL_mac-1];
}

void 
LrWpanTschMac::Initial()
{
  //static std::vector<std::vector<std::vector<int> > > scheduling_table(slotframe_size_mac, std::vector<std::vector<int> >(15, std::vector<int>(2,0))); 

}

void 
LrWpanTschMac::print_scheduling_table()
{
  std::cout << " Scheduling Table : " << std::endl;
  std::cout.width(2);
  std::cout<< "    " << " ";
  for( int i=0; i<slotframe_size_mac; i++){
    std::cout.width(8);
    std::cout<< i << "";
  }
  std::cout << std::endl;
  std::ostringstream oss;


  for( int j=0; j<15; j++){
    for( int p=0; p<3; p++){
      if( p == 0 ){
        std::cout.width(2);
        std::cout<< j << " D" << " ";
      }
      else{
        std::cout.width(2);
        std::cout<< "   S" << " ";
      }

      for( int i=0; i<slotframe_size_mac; i++){
        if(scheduling_table[i][j][p][0] != -1 || scheduling_table[i][j][p][1] != -1 )
        {
          oss.str("");
          oss << scheduling_table[i][j][p][0] << "->" << scheduling_table[i][j][p][1];
          std::cout.width(8);
          std::cout<< oss.str() << "";
        }
        else{
          oss.str("");
          std::cout.width(8);
          std::cout<< oss.str() << "";
        }
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;

  }

    
}

void 
LrWpanTschMac::add_process()
{
  adding_slotframe_size_mac = 1;
  for (std::list<MacPibSlotframeAttributes>::iterator s = m_macSlotframeTable.begin(); s != m_macSlotframeTable.end();s++)
  {
    for( int k=0; k<add_link_count; k++){
        
      int bias = 0;   // for timeslot bias
      for ( int m=0; m<k; m++){
        if( add_link[m][0] < add_link[k][0] )
          if( directly_change_add[m] != 1 )
            bias++;
      }
      add_link[k][0] += bias;
      std::cout << " bias : " << bias << " Come for ts " << add_link[k][0] << " offset " << add_link[k][1] << " " << add_link[k][2] << "->" << add_link[k][3] << std::endl;

      // check the shared link is only one in the timeslot or not
      // If yes, no need to add slotframe and add link
      // directly change the current shared link to dedicated link
      //int direct_ts = -1;
      for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){
        NS_LOG_DEBUG("i Timeslot : " << int(i->macTimeslot) << " offset : " << i->macChannelOffset << " TX : " << i->macTxID << " RX : " << i->macRxID);
        NS_LOG_DEBUG("add Timeslot : " << add_link[k][0] << " offset : " << add_link[k][1] << " TX : " << add_link[k][2] << " RX : " << add_link[k][3]);
        if( int(i->macTimeslot) == add_link[k][0] && int(i->macChannelOffset) == add_link[k][1] && 
            int(i->macTxID) == add_link[k][2] && int(i->macRxID) == add_link[k][3])
        {
          //std::cout << " scheduling_table_link[int(i->macTimeslot)] : " << scheduling_table_link[int(i->macTimeslot)] << std::endl;
          //std::cout << " scheduling_table_SharedLink[int(i->macTimeslot)][int(i->macChannelOffset)] : " << scheduling_table_SharedLink[int(i->macTimeslot)][int(i->macChannelOffset)] << std::endl;
          
          
          if(AddedLink_node[int(i->macTxID)] == 0 && scheduling_table_link[int(i->macTimeslot)-bias] == 1 && scheduling_table_SharedLink[int(i->macTimeslot)-bias][int(i->macChannelOffset)] == 2)
          {
            // only 1 shared link in one timeslot can directly change?
              

            if( directly_change_add[k] == 1 )  //second come
            {
              AddedLink_node[int(i->macTxID)] = 1;
              //add_link[k][0] = int(before_DedicatedLink_count-delete_link_count+(k+1)) + bias;
            }  

            directly_change_add[k] = 1;
            i->macLinkOptions.set(2,0); // set dedicated
            std::cout << " Shared link " << add_link[k][2] << "->" << add_link[k][3] << " directly change to dedicated, timeslot " << i->macTimeslot;
            i->macTimeslot = uint16_t(before_DedicatedLink_count-delete_link_count+(k+1));
            //direct_ts = int(i->macTimeslot);
            std::cout << " change to " << i->macTimeslot << std::endl;
          }
          break;
        }
      }
      if( directly_change_add[k] == 1 ){
        //add_link[k][0] -= bias;
        //continue;
      }
      ///////////////////////////////
      //s->size += 1;
      //slotframe_size_mac = int(s->size);
      bool b=0;
      /*if (directly_change_add[k] == 1)
      for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++)
      {
        if( int(i->macTimeslot) >= direct_ts && direct_ts > 0 )
        { 
          NS_LOG_DEBUG("process add modifying link");
          NS_LOG_DEBUG("Timeslot : " << int(i->macTimeslot) << " offset : " << i->macChannelOffset << " TX : " << i->macTxID << " RX : " << i->macRxID );
          i->macTimeslot++;
          NS_LOG_DEBUG("After Timeslot : " << int(i->macTimeslot) << " offset : " << i->macChannelOffset << " TX : " << i->macTxID << " RX : " << i->macRxID );
        }
      }*/
      //else if (directly_change_add[k] != 1)
      if (directly_change_add[k] != 1)
      {
        for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin(); i != m_macLinkTable.end(); i++)
        {
          if (int(i->macTimeslot) >= add_link[k][0])
          {
            NS_LOG_DEBUG("process add modifying link");
            NS_LOG_DEBUG("Timeslot : " << int(i->macTimeslot) << " offset : " << i->macChannelOffset << " TX : " << i->macTxID << " RX : " << i->macRxID);
            i->macTimeslot++;
            NS_LOG_DEBUG("After Timeslot : " << int(i->macTimeslot) << " offset : " << i->macChannelOffset << " TX : " << i->macTxID << " RX : " << i->macRxID);
          }
        }
        for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin(); i != m_macLinkTable.end(); i++)
        {
          if (int(i->macTimeslot) - 1 == add_link[k][0] && int(i->macChannelOffset) == add_link[k][1] &&
              int(i->macTxID) == add_link[k][2] && int(i->macRxID) == add_link[k][3])
          {

            MacPibLinkAttributes entry;
            entry.macLinkHandle = i->macLinkHandle;
            entry.macLinkOptions = i->macLinkOptions; // b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.
            entry.macLinkType = i->macLinkType;
            entry.slotframeHandle = i->slotframeHandle;
            entry.macNodeAddr = i->macNodeAddr;           // not using Mac16_Address because 0xffff means the link can be used for frames destined for the boradcast address
            entry.macTimeslot = i->macTimeslot - 1;       // refer to 5.1.1.5
            entry.macChannelOffset = i->macChannelOffset; // refer to 5.1.1.5.3
            entry.macLinkFadingBias = i->macLinkFadingBias;
            entry.macTxID = i->macTxID;
            entry.macRxID = i->macRxID;

            // if (it->macLinkOptions[0] && it->macLinkOptions[2] && AddedLink_node[i->macTxID] == 0 ) {   // set shared link to dedicated link
            // if (AddedLink_node[int(i->macTxID)] == 0 && scheduling_table_link[int(i->macTimeslot)-bias]>1) {   // shared link but have other link in the timeslot. Cannot directly change to dedicated link
            if (AddedLink_node[int(i->macTxID)] == 0)
            { // shared link but have other link in the timeslot. Cannot directly change to dedicated link
              entry.macLinkOptions.set(2, 0);
              entry.macTimeslot = uint16_t(before_DedicatedLink_count - delete_link_count + (k + 1));

              m_macLinkTable.push_back(entry);
              NS_LOG_DEBUG("Adding Dedicated Link for Timeslot : " << entry.macTimeslot << " offset : " << int(i->macChannelOffset) << " TX : " << int(i->macTxID) << " RX : " << int(i->macRxID));
              if (int(i->macTxID) > int(i->macRxID) && this == dev[int(i->macTxID)]) // second come
              {
                AddedLink_node[i->macTxID]++;
              }
              else if (int(i->macTxID) < int(i->macRxID) && this == dev[int(i->macRxID)]) // second come
              {
                AddedLink_node[i->macTxID]++;
              }

              // after added dedicated link, erase shared link
              m_macLinkTable.erase(i);
              NS_LOG_DEBUG("Deleting Shared Link for Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) << " TX : " << int(i->macTxID) << " RX : " << int(i->macRxID));
            }
            else if (AddedLink_node[int(i->macTxID)] >= 1)
            { // already have dedicated link, add one more
              m_macLinkTable.push_back(entry);
              NS_LOG_DEBUG("Adding Link for Timeslot : " << int(i->macTimeslot) - 1 << " offset : " << int(i->macChannelOffset) << " TX : " << int(i->macTxID) << " RX : " << int(i->macRxID));

              if (int(i->macTxID) > int(i->macRxID) && this == dev[int(i->macTxID)]) // second come
              {
                AddedLink_node[i->macTxID]++;
                // TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count] = int(i->macTimeslot);
                // TimeslotSkipForNewAdd_count++;
              }
              else if (int(i->macTxID) > int(i->macRxID) && this == dev[int(i->macRxID)])
              {
                // TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count] = int(i->macTimeslot)-1 + AddedLink_node[int(i->macTxID)];
                // std::cout << " TimeslotSkipForNewAdd : " << TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count] << " int(i->macTimeslot)-1 : " << int(i->macTimeslot)-1
                //           << " AddedLink_node[int(i->macTxID)] : " << AddedLink_node[int(i->macTxID)] << std::endl;
                // TimeslotSkipForNewAdd_count++;
                b = 1;
              }
              // else if( int(i->macTxID) < int(i->macRxID) && this == dev[int(i->macRxID)]) // second come
              //{
              // AddedLink_node[i->macTxID]++;
              // TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count] = int(i->macTimeslot);
              // TimeslotSkipForNewAdd_count++;
              //}
            }
            else
            {
              std::cout << " AddedLink_node[int(i->macTxID)] : " << AddedLink_node[int(i->macTxID)] << " scheduling_table_link[int(i->macTimeslot)] : "
                        << scheduling_table_link[int(i->macTimeslot)] << std::endl;
              NS_FATAL_ERROR(" Error ");
            }
            break;
          }
        }
      }
      if ( b==1 ){
        int min = add_link[k][0];
        for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){
          if(int(i->macTxID) == add_link[k][2] && int(i->macRxID) == add_link[k][3]){
            if(min > int(i->macTimeslot) )
              min = int(i->macTimeslot);
          }
        }
        TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count] = min + AddedLink_node[add_link[k][2]];
        std::cout << " TimeslotSkipForNewAdd : " << TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count] << " min : " << min
                  << " AddedLink_node[add_link[k][2]] : " << AddedLink_node[add_link[k][2]] << std::endl;
        //if( node_TimeslotLength[TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count]] != 10000 )
        //{
          std::cout<< " come add shift 1 " << std::endl;
          int aa = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
          slotframe_size_mac =  1 + aa + DedicatedLink_count;
          int temp = node_TimeslotLength[TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count]];
          for ( int m=TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count]+1; m<slotframe_size_mac; m++){
            int temp1 = node_TimeslotLength[m]; 
            node_TimeslotLength[m] = temp;
            temp = temp1;
          }
        //}
        node_TimeslotLength[TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count]] = 10000;

        
        TimeslotSkipForNewAdd_count++;


      }
      

      add_link[k][0] -= bias;
    }

    if(!w5){
    int a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
    slotframe_size_mac =  1 + a + DedicatedLink_count;
    
    s->size = uint16_t(slotframe_size_mac);


    if(!w5){
      incasn_slotframe_size = slotframe_size_mac;
    }




    NS_LOG_DEBUG( " Change slotframe size to " << int(s->size) );
    std::cout << " ASN from " << m_macTschPIBAttributes.m_macASN;
    while( m_macTschPIBAttributes.m_macASN%s->size != 0 ){
      m_macTschPIBAttributes.m_macASN++;
    }
    std::cout << " change to " << m_macTschPIBAttributes.m_macASN << " to make ts = 0 "<<std::endl;
    }
  }

}

void 
LrWpanTschMac::delete_process()
{
  deleting_slotframe_size_mac = 1;
  for (std::list<MacPibSlotframeAttributes>::iterator s = m_macSlotframeTable.begin();s != m_macSlotframeTable.end();s++)
  {
    for( int k=0; k<delete_link_count; k++){
      int bias = 0;   // for timeslot bias
      for ( int m=0; m<add_link_count; m++){
        if( add_link[m][0] < delete_link[k][0] )
          if( directly_change_add[m] != 1 )
            bias++;
      }
      for ( int m=0; m<k; m++){
        if( delete_link[m][0] < delete_link[k][0] )
          //if( directly_change_delete[m] != 1 )
            bias--;
      }
      delete_link[k][0] += bias;
      // check the it is only one dedicated link or not
      // If yes, no need to delete slotframe and delete link first
      // directly change the current dedicated link to shared link
      std::cout << " bias : " << bias << " Come for ts " << delete_link[k][0] << " offset " << delete_link[k][1] << " " << delete_link[k][2] << "->" << delete_link[k][3] << std::endl;

      for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){

        //std::cout << " Check for ts " << int(i->macTimeslot) << " offset " << int(i->macChannelOffset) << " " << int(i->macTxID) << "->" << int(i->macRxID) << std::endl;

        if( int(i->macTimeslot) == delete_link[k][0] && int(i->macChannelOffset) == delete_link[k][1] && 
            int(i->macTxID) == delete_link[k][2] && int(i->macRxID) == delete_link[k][3])
        {
          if( AddedLink_node[int(i->macTxID)] == 1 )
          {
            if( directly_change_delete[k] == 1 ) // second come
            {
              AddedLink_node[int(i->macTxID)] = 0;
            }  
            if ( i->macLinkOptions[0] && !(i->macLinkOptions[2]))  // if dedicated
              i->macLinkOptions.set(2,1); // set shared
            directly_change_delete[k] = 1;
            std::cout << " Dedicated link " << delete_link[k][2] << "->" << delete_link[k][3] << " directly change to shared" << std::endl;
          }
          break;
        }
      }
      if( directly_change_delete[k] == 1 ){
        //delete_link[k][0] -= bias;
        //continue; 
      }
         
      ///////////////////////////////
      if (directly_change_delete[k] != 1)
      {

        for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin(); i != m_macLinkTable.end(); i++)
        {
          if (int(i->macTimeslot) == delete_link[k][0] && int(i->macChannelOffset) == delete_link[k][1] &&
              int(i->macTxID) == delete_link[k][2] && int(i->macRxID) == delete_link[k][3])
          {
            m_macLinkTable.erase(i);
            if (i->macLinkOptions[0] && i->macLinkOptions[2])
              NS_LOG_DEBUG("Deleting Shared Link of Sending for Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) << " TX : " << int(i->macTxID) << " RX : " << int(i->macRxID));
            else if (i->macLinkOptions[0] && !(i->macLinkOptions[2]))
              NS_LOG_DEBUG("Deleting Dedicated Link of Sending for Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) << " TX : " << int(i->macTxID) << " RX : " << int(i->macRxID));
            else
              NS_LOG_DEBUG("Deleting Link of Receiving for Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) << " TX : " << int(i->macTxID) << " RX : " << int(i->macRxID));

            if (int(i->macTxID) > int(i->macRxID) && this == dev[int(i->macTxID)]) // second come
              AddedLink_node[int(i->macTxID)]--;
            else if (int(i->macTxID) < int(i->macRxID) && this == dev[int(i->macRxID)]) // second come
              AddedLink_node[int(i->macTxID)]--;

            break;
          }
        }
      }
      //if( scheduling_table_link[delete_link[k][0]] == 1 ){      // after erase, maybe having a empty timeslot 
      bool b = 0;
      for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin(); i != m_macLinkTable.end(); i++)
      {
        b = 0;
        // std::cout << "1 Check for ts " << int(i->macTimeslot) << " offset " << int(i->macChannelOffset) << " " << int(i->macTxID) << "->" << int(i->macRxID) << std::endl;
        if (int(i->macTimeslot) > delete_link[k][0])
        {

          for (int m = 0; m < add_link_count; m++)
          {
            std::cout << "add_link[m][2] : " << add_link[m][2] << " add_link[m][3] : " << add_link[m][3] << " AddedLink_node[add_link[m][2]] : " << AddedLink_node[add_link[m][2]] << std::endl;
            if (int(i->macTxID) == add_link[m][2] && int(i->macRxID) == add_link[m][3] && (AddedLink_node[add_link[m][2]] == 1 || AddedLink_node[add_link[m][2]] == 0))
            {                                                                                                                                                     // new add only 1 dedicated link no need to shift
              if ((AddedLink_node[add_link[m][2]] == 0 && this == dev[int(i->macRxID)]) || (AddedLink_node[add_link[m][2]] == 1 && this == dev[int(i->macTxID)])) // not add AddedLink_node yet for received
              {
                b = 1;
                break;
              }
            }
          }
          // for ( int m=0; m<delete_link_count; m++){
          //     if( int(i->macTxID) == delete_link[m][2] && int(i->macRxID) == delete_link[m][3]){    // deleteadd link no need to shift
          //         b = 1;
          //         break;
          //     }
          // }

          if (b == 1)
            continue;

          NS_LOG_DEBUG("process delete modifying link");
          NS_LOG_DEBUG("Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) << " TX : " << int(i->macTxID) << " RX : " << int(i->macRxID));
          i->macTimeslot--;
          NS_LOG_DEBUG("After Timeslot : " << int(i->macTimeslot) << " offset : " << int(i->macChannelOffset) << " TX : " << int(i->macTxID) << " RX : " << int(i->macRxID));
        }
      }
      //}
      if (plan == 3)
      {
        for (int m = 0; m < TimeslotSkipForNewAdd_count; m++)
        {
          if (TimeslotSkipForNewAdd[m] > delete_link[k][0])
          {
            TimeslotSkipForNewAdd[m]--;
          }
        }
      }

      // s->size -= 1;
      // slotframe_size_mac = int(s->size);
      delete_link[k][0] -= bias;
    }
    if (plan == 3)
    {
      for (int k = 0; k < TimeslotSkipForTwoDelete_count; k++)
      {
        int max = 0;
        for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin(); i != m_macLinkTable.end(); i++)
        {

          std::cout << " int(i->macTxID) : " << int(i->macTxID) << " int(i->macRxID) : " << int(i->macRxID) << std::endl;
          std::cout << " TimeslotSkipForTwoDelete[k][0] : " << TimeslotSkipForTwoDelete[k][0]
                    << " TimeslotSkipForTwoDelete[k][1] : " << TimeslotSkipForTwoDelete[k][1] << std::endl;
          if (TimeslotSkipForTwoDelete[k][0] == int(i->macTxID) && TimeslotSkipForTwoDelete[k][1] == int(i->macRxID))
          {

            if (max == 0)
            {
              max = int(i->macTimeslot);
            }
            else if (int(i->macTimeslot) > max)
              max = int(i->macTimeslot);
          }
        }
        if (max != 0)
        {

          TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count] = max;
          std::cout << " max : " << max << std::endl;

          //if (node_TimeslotLength[TimeslotSkipForNewAdd[TimeslotSkipForNewAdd_count]] != 10000)
          //{
            //NS_FATAL_ERROR("GGG");
            //std::cout << "GGG" << std::endl;
          //}
          //node_TimeslotLength[max] = 10000;
          TimeslotSkipForNewAdd_count++;
        }
      }
    }

    if(!w5){
    int a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
    //std::cout << " a : " << a << " DedicatedLink_count : " << DedicatedLink_count << std::endl;
    if( slotframe_size_mac != 1 + a + DedicatedLink_count ){
      slotframe_size_mac =  1 + a + DedicatedLink_count;
    }
    s->size = uint16_t(slotframe_size_mac);

    if(!w5){
      incasn_slotframe_size = slotframe_size_mac;
    }


    NS_LOG_DEBUG( " Change slotframe size to " << int(s->size));
    std::cout << " ASN from " << m_macTschPIBAttributes.m_macASN;
    while( m_macTschPIBAttributes.m_macASN%s->size != 0 ){
      m_macTschPIBAttributes.m_macASN++;
    }
    std::cout << " change to " << m_macTschPIBAttributes.m_macASN << " to make ts = 0 "<<std::endl;
    }
  }
}


void 
LrWpanTschMac::SharedLink_cal()
{
  //print_scheduling_table();
  std::cout << " add_link_count : " << add_link_count << std::endl;
  std::cout << " delete_link_count : " << delete_link_count << std::endl;
  //int a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
  //slotframe_size_mac =  1 + a + DedicatedLink_count;
  //std::cout << "slotframe_size_mac : " << slotframe_size_mac << std::endl;

  // renews
  DedicatedLink_count = 0;
  SharedLink_count = 0;

  for( int i=0; i<50; i++){
    for ( int j=0; j<=1; j++){
      for( int k=0; k<4; k++){
        SharedLink_Compress[i][j][k] = 0;
      }
    }
  }

  //

  for( int i=1; i<dev_count; i++){
    if(AddedLink_node[i] >= 1 ){
      DedicatedLink_count += AddedLink_node[i];
    }
    else if( AddedLink_node[i] == 0 ){

      SharedLink_count++;

      for( int k=0; k<slotframe_size_mac; k++){
        for( int j=0; j<scheduling_table_link[k]; j++){
            for( int p=0; p<scheduling_table_SharedLink[k][j]; p++){
              if( i == scheduling_table[k][j][p][0]){
                std::cout << "SharedLink_count : " << SharedLink_count << std::endl;

                SharedLink_Compress[(SharedLink_count-1)/2][(SharedLink_count-1)%2][0] = k;
                SharedLink_Compress[(SharedLink_count-1)/2][(SharedLink_count-1)%2][1] = j;
                SharedLink_Compress[(SharedLink_count-1)/2][(SharedLink_count-1)%2][2] = scheduling_table[k][j][p][0];
                SharedLink_Compress[(SharedLink_count-1)/2][(SharedLink_count-1)%2][3] = scheduling_table[k][j][p][1];
                
            }
          }
        }
      }
    }
  }

  


  
  int a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
  std::vector<int> temp(4,0);
  for( int k=0; k<a-1; k++){
    for( int i=0; i<a-k-1; i++){      // order
      for ( int j=0; j<=1; j++){

        /*
        std::cout <<" before " << std::endl;
        std::cout << "SharedLink_Compress[i][j][0] : " << SharedLink_Compress[i][j][0] << std::endl;
        std::cout << "SharedLink_Compress[i][j][1] : " << SharedLink_Compress[i][j][1] << std::endl;
        std::cout << "SharedLink_Compress[i][j][2] : " << SharedLink_Compress[i][j][2] << std::endl;
        std::cout << "SharedLink_Compress[i][j][3] : " << SharedLink_Compress[i][j][3] << std::endl;
        */
        if( j==0 && SharedLink_Compress[i][j][0] > SharedLink_Compress[i][j+1][0] && ( SharedLink_Compress[i][j+1][2] != 0 || SharedLink_Compress[i][j+1][3] != 0 ) ){

          temp[0] = SharedLink_Compress[i][j][0];
          temp[1] = SharedLink_Compress[i][j][1];
          temp[2] = SharedLink_Compress[i][j][2];
          temp[3] = SharedLink_Compress[i][j][3];

          SharedLink_Compress[i][j][0] = SharedLink_Compress[i][j+1][0];
          SharedLink_Compress[i][j][1] = SharedLink_Compress[i][j+1][1];
          SharedLink_Compress[i][j][2] = SharedLink_Compress[i][j+1][2];
          SharedLink_Compress[i][j][3] = SharedLink_Compress[i][j+1][3];

          SharedLink_Compress[i][j+1][0] = temp[0];
          SharedLink_Compress[i][j+1][1] = temp[1];
          SharedLink_Compress[i][j+1][2] = temp[2];
          SharedLink_Compress[i][j+1][3] = temp[3];

        }
        else if( j==1 && SharedLink_Compress[i][j][0] > SharedLink_Compress[i+1][0][0] && ( SharedLink_Compress[i+1][0][2] != 0 || SharedLink_Compress[i+1][0][3] != 0 ))
        {
          temp[0] = SharedLink_Compress[i][j][0];
          temp[1] = SharedLink_Compress[i][j][1];
          temp[2] = SharedLink_Compress[i][j][2];
          temp[3] = SharedLink_Compress[i][j][3];

          SharedLink_Compress[i][j][0] = SharedLink_Compress[i+1][0][0];
          SharedLink_Compress[i][j][1] = SharedLink_Compress[i+1][0][1];
          SharedLink_Compress[i][j][2] = SharedLink_Compress[i+1][0][2];
          SharedLink_Compress[i][j][3] = SharedLink_Compress[i+1][0][3];

          SharedLink_Compress[i+1][0][0] = temp[0];
          SharedLink_Compress[i+1][0][1] = temp[1];
          SharedLink_Compress[i+1][0][2] = temp[2];
          SharedLink_Compress[i+1][0][3] = temp[3];
        }
        /*
        std::cout <<" after " << std::endl;
        std::cout << "SharedLink_Compress[i][j][0] : " << SharedLink_Compress[i][j][0] << std::endl;
        std::cout << "SharedLink_Compress[i][j][1] : " << SharedLink_Compress[i][j][1] << std::endl;
        std::cout << "SharedLink_Compress[i][j][2] : " << SharedLink_Compress[i][j][2] << std::endl;
        std::cout << "SharedLink_Compress[i][j][3] : " << SharedLink_Compress[i][j][3] << std::endl;
        */
      }
    }
  }
  
  
  std::cout<<std::endl;

  for( int i=0; i<a; i++){
    std::cout << "SharedLink_Compress[i][0][0] : " << SharedLink_Compress[i][0][0] << std::endl;
    std::cout << "SharedLink_Compress[i][0][1] : " << SharedLink_Compress[i][0][1] << std::endl;
    std::cout << "SharedLink_Compress[i][0][2] : " << SharedLink_Compress[i][0][2] << std::endl;
    std::cout << "SharedLink_Compress[i][0][3] : " << SharedLink_Compress[i][0][3] << std::endl;
    if( SharedLink_Compress[i][1][2] != 0 ){
      std::cout << "SharedLink_Compress[i][1][0] : " << SharedLink_Compress[i][1][0] << std::endl;
      std::cout << "SharedLink_Compress[i][1][1] : " << SharedLink_Compress[i][1][1] << std::endl;
      std::cout << "SharedLink_Compress[i][1][2] : " << SharedLink_Compress[i][1][2] << std::endl;
      std::cout << "SharedLink_Compress[i][1][3] : " << SharedLink_Compress[i][1][3] << std::endl;
    }

  }

  std::cout << " before DedicatedLink_count : " << DedicatedLink_count << std::endl;
  std::cout << " before SharedLink_count : " << SharedLink_count << std::endl;
  before_DedicatedLink_count = DedicatedLink_count;
  before_SharedLink_count = SharedLink_count;
  int bias = 0;
  for( int k=0; k<add_link_count; k++){
    std::cout << " AddedLink_node[add_link[k][2]] : " << AddedLink_node[add_link[k][2]] << std::endl;
    std::cout << " add_link[k][0] : " << add_link[k][0] << std::endl;
    std::cout << " add_link[k][1] : " << add_link[k][1] << std::endl;
    std::cout << " add_link[k][2] : " << add_link[k][2] << std::endl;
    std::cout << " add_link[k][3] : " << add_link[k][3] << std::endl;

    //if( AddedLink_node[add_link[k][3]] == 0 && scheduling_table_SharedLink[add_link[k][0]][add_link[k][1]]>2 ){
    if( AddedLink_node[add_link[k][2]] == 0 ){
      SharedLink_count--;
      DedicatedLink_count++;
      bias++;
      a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
      std::cout << " SharedLink_count : " << SharedLink_count << " a : " << a << std::endl;

      bool b=0;
      for( int i=0; i<=a; i++){
        for ( int j=0; j<=1; j++){
            std::cout << "SharedLink_Compress[i][j][2] : " << SharedLink_Compress[i][j][2] <<" come add_link[k][2] : " << add_link[k][2] << " b : " << b << std::endl;

          if( SharedLink_Compress[i][j][2] == add_link[k][2] && b == 0)
          {
            std::cout << " come add_link[k][2] : " << add_link[k][2] << " i : " << i << " j : " << j << std::endl;
            b = 1;
            SharedLink_Compress[i][j][0] = 0;
            SharedLink_Compress[i][j][1] = 0;
            SharedLink_Compress[i][j][2] = 0;
            SharedLink_Compress[i][j][3] = 0;
          }
          if(b){
            if( j == 0 && ( SharedLink_Compress[i][1][2] != 0 || SharedLink_Compress[i][1][3] != 0 ) ){
              SharedLink_Compress[i][j][0] = SharedLink_Compress[i][1][0];
              SharedLink_Compress[i][j][1] = SharedLink_Compress[i][1][1];
              SharedLink_Compress[i][j][2] = SharedLink_Compress[i][1][2];
              SharedLink_Compress[i][j][3] = SharedLink_Compress[i][1][3];

              SharedLink_Compress[i][1][0] = 0;
              SharedLink_Compress[i][1][1] = 0;
              SharedLink_Compress[i][1][2] = 0;
              SharedLink_Compress[i][1][3] = 0;

            }
            else if ( j == 1 && (SharedLink_Compress[i+1][0][2] != 0 || SharedLink_Compress[i+1][0][3] != 0 ) ){
              SharedLink_Compress[i][j][0] = SharedLink_Compress[i+1][0][0];
              SharedLink_Compress[i][j][1] = SharedLink_Compress[i+1][0][1];
              SharedLink_Compress[i][j][2] = SharedLink_Compress[i+1][0][2];
              SharedLink_Compress[i][j][3] = SharedLink_Compress[i+1][0][3];

              SharedLink_Compress[i+1][0][0] = 0;
              SharedLink_Compress[i+1][0][1] = 0;
              SharedLink_Compress[i+1][0][2] = 0;
              SharedLink_Compress[i+1][0][3] = 0;

            }
              
          }

        }
      }


    }
    else if( AddedLink_node[add_link[k][2]] > 0 ){
        DedicatedLink_count++;
        bias++;
    }
  }
  for( int k=0; k<delete_link_count; k++){

    std::cout << " AddedLink_node[delete_link[k][2]] : " << AddedLink_node[delete_link[k][2]] << std::endl;
    std::cout << " delete_link[k][0] : " << delete_link[k][0] << std::endl;
    std::cout << " delete_link[k][1] : " << delete_link[k][1] << std::endl;
    std::cout << " delete_link[k][2] : " << delete_link[k][2] << std::endl;
    std::cout << " delete_link[k][3] : " << delete_link[k][3] << std::endl;
        
    //if( AddedLink_node[delete_link[k][3]] == 1 && scheduling_table_link[delete_link[k][0]] == 1 ){
    if( AddedLink_node[delete_link[k][2]] == 1 ){
      SharedLink_count++;
      DedicatedLink_count--;
      a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
      bool b=0;
      std::vector<int> temp(4,0);
      for( int i=0; i<a; i++){
        for ( int j=0; j<=1; j++){

          if( delete_link[k][0] <= SharedLink_Compress[i][j][0] && b==0)
          {
            temp[0] = SharedLink_Compress[i][j][0];
            temp[1] = SharedLink_Compress[i][j][1];
            temp[2] = SharedLink_Compress[i][j][2];
            temp[3] = SharedLink_Compress[i][j][3];

            SharedLink_Compress[i][j][0] = delete_link[k][0];
            SharedLink_Compress[i][j][1] = delete_link[k][1];
            SharedLink_Compress[i][j][2] = delete_link[k][2];
            SharedLink_Compress[i][j][3] = delete_link[k][3];
            b = 1;

          }
          else if( b==1 ){
            int t0 = SharedLink_Compress[i][j][0];
            int t1 = SharedLink_Compress[i][j][1];
            int t2 = SharedLink_Compress[i][j][2];
            int t3 = SharedLink_Compress[i][j][3];

            SharedLink_Compress[i][j][0] = temp[0];
            SharedLink_Compress[i][j][1] = temp[1];
            SharedLink_Compress[i][j][2] = temp[2];
            SharedLink_Compress[i][j][3] = temp[3];

            temp[0] = t0;
            temp[1] = t1;
            temp[2] = t2;
            temp[3] = t3;

          }
        }
      }
      if(b==0){
        //NS_FATAL_ERROR("ppp");
        for( int i=0; i<=a; i++){
          for ( int j=0; j<=1; j++){
            if( SharedLink_Compress[i][j][2] == 0 && SharedLink_Compress[i][j][3] == 0 ){    // add at the last one
              SharedLink_Compress[i][j][0] = delete_link[k][0];
              SharedLink_Compress[i][j][1] = delete_link[k][1];
              SharedLink_Compress[i][j][2] = delete_link[k][2];
              SharedLink_Compress[i][j][3] = delete_link[k][3];
              b=1;
              break;
            }
          }
          if(b)
            break;
        }
      }
    }
    else if( AddedLink_node[delete_link[k][2]] > 1 ){
      DedicatedLink_count--;
      bias--;
    }
  }

  for( int i=0; i<a; i++){      // order
    for ( int j=0; j<=1; j++){
      if( i == 0 && j == 0 ){
        if( SharedLink_Compress[0][0][0] != DedicatedLink_count + 1 ){
            SharedLink_Compress[0][0][0] = DedicatedLink_count + 1;
        }
      }
      else if ( j == 0 ){
        SharedLink_Compress[i][0][0] = SharedLink_Compress[i-1][0][0]+1;
      }
      else if ( j == 1 ){
        SharedLink_Compress[i][1][0] = SharedLink_Compress[i][0][0];
      } 
    }
  }
  /*
  if(w5){
    for( int k=0; k<a-1; k++){
      for( int i=0; i<a-k-1; i++){      // order for zero_queue_count
        for ( int j=0; j<=1; j++){

          if( j == 0 && ( SharedLink_Compress[i][1][2] != 0 || SharedLink_Compress[i][1][3] != 0 ) ){

            if( zero_queue_count[SharedLink_Compress[i][0][2]] > zero_queue_count[SharedLink_Compress[i][1][2]] ){

              //int t0 = SharedLink_Compress[i][0][0];
              //int t1 = SharedLink_Compress[i][0][1];
              int t2 = SharedLink_Compress[i][0][2];
              int t3 = SharedLink_Compress[i][0][3];

              //SharedLink_Compress[i][0][0] = SharedLink_Compress[i][1][0];
              //SharedLink_Compress[i][0][1] = SharedLink_Compress[i][1][1];
              SharedLink_Compress[i][0][2] = SharedLink_Compress[i][1][2];
              SharedLink_Compress[i][0][3] = SharedLink_Compress[i][1][3];

              //SharedLink_Compress[i][1][0] = t0;
              //SharedLink_Compress[i][1][1] = t1;
              SharedLink_Compress[i][1][2] = t2;
              SharedLink_Compress[i][1][3] = t3;
            
            }
          }
          else if ( j == 1 && (SharedLink_Compress[i+1][0][2] != 0 || SharedLink_Compress[i+1][0][3] != 0 ) ){
            if( zero_queue_count[SharedLink_Compress[i][1][2]] > zero_queue_count[SharedLink_Compress[i+1][0][2]] ){

              //int t0 = SharedLink_Compress[i][1][0];
              //int t1 = SharedLink_Compress[i][1][1];
              int t2 = SharedLink_Compress[i][1][2];
              int t3 = SharedLink_Compress[i][1][3];

              //SharedLink_Compress[i][0][0] = SharedLink_Compress[i+1][0][0];
              //SharedLink_Compress[i][0][1] = SharedLink_Compress[i+1][0][1];
              SharedLink_Compress[i][1][2] = SharedLink_Compress[i+1][0][2];
              SharedLink_Compress[i][1][3] = SharedLink_Compress[i+1][0][3];

              //SharedLink_Compress[i+1][0][0] = t0;
              //SharedLink_Compress[i+1][0][1] = t1;
              SharedLink_Compress[i+1][0][2] = t2;
              SharedLink_Compress[i+1][0][3] = t3;
            
            }
          }
        }
      }
    }
  }
  */






  //std::cout << " bias : " << bias << std::endl;
  /*
  if( bias != 0){     // timeslot bias
    for( int i=0; i<a; i++){
      for ( int j=0; j<=1; j++){
        SharedLink_Compress[i][j][0] = SharedLink_Compress[i][j][0]+bias;
      }
    }
  }*/
  /*
  for( int i=0; i<dev_count; i++){
    std::cout << " i : " << i << "zero_queue_count[i] : " << zero_queue_count[i] << std::endl;
  }
  std::cout<<std::endl;
  for( int i=0; i<dev_count; i++){
    std::cout << " i : " << i << "zero_shared_queue_count[i] : " << zero_shared_queue_count[i] << std::endl;
  }*/

  for( int i=0; i<a; i++){
    
    //std::cout << "zero_queue_count[SharedLink_Compress[i][0][2]] : " << zero_queue_count[SharedLink_Compress[i][0][2]] << std::endl;
    std::cout << "SharedLink_Compress[i][0][0] : " << SharedLink_Compress[i][0][0] << std::endl;
    std::cout << "SharedLink_Compress[i][0][1] : " << SharedLink_Compress[i][0][1] << std::endl;
    std::cout << "SharedLink_Compress[i][0][2] : " << SharedLink_Compress[i][0][2] << std::endl;
    std::cout << "SharedLink_Compress[i][0][3] : " << SharedLink_Compress[i][0][3] << std::endl;

    if( SharedLink_Compress[i][1][2] != 0 ){
      //std::cout << "zero_queue_count[SharedLink_Compress[i][1][2]] : " << zero_queue_count[SharedLink_Compress[i][1][2]] << std::endl;
      std::cout << "SharedLink_Compress[i][1][0] : " << SharedLink_Compress[i][1][0] << std::endl;
      std::cout << "SharedLink_Compress[i][1][1] : " << SharedLink_Compress[i][1][1] << std::endl;
      std::cout << "SharedLink_Compress[i][1][2] : " << SharedLink_Compress[i][1][2] << std::endl;
      std::cout << "SharedLink_Compress[i][1][3] : " << SharedLink_Compress[i][1][3] << std::endl;
    }
  }

  std::cout << " after DedicatedLink_count : " << DedicatedLink_count << std::endl;
  std::cout << " after SharedLink_count : " << SharedLink_count << std::endl;


}

void 
LrWpanTschMac::update_scheduling()
{

  if( DedicatedLink_count != 0 || SharedLink_count != 0){
    int a = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
    slotframe_size_mac =  1 + a + DedicatedLink_count;
  }
  std::cout << "slotframe_size_mac : " << slotframe_size_mac << std::endl;


  
  // empty

  if (this == dev[0])
  {
    for (int j = 0; j < 15; j++)
    {
      for (int p = 0; p < 3; p++)
      {
        for (int i = 0; i < slotframe_size_mac; i++)
        {
          if (scheduling_table_temp[i][j][p][0] != -1 || scheduling_table_temp[i][j][p][1] != -1)
          {
            scheduling_table_temp[i][j][p][0] = -1;
            scheduling_table_temp[i][j][p][1] = -1;
          }
          scheduling_table_link_temp[i] = 0;
          scheduling_table_SharedLink_temp[i][j] = 1;
        }
      }
    }
  }

    // update scheduling
    for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin();i != m_macLinkTable.end();i++){

      int t = int(i->macTimeslot);
      int c = int(i->macChannelOffset);
      int TxID = int(i->macTxID);
      int RxID = int(i->macRxID);


      if ( i->macLinkOptions[0] && i->macLinkOptions[2] == 0 ) {

        scheduling_table_temp[t][scheduling_table_link_temp[t]][0][0] = TxID;
        scheduling_table_temp[t][scheduling_table_link_temp[t]][0][1] = RxID;
        scheduling_table_link_temp[t]++;
        std::cout << " set dedicated link " << scheduling_table_temp[t][scheduling_table_link_temp[t]-1][0][0]
                  << "->" << scheduling_table_temp[t][scheduling_table_link_temp[t]-1][0][1] << " for timeslot " << t << std::endl;
        
      }
      else if (i->macLinkOptions[0] && i->macLinkOptions[2] ) {

        if(scheduling_table_SharedLink_temp[t][c]) // first in for shared link
        {
          scheduling_table_link_temp[t]++;
        }

        scheduling_table_temp[t][c][scheduling_table_SharedLink_temp[t][c]][0] = TxID;
        scheduling_table_temp[t][c][scheduling_table_SharedLink_temp[t][c]][1] = RxID;
        //scheduling_table_link[t]++;
        scheduling_table_SharedLink_temp[t][c]++;
        std::cout << " set shared link " << scheduling_table_temp[t][c][scheduling_table_SharedLink_temp[t][c]-1][0]
                  << "->" << scheduling_table_temp[t][c][scheduling_table_SharedLink_temp[t][c]-1][1] << " for timeslot " << t << std::endl;

      }


    }
    if( this == dev[dev_count-1]){

      for( int j=0; j<15; j++){
        for( int p=0; p<3; p++){
          for( int i=0; i<slotframe_size_mac; i++){
            scheduling_table[i][j][p][0] = scheduling_table_temp[i][j][p][0];
            scheduling_table[i][j][p][1] = scheduling_table_temp[i][j][p][1];
            scheduling_table_link[i] = scheduling_table_link_temp[i];
            scheduling_table_SharedLink[i][j] = scheduling_table_SharedLink_temp[i][j];
          }
        }
      }

      std::cout << " SharedLink_count : " << SharedLink_count << std::endl;
      std::cout << " adaptive_slotframe_size : " << adaptive_slotframe_size << std::endl;

      print_scheduling_table();


    }



}

void 
LrWpanTschMac::adaptive_slotframe()
{
  int Shared_Timeslot = SharedLink_count%2==0 ? SharedLink_count/2 : SharedLink_count/2+1;
  int Before_Shared_Timeslot = before_SharedLink_count%2==0 ? before_SharedLink_count/2 : before_SharedLink_count/2+1;
  int Dedicated_Timeslot = DedicatedLink_count;

  std::cout<< "Dedicated_Timeslot : " << Dedicated_Timeslot << std::endl;
  std::cout<< "Shared_Timeslot : " << Shared_Timeslot << std::endl; 
  std::cout<< "Before_Shared_Timeslot : " << Before_Shared_Timeslot << std::endl; 

  /*
  if( adaptive_slotframe_size >= ( 1+Dedicated_Timeslot+Shared_Timeslot))
  {
    adaptive_slotframe_size = 1+Dedicated_Timeslot;
    adaptive_slotframe_index = 0;
  }  
  else
  {
    //if( before_SharedLink_count == SharedLink_count && before_DedicatedLink_count < DedicatedLink_count )//for add more dedicated link but not add more shared link and all shared link have experienced
    //{
    //  adaptive_slotframe_size = 1+Dedicated_Timeslot;
    //  adaptive_slotframe_index = 0;
    //}
    //else{
      if( adaptive_slotframe_index >= Shared_Timeslot ){
        //NS_FATAL_ERROR("HHHERE");
        std::cout<< "HHH" << std::endl;
        if(before_SharedLink_count != SharedLink_count ) // last shared link is going to experience
        {
          adaptive_slotframe_index = Shared_Timeslot;    // slotframe size should not bigger than 1+Dedicated_Timeslot+Shared_Timeslot
          adaptive_slotframe_size = 1+Dedicated_Timeslot+adaptive_slotframe_index;
          break;
        }
        else if(before_SharedLink_count != SharedLink_count ) // last shared link have experienced
        {
          adaptive_slotframe_size = 1+Dedicated_Timeslot;
          adaptive_slotframe_index = 0;
          break;
        }
      }

    adaptive_slotframe_size = 1+Dedicated_Timeslot+adaptive_slotframe_index;
    //}
    

  }  */

  if( adaptive_slotframe_index >= Before_Shared_Timeslot+1 ){   // all shared link have experienced one time
    adaptive_slotframe_size = 1+Dedicated_Timeslot;
    adaptive_slotframe_index = 0;
    //adaptive_slotframe_index = 1;
    new_slotframe = 1;
    std::cout << " new_slotframe is coming " << std::endl;
  }
  else if( adaptive_slotframe_index < Before_Shared_Timeslot+1 ){
    adaptive_slotframe_size = 1+Dedicated_Timeslot+adaptive_slotframe_index;
    if( adaptive_slotframe_size > 1 + Dedicated_Timeslot + Shared_Timeslot ){   // Before_Shared_Timeslot > Shared_Timeslot , shared link decrease. make it bigger than maximum
      adaptive_slotframe_size = 1 + Dedicated_Timeslot + Shared_Timeslot;
    }
  }


  std::cout<< "adaptive_slotframe_index : " << adaptive_slotframe_index << std::endl;
  std::cout<< "adaptive_slotframe_size : " << adaptive_slotframe_size << std::endl;  
  adaptive_slotframe_index++;

  /*
  if( adaptive_slotframe_size == 1 && Dedicated_Timeslot == 0 && adaptive_slotframe_index == 1 ){
    adaptive_slotframe_size = 1+Dedicated_Timeslot+adaptive_slotframe_index;
    //adaptive_slotframe_size = 1+Dedicated_Timeslot+Shared_Timeslot;
    new_slotframe = 1;
    std::cout<< "no dedicated link. new slotframe for adaptive_slotframe_size : " << adaptive_slotframe_size << std::endl;  
    adaptive_slotframe_index++;
    //adaptive_slotframe_index = 0;

  }*/


}

void 
LrWpanTschMac::e_TSCH_Orch_process()
{

  for (int dev_index = 0; dev_index < dev_count; dev_index++)
  {
    LrWpanTschMac *device = dev[dev_index];
    std::cout << " *device : " << device << std::endl;
    if (device != NULL)
    {

      for (std::list<MacPibSlotframeAttributes>::iterator s = device->m_macSlotframeTable.begin(); s != device->m_macSlotframeTable.end(); s++)
      {
        //std::cout << " s->size : " << int(s->size)  << " slotframe_size_mac : " << slotframe_size_mac << std::endl;
        if (s->size == slotframe_size_mac)
        {
          break;
        }
        else
        {
          s->size = slotframe_size_mac;
          NS_LOG_DEBUG(" Change slotframe size to " << int(s->size));

          if(dev_index != 0)
            device->m_macTschPIBAttributes.m_macASN++;   // need ++ since 0 have incasn. and other not yet
          

          std::cout << " ASN from " << device->m_macTschPIBAttributes.m_macASN;
          while (device->m_macTschPIBAttributes.m_macASN % int(s->size) != 0)
          {
            device->m_macTschPIBAttributes.m_macASN++;
          }
          
          std::cout << " change to " << device->m_macTschPIBAttributes.m_macASN << " to make ts = 0 " << std::endl;
          if( dev_index != 0)   
            device->m_macTschPIBAttributes.m_macASN--; // need -- other device have not incre ASN
        }
      }
    }
  }

  int TX = -1;
  for (std::list<MacPibLinkAttributes>::iterator i = m_macLinkTable.begin(); i != m_macLinkTable.end(); i++)
  {
    if (e_TSCH_Orch_next_slotframe_queue[int(i->macTxID)] != e_TSCH_Orch_AddedLink[int(i->macTxID)] && TX != int(i->macTxID))
    {
      int count = 0;
      int ts = int(i->macTimeslot);
      TX = int(i->macTxID);
      for (std::list<MacPibLinkAttributes>::iterator j = m_macLinkTable.begin(); j != m_macLinkTable.end(); j++)
      {
        if (TX == int(j->macTxID))
        {
          count++;
          if (int(j->macTimeslot) < ts) // find the smallest timeslot of the link
            ts = int(j->macTimeslot);
        }
      }
      if (count == e_TSCH_Orch_next_slotframe_queue[int(i->macTxID)])
      { // added or deleted before
        continue;
      }

      for (int dev_index = 0; dev_index < dev_count; dev_index++)
      {
        LrWpanTschMac *device = dev[dev_index];
        std::cout << " *device : " << device << std::endl;
        if (device != NULL)
        {

          if (e_TSCH_Orch_next_slotframe_queue[TX] > e_TSCH_Orch_AddedLink[TX])
          { // add
            int add = e_TSCH_Orch_next_slotframe_queue[TX] - e_TSCH_Orch_AddedLink[TX];
            for (int m = 1; m <= add; m++)
            {
              std::cout << " m : " << m << std::endl;
              for (std::list<MacPibLinkAttributes>::iterator j = device->m_macLinkTable.begin(); j != device->m_macLinkTable.end(); j++) // shift timeslot for link
              {

                if (int(j->macTimeslot) >= ts)
                {
                  NS_LOG_DEBUG("process add modifying link");
                  NS_LOG_DEBUG("Timeslot : " << int(j->macTimeslot) << " offset : " << j->macChannelOffset << " TX : " << j->macTxID << " RX : " << j->macRxID);
                  j->macTimeslot++;
                  NS_LOG_DEBUG("After Timeslot : " << int(j->macTimeslot) << " offset : " << j->macChannelOffset << " TX : " << j->macTxID << " RX : " << j->macRxID);
                }
              }
            }

            for (int m = 1; m <= add; m++)
            {
              std::cout << " m : " << m << std::endl;
              for (std::list<MacPibLinkAttributes>::iterator j = device->m_macLinkTable.begin(); j != device->m_macLinkTable.end(); j++)
              {
                if (int(j->macTimeslot) - add == ts && int(j->macTxID) == TX)
                {

                  MacPibLinkAttributes entry;
                  entry.macLinkHandle = j->macLinkHandle;
                  entry.macLinkOptions = j->macLinkOptions; // b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.
                  entry.macLinkType = j->macLinkType;
                  entry.slotframeHandle = j->slotframeHandle;
                  entry.macNodeAddr = j->macNodeAddr;           // not using Mac16_Address because 0xffff means the link can be used for frames destined for the boradcast address
                  entry.macTimeslot = j->macTimeslot - m;       // refer to 5.1.1.5
                  entry.macChannelOffset = j->macChannelOffset; // refer to 5.1.1.5.3
                  entry.macLinkFadingBias = j->macLinkFadingBias;
                  entry.macTxID = j->macTxID;
                  entry.macRxID = j->macRxID;
                  device->m_macLinkTable.push_back(entry);
                  NS_LOG_DEBUG("Adding Link for Timeslot : " << int(j->macTimeslot) - m << " offset : " << int(j->macChannelOffset) << " TX : " << int(j->macTxID) << " RX : " << int(j->macRxID));
                  //if (device == dev[TX])
                  //{
                  //  e_TSCH_Orch_AddedLink[TX]++;
                  //}
                }
              }
            }
          }
          else if (e_TSCH_Orch_next_slotframe_queue[TX] < e_TSCH_Orch_AddedLink[TX])
          { // delete
            int del = e_TSCH_Orch_AddedLink[TX] - e_TSCH_Orch_next_slotframe_queue[TX];
            for (int m = 1; m <= del; m++)
            {

              for (std::list<MacPibLinkAttributes>::iterator j = device->m_macLinkTable.begin(); j != device->m_macLinkTable.end(); j++) // delete link
              {
                if (int(j->macTimeslot) == ts && int(j->macTxID) == TX)
                {
                  device->m_macLinkTable.erase(j);
                  NS_LOG_DEBUG("Deleting Link for Timeslot : " << int(j->macTimeslot) << " offset : " << int(j->macChannelOffset) << " TX : " << int(j->macTxID) << " RX : " << int(j->macRxID));
                  //if ( device == dev[TX])
                  //{
                  //  e_TSCH_Orch_AddedLink[TX]--;
                  //}
                  break;
                }
              }

              for (std::list<MacPibLinkAttributes>::iterator j = device->m_macLinkTable.begin(); j != device->m_macLinkTable.end(); j++) // shift timeslot for link
              {
                if (int(j->macTimeslot) > ts)
                {
                  NS_LOG_DEBUG("process delete modifying link");
                  NS_LOG_DEBUG("Timeslot : " << int(j->macTimeslot) << " offset : " << j->macChannelOffset << " TX : " << j->macTxID << " RX : " << j->macRxID);
                  j->macTimeslot--;
                  NS_LOG_DEBUG("After Timeslot : " << int(j->macTimeslot) << " offset : " << j->macChannelOffset << " TX : " << j->macTxID << " RX : " << j->macRxID);
                }
              }
            }
          }
        }
      }
    }
  }

  for( int i=0; i<dev_count; i++ ){
    e_TSCH_Orch_AddedLink[i] = e_TSCH_Orch_next_slotframe_queue[i];
  }



}





}





// namespace ns3
