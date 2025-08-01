#include <iostream>
#include <cstdlib>
#include <ctime>

#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/spectrum-model-ism2400MHz-res1MHz.h>
#include <ns3/spectrum-model-300kHz-300GHz-log.h>
#include <ns3/wifi-spectrum-value-helper.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/waveform-generator.h>
#include <ns3/spectrum-analyzer.h>
#include <ns3/spectrum-converter.h>
#include <ns3/log.h>
#include <string>
#include <ns3/friis-spectrum-propagation-loss.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/mobility-module.h>
#include <ns3/spectrum-helper.h>
#include <ns3/applications-module.h>
#include <ns3/adhoc-aloha-noack-ideal-phy-helper.h>
#include <ns3/waveform-generator-helper.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/non-communicating-net-device.h>
#include <ns3/flow-monitor-module.h> //change by ding
#include <ns3/flow-monitor-helper.h> //chaneg by ding

#include "ns3/internet-module.h"
#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/ipv6-routing-table-entry.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/flow-monitor-module.h"

NS_LOG_COMPONENT_DEFINE ("TwoNodesTest");

using namespace ns3;
using namespace std;

/////////////////////////////////
// Configuration
/////////////////////////////////
int pktsize = 50;           //size of packets, in bytes
int nrnodes = 1;             //number of nodes, not including the coordinator
double duty_cycle = 0.2;      //proportion of assigned timeslots with actual packets to send
double duration = 10.0;       //simulation total duration, in seconds
bool verbose = false;       //enable logging (different from trace)

//Trace Tx & Rx
double TotalTxByte = 0;
double TotalRxByte = 0;
double TxPkt = 0;
double RxPkt = 0;
double throughputarray[50] = {0};  //throughput
uint32_t packetarray[50]={0};      //packet

void MacTxCallback(Ptr<const Packet> packet){
  //NS_LOG_UNCOND("transmit time:"<<Simulator::Now().GetSeconds()<<" packet size "<<packet->GetSize());
  TotalTxByte += packet->GetSize();
  TxPkt += 1;
}

void MacRxCallback(Ptr<const Packet> packet){
  // NS_LOG_UNCOND("receive time:"<<Simulator::Now().GetSeconds()<<" packet size "<<packet->GetSize());;
  TotalRxByte += packet->GetSize();
  RxPkt += 1;
}
/*
static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND ("Receive time:"<<Simulator::Now().GetSeconds()<<" Received packet of size " << p->GetSize ()<<" from "<<params.m_srcAddr<<" to "<<params.m_dstAddr);
  // NS_LOG_UNCOND ("Source address mode "<<params.m_srcAddrMode<<" Source PAN identifier "<<params.m_srcPanId<<" Source address "<<params.m_srcAddr<<" Destination address mode "<<params.m_dstAddrMode<<" Destination PAN identifier "<<params.m_dstPanId<<" Destination address "<<params.m_dstAddr<<" LQI value measured during reception of the MPDU "<<params.m_mpduLinkQuality<<" The DSN of the received data frame "<<params.m_dsn);
  TotalRxByte += p->GetSize();
  RxPkt += 1;
  // cout<<"in data indication"<<endl;
}
*/
/*
static void DataIndicationTransmit(McpsDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND ("Transmit time:"<<Simulator::Now().GetSeconds()<<" Trasnmit packet of size " << p->GetSize ()<<" from "<<params.m_srcAddr<<" to "<<params.m_dstAddr);
  TotalTxByte += p->GetSize();
  TxPkt += 1;
  cout<<"in data indication trasnmit"<<endl;
}
*/
/*
static void DataConfirm (McpsDataConfirmParams params)
{
  NS_LOG_UNCOND ("LrWpanMcpsDataConfirmStatus = " << params.m_status);
  cout<<"in data confirm"<<endl;
}
*/
//Print Nodes position
void PrintNodePosion(NodeContainer allNodes)
{
  Ptr<MobilityModel> mob;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  for (int i = 0 ; i<nrnodes+1 ;i++)
  {
     mob = allNodes.Get(i)->GetObject<MobilityModel>();
     x = mob->GetPosition().x;
     y = mob->GetPosition().y;
     z = mob->GetPosition().z;
     if(i == 0)
	  std::cout << "Coordinator:"<< " x = "<< x << ", y = " << y << ", z = " << z << "\n";   
     else
     	  std::cout << "Sensor Node ID "<< i << ": x = "<< x << ", y = " << y << ", z = " << z << "\n";   
  }

}

void
CourseChange(std::string context, Ptr<const MobilityModel> model)
{
  Vector position = model->GetPosition();
  NS_LOG_UNCOND(context <<
              " x = " << position.x << ", y = " << position.y);
}

/**
 * \class StackHelper
 * \brief Helper to set or get some IPv6 information about nodes.
 */
class StackHelper
{
public:

  /**
   * \brief Add an address to a IPv6 node.
   * \param n node
   * \param interface interface index
   * \param address IPv6 address to add
using namespace std;
   */
  inline void AddAddress (Ptr<Node>& n, uint32_t interface, Ipv6Address address)
  {
    Ptr<Ipv6> ipv6 = n->GetObject<Ipv6> ();
    ipv6->AddAddress (interface, address);
  }

  /**
   * \brief Print the routing table.
   * \param n the node
   */
  inline void PrintRoutingTable (Ptr<Node>& n)
  {
    Ptr<Ipv6StaticRouting> routing = 0;
    Ipv6StaticRoutingHelper routingHelper;
    Ptr<Ipv6> ipv6 = n->GetObject<Ipv6> ();
    uint32_t nbRoutes = 0;
    Ipv6RoutingTableEntry route;

    routing = routingHelper.GetStaticRouting (ipv6);

    cout << "Routing table of " << n << " : " << endl;
    cout << "Destination\t\t\t\t" << "Gateway\t\t\t\t\t" << "Interface\t" <<  "Prefix to use" << endl;

    nbRoutes = routing->GetNRoutes ();
    for (uint32_t i = 0; i < nbRoutes; i++)
      {
        route = routing->GetRoute (i);
        cout << route.GetDest () << "\t"
                  << route.GetGateway () << "\t"
                  << route.GetInterface () << "\t"
                  << route.GetPrefixToUse () << "\t"
                  << endl;
      }
  }
};

ApplicationContainer nodesapp(NodeContainer lrwpanNodes,Ipv6InterfaceContainer deviceInterfaces,int i)
{
  ApplicationContainer nodeapp;

  Ping6Helper ping6;

  ping6.SetAttribute ("MaxPackets", UintegerValue (duration/(0.01*nrnodes*duty_cycle)));
  ping6.SetAttribute ("Interval", TimeValue (Seconds(0.01*nrnodes/duty_cycle)));
  ping6.SetAttribute ("PacketSize", UintegerValue (pktsize));

  ping6.SetLocal (deviceInterfaces.GetAddress (i, 1)); //set local node 
  ping6.SetRemote (deviceInterfaces.GetAddress (0, 1)); //set remote node(pan coordinator?)
  nodeapp =ping6.Install (lrwpanNodes.Get (i));
  nodeapp.Start(Seconds(0.0));
  nodeapp.Stop (Seconds (duration));

  return nodeapp;

}

int main (int argc, char** argv)
{
  CommandLine cmd;
  cmd.AddValue ("verbose", "Print trace information if true", verbose);
  cmd.AddValue ("nrnodes", "Input number of nodes, not including the coordinator ", nrnodes);
  cmd.Parse (argc, argv);
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  srand(time(NULL));
  
  
  //Enable PCAP and Ascii Tracing
  AsciiTraceHelper ascii;

  //Node create
  NodeContainer panCoord;
  NodeContainer sensors;
  NodeContainer allNodes;

  panCoord.Create (1);
  sensors.Create(nrnodes);
  NodeContainer lrwpanNodes(panCoord,sensors);
  allNodes.Add (lrwpanNodes);

  //Set Mobility
  // MobilityHelper mobility;
  
  // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  // positionAlloc->Add(Vector(50.0, 50.0, 0.0));
  // positionAlloc->Add(Vector(0.0, 0.0, 0.0));
  // mobility.SetPositionAllocator(positionAlloc);
  // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // mobility.Install (allNodes);
  // PrintNodePosion(allNodes);
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "GridWidth", UintegerValue(4),
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5),
                                 "DeltaY", DoubleValue (5),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel","Bounds",RectangleValue(Rectangle(-50,50,-50,50)));
  mobility.Install (allNodes);
  PrintNodePosion(allNodes);

  /*//IPv6
  InternetStackHelper internetv6;
  internetv6.SetIpv4StackInstall (false);
  internetv6.Install (lrwpanNodes);*/


  //Set Channel
  SpectrumChannelHelper channelHelper = SpectrumChannelHelper::Default ();
  channelHelper.SetChannel ("ns3::MultiModelSpectrumChannel");
  Ptr<SpectrumChannel> channel = channelHelper.Create ();

  // //create two netdevice rf lr-wpan-data
  // Ptr<LrWpanNetDevice> dev0 = CreateObject<LrWpanNetDevice> ();
  // Ptr<LrWpanNetDevice> dev1 = CreateObject<LrWpanNetDevice> ();

  // dev0->SetAddress (Mac16Address ("00:01"));
  // dev1->SetAddress (Mac16Address ("00:02"));

  // //set to the same channel
  // dev0->SetChannel (channel);
  // dev1->SetChannel (channel);

  // // To complete configuration, a LrWpanNetDevice must be added to a node
  // panCoord.Get(0)->AddDevice (dev0);
  // sensors.Get(0)->AddDevice (dev1);

  // Ptr<ConstantPositionMobilityModel> sender0Mobility = CreateObject<ConstantPositionMobilityModel> ();
  // sender0Mobility->SetPosition (Vector (0,0,0));
  // dev0->GetPhy ()->SetMobility (sender0Mobility);
  // Ptr<ConstantPositionMobilityModel> sender1Mobility = CreateObject<ConstantPositionMobilityModel> ();
  // // Configure position 10 m distance
  // sender1Mobility->SetPosition (Vector (0,10,0));
  // dev1->GetPhy ()->SetMobility (sender1Mobility);

  // // Send a packet back at time 2 seconds
  // McpsDataRequestParams params;
  // Ptr<Packet> p2 = Create<Packet> (60);  // 60 bytes of dummy data
  // params.m_dstAddr = Mac16Address ("00:01");
  // Simulator::ScheduleWithContext (2, Seconds (2.0),
  //                                 &LrWpanMac::McpsDataRequest,
  //                                 dev1->GetMac (), params, p2);

  //set channel rf lr-wpan-data
  // Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  // Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  // Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  // channel->AddPropagationLossModel (propModel);
  // channel->SetPropagationDelayModel (delayModel);

  //LrWpan Helper
  LrWpanTschHelper lrWpanHelper(channel,nrnodes+1,false,true);
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("tsch-nodes.tr");
  Ptr<OutputStreamWrapper> stream2 = ascii.CreateFileStream ("tsch-nodes-energy.tr");

  //Set Net Device
  NetDeviceContainer netdev = lrWpanHelper.Install (lrwpanNodes);


  /*//assign ipv4 address
  InternetStackHelper stack;
  stack.Install (lrwpanNodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");

  Ipv4InterfaceContainer interfaces = address.Assign (netdev);*/
  
  //Logger
  if(verbose)
  {
    // LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
    // LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    // LogComponentEnableAll (LOG_PREFIX_TIME);
    // LogComponentEnableAll (LOG_PREFIX_FUNC);
    // LogComponentEnable ("LrWpanTschMac", LOG_LEVEL_ALL);
    lrWpanHelper.EnableLogComponents();
  }

  //set up udp echo server
  // UdpEchoServerHelper echoServer (9);
  // ApplicationContainer serverApps = echoServer.Install (allNodes.Get (1));
  // serverApps.Start (Seconds (1.0));
  // serverApps.Stop (Seconds (10.0));
  
  //lrWpanHelper.EnablePcapAll ("tsch-nodes");
  lrWpanHelper.EnablePcap("tsch-nodes-pan",panCoord,true);
  lrWpanHelper.EnablePcap("tsch-nodes-sensor",sensors,true);
  lrWpanHelper.EnableAsciiAll (stream);
  
  EnergySourceContainer enc = lrWpanHelper.InstallEnergySource(lrwpanNodes);
  DeviceEnergyModelContainer devenc = lrWpanHelper.InstallEnergyDevice(netdev, enc);
  // lrWpanHelper.EnableEnergyAllPhy(stream2,enc); //original
  // lrWpanHelper.EnableReceivePower(stream2,lrwpanNodes);
  // lrWpanHelper.EnableEnergyAll(stream2);

  //Assiciate FFD RFD
  lrWpanHelper.AssociateToPan(netdev,123);     //Device , PAN id
  lrWpanHelper.ConfigureSlotframeAllToPan(netdev,0,true,false);//devices,empty slots,bidirectional=true,use broadcast cells=false
  lrWpanHelper.EnableTsch(netdev,0.0,duration);

  //CBR traffic
  //int xnode;
  // lrWpanHelper.GenerateTraffic(netdev.Get(1),netdev.Get(0)->GetAddress(),pktsize,5.0,duration,0.5);
  // lrWpanHelper.GenerateTraffic(netdev.Get(2),netdev.Get(0)->GetAddress(),pktsize,5.0,duration,0.5);
  // lrWpanHelper.GenerateTraffic(netdev.Get(3),netdev.Get(0)->GetAddress(),pktsize,5.0,duration,0.5);
  // lrWpanHelper.GenerateTraffic(netdev.Get(4),netdev.Get(0)->GetAddress(),pktsize,5.0,duration,0.5);
  // lrWpanHelper.GenerateTraffic(netdev.Get(5),netdev.Get(0)->GetAddress(),pktsize,5.0,duration,0.5);
  for (int i = 1; i <= nrnodes; i++)
  {
    // while(1)
    // {
    //   xnode=rand()%nrnodes;
    //   if (xnode!=i)
    //   {
    //     cout<<"xnode "<<xnode<<endl;
    //     break;
    //   }
    // }
    // lrWpanHelper.GenerateTraffic(netdev.Get(i),netdev.Get(xnode)->GetAddress(),pktsize,1.0,10.0,0.5); //random transmit
    lrWpanHelper.GenerateTraffic(netdev.Get(i),netdev.Get(0)->GetAddress(),pktsize,5.0,duration,0.5);  //transmit to coordiantor
    // lrWpanHelper.GenerateTraffic(netdev.Get(i),netdev.Get(0)->GetAddress(),pktsize,1.0,duration,0.025);  //transmit to coordiantor
    // lrWpanHelper.GenerateTraffic(netdev.Get(i-1),netdev.Get(i)->GetAddress(),pktsize,3.0,10.0,0.5);  //transmit to next node
  }

  /////////////////////////////////
  // Upper layers
  /////////////////////////////////
  //IPv6
  // InternetStackHelper internetv6;
  // internetv6.SetIpv4StackInstall (true);
  // internetv6.Install (lrwpanNodes);
  // //Sixlowpan stack
  // // StackHelper stackHelper;
  // SixLowPanHelper sixlowpan;
  // NetDeviceContainer devices = sixlowpan.Install (netdev);

  // Ipv6AddressHelper ipv6;
  // ipv6.SetBase (Ipv6Address("2001:1::"), Ipv6Prefix (64));
  // Ipv6InterfaceContainer deviceInterfaces = ipv6.Assign (devices);

  // ApplicationContainer apps;

  // for (int i=1;i<=nrnodes;i++) //original is i=1 i change to i=0
  // {
  //   apps.Add (nodesapp(lrwpanNodes,deviceInterfaces,i)); //used to add application
  // }


  // Ptr<LrWpanTschNetDevice> dev0 = netdev.Get(0)->GetObject<LrWpanTschNetDevice> ();
  // Ptr<LrWpanTschNetDevice> dev1 = netdev.Get(1)->GetObject<LrWpanTschNetDevice> ();
  // dev0->GetNMac()->TraceConnectWithoutContext("MacTx",MakeCallback(&MacTxCallback));
  // dev1->GetNMac()->TraceConnectWithoutContext("MacRx",MakeCallback(&MacRxCallback));

  //Rx
  // for (int i = 0; i <= nrnodes; i++)
  // {
  //   Ptr<LrWpanTschNetDevice> dev_0 = netdev.Get(i)->GetObject<LrWpanTschNetDevice> ();
  //   dev_0->GetNMac()->TraceConnectWithoutContext("MacRx",MakeCallback(&MacRxCallback));
  // }
  Ptr<LrWpanTschNetDevice> dev_0 = netdev.Get(0)->GetObject<LrWpanTschNetDevice> ();
  dev_0->GetNMac()->TraceConnectWithoutContext("MacRx",MakeCallback(&MacRxCallback));
  // McpsDataIndicationCallback cb1;
  // cb1 = MakeCallback (&DataIndication);
  // dev_0->GetNMac ()->SetMcpsDataIndicationCallback (cb1);

  //Tx
  for (int i = 1; i <= nrnodes; i++) // 0 or 1
  {
    Ptr<LrWpanTschNetDevice> dev = netdev.Get(i)->GetObject<LrWpanTschNetDevice> ();
    dev->GetNMac()->TraceConnectWithoutContext("MacTx",MakeCallback(&MacTxCallback));
    // McpsDataIndicationCallback cb2;
    // cb2 = MakeCallback (&DataIndicationTransmit);
    // dev->GetNMac ()->SetMcpsDataIndicationCallback (cb2);
  
  }

  //print node mobility  
  Ptr<Object> theObject = allNodes.Get(1);
  theObject->TraceConnectWithoutContext("CourseChange", MakeCallback(&CourseChange));

  
  //data confrim and inidcation callback
  // McpsDataConfirmCallback cb0;
  // cb0 = MakeCallback (&DataConfirm);
  // dev0->GetMac ()->SetMcpsDataConfirmCallback (cb0);

  // McpsDataIndicationCallback cb1;
  // cb1 = MakeCallback (&DataIndication);
  // dev0->GetMac ()->SetMcpsDataIndicationCallback (cb1);

  // McpsDataConfirmCallback cb2;
  // cb2 = MakeCallback (&DataConfirm);
  // dev1->GetMac ()->SetMcpsDataConfirmCallback (cb2);

  // McpsDataIndicationCallback cb3;
  // cb3 = MakeCallback (&DataIndication);
  // dev1->GetMac ()->SetMcpsDataIndicationCallback (cb3);
  
  
  /*//IPv6
  InternetStackHelper internetv6;
  internetv6.SetIpv4StackInstall (false);
  internetv6.Install (lrwpanNodes);
    
  //Sixlowpan stack
  StackHelper stackHelper;
  SixLowPanHelper sixlowpan;
  NetDeviceContainer devices = sixlowpan.Install (netdev);

  Ipv6AddressHelper ipv6;
  ipv6.SetBase (Ipv6Address("2001:1::"), Ipv6Prefix (64));
  Ipv6InterfaceContainer deviceInterfaces = ipv6.Assign (devices);

  for (int i = 1;i<=nrnodes;i++) {
	  deviceInterfaces.SetForwarding(i,true);
	  deviceInterfaces.SetDefaultRoute(i,0);
  }

  for (int i=0;i<=nrnodes;i++) {
	Ptr<Node> n=lrwpanNodes.Get(i);
  	cout << "=== node " << deviceInterfaces.GetAddress(i,0) 
	     << " (IEEE " << DynamicCast<LrWpanTschNetDevice>(netdev.Get(i))->GetMac()->GetShortAddress() << ")"
	     << " ===" << endl;
	stackHelper.PrintRoutingTable (n);
  }*/


  //Create flow monitor
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  monitor=flowmon.InstallAll();

  Simulator::Run ();

  // for (uint32_t i = 0; i < apps.GetN(); i++)
  // {
    // std::cout<<"i "<<i<<" "<<DynamicCast<UdpServer> (apps.Get(i))<<std::endl;
    // DynamicCast<UdpServer> (apps.Get(i))->GetNode();
    // uint32_t Packets = DynamicCast<UdpServer> (apps.Get(i)) ->GetReceived ();
    // packetarray[i]+=Packets;
  // }

  // for (uint32_t i = 0; i < apps.GetN(); i++)
  // {
  //   std::cout<<"Packets quantity "<<i<<":"<<packetarray[i]<<std::endl;
  // }

  //Print per-flow statistics
  // monitor->SerializeToXmlFile("tsch-nodes.xml",true,true);
  // monitor->CheckForLostPackets();
  
  Simulator::Destroy ();

  std::cout<< "Total Tx Bits = " << TotalTxByte*8 <<"\n";
  std::cout<< "Total Rx Bits = " << TotalRxByte*8 <<"\n";
  std::cout<< "Total Tx Pkt = " << TxPkt <<"\n";
  std::cout<< "Total Rx Pkt = " << RxPkt <<"\n";
  std::cout<< "Packet Deliver Ratio = " << (RxPkt / TxPkt) * 100 <<" %\n";
  cout<<"Throughput = "<<(TotalRxByte*8)/duration<<endl;
}
