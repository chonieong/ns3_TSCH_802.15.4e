/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 CTTC
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
 * Author: Nicola Baldo <nbaldo@cttc.es>
 * Author: Luis Pacheco <luisbelem@gmail.com>
 * Author: Peter Kourzanov <peter.kourzanov@gmail.com>
 */
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
#include <ns3/flow-monitor-helper.h> //change by ding
#include <ns3/ipv6-flow-classifier.h> //change by ding
#include <ns3/ipv6-flow-probe.h> //change by ding


#include "ns3/internet-module.h"
#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/ipv6-routing-table-entry.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/sixlowpan-module.h"
//#include "ns3/netanim-module.h"

NS_LOG_COMPONENT_DEFINE ("TschTest");

using namespace ns3;
using namespace std;

/////////////////////////////////
// Configuration
/////////////////////////////////
int pktsize = 60;           //size of packets, in bytes
// int pktsize = 20; 
int nrnodes = 20;             //number of nodes, not including the coordinator
double duty_cycle = 0.1;      //proportion of assigned timeslots with actual packets to send
double duration = 20;       //simulation total duration, in seconds
bool verbose = false;       //enable logging (different from trace)
bool interference = false;  //enable wifi interference
bool generator=true;        //generate others traffic
bool addnewnode=false;         //add new node
bool mobilitymode=false;           //enable mobilitymode //ieong
bool Treetopo=false;                // true is tree topo. false is star topo // ieong
double throughputarray[50] = {0};  //throughput
uint32_t packetarray[50]={0};      //packet
double interval=0.025;             //interval between each node send
//double interval=0.5;
/////////////////////////////////
// End configuration
/////////////////////////////////
void SingleWifiPacket(NodeContainer ofdmNodes, Ptr<SpectrumChannel> channel)
{
  WifiSpectrumValue5MhzFactory sf;
  double txPower = 0.1; // Watts
  WaveformGeneratorHelper waveformGeneratorHelper;
  Ptr<SpectrumValue> txPsd = sf.CreateTxPowerSpectralDensity (txPower, 6);

  
  waveformGeneratorHelper.SetTxPowerSpectralDensity (txPsd);
  waveformGeneratorHelper.SetChannel (channel);
  waveformGeneratorHelper.SetPhyAttribute ("Period", TimeValue (Seconds (1)));
  waveformGeneratorHelper.SetPhyAttribute ("DutyCycle", DoubleValue (0.00022637));
  NetDeviceContainer waveformGenerator0 = waveformGeneratorHelper.Install (ofdmNodes.Get(0));
  
  //Data  //some bug here
  Simulator::Schedule (MicroSeconds(101.5), &WaveformGenerator::Start, waveformGenerator0.Get (0)->GetObject<NonCommunicatingNetDevice> ()->GetPhy ()->GetObject<WaveformGenerator> ());
  Simulator::Schedule (MicroSeconds(101.5+226.37), &WaveformGenerator::Stop, waveformGenerator0.Get (0)->GetObject<NonCommunicatingNetDevice> ()->GetPhy ()->GetObject<WaveformGenerator> ());
  
  waveformGeneratorHelper.SetPhyAttribute ("Period", TimeValue (Seconds (1)));
  waveformGeneratorHelper.SetPhyAttribute ("DutyCycle", DoubleValue (0.00002207));

  NetDeviceContainer waveformGenerator1 = waveformGeneratorHelper.Install (ofdmNodes.Get(1));
  
  // ACK
   //Simulator::Schedule (MicroSeconds(101.5+226.37+16), &WaveformGenerator::Start, waveformGenerator1.Get (0)->GetObject<NonCommunicatingNetDevice> ()->GetPhy ()->GetObject<WaveformGenerator> ());
   //Simulator::Schedule (MicroSeconds(101.5+226.37+16+22.07), &WaveformGenerator::Stop, waveformGenerator1.Get (0)->GetObject<NonCommunicatingNetDevice> ()->GetPhy ()->GetObject<WaveformGenerator> ());
   // cout<<"here\n";
   //Simulator::Schedule (MicroSeconds(101.5+226.37+16+22.07), &SingleWifiPacket, ofdmNodes, channel);
}

void EnableTsch(LrWpanTschHelper* lrWpanHelper,NetDeviceContainer& netdev)
{
  lrWpanHelper->EnableTsch(netdev,0,duration);
}

void Addassociatetopan(LrWpanTschHelper* addlrWpanHelper,NetDeviceContainer& addnetdev)
{
  addlrWpanHelper->AssociateToPan(addnetdev,123);
}

void addconfigureslotframealltopan(LrWpanTschHelper* addlrWpanHelper,NetDeviceContainer& addnetdev)
{
  addlrWpanHelper->ConfigureSlotframeAllToPan(addnetdev,0,true,false);//devices,empty slots,bidirectional=true,use broadcast cells=false
}

void addnetdevice(LrWpanTschHelper* addlrWpanHelper,NetDeviceContainer& addnetdev,NodeContainer& addnode)
{
  addnetdev = addlrWpanHelper->Install (addnode);
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

// not print first  chon
    cout << "Routing table of " << n << " : " << endl;
    cout << "Destination\t\t\t\t" << "Gateway\t\t\t\t\t" << "Interface\t" <<  "Prefix to use" << endl;

    nbRoutes = routing->GetNRoutes ();
    for (uint32_t i = 0; i < nbRoutes; i++)
      {
        route = routing->GetRoute (i);
        cout << "Dest: "<<route.GetDest () << "\tGateway: "
                  << route.GetGateway () << "\tInterface: "
                  << route.GetInterface () << "\tPrefixToUse: "
                  << route.GetPrefixToUse () << "\t"
                  << endl;
      }
  }
};


//Trace Tx & Rx
double TotalTxByte = 0;
double TotalRxByte = 0;
double TxPkt = 0;
double RxPkt = 0;

void MacTxCallback(Ptr<const Packet> packet){
   //NS_LOG_UNCOND("t "<<Simulator::Now().GetSeconds()<<" "<<packet->GetSize());
  if(packet->GetSize()==69)
  {
    
    TotalTxByte += packet->GetSize();
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()<<"s TotalTxByte: "<<TotalTxByte);
    //std::cout<< "Total Rx Bits = " << TotalRxByte*8 <<endl;
    TxPkt += 1;
  }
   //TotalTxByte += packet->GetSize();
   //TxPkt += 1;
}

void MacRxCallback(Ptr<const Packet> packet){
   //NS_LOG_UNCOND("r "<<Simulator::Now().GetSeconds()<<" "<<packet->GetSize());
  if(packet->GetSize()==69)
  {
    // NS_LOG_UNCOND("r "<<Simulator::Now().GetSeconds()<<" "<<packet->GetSize());
    TotalRxByte += packet->GetSize();
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()<<"s TotalRxByte: "<<TotalRxByte);

    RxPkt += 1;
  }
   //TotalRxByte += packet->GetSize();
   //RxPkt += 1;
}
/*
void MacChannelBusyCallback(uint32_t obj){

  NS_LOG_UNCOND(Simulator::Now().GetSeconds()<<"s obj: "<< obj);

}

void MacLinkInformationCallback(uint32_t m_macTxID,  uint32_t m_macRxID, uint8_t m_currentChannel, double m_currentReceivedPower, double m_currentFadingBias){
  cout<<"abc"<<endl;
  NS_LOG_UNCOND(Simulator::Now().GetSeconds()<<"s  m_macTxID : "<<m_macTxID << "m_macRxID : "<< m_macRxID << "m_currentChannel : " << m_currentChannel <<"m_currentReceivedPower : "<<m_currentReceivedPower<<" m_currentFadingBias : "<< m_currentFadingBias);


}
*/

void MacSentPktCallback(uint32_t obj){

  NS_LOG_UNCOND(Simulator::Now().GetSeconds()<<"s obj: "<< obj);

}


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
                              //Address serveraddress
ApplicationContainer nodesapp(NodeContainer lrwpanNodes,Ipv6InterfaceContainer deviceInterfaces,int i)
{
  //set up udp echo client
  // UdpEchoClientHelper echoClient(serveraddress,9);

  ApplicationContainer nodeapp;

  Ping6Helper ping6;

  ping6.SetAttribute ("MaxPackets", UintegerValue (duration/(0.01*nrnodes*duty_cycle)));
  ping6.SetAttribute ("Interval", TimeValue (Seconds(0.01*nrnodes/duty_cycle)));
  ping6.SetAttribute ("PacketSize", UintegerValue (pktsize));

  ping6.SetLocal (deviceInterfaces.GetAddress (i, 1)); //set local node 
  ping6.SetRemote (deviceInterfaces.GetAddress (0, 1)); //set remote node(pan coordinator?)
  nodeapp =ping6.Install (lrwpanNodes.Get (i));

  //set up echo client
  // echoClient.SetAttribute("MaxPackets", UintegerValue (duration/(0.01*nrnodes*duty_cycle)));
  // echoClient.SetAttribute("Interval", TimeValue (Seconds(0.01*nrnodes/duty_cycle)));
  // echoClient.SetAttribute("PacketSize", UintegerValue(pktsize));
  // nodeapp=echoClient.Install(lrwpanNodes.Get(i));
  nodeapp.Start(Seconds(0.0));
  nodeapp.Stop (Seconds (duration));

  return nodeapp;

}

void
CourseChange(std::string context, Ptr<const MobilityModel> model)
{
  Vector position = model->GetPosition();
  NS_LOG_UNCOND("At time " << Simulator::Now ().GetSeconds () << " " << context <<
              " x = " << position.x << ", y = " << position.y);
  
}

void StarTopology(NodeContainer panCoord, NodeContainer sensors, MobilityHelper mobility){
  
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "GridWidth", UintegerValue(4),
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5),
                                 "DeltaY", DoubleValue (5),
                                 "LayoutType", StringValue ("RowFirst"));
                               
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility.Install (panCoord);
  if(mobilitymode)
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel","Bounds",RectangleValue(Rectangle(-50,50,-50,50)));
  mobility.Install(sensors);
 
  

}

void TreeTopology(NodeContainer allNodes, MobilityHelper mobility){

  int k = nrnodes + 1;
  int layers = 0;
  int layers_num = 1;
  double length = 0;
  int addto = 0;
  double m = 0;
  //cout<< "k : " << k <<endl;
  while(k>0){

    k/=2;
    layers++;
  }
  k = nrnodes + 1;
  //cout<< "layers : "<< layers << endl;
  //tr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  //int Xpos = 0;
  for ( int i =0; i<layers; i++){
    if(i!=0)
    {
      if(layers_num == 1)
        m = 0;
      else
        m = length/(layers_num-1);
    }
     
    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "GridWidth", UintegerValue(layers_num),
                                 "MinX", DoubleValue (0.0 - length/2 ),
                                 "MinY", DoubleValue (i*5),
                                 "DeltaX", DoubleValue (m),
                                 "DeltaY", DoubleValue (0),
                                 "LayoutType", StringValue ("RowFirst"));
                               
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    for ( int j=0; j< layers_num; j++){
      mobility.Install (allNodes.Get(addto));
      addto++;
      k--;
    }
    //cout<< "k : " << k << endl;
    if(layers_num*2 <= k)
      layers_num *= 2;
    else
      layers_num = k;
    //cout<< "layers_num : " << k << endl;
    length *= 2;
    if(i==0)       // form layer 1 length 0 , to layer 2 length 10
      length = 10;    
  }
  
}



int main (int argc, char** argv)
{
  CommandLine cmd;
  cmd.AddValue ("verbose", "Print trace information if true", verbose);
  cmd.AddValue ("nrnodes", "Input number of nodes, not including the coordinator ", nrnodes);
  cmd.AddValue ("interference", "Enable wifi interference ", interference);
  cmd.AddValue("generator","Generate others traffic",generator);
  cmd.AddValue("addnewnode","Add new node",addnewnode);
  cmd.AddValue("mobilitymode","Enable mobilitymode",mobilitymode); //ieong
  cmd.Parse (argc, argv);
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  srand(time(NULL));

  
  

  //Enable PCAP and Ascii Tracing
  AsciiTraceHelper ascii;

  //Schedule wifi device
  NodeContainer ofdmNodes;
  NodeContainer panCoord;
  NodeContainer sensors;
  NodeContainer allNodes;
  

  panCoord.Create (1);
  sensors.Create(nrnodes);
  NodeContainer lrwpanNodes(panCoord,sensors);
  //ofdmNodes number change from 1 to 2
  ofdmNodes.Create (1);
  allNodes.Add (lrwpanNodes);
  allNodes.Add (ofdmNodes);

  /////////////////////////////////
  // Mobility
  /////////////////////////////////
  MobilityHelper mobility;
  if(Treetopo)
    TreeTopology(allNodes,mobility);            // for tree topo //ieong  // 1 2 4 8 16 32
  else
    StarTopology(panCoord, sensors, mobility);  // for star topology   //ieong
  
  // not display first // chon
  PrintNodePosion(allNodes);

  //print node mobility  
  Ptr<Object> theObject = sensors.Get(0);
  theObject->TraceConnectWithoutContext("CourseChange", MakeCallback(&CourseChange));



   //MobilityHelper mobility;
   //Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
   //positionAlloc->Add(Vector(50.0, 50.0, 0.0));
   //positionAlloc->Add(Vector(0.0, 0.0, 0.0));
   //mobility.SetPositionAllocator(positionAlloc);
   //mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   //mobility.Install (allNodes);
   //PrintNodePosion(allNodes);

  

  /////////////////////////////////
  // Upper layers
  /////////////////////////////////
  //IPv6
  InternetStackHelper internetv6;
  internetv6.SetIpv4StackInstall (true);
  internetv6.Install (lrwpanNodes);
  /////////////////////////////////
  // Channel
  /////////////////////////////////

  SpectrumChannelHelper channelHelper = SpectrumChannelHelper::Default ();
  channelHelper.SetChannel ("ns3::MultiModelSpectrumChannel");
  Ptr<SpectrumChannel> channel = channelHelper.Create ();

  /////////////////////////////////
  // Configure lrwpan nodes
  /////////////////////////////////

  LrWpanTschHelper lrWpanHelper(channel,nrnodes+1,false,true);
  // ieong // not first
  
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("lr-wpan-tsch-test.tr");
  Ptr<OutputStreamWrapper> stream2 = ascii.CreateFileStream ("lr-wpan-tsch-test-energy.tr"); //change by ding
  Ptr<OutputStreamWrapper> fstream = ascii.CreateFileStream ("lr-wpan-tsch-test.fading");
  lrWpanHelper.PrintFadingBiasValues(fstream); 

  NetDeviceContainer netdev = lrWpanHelper.Install (lrwpanNodes);

  

  //logger
  //ieong // not first
  lrWpanHelper.EnablePcapAll (string ("lr-wpan-tsch-test"), true);
  lrWpanHelper.EnableAsciiAll (stream);

  //change by ding
  EnergySourceContainer enc = lrWpanHelper.InstallEnergySource(lrwpanNodes);
  DeviceEnergyModelContainer devenc = lrWpanHelper.InstallEnergyDevice(netdev, enc);
  // lrWpanHelper.EnableEnergyAllPhy(stream2,enc); //original
  // lrWpanHelper.EnableReceivePower(stream2,lrwpanNodes);
  // lrWpanHelper.EnableEnergyAll(stream2);

  if(verbose)
  {
    LogComponentEnableAll (LOG_PREFIX_TIME);
    LogComponentEnableAll (LOG_PREFIX_FUNC);
    LogComponentEnable ("LrWpanTschMac", LOG_LEVEL_ALL);
    //LogComponentEnable ("LrWpanNetDevice", LOG_LEVEL_ALL);   //netdevice 
    //LogComponentEnable ("LrWpanCsmaCa", LOG_LEVEL_ALL);
    // lrWpanHelper.EnableLogComponents();
  }

  
  lrWpanHelper.AssociateToPan(netdev,123);
  //lrWpanHelper.ConfigureSlotframeAllToPan(netdev,0,true,false);//devices,empty slots,bidirectional=true,use broadcast cells=false use in initial
  //lrWpanHelper.ConfigureSlotframeAllToPan(netdev,0,true,true); //change by ding,broadcast cells=true
  // lrWpanHelper.ConfigureSlotframeAllToPan(netdev,0,false,true); //change by ding,bidirectional=flase,broadcast cells=true use in csmaca

  if(Treetopo){
    lrWpanHelper.ConfigureSlotframeAllToPanForTree(netdev,0,false,false,nrnodes);
  }
  else{
    lrWpanHelper.ConfigureSlotframeAllToPan(netdev,0,true,false); //change by ding,bidirectional=false,use broadcast cells=false use in slot frame size adjust
  }

  Simulator::Schedule(Seconds(5),&EnableTsch,&lrWpanHelper,netdev);

  //change timeslot size for different interval use in timeslot adjust
   //if(interval>0.1)
   //{
     //lrWpanHelper.ModSlotframe(netdev,0,netdev.GetN()+3);
   //}else
   //{
     //lrWpanHelper.ModSlotframe(netdev,0,netdev.GetN()-1);
   //}

  
  //double generate_starttime = 5.0;
  //double duration_generate = duration - Now().GetSeconds();
  //generate traffic
  if(generator)
  {
    //int xnode;
    // lrWpanHelper.GenerateTraffic(netdev.Get(2),netdev.Get(4)->GetAddress(),pktsize,3.0,5.0,0.01*nrnodes/duty_cycle); //generate traffic from 0x0003 to 0x0005
    // lrWpanHelper.GenerateTraffic(netdev.Get(4),netdev.Get(0)->GetAddress(),pktsize,3.0,5.0,0.5);
    // lrWpanHelper.GenerateTraffic(netdev.Get(4),netdev.Get(2)->GetAddress(),pktsize,4.0,8.0,0.5);
    // use in csmaca
    // lrWpanHelper.GenerateTraffic(netdev.Get(1),netdev.Get(0)->GetAddress(),pktsize,5.0,10.0,0.025);
    // for (int i = 2; i <= nrnodes; i++)
    // {
    //   lrWpanHelper.GenerateTraffic(netdev.Get(i),netdev.Get(0)->GetAddress(),pktsize,5.0,10.0,0.5); //transmit to coordiantor
    // }


    // LrWpanTschHelper::GenerateTraffic(Ptr<NetDevice> dev, Address dst, int packet_size, double start, double duration, double interval)

    
    
    for (int i = 1; i <= nrnodes; i++)
    {
      // xnode=rand()%nrnodes;
      // cout<<"xnode "<<xnode<<endl;
      // lrWpanHelper.GenerateTraffic(netdev.Get(i),netdev.Get(xnode)->GetAddress(),pktsize,5.0,10.0,interval); //random transmit
      // lrWpanHelper.GenerateTraffic(netdev.Get(i),netdev.Get(0)->GetAddress(),pktsize,5.0,10.0,interval); //transmit to coordiantor


      lrWpanHelper.GenerateTraffic(netdev.Get(i),netdev.Get(0)->GetAddress(),pktsize,5.0,10.0,interval); //transmit to coordiantor
      //lrWpanHelper.GenerateTraffic(netdev.Get(0),netdev.Get(i)->GetAddress(),pktsize,5.0,10.0,interval); //coordinator transmit to each node

      //lrWpanHelper.GenerateTraffic(netdev.Get(i-1),netdev.Get(i)->GetAddress(),pktsize,3.0,10.0,interval);  //transmit to next node
    }
    
    /*
    lrWpanHelper.GenerateTraffic(netdev.Get(3),netdev.Get(1)->GetAddress(),pktsize,5.0,10.0,interval); //
    lrWpanHelper.GenerateTraffic(netdev.Get(4),netdev.Get(1)->GetAddress(),pktsize,5.0,10.0,interval); //

    lrWpanHelper.GenerateTraffic(netdev.Get(5),netdev.Get(2)->GetAddress(),pktsize,5.0,10.0,interval); //
    lrWpanHelper.GenerateTraffic(netdev.Get(6),netdev.Get(2)->GetAddress(),pktsize,5.0,10.0,interval); //
    */


  }
  

  /////////////////////////////////
  // Upper layers
  /////////////////////////////////
  // Sixlowpan stack
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
  }


  // Ping6
  Ping6Helper ping6;

  ping6.SetAttribute ("MaxPackets", UintegerValue (duration/(0.01*nrnodes*duty_cycle)));
  ping6.SetAttribute ("Interval", TimeValue (Seconds(0.01*nrnodes/duty_cycle)));
  ping6.SetAttribute ("PacketSize", UintegerValue (pktsize));

  ApplicationContainer apps;

  for (int i=0;i<=nrnodes;i++) //original is i=1 i change to i=0
  {
    //ping6.SetLocal (deviceInterfaces.GetAddress (i, 1)); //set local node 
    //ping6.SetRemote (deviceInterfaces.GetAddress (0, 1)); //set remote node(pan coordinator?)
    //apps.Add(ping6.Install (lrwpanNodes.Get (i)));
    //ApplicationContainer nodesapp =ping6.Install (lrwpanNodes.Get (i));
    apps.Add (nodesapp(lrwpanNodes,deviceInterfaces,i)); //used to add application
  }


  //set up udp echo server
  // UdpEchoServerHelper echoServer (9);
  // ApplicationContainer serverApps = echoServer.Install (panCoord.Get (0));
  // serverApps.Start (Seconds (1.0));
  // serverApps.Stop (Seconds (10.0));

  // Address serveraddress;
  // serveraddress=Address(deviceInterfaces.GetAddress(1,1));



  

  //set strat time and stop time
/*  
   for (int i=1; i<=nrnodes; i++) {
     
     // can't run? below line
     //apps.Get(i-1)->SetStartTime (Seconds (i-1));
     //
     apps.Get(i-1)->SetStartTime (Seconds (0));
   }
   apps.Stop (Seconds (duration));
*/

  if (interference) SingleWifiPacket(ofdmNodes, channel);


  //Rx
/*
   for (int i = 0; i <= nrnodes; i++)
   {
     Ptr<LrWpanTschNetDevice> dev0 = netdev.Get(i)->GetObject<LrWpanTschNetDevice> ();
     dev0->GetNMac()->TraceConnectWithoutContext("MacRx",MakeCallback(&MacRxCallback));
   }
*/

//only need coordinator? 
  Ptr<LrWpanTschNetDevice> dev0 = netdev.Get(0)->GetObject<LrWpanTschNetDevice> ();
  dev0->GetNMac()->TraceConnectWithoutContext("MacRx",MakeCallback(&MacRxCallback));

  
  //Tx
  for (int i = 0; i <= nrnodes; i++) //i=0 or i=1
  {
    Ptr<LrWpanTschNetDevice> dev = netdev.Get(i)->GetObject<LrWpanTschNetDevice> ();
    dev->GetNMac()->TraceConnectWithoutContext("MacTx",MakeCallback(&MacTxCallback));
  }


  /////////////////////////////////
  // Start and finish the simulation
  /////////////////////////////////
  //AnimationInterface anim("test.xml");

  //Create flow monitor
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  monitor=flowmon.InstallAll();

  
  // ieong // print node position
  //not print first 
  
  for ( int i =0;i<nrnodes;i++ ){
    std::ostringstream oss;
    oss << "/NodeList/"
        << sensors.Get(i)->GetId()
        << "/$ns3::MobilityModel/CourseChange";
    
    Config::Connect(oss.str(), MakeCallback(&CourseChange));
  }
  /*
  for (int i = 0; i <= nrnodes; i++) 
  {
    Ptr<LrWpanTschNetDevice> dev123 = netdev.Get(i)->GetObject<LrWpanTschNetDevice> ();
    dev123->GetNMac()->TraceConnectWithoutContext("MacChannelBusy",MakeCallback(&MacChannelBusyCallback));

    //Ptr<Object> theObject1 = sensors.Get(i);
    //theObject1->TraceConnectWithoutContext("MacTxDrop",MakeCallback(&MacTxDropCallback));


  }
  */
  Simulator::Run ();

  Simulator::Destroy ();

  std::cout<< "generator = " << generator <<endl;
  std::cout<< "nrnodes = " << nrnodes <<endl;
  //std::cout<< "generate_starttime = " << generate_starttime <<endl;
  std::cout<< "duration = " << duration <<endl;
  std::cout<< "interval = " << interval <<endl;
  std::cout<< "mobilitymode = " << mobilitymode <<endl;
  
  //std::cout<< "duration_generate = " << duration_generate <<endl;


  std::cout<< "Total Tx Bits = " << TotalTxByte*8 <<endl;
  std::cout<< "Total Rx Bits = " << TotalRxByte*8 <<endl;
  std::cout<< "Total Tx Pkt = " << TxPkt <<endl;
  std::cout<< "Total Rx Pkt = " << RxPkt <<endl;
  std::cout<< "Packet Deliver Ratio = " << (RxPkt / TxPkt) * 100 <<" %"<<endl;
  cout<<"Pakcet Loss = "<<TxPkt-RxPkt<<endl;
  cout<<"Throughput = "<<(TotalRxByte*8)/duration<<endl;

  

  return 0;
}

