#include <iostream>
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
int nrnodes = 2;             //number of nodes, not including the coordinator
//double duty_cycle = 0.2;      //proportion of assigned timeslots with actual packets to send
double duration = 10.0;       //simulation total duration, in seconds
bool verbose = false;       //enable logging (different from trace)

//Trace Tx & Rx
double TotalTxByte = 0;
double TotalRxByte = 0;
double TxPkt = 0;
double RxPkt = 0;

void MacTxCallback(Ptr<const Packet> packet){
  NS_LOG_UNCOND("t "<<Simulator::Now().GetSeconds()<<" "<<packet->GetSize());
  TotalTxByte += packet->GetSize();
  TxPkt += 1;
}

void MacRxCallback(Ptr<const Packet> packet){
  NS_LOG_UNCOND("r "<<Simulator::Now().GetSeconds()<<" "<<packet->GetSize());;
  TotalRxByte += packet->GetSize();
  RxPkt += 1;
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

int main (int argc, char** argv)
{
  CommandLine cmd;
  cmd.AddValue ("verbose", "Print trace information if true", verbose);
  cmd.Parse (argc, argv);
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

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
  MobilityHelper mobility;
  
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector(50.0, 50.0, 0.0));  // Coordinator
  positionAlloc->Add(Vector(0.0, 0.0, 0.0));    // Node 1
  positionAlloc->Add(Vector(50.0, 0.0, 0.0));    // Node 2
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (allNodes);
  PrintNodePosion(allNodes);

  //Set Channel
  SpectrumChannelHelper channelHelper = SpectrumChannelHelper::Default ();
  channelHelper.SetChannel ("ns3::MultiModelSpectrumChannel");
  Ptr<SpectrumChannel> channel = channelHelper.Create ();

  //LrWpan Helper
  LrWpanTschHelper lrWpanHelper(channel,nrnodes+1,false,true);
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("tsch-three-nodes.tr");
  Ptr<OutputStreamWrapper> stream2 = ascii.CreateFileStream ("tsch-three-nodes-energy.tr");

  //Set Net Device
  NetDeviceContainer netdev = lrWpanHelper.Install (lrwpanNodes);
  
  //Logger
  lrWpanHelper.EnablePcapAll (string ("tsch-three-nodes"), true);
  lrWpanHelper.EnableAsciiAll (stream);
  // lrWpanHelper.EnableLogComponents();
  EnergySourceContainer enc = lrWpanHelper.InstallEnergySource(lrwpanNodes);
  DeviceEnergyModelContainer devenc = lrWpanHelper.InstallEnergyDevice(netdev, enc);
  lrWpanHelper.EnableEnergyAllPhy(stream2,enc);

  //Assiciate FFD RFD
  lrWpanHelper.AssociateToPan(netdev,123);     //Device , PAN id
  lrWpanHelper.ConfigureSlotframeAllToPan(netdev,0,true,false);//devices,empty slots,bidirectional=true,use broadcast cells=false
  lrWpanHelper.EnableTsch(netdev,0.0,duration);

  //CBR traffic
  lrWpanHelper.GenerateTraffic(netdev.Get(1),netdev.Get(0)->GetAddress(),pktsize,5.0,duration,0.5);   // Node 1 -> Coordinator
  lrWpanHelper.GenerateTraffic(netdev.Get(2),netdev.Get(0)->GetAddress(),pktsize,5.0,duration,0.5);   // Node 2 -> Coordinator
  // lrWpanHelper.GenerateTraffic(netdev.Get(2),netdev.Get(1)->GetAddress(),pktsize,1.0,duration,0.5);   // Node 2 -> Node 1
  

  //Rx
  Ptr<LrWpanTschNetDevice> dev0 = netdev.Get(0)->GetObject<LrWpanTschNetDevice> ();
  dev0->GetNMac()->TraceConnectWithoutContext("MacRx",MakeCallback(&MacRxCallback));
  //Tx
  Ptr<LrWpanTschNetDevice> dev1 = netdev.Get(1)->GetObject<LrWpanTschNetDevice> ();
  Ptr<LrWpanTschNetDevice> dev2 = netdev.Get(2)->GetObject<LrWpanTschNetDevice> ();
  dev1->GetNMac()->TraceConnectWithoutContext("MacTx",MakeCallback(&MacTxCallback));
  dev2->GetNMac()->TraceConnectWithoutContext("MacTx",MakeCallback(&MacTxCallback));


  Simulator::Run ();
  std::cout<< "Total Tx Bits = " << TotalTxByte*8 <<"\n";
  std::cout<< "Total Rx Bits = " << TotalRxByte*8 <<"\n";
  std::cout<< "Total Tx Pkt = " << TxPkt <<"\n";
  std::cout<< "Total Rx Pkt = " << RxPkt <<"\n";
  std::cout<< "Packet Deliver Ratio = " << double(RxPkt / TxPkt) * 100 <<" %\n";
  Simulator::Destroy ();
}
