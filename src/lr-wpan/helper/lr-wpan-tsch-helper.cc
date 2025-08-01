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
 *  Tom Henderson <thomas.r.henderson@boeing.com>
 *  Luis Pacheco <luisbelem@gmail.com>
 *  Peishuo Li <pressthunder@gmail.com>
 *  Peter Kourzanov <peter.kourzanov@gmail.com>
 */
#include <cassert>
#include "lr-wpan-tsch-helper.h"
#include <ns3/energy-module.h>
#include <ns3/lr-wpan-error-model.h>
#include <ns3/lr-wpan-tsch-net-device.h>
#include <ns3/mobility-model.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/friis-spectrum-propagation-loss.h>
#include <ns3/log.h>
#include "lr-wpan-radio-energy-model-helper.h"
#include "lr-wpan-energy-source-helper.h"
#include <ns3/aloha-noack-net-device.h> //change by ding
#include <ns3/core-module.h> //change by ding
#include <ns3/network-module.h> //change by ding
#include <ns3/lr-wpan-module.h> //change by ding
#include <ns3/wifi-module.h> //change by ding
#include <ns3/point-to-point-module.h> //change by ding
#include <ns3/mac-low.h> //change by ding
#include <ns3/applications-module.h>
#include "ns3/application.h"

NS_LOG_COMPONENT_DEFINE ("LrWpanTschHelper");

namespace ns3 {

double send_interval;

//////////////////////
// add by ieong
std::vector<int> packetlist(99);
int Totalpacket = 0;
std::vector<std::vector<int> > adjacency_matrix;
std::vector<std::vector<int> > ranking_matrix;
std::vector<std::vector<int> > child_matrix;
std::vector<std::vector<std::string> > scheduling_matrix;
int nodes = 0;
std::vector<int> link(99,0);
std::vector<std::vector<std::vector<int> > > DCFL(99, std::vector<std::vector<int> >(16, std::vector<int>(2)));
int TimeslotForDCFL = 0;
int CurrentTimeslot = 0; 
int node_n = 0;
int node_n_child = 0;
int node_n_neighbor = 0;
int TASA_a = 0;
std::vector<double> node_x;
std::vector<double> node_y;
std::vector<double> node_z;
std::vector<std::vector<int> > Star_Link(99, std::vector<int>(2, 0)); // [timeslot, 0 or 1]
double TimeslotLength_helper;
int numberofTimeslots = 0;
int max_ping6_pktsize = 0;

//////////////////////

static void
AsciiLrWpanTschMacTransmitSinkWithContext (
  Ptr<OutputStreamWrapper> stream,
  std::string context,
  Ptr<const Packet> p)
{
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " " << context << " " << *p << std::endl;
}

static void
AsciiLrWpanTschMacMaxRetriesSinkWithContext (
  Ptr<OutputStreamWrapper> stream,
  std::string context,
  Ptr<const Packet> p)
{
  *stream->GetStream () << "m " << Simulator::Now ().GetSeconds () << " " << context << " " << *p << std::endl;
}

static void
AsciiLrWpanTschMacMaxRetriesSinkWithoutContext (
  Ptr<OutputStreamWrapper> stream,
  Ptr<const Packet> p)
{
  *stream->GetStream () << "m " << Simulator::Now ().GetSeconds () << " " << *p << std::endl;
}

static void
AsciiLrWpanTschMacTransmitSinkWithoutContext (
  Ptr<OutputStreamWrapper> stream,
  Ptr<const Packet> p)
{
  *stream->GetStream () << "t " << Simulator::Now ().GetSeconds () << " " << *p << std::endl;
}

static void
EnergyTraceWithContext (Ptr<OutputStreamWrapper> stream, std::string context,uint32_t psize)
{
  if (psize > 0)
    {
      *stream->GetStream () << Simulator::Now ().GetSeconds () << " " << context << " Packet size: "  << psize << std::endl;
    }
  else
    {
      *stream->GetStream () << Simulator::Now ().GetSeconds () << " " << context << std::endl;
    }
}

//change by ding
static void
ConsumedEnergyTracing (Ptr<OutputStreamWrapper> stream_dev, std::string context_dev, std::string prePhyState, std::string curPhyState,
                       bool sourceEnergyUnlimited, double consumedEnergy,
                       double remainingEnergy, double totalConsumedEnergy)
{
  if(!sourceEnergyUnlimited)
  { *stream_dev->GetStream () << Simulator::Now ().GetSeconds () << "s: "<< context_dev << " Device state switch to: " << curPhyState
                               << " from: " << prePhyState
                               << " Device consumed energy of previous state: " << consumedEnergy << "J"
                               << " Source remaining energy: " << remainingEnergy << "J"
                               <<" Total consumed energy: " << totalConsumedEnergy << "J" << std::endl;
  }                             
  

  else
  { *stream_dev->GetStream () << Simulator::Now ().GetSeconds () << "s: "<< context_dev << " Device state switch to: " << curPhyState
                              << " from: " << prePhyState
                              << " Device consumed energy of previous state: " << consumedEnergy << "J"
                              <<" Total consumed energy (Unlimited Source): " << totalConsumedEnergy << "J" << std::endl;

    std::cout<< Simulator::Now ().GetSeconds () << "s: "<< context_dev << " Device state switch to: " << curPhyState
                              << " from: " << prePhyState
                              << " Device consumed energy of previous state: " << consumedEnergy << "J"
                              <<" Total consumed energy (Unlimited Source): " << totalConsumedEnergy << "J" << std::endl;
  }
}

static void
ReceivedPowerTracing (Ptr<OutputStreamWrapper> stream_rec, uint32_t rxId, uint32_t txId, uint8_t channelNum, double receivedPower,
                      double fadingBiasPower)
{
   *stream_rec->GetStream () << Simulator::Now ().GetSeconds () << "s: " << "Received Node " << rxId
                            << " from Node " << txId << " on Channel: " << (int)channelNum <<" with power: " << receivedPower << " dBm"
                            <<" containing fading bias: " << fadingBiasPower << " dBm"
                            << " (with original " << receivedPower - fadingBiasPower << " dBm)"<< std::endl;
}

LrWpanTschHelper::LrWpanTschHelper (void)
{
  m_channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<FriisSpectrumPropagationLossModel> model = CreateObject<FriisSpectrumPropagationLossModel> ();
  //Ptr<LogDistancePropagationLossModel> model = CreateObject<LogDistancePropagationLossModel> ();
  m_channel->AddSpectrumPropagationLossModel (model);
  m_slotframehandle = 0;
  m_numchannel = 16;
  m_numnode = 0;
  MinFadingBias = 0;
  MaxFadingBias = 0;
}

LrWpanTschHelper::LrWpanTschHelper (Ptr<SpectrumChannel> ch)
{
  m_channel = ch;
  m_slotframehandle = 0;
  m_numchannel = 16;
  m_numnode = 0;
  m_fadingBiasMatrix = false;
  m_isDay = true;
  MinFadingBias = 0;
  MaxFadingBias = 0;
}

LrWpanTschHelper::LrWpanTschHelper (Ptr<SpectrumChannel> ch, u_int32_t num_node, bool fadingBiasMatrix, bool isDay)
{
  m_channel = ch;
  m_slotframehandle = 0;
  m_numchannel = 16;
  m_numnode = num_node;
  m_fadingBiasMatrix = fadingBiasMatrix;
  m_isDay = isDay;
  if (m_isDay){
      MinFadingBias = -10;
      MaxFadingBias = 5;
    }
  else{
      MinFadingBias = -15;
      MaxFadingBias = 10;
    }

  SetFadingBiasValues();
}

LrWpanTschHelper::~LrWpanTschHelper (void)
{
  m_channel->Dispose ();
  m_channel = 0;
}

void
LrWpanTschHelper::SetFadingBiasValues ()
{
  FadingBias.Build(m_numnode, m_numnode, m_numchannel);
  if (m_fadingBiasMatrix)
    {
      m_random = CreateObject<UniformRandomVariable> ();
      m_random->SetAttribute ("Min", DoubleValue ( MinFadingBias));
      m_random->SetAttribute ("Max", DoubleValue (MaxFadingBias));
      for (uint32_t i=0; i<m_numnode; i++)
        for (uint32_t j=0; j<=i; j++)
          for (uint32_t k=0; k<m_numchannel; k++)
            {
              FadingBias[i][j][k] = pow(10, (m_random->GetValue ()/10));
            }
	    /* for symmetric multi-path fading */
      for (uint32_t i=0; i<m_numnode; i++)
        for (uint32_t j=i+1; j<m_numnode; j++)
          for (uint32_t k=0; k<m_numchannel; k++)
            {
              FadingBias[i][j][k] = FadingBias[j][i][k];
            }
    }
  else
    {
      for (uint32_t i=0; i<m_numnode; i++)
        for (uint32_t j=0; j<m_numnode; j++)
          for (uint32_t k=0; k<m_numchannel; k++)
            {
              FadingBias[i][j][k] = 1;
            }
    }
}

void
LrWpanTschHelper::PrintFadingBiasValues(Ptr<OutputStreamWrapper> stream_fadingBias)
{
  //*stream_fadingBias -> GetStream () << "Fading Bias Values for this scenario:" << std::endl;
  for (unsigned char i=0; i<m_numnode; i++)
    {
    for (unsigned char j=0; j<m_numnode; j++)
      {
      for (unsigned char k=0; k<m_numchannel; k++)
        {
          //*stream_fadingBias -> GetStream () << "Bias[" << (int)i << "][" << (int)j << "][" << (int)k << "] = "
          //  << (double)FadingBias[i][j][k] << " ";
          //*stream_fadingBias -> GetStream () << 10*log10((double)FadingBias[i][j][k]) << " ";
          *stream_fadingBias -> GetStream () << (double)FadingBias[i][j][k] << " ";
        }
        //*stream_fadingBias -> GetStream () <<std::endl;
      }
      //*stream_fadingBias -> GetStream () <<std::endl;
    }
}

void
LrWpanTschHelper::EnableLogComponents (void)
{
  LogComponentEnableAll (LOG_PREFIX_TIME);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnable ("LrWpanCsmaCa", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanErrorModel", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanInterferenceHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanTschMac", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanTschNetDevice", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanSpectrumSignalParameters", LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanSpectrumValueHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("SimpleNetDevice", LOG_LEVEL_ALL); //change by ding
}

std::string
LrWpanTschHelper::LrWpanPhyEnumerationPrinter (LrWpanPhyEnumeration e)
{
  switch (e)
    {
    case IEEE_802_15_4_PHY_BUSY:
      return std::string ("BUSY");
    case IEEE_802_15_4_PHY_BUSY_RX:
      return std::string ("BUSY_RX");
    case IEEE_802_15_4_PHY_BUSY_TX:
      return std::string ("BUSY_TX");
    case IEEE_802_15_4_PHY_FORCE_TRX_OFF:
      return std::string ("FORCE_TRX_OFF");
    case IEEE_802_15_4_PHY_IDLE:
      return std::string ("IDLE");
    case IEEE_802_15_4_PHY_INVALID_PARAMETER:
      return std::string ("INVALID_PARAMETER");
    case IEEE_802_15_4_PHY_RX_ON:
      return std::string ("RX_ON");
    case IEEE_802_15_4_PHY_SUCCESS:
      return std::string ("SUCCESS");
    case IEEE_802_15_4_PHY_TRX_OFF:
      return std::string ("TRX_OFF");
    case IEEE_802_15_4_PHY_TX_ON:
      return std::string ("TX_ON");
    case IEEE_802_15_4_PHY_UNSUPPORTED_ATTRIBUTE:
      return std::string ("UNSUPPORTED_ATTRIBUTE");
    case IEEE_802_15_4_PHY_READ_ONLY:
      return std::string ("READ_ONLY");
    case IEEE_802_15_4_PHY_UNSPECIFIED:
      return std::string ("UNSPECIFIED");
    default:
      return std::string ("INVALID");
    }
}

std::string
LrWpanTschHelper::LrWpanMacStatePrinter (LrWpanTschMacState e)
{
  switch (e)
    {
    case TSCH_MAC_IDLE:
      return std::string ("TSCH_MAC_IDLE");   //0
    case TSCH_MAC_CCA:
      return std::string ("TSCH_MAC_CCA");   //1
    case TSCH_MAC_SENDING:
      return std::string ("TSCH_MAC_SENDING"); //2
    case TSCH_MAC_ACK_PENDING:
      return std::string ("TSCH_MAC_ACK_PENDING"); //3
    case TSCH_MAC_ACK_PENDING_END:
      return std::string ("TSCH_MAC_ACK_PENDING_END"); //4
    case TSCH_CHANNEL_ACCESS_FAILURE:
      return std::string ("TSCH_CHANNEL_ACCESS_FAILURE");//5
    case TSCH_CHANNEL_IDLE:
      return std::string ("TSCH_CHANNEL_IDLE");//6
    case TSCH_SET_PHY_TX_ON:
      return std::string ("TSCH_SET_PHY_TX_ON");//7
    case TSCH_MAC_RX:
      return std::string ("TSCH_MAC_RX");//8
    case TSCH_PKT_WAIT_END:
      return std::string ("TSCH_PKT_WAIT_END");//9
    default:
      return std::string ("INVALID");
    }
}

void
LrWpanTschHelper::AddMobility (Ptr<LrWpanPhy> phy, Ptr<MobilityModel> m)
{
  phy->SetMobility (m);
}

NetDeviceContainer
LrWpanTschHelper::Install (NodeContainer c)
{
  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); i++)
    {
      Ptr<Node> node = *i;

      Ptr<LrWpanTschNetDevice> netDevice = CreateObject<LrWpanTschNetDevice> ();
      netDevice->SetChannel (m_channel);
      node->AddDevice (netDevice);
      netDevice->SetNode (node);
      // \todo add the capability to change short address, extended
      // address and panId. Right now they are hardcoded in LrWpanTschMac::LrWpanTschMac ()
      devices.Add (netDevice);
    }
  return devices;
}

std::vector<Ptr<LrWpanTschNetDevice> >
LrWpanTschHelper::InstallVector (NodeContainer c)
{
  std::vector<Ptr<LrWpanTschNetDevice> > devices;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); i++)
    {
      Ptr<Node> node = *i;

      Ptr<LrWpanTschNetDevice> netDevice = CreateObject<LrWpanTschNetDevice> ();
      netDevice->SetChannel (m_channel);
      node->AddDevice (netDevice);
      netDevice->SetNode (node);
      // \todo add the capability to change short address, extended
      // address and panId. Right now they are hardcoded in LrWpanTschMac::LrWpanTschMac ()
      devices.push_back (netDevice);
    }
  return devices;
}

EnergySourceContainer
LrWpanTschHelper::InstallEnergySource (NodeContainer c)
{
  LrWpanEnergySourceHelper lrWpanSourceHelper;

  lrWpanSourceHelper.Set ("LrWpanEnergySourceInitialEnergyJ", DoubleValue (2));
  lrWpanSourceHelper.Set ("LrWpanEnergySupplyVoltageV", DoubleValue (1));
  lrWpanSourceHelper.Set ("LrWpanUnlimitedEnergy", BooleanValue (1));

  EnergySourceContainer sources = lrWpanSourceHelper.Install (c);

  return sources;
}

DeviceEnergyModelContainer
LrWpanTschHelper::InstallEnergyDevice (NetDeviceContainer devices, EnergySourceContainer sources)
{
  LrWpanRadioEnergyModelHelper radioEnergyHelper;

  // radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);

  return deviceModels;
}


int64_t
LrWpanTschHelper::AssignStreams (NetDeviceContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  Ptr<NetDevice> netDevice;
  for (NetDeviceContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      netDevice = (*i);
      Ptr<LrWpanTschNetDevice> lrwpan = DynamicCast<LrWpanTschNetDevice> (netDevice);
      if (lrwpan)
        {
          currentStream += lrwpan->AssignStreams (currentStream);
        }
    }
  return (currentStream - stream);
}

void
LrWpanTschHelper::AssociateToPan (NetDeviceContainer c, uint16_t panId)
{
  NetDeviceContainer devices;
  uint16_t id = 1;
  uint8_t idBuf[2];

  for (NetDeviceContainer::Iterator i = c.Begin (); i != c.End (); i++)
    {
      Ptr<LrWpanTschNetDevice> device = DynamicCast<LrWpanTschNetDevice> (*i);
      if (device)
        {
          idBuf[0] = (id >> 8) & 0xff;
          idBuf[1] = (id >> 0) & 0xff;
          Mac16Address address;
          address.CopyFrom (idBuf);

          device->GetOMac ()->SetPanId (panId);
          device->GetOMac ()->SetShortAddress (address);
          device->GetNMac ()->SetPanId (panId);
          device->GetNMac ()->SetShortAddress (address);
          id++;
        }
    }
  return;
}

static void
PcapSniffLrWpan (Ptr<PcapFileWrapper> file, Ptr<const Packet> packet)
{
  file->Write (Simulator::Now (), packet);
}

void
LrWpanTschHelper::EnablePcapInternal (std::string prefix, Ptr<NetDevice> nd, bool promiscuous, bool explicitFilename)
{
  NS_LOG_FUNCTION (this << prefix << nd << promiscuous << explicitFilename);
  //
  // All of the Pcap enable functions vector through here including the ones
  // that are wandering through all of devices on perhaps all of the nodes in
  // the system.
  //

  // In the future, if we create different NetDevice types, we will
  // have to switch on each type below and insert into the right
  // NetDevice type
  //
  Ptr<LrWpanTschNetDevice> device = nd->GetObject<LrWpanTschNetDevice> ();
  if (device == 0)
    {
      NS_LOG_INFO ("LrWpanTschHelper::EnablePcapInternal(): Device " << device << " not of type ns3::LrWpanTschNetDevice");
      return;
    }

  PcapHelper pcapHelper;

  std::string filename;
  if (explicitFilename)
    {
      filename = prefix;
    }
  else
    {
      filename = pcapHelper.GetFilenameFromDevice (prefix, device);
    }

  Ptr<PcapFileWrapper> file = pcapHelper.CreateFile (filename, std::ios::out,
                                                     PcapHelper::DLT_IEEE802_15_4);

  if (promiscuous == true)
    {
      device->GetOMac()->TraceConnectWithoutContext ("PromiscSniffer", MakeBoundCallback (&PcapSniffLrWpan, file));
      device->GetNMac()->TraceConnectWithoutContext ("PromiscSniffer", MakeBoundCallback (&PcapSniffLrWpan, file));
    }
  else
    {
      device->GetOMac()->TraceConnectWithoutContext ("Sniffer", MakeBoundCallback (&PcapSniffLrWpan, file));
      device->GetNMac()->TraceConnectWithoutContext ("Sniffer", MakeBoundCallback (&PcapSniffLrWpan, file));
    }
}

void
LrWpanTschHelper::EnableAsciiInternal (
  Ptr<OutputStreamWrapper> stream,
  std::string prefix,
  Ptr<NetDevice> nd,
  bool explicitFilename)
{
  uint32_t nodeid = nd->GetNode ()->GetId ();
  uint32_t deviceid = nd->GetIfIndex ();
  std::ostringstream oss;

  Ptr<LrWpanTschNetDevice> device = nd->GetObject<LrWpanTschNetDevice> ();
  if (device == 0)
    {
      NS_LOG_INFO ("LrWpanTschHelper::EnableAsciiInternal(): Device " << device << " not of type ns3::LrWpanTschNetDevice");
      return;
    }

  //
  // Our default trace sinks are going to use packet printing, so we have to
  // make sure that is turned on.
  //
  Packet::EnablePrinting ();

  //
  // If we are not provided an OutputStreamWrapper, we are expected to create
  // one using the usual trace filename conventions and do a Hook*WithoutContext
  // since there will be one file per context and therefore the context would
  // be redundant.
  //
  if (stream == 0)
    {
      //
      // Set up an output stream object to deal with private ofstream copy
      // constructor and lifetime issues.  Let the helper decide the actual
      // name of the file given the prefix.
      //
      AsciiTraceHelper asciiTraceHelper;

      std::string filename;
      if (explicitFilename)
        {
          filename = prefix;
        }
      else
        {
          filename = asciiTraceHelper.GetFilenameFromDevice (prefix, device);
        }

      Ptr<OutputStreamWrapper> theStream = asciiTraceHelper.CreateFileStream (filename);

      // Ascii traces typically have "+", '-", "d", "r", and sometimes "t"
      // The Mac and Phy objects have the trace sources for these
      //

      asciiTraceHelper.HookDefaultReceiveSinkWithoutContext<LrWpanTschNetDevice> (device, "MacRx", theStream);

      device->GetOMac()->TraceConnectWithoutContext ("MacTx", MakeBoundCallback (&AsciiLrWpanTschMacTransmitSinkWithoutContext, theStream));
      device->GetNMac()->TraceConnectWithoutContext ("MacTx", MakeBoundCallback (&AsciiLrWpanTschMacTransmitSinkWithoutContext, theStream));
      device->GetOMac()->TraceConnectWithoutContext ("MacMaxRetries", MakeBoundCallback (&AsciiLrWpanTschMacMaxRetriesSinkWithoutContext, theStream));
      device->GetNMac()->TraceConnectWithoutContext ("MacMaxRetries", MakeBoundCallback (&AsciiLrWpanTschMacMaxRetriesSinkWithoutContext, theStream));
      asciiTraceHelper.HookDefaultEnqueueSinkWithoutContext<LrWpanTschNetDevice> (device, "MacTxEnqueue", theStream);
      asciiTraceHelper.HookDefaultDequeueSinkWithoutContext<LrWpanTschNetDevice> (device, "MacTxDequeue", theStream);
      asciiTraceHelper.HookDefaultDropSinkWithoutContext<LrWpanTschNetDevice> (device, "MacTxDrop", theStream);

      return;
    }

  //
  // If we are provided an OutputStreamWrapper, we are expected to use it, and
  // to provide a context.  We are free to come up with our own context if we
  // want, and use the AsciiTraceHelper Hook*WithContext functions, but for
  // compatibility and simplicity, we just use Config::Connect and let it deal
  // with the context.
  //
  // Note that we are going to use the default trace sinks provided by the
  // ascii trace helper.  There is actually no AsciiTraceHelper in sight here,
  // but the default trace sinks are actually publicly available static
  // functions that are always there waiting for just such a case.
  //


  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacRx";
  device->GetNMac()->TraceConnect ("MacRx", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultReceiveSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacTx";
  device->GetNMac()->TraceConnect ("MacTx", oss.str (), MakeBoundCallback (&AsciiLrWpanTschMacTransmitSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacMaxRetries";
  device->GetNMac()->TraceConnect ("MacMaxRetries", oss.str (), MakeBoundCallback (&AsciiLrWpanTschMacMaxRetriesSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacTxEnqueue";
  device->GetNMac()->TraceConnect ("MacTxEnqueue", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultEnqueueSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacTxDequeue";
  device->GetNMac()->TraceConnect ("MacTxDequeue", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultDequeueSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacTxDrop";
  device->GetNMac()->TraceConnect ("MacTxDrop", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultDropSinkWithContext, stream));

  //

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacRx";
  device->GetOMac()->TraceConnect ("MacRx", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultReceiveSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTx";
  device->GetOMac()->TraceConnect ("MacTx", oss.str (), MakeBoundCallback (&AsciiLrWpanTschMacTransmitSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacMaxRetries";
  device->GetOMac()->TraceConnect ("MacMaxRetries", oss.str (), MakeBoundCallback (&AsciiLrWpanTschMacMaxRetriesSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxEnqueue";
  device->GetOMac()->TraceConnect ("MacTxEnqueue", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultEnqueueSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxDequeue";
  device->GetOMac()->TraceConnect ("MacTxDequeue", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultDequeueSinkWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxDrop";
  device->GetOMac()->TraceConnect ("MacTxDrop", oss.str (), MakeBoundCallback (&AsciiTraceHelper::DefaultDropSinkWithContext, stream));

}

void
LrWpanTschHelper::EnableEnergyAll(Ptr<OutputStreamWrapper> stream)
{
  NodeContainer n = NodeContainer::GetGlobal ();

  for (NodeContainer::Iterator i = n.Begin (); i != n.End (); ++i)
    {
      Ptr<Node> node = *i;
      for (uint32_t j = 0; j < node->GetNDevices (); ++j)
        {
          EnableEnergyInternal (stream, std::string(), node->GetDevice (j), false);
        }
    }
}

void
LrWpanTschHelper::EnableEnergyAllPhy(Ptr<OutputStreamWrapper> stream, EnergySourceContainer sources)
{
    Ptr<EnergySource> sourcePtr;
    for (EnergySourceContainer::Iterator i = sources.Begin (); i != sources.End (); ++i)
      {
        sourcePtr = (*i);
        Ptr<LrWpanEnergySource> lrWpanSourcePtr = DynamicCast<LrWpanEnergySource> (sourcePtr);
        DeviceEnergyModelContainer devicePerSrcPtr = lrWpanSourcePtr->FindDeviceEnergyModels ("ns3::LrWpanRadioEnergyModel");

        uint32_t nodeid = lrWpanSourcePtr-> GetNode() -> GetId();

        for (uint32_t j = 0; j < devicePerSrcPtr.GetN(); j++)
           {
            std::ostringstream oss;
            oss.str ("");
            oss << "/NodeList/" << nodeid << "/DeviceList/" << j << ":";

            Ptr<LrWpanRadioEnergyModel> lrWpanRadioModelPtr = DynamicCast<LrWpanRadioEnergyModel> (devicePerSrcPtr.Get(j));
            NS_ASSERT (lrWpanRadioModelPtr != NULL);

            lrWpanRadioModelPtr->TraceConnect ("CurrentEnergyState", oss.str(), MakeBoundCallback (&ConsumedEnergyTracing, stream));
          }
      }

}

void
LrWpanTschHelper::EnableReceivePower (Ptr<OutputStreamWrapper> stream_recPower, NodeContainer lrwpanNodes)
{
  for (NodeContainer::Iterator i = lrwpanNodes.Begin (); i != lrwpanNodes.End (); ++i)
    {
      Ptr<Node> node = *i;
      for (uint32_t j = 0; j < node->GetNDevices (); ++j)
        {
          Ptr<LrWpanTschNetDevice> device = node->GetDevice(j)->GetObject<LrWpanTschNetDevice> ();
          device->GetPhy ()->TraceConnectWithoutContext ("PhyLinkInformation", MakeCallback (&LrWpanTschMac::GetPhylinkInformation, device->GetNMac()));
          device->GetNMac()->TraceConnectWithoutContext ("MacLinkInformation", MakeBoundCallback (&ReceivedPowerTracing, stream_recPower));
        }
    }
}

void
LrWpanTschHelper::EnableEnergy(Ptr<OutputStreamWrapper> stream,Ptr<NetDevice> dev)
{
  EnableAsciiInternal (stream, std::string(), dev, false);
}


void
LrWpanTschHelper::EnableEnergyInternal (
  Ptr<OutputStreamWrapper> stream,
  std::string prefix,
  Ptr<NetDevice> nd,
  bool explicitFilename)
{
  uint32_t nodeid = nd->GetNode ()->GetId ();
  uint32_t deviceid = nd->GetIfIndex ();
  std::ostringstream oss;

  Ptr<LrWpanTschNetDevice> device = nd->GetObject<LrWpanTschNetDevice> ();
  if (device == 0)
    {
      NS_LOG_INFO ("LrWpanTschHelper::EnableEnergyInternal(): Device " << device << " not of type ns3::LrWpanTschNetDevice");
      return;
    }

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacRxDataTxAck";
  device->GetNMac()->TraceConnect ("MacRxDataTxAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacTxData";
  device->GetNMac()->TraceConnect ("MacTxData", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacRxData";
  device->GetNMac()->TraceConnect ("MacRxData", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacTxDataRxAck";
  device->GetNMac()->TraceConnect ("MacTxDataRxAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacSleep";
  device->GetNMac()->TraceConnect ("MacSleep", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacIdle";
  device->GetNMac()->TraceConnect ("MacIdle", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacChannelBusy";
  device->GetNMac()->TraceConnect ("MacChannelBusy", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacWaitAck";
  device->GetNMac()->TraceConnect ("MacWaitAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/TschMac/MacEmptyBuffer";
  device->GetNMac()->TraceConnect ("MacEmptyBuffer", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  // 

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacRxDataTxAck";
  device->GetOMac()->TraceConnect ("MacRxDataTxAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxData";
  device->GetOMac()->TraceConnect ("MacTxData", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacRxData";
  device->GetOMac()->TraceConnect ("MacRxData", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacTxDataRxAck";
  device->GetOMac()->TraceConnect ("MacTxDataRxAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacSleep";
  device->GetOMac()->TraceConnect ("MacSleep", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacIdle";
  device->GetOMac()->TraceConnect ("MacIdle", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacChannelBusy";
  device->GetOMac()->TraceConnect ("MacChannelBusy", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacWaitAck";
  device->GetOMac()->TraceConnect ("MacWaitAck", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

  oss.str ("");
  oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid << "/$ns3::LrWpanTschNetDevice/Mac/MacEmptyBuffer";
  device->GetOMac()->TraceConnect ("MacEmptyBuffer", oss.str (), MakeBoundCallback (&EnergyTraceWithContext, stream));

}

void
LrWpanTschHelper::AddSlotframe(NetDeviceContainer devs, uint8_t slotframehandle, uint16_t size)
{
  MlmeSetSlotframeRequestParams slotframeRequest;
  slotframeRequest.slotframeHandle = slotframehandle;
  slotframeRequest.Operation = MlmeSlotframeOperation_ADD;
  slotframeRequest.size = size;

  for (NetDeviceContainer::Iterator i = devs.Begin (); i != devs.End (); i++)
    {
      Ptr<LrWpanTschNetDevice> lrDevice = DynamicCast<LrWpanTschNetDevice> (*i);
      lrDevice->GetNMac()->MlmeSetSlotframeRequest (slotframeRequest);
    }
}

void
LrWpanTschHelper::AddSlotframe(Ptr<NetDevice> dev, uint8_t slotframehandle, uint16_t size)
{

  MlmeSetSlotframeRequestParams slotframeRequest;
  slotframeRequest.slotframeHandle = slotframehandle;
  slotframeRequest.Operation = MlmeSlotframeOperation_ADD;
  slotframeRequest.size = size;
  dev->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetSlotframeRequest (slotframeRequest);
}

void
LrWpanTschHelper::ModSlotframe(NetDeviceContainer devs, uint8_t slotframehandle, uint16_t size)
{


  MlmeSetSlotframeRequestParams slotframeRequest;
  slotframeRequest.slotframeHandle = slotframehandle;
  slotframeRequest.Operation = MlmeSlotframeOperation_MODIFY;
  slotframeRequest.size = size;

  for (NetDeviceContainer::Iterator i = devs.Begin (); i != devs.End (); i++)
    {
      Ptr<LrWpanTschNetDevice> lrDevice = DynamicCast<LrWpanTschNetDevice> (*i);
      lrDevice->GetNMac()->MlmeSetSlotframeRequest (slotframeRequest);
    }
  
  printf("after mod m_slotframehandle size %d\n",size);
}

//can change here by ding
void
LrWpanTschHelper::AddLink(Ptr<NetDevice> src, Ptr<NetDevice> dst, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_ADD_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;


  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(dst->GetAddress());
  src->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(src->GetAddress());
  dst->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::AddLink(NetDeviceContainer devs, u_int32_t srcPos, u_int32_t dstPos, AddLinkParams params, bool sharedLink)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_ADD_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;
  NS_LOG_UNCOND("Print timeslot for add link in Timeslot " <<  linkRequest.Timeslot ); //ieong // for confirm

  //10000000 to transmit  so 1010000 should be sharedlink by ding
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  if (sharedLink){
      linkRequest.linkOptions.set(2,1);
  }
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(dstPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  NS_LOG_UNCOND("setting link request for add link of sending  " ); //ieong // for confirm
  devs.Get(srcPos)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(srcPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  NS_LOG_UNCOND("setting link request for add link of receiving  " ); //ieong // for confirm
  devs.Get(dstPos)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::AddAdvLink(NetDeviceContainer devs,u_int32_t senderPos, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_ADD_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;

  NS_LOG_UNCOND("Print timeslot for advertise link in Timeslot " <<  linkRequest.Timeslot ); //ieong // for confirm

  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_ADVERTISING;
  linkRequest.nodeAddr = Mac16Address("ff:ff");
  linkRequest.linkFadingBias = NULL;
  linkRequest.TxID = senderPos;
  linkRequest.RxID = 0;
  NS_LOG_UNCOND("setting link request for advertise link of sending  " ); //ieong // for confirm
  devs.Get(senderPos)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(senderPos)->GetAddress());
  for ( u_int32_t i = 0;i < devs.GetN ();i++)
      if (i != senderPos)
        {
          linkRequest.linkFadingBias = FadingBias[i][senderPos];
          linkRequest.TxID = senderPos;
          linkRequest.RxID = i;
          NS_LOG_UNCOND("setting link request for advertise link of receiving  " ); // ieong // for confirm

          /*
          if(i == devs.GetN()-1){                // add by ieong
            linkRequest.nrnodesforuse = i; 
            std::cout<<"sending to mac :"<<i <<std::endl;
          }
*/
          devs.Get(i)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
        }

}

void LrWpanTschHelper::AddBcastLinks(NetDeviceContainer devs,u_int32_t coordinatorPos, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_ADD_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;

  //10000000 to transmit  it should be 10100000 by ding  // share link?
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkOptions.set(2,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_ADVERTISING;
  linkRequest.nodeAddr = Mac16Address("ff:ff");
  for ( u_int32_t i = 0;i < devs.GetN ();i++)
    {

          linkRequest.TxID = i;
          linkRequest.RxID = coordinatorPos;
          linkRequest.linkFadingBias = FadingBias[coordinatorPos][i];
          NS_LOG_UNCOND("setting link request for boardcast link of coordinator to node  " ); //ieong // for confirm

          devs.Get(i)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
    }

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_ADVERTISING;
  linkRequest.nodeAddr = Mac16Address("ff:ff");
  for ( u_int32_t i = 0;i < devs.GetN ();i++)
    {
          linkRequest.TxID = coordinatorPos;
          linkRequest.RxID = i;
          linkRequest.linkFadingBias = FadingBias[i][coordinatorPos];
          NS_LOG_UNCOND("setting link request for boardcast link of node to coordinator  " ); //ieong // for confirm
          devs.Get(i)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
    }
}

void
LrWpanTschHelper::ModifyLink(Ptr<NetDevice> src, Ptr<NetDevice> dst, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_MODIFY_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;

  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(dst->GetAddress());
  src->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(src->GetAddress());
  dst->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::ModifyLink(NetDeviceContainer devs, u_int32_t srcPos, u_int32_t dstPos, AddLinkParams params, bool sharedLink)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_MODIFY_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;

  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  if (sharedLink){
      linkRequest.linkOptions.set(2,1);
  }
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(dstPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(srcPos)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(srcPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(dstPos)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::DeleteLink(Ptr<NetDevice> src, Ptr<NetDevice> dst, AddLinkParams params)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_DELETE_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;


  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(dst->GetAddress());
  src->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(src->GetAddress());
  dst->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
}

void
LrWpanTschHelper::DeleteLink(NetDeviceContainer devs, u_int32_t srcPos, u_int32_t dstPos, AddLinkParams params, bool sharedLink)
{
  MlmeSetLinkRequestParams linkRequest;
  linkRequest.Operation = MlmeSetLinkRequestOperation_DELETE_LINK;
  linkRequest.linkHandle = params.linkHandle;

  linkRequest.slotframeHandle = params.slotframeHandle;
  linkRequest.Timeslot = params.timeslot;
  linkRequest.ChannelOffset = params.channelOffset;


  //10000000 to transmit
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(0,1);
  if (sharedLink){
      linkRequest.linkOptions.set(2,1);
  }
  linkRequest.linkType = MlmeSetLinkRequestlinkType_NORMAL;
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(dstPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(srcPos)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);

  //01010000 to receive
  linkRequest.linkOptions.reset();
  linkRequest.linkOptions.set(1,1);
  linkRequest.linkOptions.set(3,1);
  linkRequest.nodeAddr = Mac16Address::ConvertFrom(devs.Get(srcPos)->GetAddress());
  linkRequest.linkFadingBias = FadingBias[dstPos][srcPos];
  linkRequest.TxID = srcPos;
  linkRequest.RxID = dstPos;
  devs.Get(dstPos)->GetObject<LrWpanTschNetDevice> ()->GetNMac()->MlmeSetLinkRequest (linkRequest);
}

//can change here by ding
void
LrWpanTschHelper::ConfigureSlotframeAllToPan(NetDeviceContainer devs, int empty_timeslots, bool bidir, bool bcast)
{
  // orgin
  int size = (bcast ? 2 : 1) + (bidir ? 2 : 1)*(devs.GetN ()-1) + empty_timeslots;
  //int size=3; // fort test // ieong



  //int size = (bcast ? 5 : 1) + (bidir ? 2 : 1)*(devs.GetN ()-1) + empty_timeslots; //change by ding
  // int size=devs.GetN (); //use in slot frame size adjust
  //int size = (bcast ? 2 : 1) + (bidir ? 2 : 1)*(devs.GetN ()-5) + empty_timeslots;//test in csmaca
  printf("configure slot frame bcast %d bidir %d devs.getn() %d size %d\n",bcast,bidir,devs.GetN()-1,size);
  // std::cout<<"send interval in configure "<<send_interval<<std::endl;
  //printf("m_slotframehandle : %d \n",m_slotframehandle);

  // testing
  //int size = 20;

  AddSlotframe(devs,m_slotframehandle,size);
  // printf("before mod m_slotframehandle %d\n",m_slotframehandle);

  //change timeslot size
  // size-=3;
  // ModSlotframe(devs,m_slotframehandle,size);
  // printf("after mod configure slot frame bcast %d bidir %d devs.getn() %d size %d\n",bcast,bidir,devs.GetN()-1,size);

  //Add links
  AddLinkParams alparams;
  alparams.slotframeHandle = m_slotframehandle;
  alparams.channelOffset = 0;

  alparams.linkHandle = 0;
  alparams.timeslot = 0;
  AddAdvLink (devs,0,alparams);
  



  if (bcast) {
	  alparams.linkHandle = 1;
	  alparams.timeslot = 1;
    //change by ding
    // for (int i = 0; i < 5; i++)
    // {
    //   AddBcastLinks (devs,0,alparams);
    // }
	  AddBcastLinks (devs,0,alparams);
  }

  
  uint16_t c=(bcast ? 2 : 1);
  
  for (u_int32_t i = 0; i < devs.GetN ()-1; i++,c++)
    {
      NS_LOG_UNCOND("for first of add link"); //ieong // for confirm
      alparams.linkHandle = c;
      alparams.timeslot = c;
      AddLink(devs,i+1,0,alparams,false);
      //DCFL[c][int(alparams.channelOffset)][0] = i+1;
      //DCFL[c][int(alparams.channelOffset)][1] = 0;
      //link[c]++;
      //Star_Link[c][0] = i+1;
      //Star_Link[c][1] = 0;
    }

  if (bidir)
    for (u_int32_t i = 0; i < devs.GetN ()-1; i++,c++)
    {
      NS_LOG_UNCOND("for bidir of add link" ); //ieong // for confirm
      alparams.linkHandle = c;
      alparams.timeslot = c;
      AddLink(devs,0,i+1,alparams,false);
    }

/*
  // for test // add more 2 link to make consecutive
  NS_LOG_UNCOND("for first of add link"); //ieong // for confirm
  alparams.linkHandle = c;
  alparams.timeslot = c;
  AddLink(devs,3,0,alparams,false);
  Star_Link[c][0] = 3;
  Star_Link[c][1] = 0;
  NS_LOG_UNCOND("for first of add link"); //ieong // for confirm
  alparams.linkHandle = c+1;
  alparams.timeslot = c+1;
  AddLink(devs,3,0,alparams,false);
  Star_Link[c+1][0] = 3;
  Star_Link[c+1][1] = 0;
*/

  //TimeslotForDCFL = c+1;
  numberofTimeslots = c+1;
  m_slotframehandle++;


  

  ////////////////////////////
   
}

void
LrWpanTschHelper::ConfigureSlotframeAllToPanForSharedLink_1high(NetDeviceContainer devs, int empty_timeslots, bool bidir, bool bcast, int NumberOfSharedLink)
{
  // orgin
  //int size = (bcast ? 2 : 1) + (bidir ? 2 : 1)*(devs.GetN ()-1) + empty_timeslots;
  //int size=3; // fort test // ieong
  int t = 0;
  int remain = 0;
  if ( (devs.GetN ()-2) % NumberOfSharedLink != 0 ){
    t = (devs.GetN ()-2) / NumberOfSharedLink + 1;
    remain = (devs.GetN ()-2) % NumberOfSharedLink; 
  }
  else{
    t = (devs.GetN ()-2) / NumberOfSharedLink;
  }
  std::cout << " t : " << t << std::endl;

  int size = (bcast ? 2 : 1) + 1 + t + empty_timeslots;

  std::cout << " slotframe size : " << size << std::endl;


  AddSlotframe(devs,m_slotframehandle,size);

  //Add links
  AddLinkParams alparams;
  alparams.slotframeHandle = m_slotframehandle;
  alparams.channelOffset = 0;

  alparams.linkHandle = 0;
  alparams.timeslot = 0;
  AddAdvLink (devs,0,alparams);

  if (bcast) {
	  alparams.linkHandle = 1;
	  alparams.timeslot = 1;
   
	  AddBcastLinks (devs,0,alparams);
  }

  
  uint16_t c=(bcast ? 2 : 1);
 

  //LrWpanTschHelper::AddLink(NetDeviceContainer devs, u_int32_t srcPos, u_int32_t dstPos, AddLinkParams params, bool sharedLink)

  //for test
  NS_LOG_UNCOND("for 1 of dedicated link"); //ieong // for confirm
  alparams.linkHandle = c;
  alparams.timeslot = c;
  AddLink(devs,1,0,alparams,false);

  for ( int i=0; i<t; i++,c=c+NumberOfSharedLink ){
    for( int j=1; j<=NumberOfSharedLink; j++){

      if( i==t-1 && j > remain && remain > 0 ){
        break;
      }
      NS_LOG_UNCOND("for " << c+j <<" of shared link"); //ieong // for confirm
      alparams.linkHandle = c+j;
      alparams.timeslot = 2+i;
      AddLink(devs,c+j,0,alparams,true);


    }
    
  }


  numberofTimeslots = c+1;
  m_slotframehandle++;


  

  ////////////////////////////
   
}

void
LrWpanTschHelper::ConfigureSlotframeAllToPanForSharedLink_RandomTraffic(NetDeviceContainer devs, int empty_timeslots, bool bidir, bool bcast, int NumberOfSharedLink)
{
  // orgin
  //int size = (bcast ? 2 : 1) + (bidir ? 2 : 1)*(devs.GetN ()-1) + empty_timeslots;
  //int size=3; // fort test // ieong
  int t = 0;
  int remain = 0;
  if ( (devs.GetN ()-1) % NumberOfSharedLink != 0 ){
    t = (devs.GetN ()-1) / NumberOfSharedLink + 1;
    remain = (devs.GetN ()-1) % NumberOfSharedLink; 
  }
  else{
    t = (devs.GetN ()-1) / NumberOfSharedLink;
  }
  std::cout << " t : " << t << std::endl;

  int size = (bcast ? 2 : 1) + t + empty_timeslots;

  std::cout << " slotframe size : " << size << std::endl;


  AddSlotframe(devs,m_slotframehandle,size);

  //Add links
  AddLinkParams alparams;
  alparams.slotframeHandle = m_slotframehandle;
  alparams.channelOffset = 0;

  alparams.linkHandle = 0;
  alparams.timeslot = 0;
  AddAdvLink (devs,0,alparams);

  if (bcast) {
	  alparams.linkHandle = 1;
	  alparams.timeslot = 1;
   
	  AddBcastLinks (devs,0,alparams);
  }

  
  uint16_t c=(bcast ? 2 : 1);
 

  //LrWpanTschHelper::AddLink(NetDeviceContainer devs, u_int32_t srcPos, u_int32_t dstPos, AddLinkParams params, bool sharedLink)

  //for test
  //NS_LOG_UNCOND("for 1 of dedicated link"); //ieong // for confirm
  //alparams.linkHandle = c;
  //alparams.timeslot = c;
  //AddLink(devs,1,0,alparams,false);
  c = 0;
  for ( int i=0; i<t; i++,c=c+NumberOfSharedLink ){
    for( int j=1; j<=NumberOfSharedLink; j++){

      if( i==t-1 && j > remain && remain > 0 ){
        break;
      }
      NS_LOG_UNCOND("for " << c+j <<" of shared link"); //ieong // for confirm
      alparams.linkHandle = c+j;
      alparams.timeslot = 1+i;
      AddLink(devs,c+j,0,alparams,true);


    }
    
  }


  numberofTimeslots = c+1;
  m_slotframehandle++;


  

  ////////////////////////////
   
}



void
LrWpanTschHelper::ConfigureSlotframeAllToPanwithSlotframe(NetDeviceContainer devs, int empty_timeslots, bool bidir, bool bcast, int SlotframeSize)
{
  
  AddSlotframe(devs,m_slotframehandle,SlotframeSize);

  //Add links
  AddLinkParams alparams;
  alparams.slotframeHandle = m_slotframehandle;
  alparams.channelOffset = 0;

  alparams.linkHandle = 0;
  alparams.timeslot = 0;
  AddAdvLink (devs,0,alparams);

  if (bcast) {
	  alparams.linkHandle = 1;
	  alparams.timeslot = 1;
    //change by ding
    // for (int i = 0; i < 5; i++)
    // {
    //   AddBcastLinks (devs,0,alparams);
    // }
	  AddBcastLinks (devs,0,alparams);
  }

  uint16_t c=(bcast ? 2 : 1);

  for (u_int32_t i = 0; i < devs.GetN ()-1; i++,c++)
    {
      NS_LOG_UNCOND("for first of add link"); //ieong // for confirm
      alparams.linkHandle = c;
      alparams.timeslot = c;
      AddLink(devs,i+1,0,alparams,false);
      Star_Link[c][0] = i+1;
      Star_Link[c][1] = 0;
    }

  if (bidir)
    for (u_int32_t i = 0; i < devs.GetN ()-1; i++,c++)
    {
      NS_LOG_UNCOND("for bidir of add link" ); //ieong // for confirm
      alparams.linkHandle = c;
      alparams.timeslot = c;
      AddLink(devs,0,i+1,alparams,false);
    }

  
  // for test // add more 10 link for 3->0 to make consecutive

/*
  for ( int i=0; i<10; i++ ){
    NS_LOG_UNCOND("for first of add link"); //ieong // for confirm
    alparams.linkHandle = c;
    alparams.timeslot = c;
    AddLink(devs,3,0,alparams,false);
    Star_Link[c][0] = 3;
    Star_Link[c][1] = 0;
    c++;
  }
*/

  numberofTimeslots = c+1;
  m_slotframehandle++;
   
}

// ieong // for tree topo
void
LrWpanTschHelper::ConfigureSlotframeAllToPanForTree(NetDeviceContainer devs, int empty_timeslots, bool bidir, bool bcast, int nrnodes)
{
  int size = (bcast ? 2 : 1) + (bidir ? 2 : 1)*(devs.GetN ()-1) + empty_timeslots;
  printf("configure slot frame bcast %d bidir %d devs.getn() %d size %d\n",bcast,bidir,devs.GetN()-1,size);
  AddSlotframe(devs,m_slotframehandle,size);
  //Add Adv links
  AddLinkParams alparams;
  alparams.slotframeHandle = m_slotframehandle;
  alparams.channelOffset = 0;

  alparams.linkHandle = 0;
  alparams.timeslot = 0;
  AddAdvLink (devs,0,alparams);

  uint16_t c=(bcast ? 2 : 1);
  
  int k = nrnodes + 1;
  int layers = 0;
  int layers_num = 1;
  int now = 0;
  while(k>0){

    k/=2;
    layers++;
  }
  k = nrnodes + 1;

  for ( int i =0; i<layers; i++){
    
    for( int j=1; j<=layers_num; j++,c++){

      NS_LOG_UNCOND("for first of add link"); //ieong // for confirm
      alparams.linkHandle = c;
      alparams.timeslot = c;
      int g = layers_num - j + (j-1) * 2; 
      if(now+1+g > nrnodes)
        break;
      NS_LOG_UNCOND(now+1+g << " to " << now<< " c: "<< c ); //ieong // for confirm
      AddLink(devs,now+1+g,now,alparams,false);

      if(now+2+g > nrnodes)
        break;
      c++;
      alparams.linkHandle = c;
      alparams.timeslot = c;
      NS_LOG_UNCOND(now+2+g << " to " << now << " c: "<< c ); //ieong // for confirm
      AddLink(devs,now+2+g,now,alparams,false);
      now++;

    }
    
    layers_num *= 2;

  }

  m_slotframehandle++;


}
/*
void
LrWpanTschHelper::ConfigureSchedulingnForTree(NetDeviceContainer devs, int empty_timeslots, bool bidir, bool bcast, int nrnodes){



}*/



void
LrWpanTschHelper::EnableTsch(NetDeviceContainer devs, double start, double duration)
{
  /*
  MlmeTschModeRequestParams modeRequest,modeRequestoff;
  modeRequest.TSCHMode = MlmeTschMode_ON;
  modeRequestoff.TSCHMode = MlmeTschMode_OFF;
  */  

  for (u_int32_t i = 0;i<devs.GetN ();i++)
    {
      Simulator::Schedule(Seconds(start),
      	&LrWpanTschNetDevice::SetTschMode,devs.Get (i)->GetObject<LrWpanTschNetDevice> (),
	true);
      Simulator::Schedule(Seconds(start+duration),
      	&LrWpanTschNetDevice::SetTschMode,devs.Get (i)->GetObject<LrWpanTschNetDevice> (),
	false);
    }
}

void
LrWpanTschHelper::GenerateTraffic(Ptr<NetDevice> dev, Address dst, int packet_size, double start, double duration, double interval)
{
  double end = start+duration;
  send_interval=interval;
  // std::cout<<"send interval "<<send_interval<<std::endl;
  //change timeslot size
  // size-=3;
  // ModSlotframe(dev,m_slotframehandle,10);
  // printf("after mod m_slotframehandle %d\n",m_slotframehandle);
  Simulator::Schedule(Seconds(start),&LrWpanTschHelper::SendPacket,this,dev,dst,packet_size,interval,end);
}

void
LrWpanTschHelper::SendPacket(Ptr<NetDevice> dev, Address dst, int packet_size, double interval,double end)
{
  if (Now().GetSeconds() <= end)
    {
      //change netdevice to tsch net device ?? by ding
      //how to get into lr-wpan-tsch-net-device Send func?? by ding 

      // for random packet size // ieong
      m_random = CreateObject<UniformRandomVariable> ();
      packet_size = 10 + (uint8_t)m_random->GetInteger (0, 107);
      std::cout << " To make random packet : " << packet_size << std::endl;

      Ptr<Packet> pkt = Create<Packet> (packet_size);

      dev->Send(pkt,dst,0x86DD);


      // dev->Send(pkt); //change by ding
      

      /*
      Ptr<AlohaNoackNetDevice> devaloha; //change by ding
      Ptr<LrWpanNetDevice> devlrwpan = CreateObject<LrWpanNetDevice> (); //change by ding
      

      Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
      // Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
      // Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
      // channel->AddPropagationLossModel (propModel);
      // channel->SetPropagationDelayModel (delayModel);
      devlrwpan->SetChannel (channel);

      dev->GetNode()->AddDevice(devlrwpan);

      McpsDataRequestParams params;
      params.m_srcAddrMode = SHORT_ADDR;
      params.m_dstAddrMode = SHORT_ADDR;
      params.m_dstPanId = 123;
      params.m_dstAddr = Mac16Address ("00:02");//2001:1::ff:fe00:1
      params.m_msduHandle = 0;
      params.m_txOptions = TX_OPTION_ACK;
      // devlrwpan->GetMac ()->McpsDataRequest (params, pkt);

      NS_LOG_UNCOND(" params.m_srcAddrMode : " << params.m_srcAddrMode  ); //ieong // for confirm
      NS_LOG_UNCOND(" params.m_dstAddrMode : " << params.m_dstAddrMode  ); //ieong // for confirm
      NS_LOG_UNCOND(" params.m_dstPanId : " << params.m_dstPanId  ); //ieong // for confirm
      NS_LOG_UNCOND(" params.m_dstAddr : " << params.m_dstAddr  ); //ieong // for confirm
      NS_LOG_UNCOND(" params.m_msduHandle : " << params.m_msduHandle  ); //ieong // for confirm
      NS_LOG_UNCOND(" params.m_txOptions : " << params.m_txOptions  ); //ieong // for confirm

      */


    }

  if (Now().GetSeconds() <= end+interval)
    {
      Simulator::Schedule(Seconds(interval),&LrWpanTschHelper::SendPacket,this,dev,dst,packet_size,interval,end);
    }
}


//add by ieong
void 
LrWpanTschHelper::Sendmatrixtohelper(std::vector<std::vector<int> >& matrix, Ptr<NetDevice> dev){

  adjacency_matrix = matrix;
  nodes = static_cast<int>(adjacency_matrix[0].size());
  //std::cout<< "In Sendmatrixtohelper :" << std::endl;
  /*
  for ( int i=0; i<nodes; i++){
    for( int j=0; j<nodes; j++){
        std::cout<< adjacency_matrix[i][j] << " ";

    }
    std::cout<<std::endl;
  }*/
  
  //dev->GetObject<LrWpanTschNetDevice> ()->GetNMac()->Sendmatrixtomac (adjacency_matrix);

}

void 
LrWpanTschHelper::Sendpositiontohelper(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z){

  node_x = x;
  node_y = y;
  node_z = z;

  //std::cout << " Distance : " << sqrt((node_x[14]-node_x[15])*(node_x[14]-node_x[15])+(node_y[14]-node_y[15])*(node_y[14]-node_y[15])) << std::endl;


  /*
	std::cout << "In Send position to helper :" << std::endl;   

  for (int i = 0 ; i<nodes ;i++)
  {
 
     if(i == 0)
	  std::cout << "Coordinator:"<< " x = "<< node_x[i] << ", y = " << node_y[i] << ", z = " << node_z[i] << "\n";   
     else
     	  std::cout << "Sensor Node ID "<< i << ": x = "<< node_x[i] << ", y = " << node_y[i] << ", z = " << node_z[i] << "\n";   
  }*/

}


void
LrWpanTschHelper::generate_ranking_matrix()   // add by ieong
{

  ranking_matrix = adjacency_matrix;
  int k = 1, k1 = 1, rank = 1;
  for( int i=0; i<nodes; i++,k1--){
    for( int j=i+1; j<nodes; j++){
      if( ranking_matrix[i][j] == 1){
        ranking_matrix[i][j] = rank;
        ranking_matrix[j][i] = rank;

        //std::cout << " k : " << k << std::endl;
        //std::cout << " k1 : " << k1 << std::endl;
        //std::cout << " i : " << i << std::endl;
        //std::cout << " j : " << j << std::endl;
        //std::cout << " rank : " << rank << std::endl;


      }
      if( k1 == 0)
      {
        k = k*2;
        k1 = k;
        rank++;
      }
    }  
  }

  /*
  std::cout<< "In ranking matrix :" << std::endl;
  for ( int i=0; i<nodes; i++){
    for( int j=0; j<nodes; j++){
        std::cout<< ranking_matrix[i][j] << " ";
    }
    std::cout<<std::endl;
  }*/

}



void
LrWpanTschHelper::generate_packet()   // add by ieong
{

  m_random = CreateObject<UniformRandomVariable> ();

  int random_generate_packet = 0;

  packetlist[0] = 0;

  for( int i=1; i<nodes; i++){
    //random_generate_packet = (uint8_t)m_random->GetInteger (0, 8);    // random packet 
    random_generate_packet = 0;
    packetlist[i] = 1 + random_generate_packet;  // q[k]
    Totalpacket += packetlist[i];   // Q 
    //std::cout<< packetlist[i]<< " ";
  }
  //std::cout << std::endl;
  //std::cout<< "Totalpacket : "<< Totalpacket << std::endl;
  
}

/*
void
LrWpanTschHelper::scheduling()   // add by ieong
{

  //int running[99] = {0};
  //int link[99] = {0};
  //int DCFL[99][16][2] = {0};
  //int childofsink[99] = {0};
  
  std::vector<int> running(nodes);
  std::vector<int> childofsink(10);

  int k = 0, k1 = 0;
  int sinkhavedata = 0;


  for (int i=1; i<nodes; i++){
    if(adjacency_matrix[0][i] == 1)
    {
      childofsink[k] = i;
      k++;
    }
  }
  std::cout<< "k : " << k << std::endl;


  while(packetlist[0] != Totalpacket){

    
    sinkhavedata = 0;

    for (int i=0; i<nodes; i++)
      running[i] = 0;


    // change i = nodes - 1 from i = nodes
    for( int i=nodes-1; i>=1; i--){
        
      k1 = 0;

      if(running[i] == 1 || packetlist[i] == 0)
        continue;

      if( sinkhavedata == 0 ){
        for( int m=0; m<k ; m++){
          if( running[childofsink[m]] == 1 || packetlist[childofsink[m]] == 0)
          {
            k1++;
          }  
        }

      }
      

      if( k1 == k - 1){
        for( int m=0; m<=k ; m++){
          
          if( running[childofsink[m]] == 0 && packetlist[childofsink[m]] != 0){
            DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] = childofsink[m];
            DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] = 0;
            link[TimeslotForDCFL]++;

            packetlist[childofsink[m]]--;
            packetlist[0]++;

            running[childofsink[m]] = 1;
            running[0] = 1;

            sinkhavedata = 1;

            break;

          }
        }
      }
        

      for(int j=i-1; j>=0; j--){
          // add packtlist[i] != 0
        if( adjacency_matrix[i][j] == 1 && running[j] != 1 && packtlist[i] != 0 ){

          DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] = i;
          DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] = j;
          link[TimeslotForDCFL]++;

          packetlist[i]--;
          packetlist[j]++;

          running[i] = 1;
          running[j] = 1;

          break;

        }


      }
    }
    //std::cout<< "packet of root in timeslot " << timeslot <<" is "<<packetlist[0]<< std::endl;
    for( int x=0; x<nodes; x++){
      std::cout<< packetlist[x] << " ";
    }
    std::cout<<std::endl;
    TimeslotForDCFL ++;
  }    

  
  for( int i=0; i<TimeslotForDCFL ;i++){

    std::cout<< " In timeslot : " << i+1 << std::endl;
    std::cout<< "link[timeslot] : "<< link[i]<<std::endl;
    for( int j=0; j<link[i]; j++){

      std::cout<< DCFL[i][j][0] << " to " << DCFL[i][j][1] << std::endl;

    }

  }

  //ChangingTimeslotTable(DCFL, timeslot, link );

}
*/

/*
void
LrWpanTschHelper::scheduling()   // add by ieong
{

  //int running[99] = {0};
  //int link[99] = {0};
  //int DCFL[99][16][2] = {0};
  //int childofsink[99] = {0};
  
  std::vector<int> running(nodes);
  std::vector<int> rank_x_running(nodes);


  while(packetlist[0] != Totalpacket){

    for (int i=0; i<nodes; i++){
      running[i] = 0;
      rank_x_running[i] = 0;
    }

    for ( int i=0; i<nodes; i++){
      if( running[i] == 1)
      {
        continue;
      }

      // To make each rank at least have 1 packet to send first
      for( int j=i+1; j<nodes; j++){
        if( ranking_matrix[i][j] != 0 && running[j] != 1 && packetlist[j] != 0 && rank_x_running[ranking_matrix[i][j]] == 0)
        {
          
            DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] = j;
            DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] = i;
            link[TimeslotForDCFL]++;

            packetlist[j]--;
            packetlist[i]++;

            running[i] = 1;
            running[j] = 1;
            
            rank_x_running[ranking_matrix[i][j]] = 1;

            break;
          
        }

      }
    }


    // from leaf begin to schedule since each rank have packet already 
    for ( int i=nodes-1; i>=1; i--){
      if( running[i] == 1)
      {
        continue;
      }
      for(int j=i-1; j>=0; j--){
          // add packtlist[i] != 0
        if( adjacency_matrix[i][j] == 1 && running[j] != 1 && packetlist[i] != 0 ){

          DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] = i;
          DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] = j;
          link[TimeslotForDCFL]++;

          packetlist[i]--;
          packetlist[j]++;

          running[i] = 1;
          running[j] = 1;

          break;

        }
      }
    }

    std::cout << " packetlist : ";
    for( int i=0; i<nodes; i++){
      std::cout<< packetlist[i] << " ";
    }
    std::cout<<std::endl;
    TimeslotForDCFL ++;
  }    


  std::cout << " Scheduling Table : " << std::endl;

  for( int i=0; i<TimeslotForDCFL; i++){
    
    std::cout.width(8);
    std::cout<< i+1 << "";

  }
  std::cout << std::endl;
  
  std::ostringstream oss;
  for( int j=0; j<16; j++){
    for( int i=0; i<TimeslotForDCFL; i++){
      if( link[i] > j){
        oss.str("");
        oss << DCFL[i][j][0] << "->" << DCFL[i][j][1];
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
  

}
*/


void
LrWpanTschHelper::scheduling()   // add by ieong
{
  
  std::vector<int> running(nodes);
  std::vector<int> rank_x_running(nodes);


  while(packetlist[0] != Totalpacket){

    for (int i=0; i<nodes; i++){
      running[i] = 0;
      rank_x_running[i] = 0;
    }

    for ( int i=0; i<nodes; i++){
      if( running[i] == 1)
      {
        continue;
      }

      // To make each rank at least have 1 packet to send first
      for( int j=i+1; j<nodes; j++){
        if( ranking_matrix[i][j] != 0 && running[j] != 1 && packetlist[j] != 0 && rank_x_running[ranking_matrix[i][j]] == 0)
        {
          
            DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] = j;
            DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] = i;
            link[TimeslotForDCFL]++;

            packetlist[j]--;
            packetlist[i]++;

            running[i] = 1;
            running[j] = 1;
            
            rank_x_running[ranking_matrix[i][j]] = 1;

            break;
          
        }

      }
    }


    // from leaf begin to schedule since each rank have packet already 
    for ( int i=nodes-1; i>=1; i--){
      if( running[i] == 1)
      {
        continue;
      }
      for(int j=i-1; j>=0; j--){
          // add packtlist[i] != 0
        if( adjacency_matrix[i][j] == 1 && running[j] != 1 && packetlist[i] != 0 ){

          DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] = i;
          DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] = j;
          link[TimeslotForDCFL]++;

          packetlist[i]--;
          packetlist[j]++;

          running[i] = 1;
          running[j] = 1;

          break;

        }
      }
    }

    // found that if some node have packet but cannot send in this timeslot
    // then make more link to transmit but not orgin one 
    // the additional link cannot exceed the number of channel

    if( link[TimeslotForDCFL]-1 < 15 ) // channel list = 15
    {
      for( int i=nodes-1; i>0; i--){
        if( packetlist[i] != 0 && running[i] != 1){       // who have packet to send and not running

          std::cout << " In 1 " << std::endl;

          int rankofnode_i = 0;
          for ( int k=0; k<i; k++){                       // behind of i no need to check
            if( ranking_matrix[i][k] != 0){
              rankofnode_i = ranking_matrix[i][k];       // the first one is smallest and it is the rank
              break;
            }
          }

            
          std::cout << " In 2 " << std::endl;
          std::cout << " node i : " << i << std::endl;
          std::cout << " rank of node i : " << rankofnode_i << std::endl;

          for(int j=i-1; j>0; j--){                     

            if( running[j] != 1){                        // only for this node is not running
              std::cout << " In 3 " << std::endl;

              // find the smaller rank to transmit
              int rankofnode_j = 0;
              for ( int k=0; k<j; k++){                 // behind of j no need to check
                if( ranking_matrix[j][k] != 0){      
                  rankofnode_j = ranking_matrix[j][k];  // the first one is smallest and it is the rank
                  break;
                }
              }
                
              std::cout << " In 4 " << std::endl;
              std::cout << " node i : " << i << std::endl;
              std::cout << " node j : " << j << std::endl;

              std::cout << " rank of node i : " << rankofnode_i << std::endl;
              std::cout << " rank of node j : " << rankofnode_j << std::endl;

              if( rankofnode_i - rankofnode_j ==  1 ){   // gap of rank is 1. It may transmit to this node

                if(CheckOverDistance(i,j))   // if Over distance, the link will occur packet loss. So no add link if over distance.
                  continue;


                DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] = i;
                DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] = j;
                std::cout<< "Adding " << DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] << " to " << DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] << " in timeslot " << TimeslotForDCFL+1 << "for additional" << std::endl;

                link[TimeslotForDCFL]++;


                packetlist[i]--;
                packetlist[j]++;

                running[i] = 1;
                running[j] = 1;

                break;

              }
              else if ( rankofnode_i - rankofnode_j > 1 ) // if the gap of j already bigger than 1 . No need to check other.
              {
                break;
              } 

            }
          }
        }

        if( link[TimeslotForDCFL] >= 15 )
          break;

      }

      
    }





    /*
    std::cout << " rank_x_running : ";
    for( int x=0; x<nodes; x++){
      std::cout<< rank_x_running[x] << " ";
    }
    std::cout<<std::endl;
    */
    std::cout << " packetlist : ";
    for( int i=0; i<nodes; i++){
      std::cout<< packetlist[i] << " ";
    }
    std::cout<<std::endl;
    TimeslotForDCFL ++;
  }    

/*
  for( int i=0; i<TimeslotForDCFL ;i++){

    std::cout<< " In timeslot : " << i+1 << std::endl;
    std::cout<< "link[timeslot] : "<< link[i]<<std::endl;
    for( int j=0; j<link[i]; j++){

      std::cout<< DCFL[i][j][0] << " to " << DCFL[i][j][1] << std::endl;

    }

  }*/
  

  std::cout << " Scheduling Table : " << std::endl;

  for( int i=0; i<TimeslotForDCFL; i++){
    
    std::cout.width(8);
    std::cout<< i+1 << "";

  }
  std::cout << std::endl;
  
  std::ostringstream oss;
  for( int j=0; j<16; j++){
    for( int i=0; i<TimeslotForDCFL; i++){
      if( link[i] > j){
        oss.str("");
        oss << DCFL[i][j][0] << "->" << DCFL[i][j][1];
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
}
  












void
LrWpanTschHelper::ConfigureSlotframeAllToPanForTreeBaseOnPacket(NetDeviceContainer devs, int size)
{
  // for slotframe size
  
  AddSlotframe(devs,m_slotframehandle,size);

  //Add Adv links
  AddLinkParams alparams;
  alparams.slotframeHandle = m_slotframehandle;
  alparams.channelOffset = 0;

  alparams.linkHandle = 0;
  alparams.timeslot = 0;
  //AddAdvLink (devs,0,alparams);  // for beacon

  int linkhandleForDCFL = 1;

  for( int i=0; i<TimeslotForDCFL ;i++){   // i = 1 for beacon
  
    std::cout<< " In timeslot : " << i 
             << ", have link[timeslot] : "<< link[i]<<std::endl;
    for( int j=0; j<link[i]; j++){
      if( DCFL[i][j][0] == 0 && DCFL[i][j][1] == 0 )
        continue;
      std::cout<< DCFL[i][j][0] << " to " << DCFL[i][j][1] << std::endl;

      alparams.linkHandle = linkhandleForDCFL;
      //alparams.timeslot = i+1;  // i+1 since Adv Link use timeslot 0 // for beacon
      alparams.timeslot = i;
      alparams.channelOffset = j;

      AddLink(devs,DCFL[i][j][0],DCFL[i][j][1],alparams,false);

      linkhandleForDCFL++;

    }

  }
  std::cout << "linkhandleForDCFL : " << linkhandleForDCFL << std::endl;
  m_slotframehandle++;


}


void
LrWpanTschHelper::GenerateTrafficBaseOnPacket(NetDeviceContainer devs,int packet_size, double start, double duration, double interval)
{
  
  
  // macTsTimeslotLength = 10000us = 0.01s
  // so period to call send packet for each timeslot to send packet
  //double end = start + 0.01*TimeslotForDCFL;
  //double interval = 0.01;
  double end = start+duration;

  std::cout<< " Start Generate Traffic "  << std::endl; 
  // schedule in period for all timeslot of send
  Simulator::Schedule(Seconds(start),&LrWpanTschHelper::SendPacketBaseOnPacket,this,devs,packet_size,interval,end);

  



}

void
LrWpanTschHelper::SendPacketBaseOnPacket(NetDeviceContainer devs,int packet_size, double interval, double end)
{

  if (Now().GetSeconds() <= end)
  {
    for( int i=0; i<TimeslotForDCFL; i++){
      for( int j=0; j<link[i]; j++){
        if( DCFL[i][j][0] == 0 && DCFL[i][j][1] == 0 )
          continue;
        std::cout<< "Sending Packet From " << DCFL[i][j][0] << " to " << DCFL[i][j][1] << " at timeslot " << i << std::endl;

        Address dst = devs.Get(DCFL[i][j][1])->GetAddress();


        m_random = CreateObject<UniformRandomVariable> ();
        //packet_size = 10 + (uint8_t)m_random->GetInteger (0, 107);
        packet_size = 10 + (uint8_t)m_random->GetInteger (0, 235);
        Ptr<Packet> pkt = Create<Packet> (packet_size);
      
        devs.Get(DCFL[i][j][0])->Send(pkt,dst,0x86DD);

      }

    }
    
    //CurrentTimeslot++;
  }

  if (Now().GetSeconds() <= end+interval)
  {
      Simulator::Schedule(Seconds(interval),&LrWpanTschHelper::SendPacketBaseOnPacket,this,devs,packet_size,interval,end);
  }
  


}

void
LrWpanTschHelper::GenerateTrafficBaseOnPacketForETree(NetDeviceContainer devs,int packet_size, double start, double duration, double interval, int nrnodes)
{
  
  // macTsTimeslotLength = 10000us = 0.01s
  // so period to call send packet for each timeslot to send packet
  //double end = start + 0.01*TimeslotForDCFL;
  //double interval = 0.01;
  double end = start+duration;

  std::cout<< " Start Generate Traffic "  << std::endl; 
  // schedule in period for all timeslot of send
  Simulator::Schedule(Seconds(start),&LrWpanTschHelper::SendPacketBaseOnPacketForETree,this,devs,packet_size,interval,end,nrnodes);

}

void
LrWpanTschHelper::SendPacketBaseOnPacketForETree(NetDeviceContainer devs,int packet_size, double interval, double end, int nrnodes)
{

  if (Now().GetSeconds() <= end)
  {

    /*
    int k = link[0];
    int p = TimeslotForDCFL;

    while(k!=0){

      m_random = CreateObject<UniformRandomVariable> ();
      packet_size = 10 + (uint8_t)m_random->GetInteger (0, 107);
      Ptr<Packet> pkt = Create<Packet> (packet_size);
      int j = k-1;
      
      for( int i=0; i<p; i++){

        
        std::cout<< "Sending Packet From " << DCFL[i][j][0] << " to " << DCFL[i][j][1] << " at timeslot " << i << std::endl;
        Address dst = devs.Get(DCFL[i][j][1])->GetAddress();
        devs.Get(DCFL[i][j][0])->Send(pkt,dst,0x86DD);
        j--;

      }
      p--;
      k--;
    }
    */

    for( int i=1; i<=(nrnodes/4); i++){


      m_random = CreateObject<UniformRandomVariable> ();

      
      packet_size = 10 + (uint8_t)m_random->GetInteger (0, 107);
      //Ptr<Packet> pkt = Create<Packet> (packet_size);
      int k1 = 0, k2 = i;
      //std::cout<< " pkt size " << pkt->GetSize() << std::endl;
      while( k2 <= nrnodes ){

        Ptr<Packet> pkt = Create<Packet> (packet_size);
        std::cout<< "Sending Packet From " << k2 << " to " << k1 << " with pkt " << pkt->GetSize() << " packet_size " <<packet_size << std::endl;
        Address dst = devs.Get(k1)->GetAddress();
        devs.Get(k2)->Send(pkt,dst,0x86DD);

        k1 = k2;
        k2 = k2 + 4;


      }



    }



  }

  if (Now().GetSeconds() <= end+interval)
  {
      Simulator::Schedule(Seconds(interval),&LrWpanTschHelper::SendPacketBaseOnPacket,this,devs,packet_size,interval,end);
  }
  


}

void
LrWpanTschHelper::scheduling_ETree(int nrnodes, int slotframe_size)
{


  /*
  int k = 0;
  int k1;
  for( int i=0; i<slotframe_size; i++ ){
  
    if( i == 0 ){
      k1 = 0;
    }
    else if( i >= 1)
      k1 += (i+1);

    k = i;

    for( int j=nrnodes-k1; j>0 ; j=j-k)
    {
      k++;
      DCFL[i][link[i]][0] = j;
      if( j-k-1<0 )
        DCFL[i][link[i]][1] = 0;
      else 
        DCFL[i][link[i]][1] = j-k-1;
      link[i]++;
    }
    for ( int j=0; j<link[i]/2; j++ ){

      int temp0 = DCFL[i][j][0];
      int temp1 = DCFL[i][j][1];

      DCFL[i][j][0] = DCFL[i][link[i]-j-1][0];
      DCFL[i][j][1] = DCFL[i][link[i]-j-1][1];

      DCFL[i][link[i]-j-1][0] = temp0;
      DCFL[i][link[i]-j-1][1] = temp1;
    }
    TimeslotForDCFL = i+1;

  }
  */

  int c = 0;
  int t1 = 0;
  for ( int i=nrnodes; i>nrnodes-4; i-- ){
    
    int k1 = i, k2 = i-4;
    int t = 0;
    while( k2 > 0 ){

      DCFL[t][c][0] = k1;
      DCFL[t][c][1] = k2;
      link[t]++;
      k1 = k2;
      k2 = k2 - 4;
      t++;

    }
    if( k2 <= 0 ){
      if( t1 < t ){
        t1 = t;
      }

      k2 = 0;
      DCFL[t1][0][0] = k1;
      DCFL[t1][0][1] = k2;
      link[t1]++;
      t1++;
    }
    c++;
    TimeslotForDCFL = t1;
  }
  



  std::cout << " Scheduling Table : " << std::endl;

  for( int i=0; i<TimeslotForDCFL; i++){
    
    std::cout.width(8);
    std::cout<< i << "";

  }
  std::cout << std::endl;
  
  std::ostringstream oss;
  for( int j=0; j<16; j++){
    std::cout<< j << " ";
    for( int i=0; i<TimeslotForDCFL; i++){
      if( link[i] > j){
        oss.str("");
        oss << DCFL[i][j][0] << "->" << DCFL[i][j][1];
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

void
LrWpanTschHelper::scheduling_TASA()
{
  


  while(packetlist[0] != Totalpacket){

    // reset
    child_matrix = adjacency_matrix;
    node_n = 0;
    TASA_a = 0;
    // matching
    matching_TASA(node_n);

    for ( int i=0; i<nodes; i++)
    {
      std::cout << packetlist[i] << " ";
    }
    std::cout<<std::endl;

    TimeslotForDCFL++;

  }

  std::cout << " Scheduling Table : " << std::endl;

  for( int i=0; i<TimeslotForDCFL; i++){
    
    std::cout.width(8);
    std::cout<< i << "";

  }
  std::cout << std::endl;
  
  std::ostringstream oss;
  for( int j=0; j<16; j++){
    std::cout<< j << " ";
    for( int i=0; i<TimeslotForDCFL; i++){
      if( DCFL[i][j][0]!=0 || DCFL[i][j][1]!=0){
        oss.str("");
        oss << DCFL[i][j][0] << "->" << DCFL[i][j][1];
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

/*
///////////////////////// 
// for prevent CCA failure since will have concurrent transmit for current ts and next ts of same channel.
// rescheduling same node of current ts and next ts have same  channel offset ( ASN+offset = ASN+1+offset ) -> ( j = k -1 )

  //int k = TimeslotForDCFL%2;
  //for( int i=TimeslotForDCFL-1-k; i>0; i--){

  for( int i=0; i<TimeslotForDCFL; i++){
  //for( int i=0; i<=0; i++){
    std::cout<< " In ts " << i << " link " << link[i] << std::endl;
    std::vector<std::vector<int> > found(link[i], std::vector<int>(2, -1));   // need change for different nrnodes and timeslot
    int add = 0;
    //int extending = 0;
    for( int j=1; j<link[i]; j++){
      if( DCFL[i][j][0] == 0 && DCFL[i][j][1] == 0 ){
        continue;
      }

      for ( int k=0; k<link[i+1]; k++ ){
        if(DCFL[i][j][0] == DCFL[i+1][k][0] || DCFL[i][j][0] == DCFL[i+1][k][1]){
          if(found[k+1][0]!=-2){
            found[j][0] = k;
          }
        }
        else if(DCFL[i][j][1] == DCFL[i+1][k][0] || DCFL[i][j][1] == DCFL[i+1][k][1]){
          if(found[k+1][0]!=-2){
            found[j][1] = k;
          }
        }
      }
      if( ( found[j][0] == -1 && found[j][1]!=-1 ) || ( found[j][0] != -1 && found[j][1]== -1 ) ){

        int change;
        if(found[j][0] != -1){
          change = found[j][0];
        }
        else{
          change = found[j][1];
        }
        std::cout<< " change found[j][0] at j " << j << std::endl;
        found[j][0] = -2;
        found[j][1] = -2;
        int temp0 = DCFL[i+1][change][0];
        int temp1 = DCFL[i+1][change][1];
        int change_offset;
        if( j==0){
          change_offset = 15 - 1;   // channel list = 15
        }
        else{
          change_offset = j - 1;
        }
        if( change == change_offset )
          continue;  
        DCFL[i+1][change][0] = DCFL[i+1][change_offset][0];
        DCFL[i+1][change][1] = DCFL[i+1][change_offset][1];

        DCFL[i+1][change_offset][0] = temp0;
        DCFL[i+1][change_offset][1] = temp1;
        std::cout<< " DCFL[i+1][change_offset][0] : " << DCFL[i+1][change_offset][0] 
                 << " DCFL[i+1][change_offset][1] : " << DCFL[i+1][change_offset][1]
                 << " i+1 : " << i+1 << " change_offset : " << change_offset << std::endl;
        std::cout<< " DCFL[i+1][change][0] : " << DCFL[i+1][change][0] 
                 << " DCFL[i+1][change][1] : " << DCFL[i+1][change][1] << " change : " << change << std::endl;
        if( change_offset > link[i+1]-1 ){
          link[i+1] = change_offset+1;
          //extending = change_offset+1;
          std::cout << " 1Extending link " << link[i+1] << std::endl;
        }

      }
    }

    for ( int m=0; m<link[i]; m++){
      std::cout << " found[i][0] : " << found[m][0] << std::endl;    
    }

  std::cout << " Rescheduling Table : " << std::endl;
  std::cout.width(8);
  std::cout << " ts/ch ";
  for( int m=0; m<TimeslotForDCFL; m++){
    
    std::cout.width(8);
    std::cout<< m << "";

  }
  std::cout << std::endl;
  
  for( int n=0; n<16; n++){
    std::cout.width(8);
    std::cout<< n << " ";
    for( int m=0; m<TimeslotForDCFL; m++){
      if( DCFL[m][n][0]!=0 || DCFL[m][n][1]!=0){
        oss.str("");
        oss << DCFL[m][n][0] << "->" << DCFL[m][n][1];
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
    
    for( int j=1; j<link[i]; j++){   // not count 0 first?
      if( DCFL[i][j][0] == 0 && DCFL[i][j][1] == 0 ){
        continue;
      }
      if( found[j][0] < 0 )
        continue;
      found[j][0] = -1;
      found[j][1] = -1;
      for ( int k=0; k<link[i+1]; k++ ){
        if(DCFL[i][j][0] == DCFL[i+1][k][0] || DCFL[i][j][0] == DCFL[i+1][k][1]){
          std::cout<< " come but got at j " << j << " k " << k << std::endl;
          if(found[k+1][0]!=-2){
            found[j][0] = k;
            std::cout<< " come but got at j " << j << " k " << k << std::endl;

          }
        }
        else if(DCFL[i][j][1] == DCFL[i+1][k][0] || DCFL[i][j][1] == DCFL[i+1][k][1]){
          std::cout<< " come but got at j " << j << " k " << k << std::endl;
          if(found[k+1][0]!=-2){
            found[j][1] = k;
            std::cout<< " come but got at j " << j << " k " << k << std::endl;

          }
        }
      }
      if( found[j][1]!=-1 || found[j][0] != -1 ){
        int change;
        if(found[j][0] != -1){
          change = found[j][0];
        }
        else{
          change = found[j][1];
        }
        std::cout<< " change found[j][0] at j " << j << std::endl;

        found[j][0] = -2;
        found[j][1] = -2;
        int temp0 = DCFL[i+1][change][0];
        int temp1 = DCFL[i+1][change][1];
        int change_offset;
        if( j==0){
          change_offset = 15 - 1;   // channel list = 15
        }
        else{
          change_offset = j - 1;
        }
        if( change == change_offset )
          continue;  
        DCFL[i+1][change][0] = DCFL[i+1][change_offset][0];
        DCFL[i+1][change][1] = DCFL[i+1][change_offset][1];

        DCFL[i+1][change_offset][0] = temp0;
        DCFL[i+1][change_offset][1] = temp1;
        std::cout<< " DCFL[i+1][change_offset][0] : " << DCFL[i+1][change_offset][0] 
                 << " DCFL[i+1][change_offset][1] : " << DCFL[i+1][change_offset][1]
                 << " i+1 : " << i+1 << " change_offset : " << change_offset << std::endl;
        std::cout<< " DCFL[i+1][change][0] : " << DCFL[i+1][change][0] 
                 << " DCFL[i+1][change][1] : " << DCFL[i+1][change][1] << " change : " << change << std::endl;
        if( change_offset > link[i+1]-1 ){
          link[i+1] = change_offset+1;
          std::cout << " 2Extending link " << link[i+1] << std::endl;
        }

      }
    }
    for ( int m=2; m<link[i]; m++){
      if( DCFL[i][m][0] == 0 && DCFL[i][m][1] == 0 ){
        continue;
      }
      if( (found[m][0] == -1 && DCFL[i+1][m-1][0] != 0 && DCFL[i+1][m-1][1] != 0) &&
          (DCFL[i+1][0][0] == 0 && DCFL[i+1][0][1] == 0 ) ){
          DCFL[i+1][0][0] = DCFL[i+1][m-1][0];
          DCFL[i+1][0][1] = DCFL[i+1][m-1][1];
          DCFL[i+1][m-1][0] = 0;
          DCFL[i+1][m-1][1] = 0;

      }
      else if(found[m][0] == -1 && DCFL[i+1][m-1][0] != 0 && DCFL[i+1][m-1][1] != 0){
        while(add<15){
          if(DCFL[i+1][link[i]-1+add][0] == 0 && DCFL[i+1][link[i]-1+add][1] == 0 )
          {
            DCFL[i+1][link[i]-1+add][0] = DCFL[i+1][m-1][0];
            DCFL[i+1][link[i]-1+add][1] = DCFL[i+1][m-1][1];
            DCFL[i+1][m-1][0] = 0;
            DCFL[i+1][m-1][1] = 0;
            link[i+1] = link[i]+add;
          std::cout << " 3Extending link " << link[i+1] << std::endl;

            break;
          }
          else{
            add++;
          }
          if( add == 15 )
          {
            std::cout<< " too much " << std::endl;
          }

        }
      }

    }
    for ( int m=0; m<link[i]; m++){
      std::cout << " found[i][0] : " << found[m][0] << std::endl;    
    }
    std::cout << " Rescheduling Table : " << std::endl;
  std::cout.width(8);
  std::cout << " ts/ch ";
  for( int m=0; m<TimeslotForDCFL; m++){
    
    std::cout.width(8);
    std::cout<< m << "";

  }
  std::cout << std::endl;
  
  for( int n=0; n<16; n++){
    std::cout.width(8);
    std::cout<< n << " ";
    for( int m=0; m<TimeslotForDCFL; m++){
      if( DCFL[m][n][0]!=0 || DCFL[m][n][1]!=0){
        oss.str("");
        oss << DCFL[m][n][0] << "->" << DCFL[m][n][1];
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

  }


  std::cout << " Rescheduling Table : " << std::endl;
  std::cout.width(8);
  std::cout << " ts/ch ";
  for( int i=0; i<TimeslotForDCFL; i++){
    
    std::cout.width(8);
    std::cout<< i << "";

  }
  std::cout << std::endl;
  
  for( int j=0; j<16; j++){
    std::cout.width(8);
    std::cout<< j << " ";
    for( int i=0; i<TimeslotForDCFL; i++){
      if( DCFL[i][j][0]!=0 || DCFL[i][j][1]!=0){
        oss.str("");
        oss << DCFL[i][j][0] << "->" << DCFL[i][j][1];
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


}

void
LrWpanTschHelper::matching_TASA(int node_n)
{
  int maxpacket = 0;
  int child = 0;
  node_n_child = 0;
  node_n_neighbor = 0;
  for ( int i=node_n+1; i<nodes; i++){
      if( child_matrix[node_n][i] == 1 ){
        if(packetlist[i] > maxpacket)
        {
          maxpacket = packetlist[i];
          child = i;
          TASA_a = 1;
        }        
      }
    }
  if( child == 0 && TASA_a == 1)
  {
    return;
  }
  else if( child == 0 && TASA_a == 0 )
  {
    matching_TASA(node_n + 1);
    return;
  }
  //std::cout << child << " to " << node_n << " in timeslot : " << TimeslotForDCFL << " offset : " << link[TimeslotForDCFL] << std::endl;

  DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][0] = child;
  DCFL[TimeslotForDCFL][link[TimeslotForDCFL]][1] = node_n;
  link[TimeslotForDCFL]++;
  
  packetlist[child]--;
  packetlist[node_n]++;
  
  child_matrix[node_n][child] = 0;
  child_matrix[child][node_n] = 0;

  for ( int i=child+1; i<nodes; i++){
    if( child_matrix[child][i] == 1 ){
      node_n_child = i;
      //std::cout << " node_n_child : " << node_n_child << std::endl;
      matching_TASA(node_n_child);       
    }
  }

  for ( int i=node_n+1; i<nodes; i++){
    if( child_matrix[node_n][i] == 1 ){
      node_n_neighbor = i;
      //std::cout << " node_n_neighbor : " << node_n_neighbor << std::endl;
      matching_TASA(node_n_neighbor);
    }
  }

}

bool
LrWpanTschHelper::CheckOverDistance(int i, int j)
{

  double distance = sqrt((node_x[i]-node_x[j])*(node_x[i]-node_x[j])+(node_y[i]-node_y[j])*(node_y[i]-node_y[j]));

  std::cout << " Distance : " << distance << std::endl;

  if( distance >= 50)  // 50 is max range
    return 1;
  else
    return 0;

}

int
LrWpanTschHelper::GetActiveTimeslot()
{
  return TimeslotForDCFL;
}

int
LrWpanTschHelper::GetTotalpacket()
{
  return Totalpacket;
}

void
LrWpanTschHelper::SendConfigurationToHelper(NetDeviceContainer devs, double TimeslotLength, int pktsize, double interval, ApplicationContainer apps)
{
  //dev->GetObject<LrWpanTschNetDevice> ()->GetNMac()->SendConfigurationToMac (TimeslotLength, pktsize, interval);
  for (NetDeviceContainer::Iterator i = devs.Begin (); i != devs.End (); i++)
  {
    Ptr<LrWpanTschNetDevice> lrDevice = DynamicCast<LrWpanTschNetDevice> (*i);
    lrDevice->GetNMac()->SendConfigurationToMac (TimeslotLength, pktsize, interval, DCFL, TimeslotForDCFL, link, apps);
    break;
  }

}

void
LrWpanTschHelper::GenerateTrafficBaseOnPacketForStar(NetDeviceContainer devs,int packet_size, double start,double end)
{
  
  double end1 = start+end;
  // macTsTimeslotLength = 10000us = 0.01s
  // so period to call send packet for each timeslot to send packet
  
  double interval = TimeslotLength_helper / 1000000;  // timeslot length
  //double interval = 0.5;  // timeslot length


  Simulator::Schedule(Seconds(start),&LrWpanTschHelper::SendPacketBaseOnPacketForStar,this,devs,packet_size,interval,end1);

  



}

void
LrWpanTschHelper::SendPacketBaseOnPacketForStar(NetDeviceContainer devs,int packet_size, double interval, double end)
{

  if (Now().GetSeconds() <= end )
  {
    for( int i=1; i<numberofTimeslots; i++){

      std::cout<< "Sending Packet From " << Star_Link[i][0] << " to " << Star_Link[i][1] << " at timeslot " << i << std::endl;

      Address dst = devs.Get(Star_Link[i][1])->GetAddress();

      Ptr<Packet> pkt = Create<Packet> (packet_size);
      devs.Get(Star_Link[i][0])->Send(pkt,dst,0x86DD);

    }
  }

  if (Now().GetSeconds() <= end+interval)
  {
      Simulator::Schedule(Seconds(interval),&LrWpanTschHelper::SendPacketBaseOnPacketForStar,this,devs,packet_size,interval,end);
  }
  


}

void
LrWpanTschHelper::GenerateTrafficAndUpdatePing6(Ptr<NetDevice> dev, Address dst, double start, double interval, ApplicationContainer apps, int nrnodes)
{
  //double end = start+duration;
  //send_interval=interval;

  Simulator::Schedule(Seconds(start),&LrWpanTschHelper::SendPacketAndUpdatePing6,this,dev,dst,interval,apps,nrnodes);
  //Simulator::Schedule(Seconds(start),&LrWpanTschHelper::SendPacket,this,dev,dst,packet_size,interval,end);

}

void
LrWpanTschHelper::SendPacketAndUpdatePing6(Ptr<NetDevice> dev, Address dst, double interval, ApplicationContainer apps, int nrnodes)
{
  if (Now().GetSeconds() <= 25)
    {

      //for random packet size // ieong
      m_random = CreateObject<UniformRandomVariable> ();
      int packet_size = 10 + (uint8_t)m_random->GetInteger (0, 107);
      std::cout << " To make random packet : " << packet_size << std::endl;

      Ptr<Packet> pkt = Create<Packet> (packet_size);

      dev->Send(pkt,dst,0x86DD);

      /*
      if( packet_size > max_ping6_pktsize  ){   // 74+43 = 117        // 69+48 = 117
        max_ping6_pktsize = packet_size;
        //if( max_ping6_pktsize > 74 )
          //max_ping6_pktsize = 74;
        for (int i = 0; i <= nrnodes; i++)
        {
          Ptr<Ping6> ping6App = DynamicCast<Ping6>(apps.Get(i));
          ping6App->SetSize(max_ping6_pktsize);
          if( i==0 )
            std::cout << "change from random packet to ping6App : " << ping6App->GetSize() << std::endl;
        }

      }
      */

    }

  if (Now().GetSeconds() <= 25+interval)
    {
      Simulator::Schedule(Seconds(interval),&LrWpanTschHelper::SendPacketAndUpdatePing6,this,dev,dst,interval,apps,nrnodes);
    }
}



} // namespace ns3

