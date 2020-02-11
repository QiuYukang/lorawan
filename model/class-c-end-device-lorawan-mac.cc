/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
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
 * Author: Peggy Anderson <peggy.anderson@usask.ca>
 */

#include "ns3/class-c-end-device-lorawan-mac.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/log.h"

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("ClassCEndDeviceLorawanMac");

NS_OBJECT_ENSURE_REGISTERED (ClassCEndDeviceLorawanMac);

TypeId
ClassCEndDeviceLorawanMac::GetTypeId (void)
{
static TypeId tid = TypeId ("ns3::ClassCEndDeviceLorawanMac")
  .SetParent<EndDeviceLorawanMac> ()
  .SetGroupName ("lorawan")
  .AddTraceSource ("NumberOfOpenSecondReceiveWindowCalls",
                   "Whether or not the second occurence of the "
                    "second receive window is cancelled.",
                    MakeTraceSourceAccessor
                      (&EndDeviceLorawanMac::m_numCloseSecondReceiveWindowCalls),
                    "ns3::TracedValueCallback::Int")
  .AddConstructor<ClassCEndDeviceLorawanMac> ();
return tid;
}

ClassCEndDeviceLorawanMac::ClassCEndDeviceLorawanMac () 
{
  NS_LOG_FUNCTION (this);

  m_deviceClass = EndDeviceLorawanMac::CLASS_C;
}

ClassCEndDeviceLorawanMac::~ClassCEndDeviceLorawanMac ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

//////////////////////////
//  Receiving methods   //
//////////////////////////

void
ClassCEndDeviceLorawanMac::Receive (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this << packet);

  NS_LOG_DEBUG ("Receive");
}

void
ClassCEndDeviceLorawanMac::FailedReception (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this << packet);

  NS_LOG_DEBUG ("FailedReception");
}

void
ClassCEndDeviceLorawanMac::TxFinished (Ptr<const Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  NS_LOG_DEBUG ("TxFinished");

  // Schedule the opening of the second receive window
  m_secondReceiveWindow = Simulator::Schedule (m_receiveDelay1,
                                               &ClassCEndDeviceLorawanMac::OpenSecondReceiveWindow,
                                               this);

  // Schedule the opening of the first receive window
  Simulator::Schedule ((m_receiveDelay2 - m_receiveDelay1),
                       &ClassCEndDeviceLorawanMac::OpenFirstReceiveWindow, this);

  // Schedule the opening of the second receive window
  m_secondReceiveWindow = Simulator::Schedule (Simulator::GetMaximumSimulationTime (),
                                               &ClassCEndDeviceLorawanMac::OpenSecondReceiveWindow,
                                               this);
  
}

} /* namespace lorawan */
} /* namespace ns3 */
