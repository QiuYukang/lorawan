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
 * Author: QiuYukang <b612n@qq.com>
 */

#include "ns3/class-c-end-device-lorawan-mac.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/log.h"
#include <algorithm>

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
                          .AddConstructor<ClassCEndDeviceLorawanMac> ();
  return tid;
}

ClassCEndDeviceLorawanMac::ClassCEndDeviceLorawanMac ()
    : // LoraWAN default
      m_receiveDelay1 (Seconds (1)),
      // LoraWAN default
      m_receiveDelay2 (Seconds (2)),
      m_rx1DrOffset (0),
      m_windowRX2BeforeRX1 (true)
{
  NS_LOG_FUNCTION (this);

  // Void the two receiveWindow events
  m_closeFirstWindow = EventId ();
  m_closeFirstWindow.Cancel ();
  m_closeSecondWindow = EventId ();
  m_closeSecondWindow.Cancel ();
  m_secondReceiveWindow = EventId ();
  m_secondReceiveWindow.Cancel ();
}

ClassCEndDeviceLorawanMac::~ClassCEndDeviceLorawanMac ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

/////////////////////
// Sending methods //
/////////////////////

void
ClassCEndDeviceLorawanMac::SendToPhy (Ptr<Packet> packetToSend)
{
  /////////////////////////////////////////////////////////
  // Add headers, prepare TX parameters and send the packet
  /////////////////////////////////////////////////////////

  NS_LOG_DEBUG ("PacketToSend: " << packetToSend);

  // Data Rate Adaptation as in LoRaWAN specification, V1.0.2 (2016)
  if (m_enableDRAdapt && (m_dataRate > 0) && (m_retxParams.retxLeft < m_maxNumbTx) &&
      (m_retxParams.retxLeft % 2 == 0))
    {
      m_txPower = 14; // Reset transmission power
      m_dataRate = m_dataRate - 1;
    }

  // Craft LoraTxParameters object
  LoraTxParameters params;
  params.sf = GetSfFromDataRate (m_dataRate);
  params.headerDisabled = m_headerDisabled;
  params.codingRate = m_codingRate;
  params.bandwidthHz = GetBandwidthFromDataRate (m_dataRate);
  params.nPreamble = m_nPreambleSymbols;
  params.crcEnabled = 1;
  params.lowDataRateOptimizationEnabled = 0;

  // Wake up PHY layer and directly send the packet

  Ptr<LogicalLoraChannel> txChannel = GetChannelForTx ();

  NS_LOG_DEBUG ("PacketToSend: " << packetToSend);
  m_phy->Send (packetToSend, params, txChannel->GetFrequency (), m_txPower);

  //////////////////////////////////////////////
  // Register packet transmission for duty cycle
  //////////////////////////////////////////////

  // Compute packet duration
  Time duration = m_phy->GetOnAirTime (packetToSend, params);

  // Register the sent packet into the DutyCycleHelper
  m_channelHelper.AddEvent (duration, txChannel);

  //////////////////////////////
  // Prepare for the downlink //
  //////////////////////////////

  // Switch the PHY to the channel so that it will listen here for downlink
  m_phy->GetObject<EndDeviceLoraPhy> ()->SetFrequency (txChannel->GetFrequency ());
  m_firstReceiveWindowFrequency = txChannel->GetFrequency ();

  // Instruct the PHY on the right Spreading Factor to listen for during the window
  // create a SetReplyDataRate function?
  uint8_t replyDataRate = GetFirstReceiveWindowDataRate ();
  NS_LOG_DEBUG ("m_dataRate: " << unsigned(m_dataRate)
                               << ", m_rx1DrOffset: " << unsigned(m_rx1DrOffset)
                               << ", replyDataRate: " << unsigned(replyDataRate) << ".");

  m_phy->GetObject<EndDeviceLoraPhy> ()->SetSpreadingFactor (GetSfFromDataRate (replyDataRate));
}

//////////////////////////
//  Receiving methods   //
//////////////////////////
void
ClassCEndDeviceLorawanMac::Receive (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this << packet);

  // Work on a copy of the packet
  Ptr<Packet> packetCopy = packet->Copy ();

  // Remove the Mac Header to get some information
  LorawanMacHeader mHdr;
  packetCopy->RemoveHeader (mHdr);

  NS_LOG_DEBUG ("Mac Header: " << mHdr);

  // Only keep analyzing the packet if it's downlink
  if (!mHdr.IsUplink ())
    {
      NS_LOG_INFO ("Found a downlink packet.");

      // Remove the Frame Header
      LoraFrameHeader fHdr;
      fHdr.SetAsDownlink ();
      packetCopy->RemoveHeader (fHdr);

      NS_LOG_DEBUG ("Frame Header: " << fHdr);

      // Determine whether this packet is for us
      bool messageForUs = (m_address == fHdr.GetAddress ());

      if (messageForUs)
        {
          NS_LOG_INFO ("The message is for us!");

          // If it exists, cancel the second receive window event
          // THIS WILL BE GetReceiveWindow()
          Simulator::Cancel (m_secondReceiveWindow);

          // Parse the MAC commands
          ParseCommands (fHdr);

          // TODO Pass the packet up to the NetDevice

          // Call the trace source
          m_receivedPacket (packet);
        }
      else
        {
          NS_LOG_DEBUG ("The message is intended for another recipient.");

          // In this case, we are either receiving in the first receive window
          // and finishing reception inside the second one, or receiving a
          // packet in the second receive window and finding out, after the
          // fact, that the packet is not for us. In either case, if we no
          // longer have any retransmissions left, we declare failure.
          if (m_retxParams.waitingAck && m_secondReceiveWindow.IsExpired ())
            {
              if (m_retxParams.retxLeft == 0)
                {
                  uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
                  m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
                  NS_LOG_DEBUG ("Failure: no more retransmissions left. Used "
                                << unsigned(txs) << " transmissions.");

                  // Reset retransmission parameters
                  resetRetransmissionParameters ();
                }
              else // Reschedule
                {
                  this->Send (m_retxParams.packet);
                  NS_LOG_INFO ("We have " << unsigned(m_retxParams.retxLeft)
                                          << " retransmissions left: rescheduling transmission.");
                }
            }
        }
    }
  else if (m_retxParams.waitingAck && m_secondReceiveWindow.IsExpired ())
    {
      NS_LOG_INFO ("The packet we are receiving is in uplink.");
      if (m_retxParams.retxLeft > 0)
        {
          this->Send (m_retxParams.packet);
          NS_LOG_INFO ("We have " << unsigned(m_retxParams.retxLeft)
                                  << " retransmissions left: rescheduling transmission.");
        }
      else
        {
          uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
          m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
          NS_LOG_DEBUG ("Failure: no more retransmissions left. Used " << unsigned(txs)
                                                                       << " transmissions.");

          // Reset retransmission parameters
          resetRetransmissionParameters ();
        }
    }

  m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToSleep ();
}

void
ClassCEndDeviceLorawanMac::FailedReception (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this << packet);

  // Switch to sleep after a failed reception
  m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToSleep ();

  if (m_secondReceiveWindow.IsExpired () && m_retxParams.waitingAck)
    {
      if (m_retxParams.retxLeft > 0)
        {
          this->Send (m_retxParams.packet);
          NS_LOG_INFO ("We have " << unsigned(m_retxParams.retxLeft)
                                  << " retransmissions left: rescheduling transmission.");
        }
      else
        {
          uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
          m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
          NS_LOG_DEBUG ("Failure: no more retransmissions left. Used " << unsigned(txs)
                                                                       << " transmissions.");

          // Reset retransmission parameters
          resetRetransmissionParameters ();
        }
    }
}

// void
// ClassCEndDeviceLorawanMac::TxFinished (Ptr<const Packet> packet)
// {
//   NS_LOG_FUNCTION_NOARGS ();

//   // Schedule the opening of the first receive window
//   Simulator::Schedule (m_receiveDelay1,
//                        &ClassCEndDeviceLorawanMac::OpenFirstReceiveWindow, this);

//   // Schedule the opening of the second receive window
//   m_secondReceiveWindow = Simulator::Schedule (m_receiveDelay2,
//                                                &ClassCEndDeviceLorawanMac::OpenSecondReceiveWindow,
//                                                this);
//   // // Schedule the opening of the first receive window
//   // Simulator::Schedule (m_receiveDelay1,
//   //                      &ClassCEndDeviceLorawanMac::OpenFirstReceiveWindow, this);
//   //
//   // // Schedule the opening of the second receive window
//   // m_secondReceiveWindow = Simulator::Schedule (m_receiveDelay2,
//   //                                              &ClassCEndDeviceLorawanMac::OpenSecondReceiveWindow,
//   //                                              this);

//   // Switch the PHY to sleep
//   m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToSleep ();
// }

void
ClassCEndDeviceLorawanMac::TxFinished (Ptr<const Packet> packet)
{
  NS_LOG_FUNCTION_NOARGS ();

  // Schedule the opening of the second receive window before opening the first one
  OpenSecondReceiveWindow (true);
  m_closeSecondWindow = Simulator::Schedule (
      m_receiveDelay1, &ClassCEndDeviceLorawanMac::CloseSecondReceiveWindow, this);

  // Schedule the opening of the first receive window
  Simulator::Schedule (m_receiveDelay1, &ClassCEndDeviceLorawanMac::OpenFirstReceiveWindow, this);

  // // Schedule the opening of the second receive window
  // m_secondReceiveWindow = Simulator::Schedule (m_receiveDelay2,
  //                                              &ClassCEndDeviceLorawanMac::OpenSecondReceiveWindow,
  //                                              this);

  // // Switch the PHY to sleep
  // m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToSleep ();
}

void
ClassCEndDeviceLorawanMac::DoSend (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this);

  // Close the second recieve window before TX
  NS_LOG_DEBUG ("Try to close RX2 window before DoSend.");
  Simulator::Cancel (m_closeSecondWindow);
  CloseSecondReceiveWindow ();

  // Checking if this is the transmission of a new packet
  if (packet != m_retxParams.packet)
    {
      NS_LOG_DEBUG ("Received a new packet from application. Resetting retransmission parameters.");
      m_currentFCnt++;
      NS_LOG_DEBUG ("APP packet: " << packet << ".");

      // Add the Lora Frame Header to the packet
      LoraFrameHeader frameHdr;
      ApplyNecessaryOptions (frameHdr);
      packet->AddHeader (frameHdr);

      NS_LOG_INFO ("Added frame header of size " << frameHdr.GetSerializedSize () << " bytes.");

      // Add the Lora Mac header to the packet
      LorawanMacHeader macHdr;
      ApplyNecessaryOptions (macHdr);
      packet->AddHeader (macHdr);

      // Reset MAC command list
      m_macCommandList.clear ();

      if (m_retxParams.waitingAck)
        {
          // Call the callback to notify about the failure
          uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
          m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
          NS_LOG_DEBUG (" Received new packet from the application layer: stopping retransmission "
                        "procedure. Used "
                        << unsigned(txs) << " transmissions out of a maximum of "
                        << unsigned(m_maxNumbTx) << ".");
        }

      // Reset retransmission parameters
      resetRetransmissionParameters ();

      // If this is the first transmission of a confirmed packet, save parameters for the (possible) next retransmissions.
      if (m_mType == LorawanMacHeader::CONFIRMED_DATA_UP)
        {
          m_retxParams.packet = packet->Copy ();
          m_retxParams.retxLeft = m_maxNumbTx;
          m_retxParams.waitingAck = true;
          m_retxParams.firstAttempt = Simulator::Now ();
          m_retxParams.retxLeft =
              m_retxParams.retxLeft - 1; // decreasing the number of retransmissions

          NS_LOG_DEBUG ("Message type is " << m_mType);
          NS_LOG_DEBUG ("It is a confirmed packet. Setting retransmission parameters and "
                        "decreasing the number of transmissions left.");

          NS_LOG_INFO ("Added MAC header of size " << macHdr.GetSerializedSize () << " bytes.");

          // Sent a new packet
          NS_LOG_DEBUG ("Copied packet: " << m_retxParams.packet);
          m_sentNewPacket (m_retxParams.packet);

          // static_cast<ClassAEndDeviceLorawanMac*>(this)->SendToPhy (m_retxParams.packet);
          SendToPhy (m_retxParams.packet);
        }
      else
        {
          m_sentNewPacket (packet);
          // static_cast<ClassAEndDeviceLorawanMac*>(this)->SendToPhy (packet);
          SendToPhy (packet);
        }
    }
  // this is a retransmission
  else
    {
      if (m_retxParams.waitingAck)
        {

          m_currentFCnt++;

          // Remove the headers
          LorawanMacHeader macHdr;
          LoraFrameHeader frameHdr;
          packet->RemoveHeader (macHdr);
          packet->RemoveHeader (frameHdr);

          // Add the Lora Frame Header to the packet
          frameHdr = LoraFrameHeader ();
          ApplyNecessaryOptions (frameHdr);
          packet->AddHeader (frameHdr);

          NS_LOG_INFO ("Added frame header of size " << frameHdr.GetSerializedSize () << " bytes.");

          // Add the Lorawan Mac header to the packet
          macHdr = LorawanMacHeader ();
          ApplyNecessaryOptions (macHdr);
          packet->AddHeader (macHdr);
          m_retxParams.retxLeft =
              m_retxParams.retxLeft - 1; // decreasing the number of retransmissions
          NS_LOG_DEBUG ("Retransmitting an old packet.");

          // static_cast<ClassAEndDeviceLorawanMac*>(this)->SendToPhy (m_retxParams.packet);
          SendToPhy (m_retxParams.packet);
        }
    }
}

void
ClassCEndDeviceLorawanMac::OpenFirstReceiveWindow (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  // If the second recieve window is open, close it?
  if (m_closeSecondWindow.IsRunning ())
    {
      CloseSecondReceiveWindow ();
      m_closeSecondWindow.Cancel ();
    }

  // Set Phy in Standby mode
  m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToStandby ();

  // Set the frequency and sf of the first receive window, because
  // they are changed when opening the second recieve window.
  m_phy->GetObject<EndDeviceLoraPhy> ()->SetFrequency (m_firstReceiveWindowFrequency);
  m_phy->GetObject<EndDeviceLoraPhy> ()->SetSpreadingFactor (
      GetSfFromDataRate (GetFirstReceiveWindowDataRate ()));

  // Calculate the duration of a single symbol for the first receive window DR
  double tSym = pow (2, GetSfFromDataRate (GetFirstReceiveWindowDataRate ())) /
                GetBandwidthFromDataRate (GetFirstReceiveWindowDataRate ());

  // Schedule return to sleep after "at least the time required by the end
  // device's radio transceiver to effectively detect a downlink preamble"
  // (LoraWAN specification)
  m_closeFirstWindow = Simulator::Schedule (Seconds (m_receiveWindowDurationInSymbols * tSym),
                                            &ClassCEndDeviceLorawanMac::CloseFirstReceiveWindow,
                                            this); //m_receiveWindowDuration
}

void
ClassCEndDeviceLorawanMac::CloseFirstReceiveWindow (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy> ();

  // Check the Phy layer's state:
  // - RX -> We are receiving a preamble.
  // - STANDBY -> Nothing was received.
  // - SLEEP -> We have received a packet.
  // We should never be in TX or SLEEP mode at this point
  switch (phy->GetState ())
    {
    case EndDeviceLoraPhy::TX:
      NS_ABORT_MSG ("PHY was in TX mode when attempting to "
                    << "close a receive window.");
      break;
    case EndDeviceLoraPhy::RX:
      // PHY is receiving: let it finish. The Receive method will switch it back to SLEEP.
      break;
    case EndDeviceLoraPhy::SLEEP:
      // PHY has received, and the MAC's Receive already put the device to sleep
      break;
    case EndDeviceLoraPhy::STANDBY:
      // Turn PHY layer to SLEEP
      phy->SwitchToSleep ();
      break;
    }

  // Open the second recieve window (fixme:if phy is in 'RX' state?)
  OpenSecondReceiveWindow (false);
}

void
ClassCEndDeviceLorawanMac::OpenSecondReceiveWindow (bool beforeRX1)
{
  NS_LOG_FUNCTION (this << beforeRX1);

  m_windowRX2BeforeRX1 = beforeRX1;

  // Check for receiver status: if it's locked on a packet, don't open this
  // window at all.
  if (m_phy->GetObject<EndDeviceLoraPhy> ()->GetState () == EndDeviceLoraPhy::RX)
    {
      NS_LOG_INFO ("Won't open second receive window since we are in RX mode.");

      return;
    }

  // Set Phy in Standby mode
  m_phy->GetObject<EndDeviceLoraPhy> ()->SwitchToStandby ();

  // Switch to appropriate channel and data rate
  NS_LOG_INFO ("Using parameters: " << m_secondReceiveWindowFrequency << "Hz, DR"
                                    << unsigned(m_secondReceiveWindowDataRate));
  // NS_LOG_DEBUG("---qiu--debug-- set frequency(RX2 f) of phy listen:" << m_secondReceiveWindowFrequency << "MHz");
  m_phy->GetObject<EndDeviceLoraPhy> ()->SetFrequency (m_secondReceiveWindowFrequency);
  m_phy->GetObject<EndDeviceLoraPhy> ()->SetSpreadingFactor (
      GetSfFromDataRate (m_secondReceiveWindowDataRate));

  // Close the second recieve window after a long long time
  m_closeSecondWindow = Simulator::Schedule (
      Time::Max (), &ClassCEndDeviceLorawanMac::CloseSecondReceiveWindow, this);

  // qiuyukang comment
  // Don't close the second recieve window
  /*
  //Calculate the duration of a single symbol for the second receive window DR
  double tSym = pow (2, GetSfFromDataRate (GetSecondReceiveWindowDataRate ())) / GetBandwidthFromDataRate ( GetSecondReceiveWindowDataRate ());

  // Schedule return to sleep after "at least the time required by the end
  // device's radio transceiver to effectively detect a downlink preamble"
  // (LoraWAN specification)
  m_closeSecondWindow = Simulator::Schedule (Seconds (m_receiveWindowDurationInSymbols*tSym),
                                             &ClassCEndDeviceLorawanMac::CloseSecondReceiveWindow, this);
  */
}

void
ClassCEndDeviceLorawanMac::CloseSecondReceiveWindow (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy> ();

  // NS_ASSERT (phy->m_state != EndDeviceLoraPhy::TX &&
  // phy->m_state != EndDeviceLoraPhy::SLEEP);

  // Check the Phy layer's state:
  // - RX -> We have received a preamble.
  // - STANDBY -> Nothing was detected.
  switch (phy->GetState ())
    {
    case EndDeviceLoraPhy::TX:
      break;
    case EndDeviceLoraPhy::SLEEP:
      break;
    case EndDeviceLoraPhy::RX:
      // PHY is receiving: let it finish
      NS_LOG_DEBUG ("PHY is receiving: Receive will handle the result.");
      return;
    case EndDeviceLoraPhy::STANDBY:
      // Turn PHY layer to sleep
      phy->SwitchToSleep ();
      break;
    }

  // fixme: How to detect timeout of ack
  if (m_retxParams.waitingAck)
    {
      // If this is the first RX2 recieve window after TxFinish, just don't detect ack_timeout
      if (m_windowRX2BeforeRX1)
        {
          NS_LOG_DEBUG("This is the first time to close RX2 recieve window.");
          return;
        }

      NS_LOG_DEBUG ("No reception initiated by PHY: rescheduling transmission.");
      if (m_retxParams.retxLeft > 0)
        {
          NS_LOG_INFO ("We have " << unsigned(m_retxParams.retxLeft)
                                  << " retransmissions left: rescheduling transmission.");
          this->Send (m_retxParams.packet);
        }

      else if (m_retxParams.retxLeft == 0 &&
               m_phy->GetObject<EndDeviceLoraPhy> ()->GetState () != EndDeviceLoraPhy::RX)
        {
          uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
          m_requiredTxCallback (txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
          NS_LOG_DEBUG ("Failure: no more retransmissions left. Used " << unsigned(txs)
                                                                       << " transmissions.");

          // Reset retransmission parameters
          resetRetransmissionParameters ();
        }

      else
        {
          NS_ABORT_MSG ("The number of retransmissions left is negative ! ");
        }
    }
  else
    {
      uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
      m_requiredTxCallback (txs, true, m_retxParams.firstAttempt, m_retxParams.packet);
      NS_LOG_INFO (
          "We have " << unsigned(m_retxParams.retxLeft)
                     << " transmissions left. We were not transmitting confirmed messages.");

      // Reset retransmission parameters
      resetRetransmissionParameters ();
    }
}

/////////////////////////
// Getters and Setters //
/////////////////////////

Time
ClassCEndDeviceLorawanMac::GetNextClassTransmissionDelay (Time waitingTime)
{
  NS_LOG_FUNCTION_NOARGS ();

  if (!m_closeFirstWindow.IsExpired () || !m_closeSecondWindow.IsExpired () ||
      !m_secondReceiveWindow.IsExpired ())
    {
      NS_LOG_WARN ("Attempting to send when there are receive windows:"
                   << " Transmission postponed.");

      //Calculate the duration of a single symbol for the second receive window DR
      double tSym = pow (2, GetSfFromDataRate (GetSecondReceiveWindowDataRate ())) /
                    GetBandwidthFromDataRate (GetSecondReceiveWindowDataRate ());

      Time endSecondRxWindow =
          (m_receiveDelay2 + Seconds (m_receiveWindowDurationInSymbols * tSym));
      NS_LOG_DEBUG ("---qiu-debug---: waitingTime"
                    << waitingTime.GetSeconds ()
                    << "  endSecondRxWindow:" << endSecondRxWindow.GetSeconds ());
      waitingTime = std::max (waitingTime, endSecondRxWindow);
    }

  return waitingTime;
}

uint8_t
ClassCEndDeviceLorawanMac::GetFirstReceiveWindowDataRate (void)
{
  return m_replyDataRateMatrix.at (m_dataRate).at (m_rx1DrOffset);
}

void
ClassCEndDeviceLorawanMac::SetSecondReceiveWindowDataRate (uint8_t dataRate)
{
  m_secondReceiveWindowDataRate = dataRate;
}

uint8_t
ClassCEndDeviceLorawanMac::GetSecondReceiveWindowDataRate (void)
{
  return m_secondReceiveWindowDataRate;
}

void
ClassCEndDeviceLorawanMac::SetSecondReceiveWindowFrequency (double frequencyMHz)
{
  m_secondReceiveWindowFrequency = frequencyMHz;
}

double
ClassCEndDeviceLorawanMac::GetSecondReceiveWindowFrequency (void)
{
  return m_secondReceiveWindowFrequency;
}

/////////////////////////
// MAC command methods //
/////////////////////////

void
ClassCEndDeviceLorawanMac::OnRxClassParamSetupReq (Ptr<RxParamSetupReq> rxParamSetupReq)
{
  NS_LOG_FUNCTION (this << rxParamSetupReq);

  bool offsetOk = true;
  bool dataRateOk = true;

  uint8_t rx1DrOffset = rxParamSetupReq->GetRx1DrOffset ();
  uint8_t rx2DataRate = rxParamSetupReq->GetRx2DataRate ();
  double frequency = rxParamSetupReq->GetFrequency ();

  NS_LOG_FUNCTION (this << unsigned(rx1DrOffset) << unsigned(rx2DataRate) << frequency);

  // Check that the desired offset is valid
  if (!(0 <= rx1DrOffset && rx1DrOffset <= 5))
    {
      offsetOk = false;
    }

  // Check that the desired data rate is valid
  if (GetSfFromDataRate (rx2DataRate) == 0 || GetBandwidthFromDataRate (rx2DataRate) == 0)
    {
      dataRateOk = false;
    }

  // For now, don't check for validity of frequency
  m_secondReceiveWindowDataRate = rx2DataRate;
  m_rx1DrOffset = rx1DrOffset;
  m_secondReceiveWindowFrequency = frequency;

  // Craft a RxParamSetupAns as response
  NS_LOG_INFO ("Adding RxParamSetupAns reply");
  m_macCommandList.push_back (CreateObject<RxParamSetupAns> (offsetOk, dataRateOk, true));
}

} /* namespace lorawan */
} /* namespace ns3 */
