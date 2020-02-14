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

#ifndef CLASS_C_END_DEVICE_LORAWAN_MAC_H
#define CLASS_C_END_DEVICE_LORAWAN_MAC_H

#include "ns3/lorawan-mac.h"                // Packet
#include "ns3/end-device-lorawan-mac.h"     // EndDeviceLorawanMac
#include "ns3/lora-frame-header.h"          // RxParamSetupReq
// #include "ns3/random-variable-stream.h"
#include "ns3/lora-device-address.h"
// #include "ns3/traced-value.h"

namespace ns3 {
namespace lorawan {

/**
 * Class representing the MAC layer of a Class A LoRaWAN device.
 */
class ClassCEndDeviceLorawanMac : public EndDeviceLorawanMac
{
public:
  static TypeId GetTypeId (void);

  ClassCEndDeviceLorawanMac ();
  virtual ~ClassCEndDeviceLorawanMac ();

  /////////////////////
  // Sending methods //
  /////////////////////

  /**
  * Add headers and send a packet with the sending function of the physical layer.
  *
  * \param packet the packet to send
  */
  virtual void SendToPhy (Ptr<Packet> packet);

  //////////////////////////
  //  Receiving methods   //
  //////////////////////////

  /**
   * Receive a packet.
   *
   * This method is typically registered as a callback in the underlying PHY
   * layer so that it's called when a packet is going up the stack.
   *
   * \param packet the received packet.
   */
  virtual void Receive (Ptr<Packet const> packet);

  virtual void FailedReception (Ptr<Packet const> packet);

  /**
   * Perform the actions that are required after a packet send.
   *
   * This function handles opening of the first receive window.
   */
  virtual void TxFinished (Ptr<const Packet> packet);

  /**
   * Checking if we are performing the transmission of a new packet or a retransmission, and call SendToPhy function.
   *
   * \param packet the packet to send
   */
  virtual void DoSend (Ptr<Packet> packet);

  /**
   * Perform operations needed to open the first receive window.
   */
  void OpenFirstReceiveWindow (void);

  /**
   * Perform operations needed to open the second receive window.
   * 
   * \param beforeRX1 A flag that identifies the type of RX2 window
   */
  void OpenSecondReceiveWindow (bool beforeRX1);

  /**
   * Perform operations needed to close the first receive window.
   */
  void CloseFirstReceiveWindow (void);

  /**
   * Perform operations needed to close the second receive window.
   */
  void CloseSecondReceiveWindow (void);

  /////////////////////////
  // Getters and Setters //
  /////////////////////////

  /**
   * Find the minimum waiting time before the next possible transmission based
   * on End Device's Class Type.
   */
  virtual Time GetNextClassTransmissionDelay (Time waitingTime);

  /**
   * Get the Data Rate that will be used in the first receive window.
   *
   * \return The Data Rate
   */
  uint8_t GetFirstReceiveWindowDataRate (void);

  /**
   * Set the Data Rate to be used in the second receive window.
   *
   * \param dataRate The Data Rate.
   */
  void SetSecondReceiveWindowDataRate (uint8_t dataRate);

  /**
   * Get the Data Rate that will be used in the second receive window.
   *
   * \return The Data Rate
   */
  uint8_t GetSecondReceiveWindowDataRate (void);

  /**
   * Set the frequency that will be used for the second receive window.
   *
   * \param frequencyMHz the Frequency.
   */
  void SetSecondReceiveWindowFrequency (double frequencyMHz);

  /**
   * Get the frequency that is used for the second receive window.
   *
   * @return The frequency, in MHz
   */
  double GetSecondReceiveWindowFrequency (void);

  /////////////////////////
  // MAC command methods //
  /////////////////////////

  /**
   * Perform the actions that need to be taken when receiving a RxParamSetupReq
   * command based on the Device's Class Type.
   *
   * \param rxParamSetupReq The Parameter Setup Request, which contains:
   *                            - The offset to set.
   *                            - The data rate to use for the second receive window.
   *                            - The frequency to use for the second receive window.
   */
  virtual void OnRxClassParamSetupReq (Ptr<RxParamSetupReq> rxParamSetupReq);

private:

  /**
   * The interval between when a packet is done sending and when the first
   * receive window is opened.
   */
  Time m_receiveDelay1;

  /**
   * The interval between when a packet is done sending and when the second
   * receive window is opened.
   */
  Time m_receiveDelay2;

  /**
   * The event of the closing the first receive window.
   *
   * This Event will be canceled if there's a successful reception of a packet.
   */
  EventId m_closeFirstWindow;

  /**
   * The event of the closing the second receive window.
   *
   * This Event will be canceled if there's a successful reception of a packet.
   */
  EventId m_closeSecondWindow;

  /**
   * The event of the second receive window opening.
   *
   * This Event is used to cancel the second window in case the first one is
   * successful.
   */
  EventId m_secondReceiveWindow;

  /**
   * The frequency to listen on for the first receive window.
   */
  double m_firstReceiveWindowFrequency;

  /**
   * The frequency to listen on for the second receive window.
   */
  double m_secondReceiveWindowFrequency;

  /**
   * The Data Rate to listen for during the second downlink transmission.
   */
  uint8_t m_secondReceiveWindowDataRate;

  /**
   * The RX1DROffset parameter value
   */
  uint8_t m_rx1DrOffset;
  
  /**
   * A flag that identifies the type of RX2 window
   *  true: RX2 window after Tx finish
   *  false: RX2 widow after RX1 window close
   */
  bool m_windowRX2BeforeRX1;

}; /* ClassCEndDeviceLorawanMac */
} /* namespace lorawan */
} /* namespace ns3 */
#endif /* CLASS_C_END_DEVICE_LORAWAN_MAC_H */
