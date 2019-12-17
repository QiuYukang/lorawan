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

#ifndef CLASS_A_END_DEVICE_LORAWAN_MAC_H
#define CLASS_A_END_DEVICE_LORAWAN_MAC_H

namespace ns3 {
namespace lorawan {


/**
 * Class representing the MAC layer of a Class C LoRaWAN device.
 */
class ClassAEndDeviceLorawanMac : public EndDeviceLorawanMac
{
public:
  static TypeId GetTypeId (void);

  ClassAEndDeviceLorawanMac ();
  virtual ~ClassAEndDeviceLorawanMac ();

}; /* ClassAEndDeviceLorawanMac */

} /* namespace lorawan */
} /* namespace ns3 */

#endif /* CLASS_A_END_DEVICE_LORAWAN_MAC_H */
