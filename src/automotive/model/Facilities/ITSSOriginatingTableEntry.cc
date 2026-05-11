/* ============================================================================
 * Research Project: Data communication in the environment of
intelligent cars
 * Author: Kirill Komarov
 * Date: 2026
 * 
 * Description:
 * This file contains source code developed (or modified) as part of the 
 * research for the paper: "Data communication in the environment of
intelligent cars".
 * 
 * DISCLAIMER & ACKNOWLEDGEMENT:
 * Please note that this file contains or may contain code fragments, 
 * algorithms, or architectural solutions that were previously implemented 
 * in the "VaN3Twin" project https://github.com/DriveX-devs/VaN3Twin.git.
 * 
 * The borrowed code has been adapted and is used strictly for academic 
 * and research purposes. All rights to the original code segments belong 
 * to their respective original authors.
 * ============================================================================ */
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/

#include "ITSSOriginatingTableEntry.h"

namespace ns3 {
  ITSSOriginatingTableEntry::ITSSOriginatingTableEntry() {
    m_status = STATE_UNSET;
    m_actionid.originatingStationID = 0;
    m_actionid.sequenceNumber = -1;
    m_referenceTime = -1;
  }

  ITSSOriginatingTableEntry::ITSSOriginatingTableEntry(Packet asnDenmPacket, denm_table_state_t status, DEN_ActionID_t actionID)
  {
    m_denm_encoded = asnDenmPacket;
    m_status = status;
    m_actionid = actionID;
  }

  ITSSOriginatingTableEntry::ITSSOriginatingTableEntry(Packet asnDenmPacket, denm_table_state_t status, DEN_ActionID_t actionID, long referenceTime)
  {
    m_denm_encoded = asnDenmPacket;
    m_status = status;
    m_actionid = actionID;
    m_referenceTime = referenceTime;
  }
}
