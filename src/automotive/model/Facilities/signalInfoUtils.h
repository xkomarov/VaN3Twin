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
 *  Diego Gasco, Politecnico di Torino (diego.gasco@polito.it, diego.gasco99@gmail.com)
*/

#ifndef SIGNALINFOUTILS_H
#define SIGNALINFOUTILS_H
#include <string>
#include <cmath>
#include <fstream>

const double DEFAULT_VALUE = std::numeric_limits<double>::quiet_NaN();
const double SENTINEL_VALUE = 42000;

typedef struct {
    double timestamp;
    double rssi;
    double snr;
    double sinr;
    double rsrp;
    double size;
} SignalInfo;


class SignalInfoUtils
{
public:
    SignalInfoUtils();
    void SetSignalInfo(double timestamp, double size, double rssi, double snr, double sinr, double rsrp);
    SignalInfo GetSignalInfo();
    void WriteLastSignalInfo(std::string path, long stationID);

private:
    SignalInfo m_signalInfo;
};


#endif // SIGNALINFOUTILS_H
