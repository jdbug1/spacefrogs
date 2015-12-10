

/*********************************************************** Copyright
 **
 ** Copyright (c) 2008, German Aerospace Center (DLR)
 ** All rights reserved.
 **
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 **
 ** 1 Redistributions of source code must retain the above copyright
 **   notice, this list of conditions and the following disclaimer.
 **
 ** 2 Redistributions in binary form must reproduce the above copyright
 **   notice, this list of conditions and the following disclaimer in the
 **   documentation and/or other materials provided with the
 **   distribution.
 **
 ** 3 Neither the name of the German Aerospace Center nor the names of
 **   its contributors may be used to endorse or promote products derived
 **   from this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **
 ****************************************************************************/

#pragma once

#include "hw_udp.h"

/**
 * @file udp.h
 * @date 2008/05/20 6:20
 * @author Sergio Montenegro
 *
 * Copyright 2008 DLR
 *
 * UDP interface
 *
 */


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

///To receive data from a single UDP port (socket) - abstraction layer.
class UDPIn {
    /// socket API, used internally for the HW Implementation
    UDPReceiver udpRx;
public:
    /** Constructor, opens an UDP connection to the given port
     * @param[in] portNr port number on localhost used for reception
     */
    UDPIn(const long portNr);


    /** Constructor, opens an UDP connection to the given port
     * and incommig messages will be distriputed in the given topic
     * The get method is is the case not accecible!
     * Warning: The distribution is from Interrupt server, please use
     * Subscriber::putFromInterrupt(...)
     * do not supend (no semaphore, no yield, etc) in this method!
     */
    UDPIn(const long portNr, Topic<GenericMsgRef>* associatedTopic);

    void setAsync(Topic<GenericMsgRef>* associatedTopic);

    /** Opens a socket for UDP reception.
     * On success attribute 'initialised' is set true.
     * @param[in] portNr port number on localhost
     */
    void reopen(const long portNr);

    /**
     * Receives data from UDP socket. Writes data up to maxLen to userData.
     * @param[in,out] userData pointer to input buffer
     * @param[in] maxLen size of input buffer
     * @return length of message written to userData
     */
    long get(void* userData, int maxLen);

    /**
     * Receives data from UDP socket. Writes data up to maxLen to userData.
     * @param[in,out] userData pointer to input buffer
     * @param[in] maxLen size of input buffer
     * @param[out] ipaddr sender address
     * @return length of message written to userData
     */
    long get(void* userData, int maxLen, unsigned long *ipaddr);

    /**
     * Check for incoming data.
     * @return true - if data is available, false - otherwise
     */
    bool readyToGet();

    /**
     * Check whether port is active or not.
     * @return true - connected, false - not connected
     */
    bool isConnected();
};




///To send data to a single UPD port (socket) - abstraction layer.
class UDPOut {
    /// socket API, used internally for the HW Implementation
    UDPTransmitter udpTx;
public:

    /** Constructor for connection to remote host known by name.
         * WARNING: Negativ port number uses the abs port number and sends broadcast
     * @param portNr port number on remote host, negativ -> broadcast to this port
     * @param hostname IP-hostname of remote host
     */
    UDPOut(const long portNr, const char* hostname = "localhost");

    /** Constructor for connection to remote host known by IP-address.
     * @param _portNr port number on remote host
     * @param _ipAddr IP-address of remote host, e.g. format as hex 0x01020304,
     * where 01 = ip_part1, ...
     */
    UDPOut(const long _portNr, unsigned long _ipAddr); //MSul

    /** Constructor for connection to remote host known by IP-address.
     * @param _portNr port number on remote host
     * @param ip0, ip1, ip2, ip3 IP-address of remote host consisting of separate octets
     */
    UDPOut(const long _portNr, int ip0, int ip1, int ip2, int ip3);//MSul

    /** Send user data
     * @param[in] userData pointer to output data stream
     * @param[in] maxLen size of data block to be send, must be <= max allowed length
     * @return true - if sending was successful, false - otherwise
     */
    bool send(const void* userData, const int maxLen);

    /** Send user data to a given address
     * @param[in] userData pointer to output data stream
     * @param[in] maxLen size of data block to be send, must be <= max allowed length
     * @param[in] ipAddr new IP-address, where data is send to
     * @return true - if sending was successful, false - otherwise
     */
    bool sendTo(const void* userData, const int maxLen, unsigned long ipAddr);//MSul

    /**
    	 * Check whether port is active or not.
    	 * @return true - connected, false - not connected
    	 */
    bool isConnected();
};


class UDPInOut {
public:

    UDPIn udpIn;
    UDPOut udpOut;

    bool isBroadcastLink;

    UDPInOut(int broadcastPort);
    UDPInOut(int port,const char* hostname );
    UDPInOut(int localInputPort, int remoteOutputPort,const  char* hostname);
};


#ifndef NO_RODOS_NAMESPACE
}
#endif

