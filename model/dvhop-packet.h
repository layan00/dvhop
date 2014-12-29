#ifndef DVHOP_PACKET_H
#define DVHOP_PACKET_H

#include <iostream>
#include "ns3/header.h"
#include "ns3/enum.h"
#include "ns3/ipv4-address.h"


namespace ns3
{
  namespace dvhop
  {
    /*
    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |                            X Position                         |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |                            Y Position                         |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |           Hops                |       Sequence number         |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |                        Beacon IP address                      |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

    */
    class FloodingHeader: public Header
    {
    public:

      FloodingHeader();
      FloodingHeader(uint32_t xPos, uint32_t yPos, uint16_t seqNo, uint16_t hopCount, Ipv4Address beacon);

      //Serializing and deserializing
      //{
      static TypeId    GetTypeId (void);
      TypeId           GetInstanceTypeId () const;
      virtual void     Serialize (Buffer::Iterator start) const;
      virtual uint32_t Deserialize (Buffer::Iterator start);
      virtual uint32_t GetSerializedSize () const;
      virtual void     Print (std::ostream &os) const;
      //}

      //Getters and setters
      void SetHopCount(uint16_t count)     { m_hopCount = count; }
      void SetXPosition(float pos)         { m_xPos = pos;   }
      void SetYPosition(float pos)         { m_yPos = pos;   }
      void SetSequenceNumber(uint16_t sn)  { m_seqNo = sn;   }
      void SetBeaconAddress(Ipv4Address a) { m_beaconId = a; }

      float    GetXPosition()        {   return m_xPos;     }
      float    GetYPosition()        {   return m_yPos;     }
      uint16_t GetHopCount()         {   return m_hopCount; }
      uint16_t GetSequenceNumber()   {   return m_seqNo;    }
      Ipv4Address GetBeaconAddress() {   return m_beaconId; }


    private:
      float        m_xPos;
      float        m_yPos;
      uint16_t     m_seqNo;
      uint16_t     m_hopCount;
      Ipv4Address  m_beaconId;
    };

    std::ostream & operator<< (std::ostream & os, FloodingHeader const &);


  }
}

#endif // DVHOP_PACKET_H
