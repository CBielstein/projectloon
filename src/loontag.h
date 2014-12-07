/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/tag.h"
#include "ns3/core-module.h"
#include <iostream>

using namespace ns3;

class LoonTag : public Tag 
{
public:

  LoonTag ();

  void IncrementHopCount ();
  uint16_t GetHopCount ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual void Serialize (TagBuffer start) const;
  virtual void Deserialize (TagBuffer start);
  virtual uint32_t GetSerializedSize (void) const;

private:
  uint16_t hopCount = 1;
};
