/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/header.h"
#include "loontag.h"
#include <iostream>

using namespace ns3;

LoonTag::LoonTag ()
{
}

TypeId 
LoonTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LoonTag")
    .SetParent<Header> ()
    .AddConstructor<LoonTag> ()
    ;
  return tid;
}
TypeId 
LoonTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void 
LoonTag::Print (std::ostream &os) const
{
  // This method is invoked by the packet printing
  // routines to print the content of my header.
  os << "data=" << hopCount;
}
uint32_t
LoonTag::GetSerializedSize (void) const
{
  // we reserve 2 bytes for our header.
  return 2;
}
void
LoonTag::Serialize (TagBuffer start) const
{
  // we can serialize four bytes at the start of the buffer.
  // we write them in network byte order.
  start.WriteU16 (hopCount);
}
void
LoonTag::Deserialize (TagBuffer start)
{
  // we can deserialize two bytes from the start of the buffer.
  // we read them in network byte order and store them
  // in host byte order.
  hopCount = start.ReadU16 ();
}

void LoonTag::IncrementHopCount() {
  hopCount++;
}

uint16_t LoonTag::GetHopCount() {
  return hopCount;
}
