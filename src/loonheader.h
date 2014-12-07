/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/header.h"
#include "ns3/core-module.h"
#include <iostream>

using namespace ns3;

/* A sample Header implementation
 */
class LoonHeader : public Header 
{
public:

  LoonHeader ();
  virtual ~LoonHeader ();

  void SetDest (uint32_t data);
  uint32_t GetDest (void) const;

  void SetFinalDest (uint32_t data);
  uint32_t GetFinalDest (void) const;

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual uint32_t GetSerializedSize (void) const;
  virtual bool IsRoutingModeGreedy (void) const;
  virtual void SetRoutingModeGreedy (bool m);
  virtual void SetStartPerimeterRoutingDistance(double distance);
  virtual double GetStartPerimeterRoutingDistance (void) const;
  virtual void SetStartPerimeterRoutingLocation(ns3::Vector3D startLocation);
  virtual ns3::Vector3D GetStartPerimeterRoutingLocation();
private:
  uint32_t dest;
  double startPerimeterRoutingDistance;
  bool greedy = true;
  ns3::Vector3D startPerimeterRoutingLocation;
  uint32_t finalDest;
};
