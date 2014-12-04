#ifndef GATEWAY_H
#define GATEWAY_H

#include "balloon.h"

// TODO if there's enough time before the project deadline, fix the inheritance model to
// extend some base class instead of Balloon
// Class that extends balloon to override a few functions necessary for creating a gateway
class Gateway : public Balloon
{
    public:
        Gateway() : Balloon() {}
        Gateway(ns3::Ptr<ns3::Node> _node) : Balloon(_node) {}

    private:
        bool IsGateway() { return true; }
};

#endif
