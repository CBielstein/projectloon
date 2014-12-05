#ifndef BALLOON_H
#define BALLOON_H

#include "loonnode.h"

// Represents gateways in the network
namespace Loon
{
    class Balloon : public LoonNode
    {
        public:
            Balloon() : LoonNode() { connected = false; }
            Balloon(ns3::Ptr<ns3::Node> _node) : LoonNode(_node) { connected = false; }
            bool IsGateway() override { return false; }
    };
}

#endif
