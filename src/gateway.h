#ifndef GATEWAY_H
#define GATEWAY_H

#include "loonnode.h"

// Represents gateways in the network
namespace Loon
{
    class Gateway : public LoonNode
    {
        public:
            Gateway() : LoonNode() { connected = true; }
            Gateway(ns3::Ptr<ns3::Node> _node) : LoonNode(_node) { connected = true; }
            LoonNodeType GetType() const override { return LoonNodeType::GATEWAY; }

            // Gateways don't move, so disable setting velocity.
            bool SetVelocity(const ns3::Vector3D& velocity) { return false; }
    };
}

#endif
