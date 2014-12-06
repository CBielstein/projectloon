#ifndef CLIENT_H
#define CLIENT_H

#include "loonnode.h"

// Represents gateways in the network
namespace Loon
{
    class Client : public LoonNode
    {
        public:
            Client() : LoonNode() { connected = false; }
            Client(ns3::Ptr<ns3::Node> _node) : LoonNode(_node) { connected = false; }
            LoonNodeType GetType() const override { return LoonNodeType::CLIENT; }
    };
}

#endif
