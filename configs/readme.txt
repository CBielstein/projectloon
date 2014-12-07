How to write config files for our tests:

Specify the nodes in your network. G stands for Gateway, B stands for Balloon, and C stands for client.
Each should be listed on its own line. In addition to the type of node, please choose the location in the form of
3 doubles. Separate each coordinate with a space. It is recommended that you list your nodes in the order of all
gateways, all balloons, and then all clients. Not doing so will not break anything, but it makes reading the results
less confusing.

An example of listing nodes would be:
G 0.0 0.0 0.0
B -30066.0 0.0 20000.0
B -40066.0 0.0 20000.0
B -80066.0 0.0 20000.0
B -80066.0 10000.0 0.0
C -60000.0 30000.0 0.0
This lists 1 gateway, 4 balloons, and 1 client at the various locations.

The S label stands for Send. Send has 4 parameters - the id of the sender, the final destination, the number of packets
to send, and the interval in which to send the packets. Each of the parameters should be separated by a space. The id
of the sender is the order in which the nodes appear (in respect to the order gateways, balloons, and then clients).
So, for example, the line "S 5 0 10 1" states that node 5 should send 10 packets to node 0 ever 1 second.

Finally, end your config file with an X. The X marks the end of the config file.
