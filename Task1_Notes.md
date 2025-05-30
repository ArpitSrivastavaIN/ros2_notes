# What DDS and peer-to-peer communication are??
## What is DDS:
- DDS standing for **data distribution service** is a middle-ware that carries out the task of **data exchange between nodes** in a ros package
- DDS provides a publish subscribe transport using the "**Interface Description Language**"
-  ### What is the IDL?? :
	-  The IDL is used to define, messages, services and actions which is used by the DDS. It gives structure to the data that needs to be sent which is then used by the DDS.
	
	- `.msg`, `.srv` and `.action` files get translated into IDL during the build process. Then the IDL is used to generate the serialization/deserialization code to be further sent by the DDS.
	- IDL can generate code for various DDS implementations and is independent of the middleware type being used when defining message/service types.

- DDS is a type of an end to end middleware which means there is *less code to maintain* and the nodes making it easier for certain use cases in others the feature may not be flexible and others means must be approached.

- DDS provides discovery, message definition, message serialization, and publish-subscribe transport. Therefore, DDS would provide discovery, publish-subscribe transport, and at least the underlying message serialization for ROS.
- ### Different Implementations of DDS in ROS:
	- 1.**FastDDS**: Is the default DDS vendor in ROS2. Designed for speed and efficiency. Includes discovery server support and DDS security.

	- 2.**CycloneDDS**: Is high performance and focuses on real time capabilities. Well suited for edge and embedded systems due to its low latency and small footprint.
	
	- 3.**RTI ConnectDDS**: Highly advanced and feature rich with usage in aerospace and defence sector. Advanced QoS tuning options are available. Is the most advanced of all the implementations mentioned above.
- **How does DDS work in ROS2**:
	1. Discovery: When a node is running it broadcasts it's presence and declares the topics it will publish/subscribe form. DDS uses peer-to-peer discovery protocols often over UDP multicast to detect and connect the nodes with identical topics. Packets are sent via UDP(RTPS) that advertise who they are the topics they/publish and subscribe to and their QoS settings.

	2. Once topic name, message type are matched a communication channel is established thus creating end to end connection between the two nodes.Matching occurs automatically peer to peer.
	3. Transfer of Data: Publishers then send data using DDS serialisation and Subscribers receive that data.  

## What is UDP:
- **UDPROS** is a transport layer for ROS messages and services which uses standard UDP datagram packets to transport serialized message data. Is most useful when latency is more important than reliable transport.

-  **What is RTPS**: RTPS (Real-Time Publish Subscribe) runs on top of UDP, but adds its own logic for reliability, delivery, fragmentation, and discovery. This makes UDP suitable for real-time systems like ROS 2 without giving up key guarantees. UDP sends individual packets called datagram with no guarantee for delivery has no reliability or ordering to enforce reliability  RTPS is needed.

-Message Structure: RTPS messages are framed with headers and sub-messages that define their purpose thus each UDP diagram carries the data, heartbeat (to track delivery) acknack (for feedback/ retransmission) which makes UDP packets become part of a larger, structured messaging system

-  How this works? ROS 2 applications don't deal with UDP or RTPS directly but when nodes are run ROS 2 talks to the RMW (ROS middle-ware) talks to DDS which further sends RTPS messages using UDP thus providing the performance of UDP with the structure and discipline of RTPS.

### Peer to Peer Discovery:
- ROS 2 is built on the DDS middleware which supports true peer to peer discovery. It eliminates the need for a mediator.
- In P2P networks, resources can be shared among all computers in the network without requiring a separate server computer. Each computer acts as both a client and a server, enabling them to exchange information directly with each other.
- This model eliminates a single point of failure by removing the bottle-necks and allowing the system to scale up and down dynamically according to the number and availability of nodes
- P2P networks face certain challenges such as discovery, routing and consistency making it hard for nodes to establish connections in a large and dynamic network.

## Why ROS2 dropped ROS master:

- ROS2 dropped the master server to enhance the **fault tolerance and simplify the system architecture**. In ROS2 the master server acted as a central mediator (post office) communication between nodes. This often caused the whole setup to fail if the master server didn't work properly or failed to load. So in ROS2 peer to peer communication was introduced allowing Nodes to communicate independently, reducing the risk of orphaned nodes and improving the overall robustness of the system.

- ROS2 was built from the ground up with certain design requirements  and DDS was chosen as the network protocol making all security concerns disappear. DDS provided the **security guarantee** that was not present in ROS1 opening ROS platform up for commercial and defence usage. Allowing ROS systems to access unsafe networks such as the internet.
 
- ROS1 has a **single point of failure** whereas ROS2 doesn't. In ROS1 the ROS master provides naming and registration services for ROS nodes. If a package has two nodes node1 and node2, both nodes introduce themselves to the ROS master and exchange information about the messages and services they will publish and subscribe. ROS master then assigns a designated port and a direct connection is established between the two nodes. The problem here is that is ROS master dies even though node1 and node2 will still be communication with each other if a new node pops up and tries to communicate with the existing nodes, it won't be able to do so resulting in what is called an **orphaned node**.
