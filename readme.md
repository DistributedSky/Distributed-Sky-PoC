# Distributed-Sky
Distributed sky is a global traffic management system running on Ethereum world computer.

## Abstract
The project’s team sees the lack of universal air traffic management system as a major bottleneck for unlocking the potential of unmanned aerial systems (UASs). Even though there are [many ways](http://www.cnn.com/2013/11/03/business/meet-your-friendly-neighborhood-drones) “drones” can benefit humanity, applications are still very limited because the system that would allow millions of flights to be coordinated efficiently on the global scale did not exist before.
Distributed Sky is a framework for a global air traffic management system that is running on [a world computer](https://ethereum.gitbooks.io/frontier-guide/content/ethereum.html). It is not run in a data-center, but instead leverages a global network of computers to process and store identity, traffic and other crucial information.


## Distributed sky ROS prototype

This is ROS-based prototype implementation for [Distributed Sky project](http://www.distributedsky.com).
The prototype is based on [AIRA](http://aira.life) robonomics protocol.

Project is being developed, now it contains only two agents and can be used as an illustration of robonomics protocol
 usage.
 
### How to use?

It is recommended to use it as a part of [this prototype](https://github.com/XomakNet/DistributedSky_Docker), which
is a docker-compose based project, containing test UAV and ASP (in terms of Distributed Sky) containers, as well as 
parity and IPFS, required for them.

### What is included?

#### UAV
Distributed_sky_uav contains single node, which intended to be used on UAV's side as a middleware between navigation 
nodes of UAV (route planners) and Distributed Sky. This node expects route, which should be approved by Airspace Service
Provider and some params (service id, for instance), handles all the interaction with Distributed Sky and publishes the 
result on defined topic.

#### Airspace service provider
airspace_service_provider package contains single node, which is a STUB for ASP. It receives route confirmation requests
(as Asks) on the market, sends the Bid and executes the liability to confirm the route (currently, it confirms all 
routes).