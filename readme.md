# Distributed sky ROS prototype

This is ROS-based prototype implementation for [Distributed Sky project](http://www.distributedsky.com).
The prototype is based on [AIRA](http://aira.life) robonomics protocol.

Project is being developed, now it contains only two agents and can be used as an illustration of robonomics protocol
 usage.
 
# How to use?

It is recommended to use it as a part of [this prototype](https://github.com/XomakNet/DistributedSky_Docker), which
is a docker-compose based project, containing test UAV and ASP (in terms of Distributed Sky) containers, as well as 
parity and IPFS, required for them.

# What is included?

## UAV
Distributed_sky_uav contains single node, which intended to be used on UAV's side as a middleware between navigation 
nodes of UAV (route planners) and Distributed Sky. This node expects route, which should be approved by Airspace Service
Provider and some params (service id, for instance), handles all the interaction with Distributed Sky and publishes the 
result on defined topic.

## Airspace service provider
airspace_service_provider package contains single node, which is a STUB for ASP. It receives route confirmation requests
(as Asks) on the market, sends the Bid and executes the liability to confirm the route (currently, it confirms all 
routes).