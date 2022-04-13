# MAVLinkRandomWaypointMobility
Extends: [MAVLinkMobilityBase](/Modules/MAVLinkMobilityBase/)
## Description

 This module is analogous to INET's RandomWaypointMobility. The vehicle will follow random waypoints within the environment constraints
 and wait there for a set amount before traveling to the next one.

## Parameters

| Name | Type | Unit | Default value | Description |
| ---- | ---- | ---- | ------------- | ----------- |
| speed | double | mps | 20mps |  Speed at which the vehicle should travel |
| waitTime | double | s | 0s |  Time the vehicle will wait before traveling to the next waypoint |
| waypointRadius | double | m | 30m |  Radius of the waypoint. Vehicles within this radius will be considered to have reached the waypoint |
