# Routing-ROS-Module

This module is actually a ROS service, which returns the points of a polyline in the long/lat coordinates systems, from a starting point to an end point. It's using the Mapzen API to calculate the quickest path to do so.

## How does it work ?

Using CURL, the service calls Mapzen API providing coordinates of the start and the end of the route, and the API returns directions, encoded in the spline format. We decode that spline, and put it in the output, it's as simple as that.

## Installation

`git clone` in your `catkin_ws/src` and you're ready to go! 
 
## Need to display it on a map ?
 
 Check [my other repository](https://github.com/sylvainar/ROS-OSM-map-integration)!

 
