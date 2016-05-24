#include "ros/ros.h"
#include "routing_machine/ParseWpts.h"
#include <iostream>
#include <ctype.h>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <string.h>
#include <vector>
#include <ros/console.h>

using namespace std;

// ===> Prototypes
string apiCall(string website, string parameters);
string isolateShape(string input);
int decodePolyline(string encoded, std::vector<double> *latitude, std::vector<double> *longitude);
string formateParameters(double slat, double slng, double elat, double elng);
bool getRouting(routing_machine::ParseWpts::Request  &req, routing_machine::ParseWpts::Response &res);

// ===> main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "routing_machine");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("routing_machine/get_wpts", getRouting);
	ROS_INFO("Routing Machine ready !");
	ros::spin();

	return 0;
}

// ===> getRouting : principal function
bool getRouting(routing_machine::ParseWpts::Request  &req, routing_machine::ParseWpts::Response &res)
{
	// Only if the client ask for waypoints
	if(req.get_wpts == true)
	{
		// Variables declaration
		std::vector<double> latitude;
		std::vector<double> longitude;
		
		float endLatitude;
		float endLongitude;
		float startLatitude;
		float startLongitude;
		bool goTo;
		
		string apiReturn;
		string shape;

		// First test : we check if the interface has been launched. If so, it should have set the parameters
		// Second test : we check if the user has provided a route by checking goTo's value.
		if (ros::param::get("/routing_machine/destination/goTo", goTo) && goTo == true)
		{
			// If the parameters goTo has been set, the 4 others should have been too. We get their values.
			ros::param::get("/routing_machine/start/latitude", startLatitude);
			ros::param::get("/routing_machine/start/longitude", startLongitude);
			ros::param::get("/routing_machine/destination/latitude", endLatitude);
			ros::param::get("/routing_machine/destination/longitude", endLongitude);

			ROS_INFO("Start lat : %f lon : %f - End : lat : %f lon : %f", startLatitude, startLongitude, endLatitude, endLongitude);


			// Start message
			ROS_INFO("Routing Machine : WIP !");

			// First, call the API
			apiReturn = apiCall("valhalla.mapzen.com",formateParameters(startLatitude,startLongitude,endLatitude,endLongitude));

			// Test the return. If we have an error, it means that the API is disconnected or the car isn't connected
			// to the internet, so no need to go further.
			if(apiReturn == "error")
			{
				ROS_FATAL("Routing Machine : Impossible to connect to routing API. Routing failed.");
				res.success = false;
			}
			else
			{
				// Data has arrived! Next, extract the shape.
				ROS_INFO("Routing Machine : Got information from API, processing.");
				shape = isolateShape(apiReturn);

				// Here, two possibilities. 
				// - The export is a JSON object, containing the shape, so we use string function manipulation
				//   to extract it, and after we'll be able to decode it with the polyline algorith.
				// - The export is not parsable. That's mean that instead of a nice JSON object, we got an error
				//   coming from the API. It could be pretty much anything, so we display it for debugging purpose.
				//   For example, it could be a coordinate that doesn't exists.

				if(shape == "error")
				{
					ROS_FATAL("Routing Machine : Unable to parse data");
					ROS_FATAL("Check API return : \n%s",apiReturn.c_str());
					res.success = false;
				}
				else
				{
					ROS_INFO("Routing Machine : Parsed correctly");
					// Now we can decode the shape, using the google algorithm.
					// More info here : https://developers.google.com/maps/documentation/utilities/polylinealgorithm
					// We give the function two pointers, so we get our vectors directly written.
					decodePolyline(shape, &latitude, &longitude);

					// And now we make a nice output.
					res.latitude = latitude;
					res.longitude = longitude;
					res.num_wpts = latitude.size();
					res.success = true;
				}
			}

			ROS_INFO("Routing Machine : Finish! %ld waypoints.", latitude.size());
			return true;
		}
		else
		{
			// The interface has not been launched or the user hasn't provided any route
			ROS_INFO("No destination set.");
			res.success = false;
			return true;
		}
	}
	else
	{
		res.success = false;
		return true;
	}
}

// ===> apiCall : open a socket on the website asked, with the parameters, and return the whole result.
string apiCall(string website, string parameters) 
{
	// Adapted from : http://www.mzan.com/article/17685466-http-request-by-sockets-in-c.shtml

	// Variables declarations
	int sock;
	struct sockaddr_in client;
	int PORT = 80;
	string output;
	string realoutput;
	struct hostent * host = gethostbyname(website.c_str());

	// Program starts
	ROS_INFO("Routing Machine : Contact API at %s%s",website.c_str(), parameters.c_str());

	if ( (host == NULL) || (host->h_addr == NULL) ) {
		ROS_FATAL("Routing Machine : Error contacting DNS. Are you connected to the internet?");
		return "error";
	}

	bzero(&client, sizeof(client));
	client.sin_family = AF_INET;
	client.sin_port = htons( PORT );
	memcpy(&client.sin_addr, host->h_addr, host->h_length);

	sock = socket(AF_INET, SOCK_STREAM, 0);

	if (sock < 0) {
		ROS_FATAL("Routing Machine : Error creating socket.");
		return "error";
	}

	if ( connect(sock, (struct sockaddr *)&client, sizeof(client)) < 0 ) {
		close(sock);
		ROS_FATAL("Routing Machine : Could not connect to API.");
		return "error";
	}

	stringstream ss;
	ss << "GET "<< parameters << " HTTP/1.0\r\n"
		 << "Host: " << website << "\r\n"
		 << "Accept: application/json\r\n"
		 << "Content-Type: application/x-www-form-urlencoded\r\n"
		 << "\r\n\r\n";
	string request = ss.str();

	if (send(sock, request.c_str(), request.length(), 0) != (int)request.length()) {
		ROS_FATAL("Routing Machine : Error sending request.");
		return "error";
	}

	// Copy buffer to output
	char cur; 
	int i = 0;
	int j = 0;
	while ( read(sock, &cur, 1) > 0 ) {
		output += cur;
		i++;
	}

	return output;
}

// ===> isolateShape : make a little string manipulation to separate the shape from the whole API return.
string isolateShape(string input)
{
	std::size_t start = input.find("[{\"shape\":\"") + 11; 
	
	if(start == string::npos + 11) 
	{
		return "error";
	}
	else
	{	
		std::size_t end = input.find("\"", start+12);  
		return input.substr (start, end-start);
	}
}

// ===> decodePolyline : decode the shape, which is encoded with a google algorithm
int decodePolyline(string encoded, std::vector<double> *latitude, std::vector<double> *longitude)
{
	//Adapted from : https://github.com/paulobarcelos/ofxGooglePolyline
	// More info here : https://developers.google.com/maps/documentation/utilities/polylinealgorithm
	std::vector<float> points;
	int len = encoded.length();
	int index = 0;
	float lat = 0;
	float lng = 0;

	while (index < len) {
		char b;
		int shift = 0;
		int result = 0;
		do {
			b = encoded.at(index++) - 63;
			result |= (b & 0x1f) << shift;
			shift += 5;
		} while (b >= 0x20);
		float dlat = ((result & 1) ? ~(result >> 1) : (result >> 1));
		lat += dlat;

		shift = 0;
		result = 0;
		do {
			b = encoded.at(index++) - 63;
			result |= (b & 0x1f) << shift;
			shift += 5;
		} while (b >= 0x20);
		float dlng = ((result & 1) ? ~(result >> 1) : (result >> 1));
		lng += dlng;

		latitude -> push_back(lat * (float)1e-6);
		longitude -> push_back(lng * (float)1e-6);
	}

	return 0;
}

// ===> formateParameters : using the start and finish coordinates, it formates the string with the params in it
string formateParameters(double slat, double slng, double elat, double elng)
{
	char output[500];
	sprintf(output,"/route?json={\"locations\":[{\"lat\":%f,\"lon\":%f},{\"lat\":%f,\"lon\":%f}],\"costing\":\"pedestrian\",\"directions_options\":{\"units\":\"miles\"}}&api_key=valhalla-RsYgicy",slat,slng,elat,elng);
	return output;
}
