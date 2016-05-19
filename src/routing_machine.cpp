#include "ros/ros.h"
#include "routing_machine/RoutingMachine.h"
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

using namespace std;

string apiCall(string website, string parameters);
string isolateShape(string input);
std::vector<float> decodePolyline(string encoded);
string formateParameters(float slat, float slng, float elat, float elng);

bool getRouting(routing_machine::RoutingMachine::Request  &req, routing_machine::RoutingMachine::Response &res)
{

	std::vector<float> coords;
	string apiReturn;

	apiReturn = apiCall("valhalla.mapzen.com",formateParameters(39.4783618733,-0.335404928529,39.48417030015832,-0.438079833984375));

	coords = decodePolyline(isolateShape(apiReturn));

	for (int i = 0; i < coords.size(); ++i)
	{
		res.coords.push_back(coords[i]);
	}
		
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "routing_machine");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("routing_machine", getRouting);
	ROS_INFO("Routing Machine ready !");
	ros::spin();

	return 0;
}


string apiCall(string website, string parameters) 
{
	//===> Variables declarations
	int sock;
	struct sockaddr_in client;
	int PORT = 80;
	string output;
	string realoutput;

	struct hostent * host = gethostbyname(website.c_str());

	if ( (host == NULL) || (host->h_addr == NULL) ) {
		cout << "Error retrieving DNS information." << endl;
		exit(1);
	}

	bzero(&client, sizeof(client));
	client.sin_family = AF_INET;
	client.sin_port = htons( PORT );
	memcpy(&client.sin_addr, host->h_addr, host->h_length);

	sock = socket(AF_INET, SOCK_STREAM, 0);

	if (sock < 0) {
		cout << "Error creating socket." << endl;
		exit(1);
	}

	if ( connect(sock, (struct sockaddr *)&client, sizeof(client)) < 0 ) {
		close(sock);
		cout << "Could not connect" << endl;
		exit(1);
	}

	stringstream ss;
	ss << "GET "<< parameters << " HTTP/1.0\r\n"
		 << "Host: " << website << "\r\n"
		 << "Accept: application/json\r\n"
		 << "Content-Type: application/x-www-form-urlencoded\r\n"
		 << "\r\n\r\n";
	string request = ss.str();

	if (send(sock, request.c_str(), request.length(), 0) != (int)request.length()) {
		cout << "Error sending request." << endl;
		exit(1);
	}


	char cur; 
	int i = 0;
	int j = 0;
	while ( read(sock, &cur, 1) > 0 ) {
		output += cur;
		i++;
	}

	return output;
}

string isolateShape(string input)
{
	std::size_t start = input.find("[{\"shape\":\"") + 11;  
	std::size_t end = input.find("\"", start+12);  
	return input.substr (start, end-start);
}

std::vector<float> decodePolyline(string encoded)
{
	//Adapted from : https://github.com/paulobarcelos/ofxGooglePolyline
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

		points.push_back(lat * (float)1e-6);
		points.push_back(lng * (float)1e-6);
	}

	return points;
}

string formateParameters(float slat, float slng, float elat, float elng)
{
	char output[500];
	sprintf(output,"/route?json={\"locations\":[{\"lat\":%f,\"lon\":-0.335404928529},{\"lat\":39.48417030015832,\"lon\":-0.438079833984375}],\"costing\":\"pedestrian\",\"directions_options\":{\"units\":\"miles\"}}&api_key=valhalla-RsYgicy",slat);
	return output;
}