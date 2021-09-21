/* 
Copyright © 2012 NaturalPoint Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#ifndef MOTIVECLIENT_H
#define MOTIVECLIENT_H

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <limits.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <math.h>
#include <iostream>

#include "RigidBody.h"


#define MAX_NAMELENGTH              256

// NATNET message ids
#define NAT_CONNECT                 0 
#define NAT_SERVERINFO              1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999

#define MAX_PACKETSIZE				100000	// max size of packet (actual packet size is dynamic)

// This should match the multicast address listed in Motive's streaming settings.
#define MULTICAST_ADDRESS	"239.255.42.99"    

// Motive Command channel
#define PORT_COMMAND            1510

// Motive Data channel
#define PORT_DATA  		1511         


// sender
typedef struct
{
    char szName[MAX_NAMELENGTH];            // sending app's name
    unsigned char Version[4];               // sending app's version [major.minor.build.revision]
    unsigned char NatNetVersion[4];         // sending app's NatNet version [major.minor.build.revision]

} sSender;

typedef struct
{
    unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
    unsigned short nDataBytes;              // Num bytes in payload
    union
    {
        unsigned char  cData[MAX_PACKETSIZE];
        char           szData[MAX_PACKETSIZE];
        unsigned long  lData[MAX_PACKETSIZE/4];
        float          fData[MAX_PACKETSIZE/4];
        sSender        Sender;
    } Data;                                 // Payload incoming from NatNet Server

} sPacket;

// -------------------------- non-member functions -------------------------- 
int getLocalIP(char** IP) // Output IP string and takes as input address of char array
{

	char host[256];
	struct hostent *host_entry;
	int hostname;
	hostname=gethostname(host,sizeof(host));  //get host name

	

	if(hostname<0)
	{
		perror("gethostname");
		return -1;
	}

	host_entry = gethostbyname(host); //get host info

	if(host_entry == NULL)
	{
		perror("gethostbyname");
		return -1;	
	}

	*IP = inet_ntoa(*((struct in_addr*) host_entry->h_addr_list[0]));	//get IP string
	
	return 0;

}


bool DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe)
{
	bool bValid = true;

	*hour = (inTimecode>>24)&255;
	*minute = (inTimecode>>16)&255;
	*second = (inTimecode>>8)&255;
	*frame = inTimecode&255;
	*subframe = inTimecodeSubframe;

	return bValid;
}

// Takes timecode and assigns it to a string
bool TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize)
{
	bool bValid;
	int hour, minute, second, frame, subframe;
	bValid = DecodeTimecode(inTimecode, inTimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
//	sprintf_s originally
	snprintf(Buffer,BufferSize,"%2d:%2d:%2d:%2d.%d",hour, minute, second, frame, subframe);
	for(unsigned int i=0; i<strlen(Buffer); i++)
		if(Buffer[i]==' ')
			Buffer[i]='0';

	return bValid;
}

void DecodeMarkerID(int sourceID, int* pOutEntityID, int* pOutMemberID)
{
    if (pOutEntityID)
        *pOutEntityID = sourceID >> 16;

    if (pOutMemberID)
        *pOutMemberID = sourceID & 0x0000ffff;
}



int CreateCommandSocket(unsigned long IP_Address, unsigned short uPort)
{
    struct sockaddr_in my_addr;     
    static unsigned long ivalue;
    static unsigned long bFlag;
    int nlengthofsztemp = 64;  
    int sockfd;

    // Create a blocking, datagram socket
    if ((sockfd=socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
	printf("Failed 1\n");
        return -1;
    }

    // bind socket
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(uPort);
    my_addr.sin_addr.s_addr = IP_Address;
    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) < 0)
    {
	printf("Failed 2, Address: %d\n",(unsigned int)my_addr.sin_addr.s_addr);
        close(sockfd);
        return -1;
    }

    // set to broadcast mode
    ivalue = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (char *)&ivalue, sizeof(ivalue)) < 0)
    {
	printf("Failed 3\n");
        close(sockfd);
        return -1;
    }

    return sockfd;
}


class MotiveClient
{

private:


int CommandSocket;
int DataSocket;
char isInitialized;
sockaddr_in HostAddr;  // server address for sending commands

struct hostent *server;
struct hostent *client;

struct ip_mreq Mreq;

struct sockaddr_in serv_addr, cli_addr;// server and client addresses

int NatNetVersion[4] = {0,0,0,0};
int ServerVersion[4] = {0,0,0,0};

int gCommandResponse = 0;
int gCommandResponseSize = 0;
unsigned char gCommandResponseString[PATH_MAX];
int gCommandResponseCode = 0;

std::vector <RigidBody> AllBodies;

public:

MotiveClient();
int InitClient(char* MulticastAddress, char* ClientAddress, char* ServerAddress, int DataPort, int CommandPort);// Initialize client and connect to data and command ports
int RenewMembership();				// renew membership to keep receiving data on the multicast address must be called every 30 seconds (or before router timeout) 
RigidBody* GetRigidBodyData(int ID);		// Returns a pointer to rigid body object with specific ID
int GetRigidBodyIndex(int ID);			// Return the index of the rigid body in list (AllBodies), if rigid body with ID: (ID) is in the list, otherwise returns -1
std::vector <RigidBody>* GetAllRigidBodies();	// returns a pointer to vector of rigid body data
int ReadNewData();				// Listens to the data port to update rigid body data
int SendCommand(char* szCommand);		

};



#endif //MOTIVECLIENT_H
