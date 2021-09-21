#include "MotiveClient.h"
#include "RigidBody.h"

#define MULTICAST_ADDRESS "239.255.42.99"
#define DATA_PORT 1511 // data socket port for optitrack
#define COMMAND_PORT 1510 // command socket port for optitrack
#define MY_ID 1 // ID of rigid body of interest
// Initialize client and connect to data and command ports

MotiveClient::MotiveClient()
{

isInitialized = 0; // indicates that client is not initialized yet

}


int MotiveClient::InitClient(char* MulticastAddress, char* ClientAddress, char* ServerAddress, int DataPort, int CommandPort)
{


    	int optval = 0x100000;	// required buffer size of 1 MB
    	int optval_size = 4;	// size of optval variable
    	int retval;

	memset(&serv_addr, 0, sizeof(serv_addr)); 	// set server address to zero
    	memset(&cli_addr, 0, sizeof(cli_addr));		// set client address to zero

	server = gethostbyname(ServerAddress);
	if(server != NULL)
		memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	else
	{
		printf("Error: Invalid Server Address\n");
		return -1;
	}

	client = gethostbyname(ClientAddress);
	if(client != NULL)
		memcpy(&cli_addr.sin_addr.s_addr, client->h_addr, client->h_length);
	else
	{
		printf("Error: Invalid Client Address\n");
		return -1;
	}


	// --------------------------------- create command socket -------------------------------------

	CommandSocket = CreateCommandSocket(cli_addr.sin_addr.s_addr, 0); // Socket used to receive command replies. Use wildcard port 0

	if(CommandSocket == -1)
    	{
        	// error 
		printf("Error: Command Socket Creation Failed\n");
		return -1;
    	}
    	else
    	{
        	// [optional] set to non-blocking
        	// set buffer
        	setsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
        	getsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, (unsigned int *)&optval_size);
        	if (optval != 0x100000)
        	{
        		// err - actual size...
			printf("Error setting command socket buffer size to 1 MB \nCurrent size: %d Bytes\n",optval);
        	}

        	// startup our "Command Listener" thread here (if any)
	
     	}



	// --------------------------------- create data socket -------------------------------------


	// create a "Data" socket
    	DataSocket = socket(AF_INET, SOCK_DGRAM, 0);

    	// allow multiple clients on same machine to use address/port
    	int value = 1;
    	retval = setsockopt(DataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value)); // reuse address option
    	
	if (retval < 0)
    	{
        	close(DataSocket);
        	return -1;
    	}
	
	
	struct sockaddr_in DataSocketAddr;
	memset(&DataSocketAddr, 0, sizeof(DataSocketAddr));
    	DataSocketAddr.sin_family = AF_INET;
    	DataSocketAddr.sin_port = htons(DataPort);
    	DataSocketAddr.sin_addr.s_addr = inet_addr(MulticastAddress);
    	

	// make socket non-blocking
	int flags = fcntl(DataSocket, F_GETFL, 0);
	fcntl(DataSocket, F_SETFL, flags | O_NONBLOCK);
	// ----------------------------------------------

	if (bind(DataSocket, (struct sockaddr *)&DataSocketAddr, sizeof(struct sockaddr)) < 0)
    	{
		printf("Error: Data socket bind failed\n");
        	return -1;
    	}


	// join multicast group
	Mreq.imr_multiaddr.s_addr = inet_addr(MulticastAddress);
	Mreq.imr_interface = cli_addr.sin_addr;

	retval = setsockopt(DataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&Mreq, sizeof(Mreq));
 	if (retval < 0)
    	{
        	printf("Error: Multicast group join failed \n");
        	return -1;
    	}

	// create a 1MB buffer for data socket
    	setsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
    	getsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, (unsigned int *)&optval_size);
    	if (optval != 0x100000)
    	{
    		printf("Error setting data socket buffer size to 1 MB \nCurrent size: %d Bytes\n",optval);
    	}
	

	// Server address for commands

	memset(&HostAddr, 0, sizeof(HostAddr));
    	HostAddr.sin_family = AF_INET;        
    	HostAddr.sin_port = htons(CommandPort); 
    	HostAddr.sin_addr = serv_addr.sin_addr;

	// send initial connect request
    	sPacket PacketOut;
    	PacketOut.iMessage = NAT_CONNECT;
    	PacketOut.nDataBytes = 0;
    	int nTries = 3;
	int iRet;
    	while (nTries--)
    	{
        	iRet = sendto(CommandSocket, (char *)&PacketOut, 4 + PacketOut.nDataBytes, 0, (sockaddr *)&HostAddr, sizeof(HostAddr));
		printf("Connect request reply: %d\n",iRet);
        	if(iRet != -1)
            		break;
    	}
	if(iRet < 0)
	{
		printf("Error:Failed to send connect request to server.\n");
		return -1;
	}
	

	printf("Motive Client started\n\n");
	isInitialized=1;
	
}

/**********************************************************************
** RenewMembership:
**	Renews membership to the multicast group Must
**	be called every 1 minute (or before multicast 
**	membership timeout according to used router)
**********************************************************************/
int MotiveClient::RenewMembership()
{

	int retval;

	retval = setsockopt(DataSocket, IPPROTO_IP, IP_DROP_MEMBERSHIP,(char *)&Mreq, sizeof(Mreq));
	if(retval < 0 )
		printf("Error Dropping Membership\n");

   	retval = setsockopt(DataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&Mreq, sizeof(Mreq));
	if(retval < 0 )
		printf("Error Adding Membership\n");

}


/**********************************************************************
** GetAllRigidBodies:
**	returns a pointer to the list of rigid bodies
**********************************************************************/
std::vector <RigidBody>* MotiveClient::GetAllRigidBodies()
{
	return &AllBodies;
}


/**********************************************************************
** GetRigidBodyIndex:
**	returns the index of the rigid body with ID (ID)
**	if there is no rigid body with ID (ID) returns -1
**********************************************************************/
int MotiveClient::GetRigidBodyIndex(int ID)
{
	int i;
	for(i=0;i<AllBodies.size();i++)
	{
		if(AllBodies[i].getID()==ID)
			return i;	// found index of rigid body with ID
	}
	
	return -1; // ID not found
}

/**********************************************************************
** GetRigidBodyData:
**	returns the index of the rigid body with ID (ID)
**	if there is no rigid body with ID (ID) returns -1
**********************************************************************/
RigidBody* MotiveClient::GetRigidBodyData(int ID)
{
	int i;

	for (i=0;i<AllBodies.size();i++)
		if(AllBodies[i].getID()==ID)
			return &AllBodies[i];
	return NULL;
}


/**********************************************************************
** ReadNewData:
**	Reads Data and Unpacks it into the rigid body list AllBodies
**********************************************************************/
int MotiveClient::ReadNewData()		// Listens to the data port to update rigid body data in AllBodies list
{

/*
    char  pData[20000];
    int addr_len = sizeof(struct sockaddr);
    sockaddr_in TheirAddress;
   
        // Check DataSocket for new datagram sent on the network (from anyone including ourselves)
        int nDataBytesReceived = recvfrom(DataSocket, pData, sizeof(pData), 0, (sockaddr *)&TheirAddress,(unsigned int *) &addr_len);
        

	// Once we have bytes recieved Unpack organizes all the data
	if(nDataBytesReceived <= 0)
	{
		printf("Data bytes received: %d\n",nDataBytesReceived);
		return nDataBytesReceived; // no data received
	}
*/

    	char  pData[20000];
	char  tempData[20000];
    	int addr_len = sizeof(struct sockaddr);
    	sockaddr_in TheirAddress;
	int dataReceivedFlag=0; 
        
	// Check DataSocket for new datagram sent on the network (from anyone including ourselves)

	int nDataBytesReceived=0;
	
	// keep reading data until buffer is empty

	do{	

        	nDataBytesReceived = recvfrom(DataSocket, tempData, sizeof(tempData), 0, (sockaddr *)&TheirAddress,(unsigned int *) &addr_len);
        

		if(nDataBytesReceived>0)
		{
			dataReceivedFlag = 1;
			memcpy(pData,tempData,sizeof(pData)); // copy new data to pData
		}

	}while(nDataBytesReceived > 0);

	// if no data is received return

	if(dataReceivedFlag == 0)
	{

//		printf("Data bytes received: %d\n",nDataBytesReceived);
		return nDataBytesReceived; // no data received		

	}

	//---------------------- start unpacking --------------------------

    int major = NatNetVersion[0];
    int minor = NatNetVersion[1];

    char *ptr = pData;

//    printf("Begin Packet\n-------\n");

    // First 2 Bytes is message ID
    int MessageID = 0;
    memcpy(&MessageID, ptr, 2); ptr += 2;
//    printf("Message ID : %d\n", MessageID);

    // Second 2 Bytes is the size of the packet
    int nBytes = 0;
    memcpy(&nBytes, ptr, 2); ptr += 2;
//    printf("Byte count : %d\n", nBytes);
	
    if(MessageID == 7)      // FRAME OF MOCAP DATA packet
    {
        // Next 4 Bytes is the frame number
        int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
//        printf("Frame # : %d\n", frameNumber);
    	
	    // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
        int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
//        printf("Marker Set Count : %d\n", nMarkerSets);

        // Loop through number of marker sets and get name and data
        for (int i=0; i < nMarkerSets; i++)
        {    
            // Markerset name
            char szName[256];
            strcpy(szName, ptr);
            int nDataBytes = (int) strlen(szName) + 1;
            ptr += nDataBytes;
//            printf("Model Name: %s\n", szName);

        	// marker data
            int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
//            printf("Marker Count : %d\n", nMarkers);

            for(int j=0; j < nMarkers; j++)
            {
                float x = 0; memcpy(&x, ptr, 4); ptr += 4;
                float y = 0; memcpy(&y, ptr, 4); ptr += 4;
                float z = 0; memcpy(&z, ptr, 4); ptr += 4;
//                printf("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n",j,x,y,z);
            }
        }

	    // Loop through unlabeled markers
        int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
		// OtherMarker list is Deprecated
        //printf("Unidentified Marker Count : %d\n", nOtherMarkers);
        for(int j=0; j < nOtherMarkers; j++)
        {
            float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
            float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
            float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
            
			// Deprecated
			//printf("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n",j,x,y,z);
        }
        
        // Loop through rigidbodies
        int nRigidBodies = 0;
	int RBIndex;
        memcpy(&nRigidBodies, ptr, 4); ptr += 4;
//        printf("Rigid Body Count : %d\n", nRigidBodies);
        for (int j=0; j < nRigidBodies; j++)
	{
            	// Rigid body position and orientation 
            	int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
            	float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
            	float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
            	float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
            	float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
            	float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
            	float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
            	float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;

            	//update the list with new data

	    	RBIndex=GetRigidBodyIndex(ID);
		if(RBIndex==-1)
		{
			RigidBody RbNew(ID,x,y,z,qx,qy,qz,qw);// create new rigidbody if was not found in AllBodies vector
			AllBodies.push_back(RbNew);
		}else
		{
			
			AllBodies[RBIndex].UpdateData(x,y,z,qx,qy,qz,qw);
		}
		
	    //printf("ID : %d\n", ID);
            //printf("pos: [%3.2f,%3.2f,%3.2f]\n", x,y,z);
            //printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx,qy,qz,qw);

            // NatNet version 2.0 and later
            if(major >= 2)
            {
                // Mean marker error
                float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
//                printf("Mean marker error: %3.2f\n", fError);
            }

            // NatNet version 2.6 and later
            if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
            {
                // params
                short params = 0; memcpy(&params, ptr, 2); ptr += 2;
                bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
            }
        } // Go to next rigid body


        // Skeletons (NatNet version 2.1 and later)
        if( ((major == 2)&&(minor>0)) || (major>2))
        {
            int nSkeletons = 0;
            memcpy(&nSkeletons, ptr, 4); ptr += 4;
//            printf("Skeleton Count : %d\n", nSkeletons);

            // Loop through skeletons
            for (int j=0; j < nSkeletons; j++)
            {
                // skeleton id
                int skeletonID = 0;
                memcpy(&skeletonID, ptr, 4); ptr += 4;

                // Number of rigid bodies (bones) in skeleton
                int nRigidBodies = 0;
                memcpy(&nRigidBodies, ptr, 4); ptr += 4;
//                printf("Rigid Body Count : %d\n", nRigidBodies);

                // Loop through rigid bodies (bones) in skeleton
                for (int j=0; j < nRigidBodies; j++)
                {
                    // Rigid body position and orientation
                    int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
                    float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
                    float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
                    float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
                    float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
                    float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
                    float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
                    float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
//                    printf("ID : %d\n", ID);
//                    printf("pos: [%3.2f,%3.2f,%3.2f]\n", x,y,z);
//                    printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx,qy,qz,qw);

                    // Mean marker error (NatNet version 2.0 and later)
                    if(major >= 2)
                    {
                        float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
//                        printf("Mean marker error: %3.2f\n", fError);
                    }

                    // Tracking flags (NatNet version 2.6 and later)
                    if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
                    {
                        // params
                        short params = 0; memcpy(&params, ptr, 2); ptr += 2;
                        bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
                    }

                } // next rigid body

            } // next skeleton
        }
        
        // labeled markers (NatNet version 2.3 and later)
        // labeled markers - this includes all markers: Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
		if( ((major == 2)&&(minor>=3)) || (major>2))
		{
			int nLabeledMarkers = 0;
			memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
//			printf("Labeled Marker Count : %d\n", nLabeledMarkers);

            // Loop through labeled markers
			for (int j=0; j < nLabeledMarkers; j++)
			{
				// id
                // Marker ID Scheme:
                // Active Markers:
                //   ID = ActiveID, correlates to RB ActiveLabels list
                // Passive Markers: 
                //   If Asset with Legacy Labels
                //      AssetID 	(Hi Word)
                //      MemberID	(Lo Word)
                //   Else
                //      PointCloud ID
				int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
                int modelID, markerID;
//                DecodeMarkerID(ID, &modelID, &markerID);


				// x
				float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
				// y
				float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
				// z
				float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
				// size
				float size = 0.0f; memcpy(&size, ptr, 4); ptr += 4;

                // NatNet version 2.6 and later
                if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
                {
                    // marker params
                    short params = 0; memcpy(&params, ptr, 2); ptr += 2;
                    bool bOccluded = (params & 0x01) != 0;     // marker was not visible (occluded) in this frame
                    bool bPCSolved = (params & 0x02) != 0;     // position provided by point cloud solve
                    bool bModelSolved = (params & 0x04) != 0;  // position provided by model solve
                    if ((major >= 3) || (major == 0))
                    {
                        bool bHasModel = (params & 0x08) != 0;     // marker has an associated asset in the data stream
                        bool bUnlabeled = (params & 0x10) != 0;    // marker is 'unlabeled', but has a point cloud ID
                        bool bActiveMarker = (params & 0x20) != 0; // marker is an actively labeled LED marker
                    }

                }

                // NatNet version 3.0 and later
                float residual = 0.0f;
                if ((major >= 3) || (major == 0))
                {
                    // Marker residual
                    memcpy(&residual, ptr, 4); ptr += 4;
                }

//				printf("ID  : [MarkerID: %d] [ModelID: %d]\n", markerID, modelID);
//				printf("pos : [%3.2f,%3.2f,%3.2f]\n", x,y,z);
//                printf("size: [%3.2f]\n", size);
//                printf("err:  [%3.2f]\n", residual);
            }
		}

        // Force Plate data (NatNet version 2.9 and later)
        if (((major == 2) && (minor >= 9)) || (major > 2))
        {
            int nForcePlates;
            memcpy(&nForcePlates, ptr, 4); ptr += 4;
            for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++)
            {
                // ID
                int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
//                printf("Force Plate : %d\n", ID);

                // Channel Count
                int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

                // Channel Data
                for (int i = 0; i < nChannels; i++)
                {
//                    printf(" Channel %d : ", i);
                    int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
                    for (int j = 0; j < nFrames; j++)
                    {
                        float val = 0.0f;  memcpy(&val, ptr, 4); ptr += 4;
//                        printf("%3.2f   ", val);
                    }
//                    printf("\n");
                }
            }
        }

        // Device data (NatNet version 3.0 and later)
        if (((major == 2) && (minor >= 11)) || (major > 2))
        {
            int nDevices;
            memcpy(&nDevices, ptr, 4); ptr += 4;
            for (int iDevice = 0; iDevice < nDevices; iDevice++)
            {
                // ID
                int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
//                printf("Device : %d\n", ID);

                // Channel Count
                int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

                // Channel Data
                for (int i = 0; i < nChannels; i++)
                {
//                    printf(" Channel %d : ", i);
                    int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
                    for (int j = 0; j < nFrames; j++)
                    {
                        float val = 0.0f;  memcpy(&val, ptr, 4); ptr += 4;
//                        printf("%3.2f   ", val);
                    }
//                    printf("\n");
                }
            }
        }
		
		// software latency (removed in version 3.0)
        if ( major < 3 )
        {
            float softwareLatency = 0.0f; memcpy(&softwareLatency, ptr, 4);	ptr += 4;
//            printf("software latency : %3.3f\n", softwareLatency);
        }

		// timecode
		unsigned int timecode = 0; 	memcpy(&timecode, ptr, 4);	ptr += 4;
		unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4); ptr += 4;
		char szTimecode[128] = "";
//		TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

        // timestamp
        double timestamp = 0.0f;

        // NatNet version 2.7 and later - increased from single to double precision
        if( ((major == 2)&&(minor>=7)) || (major>2))
        {
            memcpy(&timestamp, ptr, 8); ptr += 8;
        }
        else
        {
            float fTemp = 0.0f;
            memcpy(&fTemp, ptr, 4); ptr += 4;
            timestamp = (double)fTemp;
        }
//        printf("Timestamp : %3.3f\n", timestamp);

        // high res timestamps (version 3.0 and later)
        if ( (major >= 3) || (major == 0) )
        {
            uint64_t cameraMidExposureTimestamp = 0;
            memcpy( &cameraMidExposureTimestamp, ptr, 8 ); ptr += 8;
//            printf( "Mid-exposure timestamp : %" PRIu64"\n", cameraMidExposureTimestamp );

            uint64_t cameraDataReceivedTimestamp = 0;
            memcpy( &cameraDataReceivedTimestamp, ptr, 8 ); ptr += 8;
//            printf( "Camera data received timestamp : %" PRIu64"\n", cameraDataReceivedTimestamp );

            uint64_t transmitTimestamp = 0;
            memcpy( &transmitTimestamp, ptr, 8 ); ptr += 8;
//            printf( "Transmit timestamp : %" PRIu64"\n", transmitTimestamp );
        }

        // frame params
        short params = 0;  memcpy(&params, ptr, 2); ptr += 2;
        bool bIsRecording = (params & 0x01) != 0;                  // 0x01 Motive is recording
        bool bTrackedModelsChanged = (params & 0x02) != 0;         // 0x02 Actively tracked model list has changed


		// end of data tag
        int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
//        printf("End Packet\n-------------\n");

    }
    else if(MessageID == 5) // Data Descriptions
    {
        // number of datasets
        int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
//        printf("Dataset Count : %d\n", nDatasets);

        for(int i=0; i < nDatasets; i++)
        {
//            printf("Dataset %d\n", i);

            int type = 0; memcpy(&type, ptr, 4); ptr += 4;
//            printf("Type : %d\n",  type);

            if(type == 0)   // markerset
            {
                // name
                char szName[256];
                strcpy(szName, ptr);
                int nDataBytes = (int) strlen(szName) + 1;
                ptr += nDataBytes;
//                printf("Markerset Name: %s\n", szName);

        	    // marker data
                int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
//                printf("Marker Count : %d\n", nMarkers);

                for(int j=0; j < nMarkers; j++)
                {
                    char szName[256];
                    strcpy(szName, ptr);
                    int nDataBytes = (int) strlen(szName) + 1;
                    ptr += nDataBytes;
//                    printf("Marker Name: %s\n", szName);
                }
            }
            else if(type ==1)   // rigid body
            {
                if(major >= 2)
                {
                    // name
                    char szName[MAX_NAMELENGTH];
                    strcpy(szName, ptr);
                    ptr += strlen(ptr) + 1;
//                    printf("Name: %s\n", szName);
                }

                int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
//                printf("ID : %d\n", ID);
             
                int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
//                printf("Parent ID : %d\n", parentID);
                
                float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
//                printf("X Offset : %3.2f\n", xoffset);

                float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
//                printf("Y Offset : %3.2f\n", yoffset);

                float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
//                printf("Z Offset : %3.2f\n", zoffset);

                // Per-marker data (NatNet 3.0 and later)
                if ( major >= 3 )
                {
                    int nMarkers = 0; memcpy( &nMarkers, ptr, 4 ); ptr += 4;

                    // Marker positions
                    nBytes = nMarkers * 3 * sizeof( float );
                    float* markerPositions = (float*)malloc( nBytes );
                    memcpy( markerPositions, ptr, nBytes );
                    ptr += nBytes;

                    // Marker required active labels
                    nBytes = nMarkers * sizeof( int );
                    int* markerRequiredLabels = (int*)malloc( nBytes );
                    memcpy( markerRequiredLabels, ptr, nBytes );
                    ptr += nBytes;

                    for ( int markerIdx = 0; markerIdx < nMarkers; ++markerIdx )
                    {
                        float* markerPosition = markerPositions + markerIdx * 3;
                        const int markerRequiredLabel = markerRequiredLabels[markerIdx];

//                        printf( "\tMarker #%d:\n", markerIdx );
//                        printf( "\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2] );

                        if ( markerRequiredLabel != 0 )
                        {
//                            printf( "\t\tRequired active label: %d\n", markerRequiredLabel );
                        }
                    }

                    free( markerPositions );
                    free( markerRequiredLabels );
                }
            }
            else if(type ==2)   // skeleton
            {
                char szName[MAX_NAMELENGTH];
                strcpy(szName, ptr);
                ptr += strlen(ptr) + 1;
//                printf("Name: %s\n", szName);

                int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
//                printf("ID : %d\n", ID);

                int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
//                printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

                for(int i=0; i< nRigidBodies; i++)
                {
                    if(major >= 2)
                    {
                        // RB name
                        char szName[MAX_NAMELENGTH];
                        strcpy(szName, ptr);
                        ptr += strlen(ptr) + 1;
//                        printf("Rigid Body Name: %s\n", szName);
                    }

                    int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
//                    printf("RigidBody ID : %d\n", ID);

                    int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
//                    printf("Parent ID : %d\n", parentID);

                    float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
//                    printf("X Offset : %3.2f\n", xoffset);

                    float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
//                    printf("Y Offset : %3.2f\n", yoffset);

                    float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
//                    printf("Z Offset : %3.2f\n", zoffset);
                }
            }

        }   // next dataset

       //printf("End Packet\n-------------\n");
	
    }
    else
    {
        printf("Unrecognized Packet Type.\n");
	return -1;
    }
	
	//---------------------- end of unpacking ------------------------
}

// main for testing
int main(int argc, char* argv[])
{

    float x,y,z,qx,qy,qz,qw; // position and orientation in NED frame
    float Mx, My, Mz; // position of motive
    float Mqx, Mqy, Mqz, Mqw; // quaternions of motive
    float roll, pitch, yaw;
    int membershipCounter = 0; // when it reaches a certain value, membership should be renewed for the optitrack connection
    int optiTO = 0; // time out variable (if it reaches a certain value while the body is not detected, it stops the program)
    MotiveClient mc;


    RigidBody* quad1;

    if (argc < 3)
    {
        std::cout << "Insufficient arguments\n usage: <file name> <client address> <server address>\n";
        return -1;
    }
    else
    {
        int flag = 0;

        flag = mc.InitClient(MULTICAST_ADDRESS, argv[1], argv[2], DATA_PORT, COMMAND_PORT);

        if (flag < 0)
        {

            std::cout << "Motive client initialization failed.\n";
            return -1;
        }
        else
        {
            std::cout << "Motive client initialized.\n";
        }

    }

    // check that quad 1 is detected by optitrack and that it is centered in the arena
    while (quad1 == NULL)
    {
        optiTO++;
        if (optiTO >= 1000)
        {
            std::cout << "Optitrack did not detect rigid body with ID: " << MY_ID << "\n";
            return -1;
        }
        usleep(5000);
        mc.ReadNewData();
        quad1 = mc.GetRigidBodyData(MY_ID);

        if (quad1 != NULL)
        {
            quad1->getData(&Mx, &My, &Mz, &Mqx, &Mqy, &Mqz, &Mqw);
            if (Mx > 0.1 || Mx < -0.1 || Mz > 0.1 || Mz < -0.1 || My > 0.1 || My < -0.1)
            {
                std::cout << "Warning: Drone not in the center of the field\n Terminating..\n";
                return -1;
            }
        }


    }
	//mc.InitClient("239.255.42.99", "192.168.1.4", "192.168.1.3", 1511, 1510);
	

	while(1)
	{
		
		
		mc.ReadNewData();

		quad1 = mc.GetRigidBodyData(1);

		if(quad1 != NULL)
			if(quad1->isUpdated())
			{

                quad1->getData(&Mx, &My, &Mz, &Mqx, &Mqy, &Mqz, &Mqw);

                // transfrom from motive axes to NED axes



		float qb = -1.0 *Mqz;
                float qc = Mqx;
                float qd = Mqy;
                float qa = -1.0 * Mqw;


                pitch = std::asin(2 * (qa * qc - qd * qb));
                roll = std::atan2(2 * (qa * qb + qc * qd), 1 - 2 * (qb * qb + qc * qc));
                yaw = std::atan2(2 * (qa * qd + qb * qc), 1 - 2 * (qc * qc + qd * qd));


/*                qx = Mqz;
                qy = -1.0 * Mqx;
                qz = -1.0 * Mqy;
                qw = -1.0 * Mqw;
*/
		x=Mz;
		y=-Mx;
		z=-My;

/*		pitch = std::asin(2 * (qx * qz - qw * qy));
                roll = std::atan2(2 * (qx * qy + qz * qw), 1 - 2 * (qy * qy + qz * qz));
                yaw = std::atan2(2 * (qx * qw + qy * qz), 1 - 2 * (qz * qz + qw * qw));
*/

				printf("Quad1 data:\nx: %f, y: %f, z: %f\nroll: %f, pitch: %f, yaw: %f\n", x, y, z, roll, pitch, yaw);


			}

        membershipCounter++;

        if (membershipCounter > 8000) // renew membership every 40 seconds (0.005 * 8000) 
        {                            //since router closes connection automatically every arrox. 50 seconds
            mc.RenewMembership();
            membershipCounter = 0;
        }

		usleep(5000); // sleep for 5ms, (remove when integrated into code since function call is already periodic)

	}

	return 0;
}


