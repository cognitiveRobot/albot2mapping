/*
    Laser2Surface - Scans laser data and finds surfaces.

    Original by ? (modified by Thomas)
*/

#include <iostream>
#include <list>
#include <stack>
//#include <cstdlib>
#include "Laser2Surface.H"
//#include "readAndwriteASCII.H"
//#include "RobotSettings.H"
//#include "GeometryFuncs.H"
//#include "Plotting.H"

//#include "Object.H"

using namespace std;


// Collects a laser reading from the robot, and corrects the XY-rotation
vector<PointXY> CollectLaser(ArSick &sick, double maxRange)
{
	vector<PointXY> laserPoints;

	sick.lockDevice();

	vector<ArSensorReading> * readings = sick.getRawReadingsAsVector();
	for(vector<ArSensorReading>::const_iterator it = readings->begin(); it != readings->end(); it++)
	{
		if(it->getRange() < maxRange)
		{
			// Correct coordinates right here so we don't have to do it later!
			double x = -it->getLocalY();
			double y =  it->getLocalX();
			laserPoints.push_back(PointXY(x, y));
		}
	}

	sick.unlockDevice();
	//ArUtil::sleep(100);

	return laserPoints;
}

/*
vector<Object> scanAndSaveView(ArSick &sick, int v)
{

	std::vector< ArSensorReading >* readings;
	std::vector< ArSensorReading >::iterator it;

 	int counter=0;
	double filter =30000;

	char lfname[50];
	sprintf(lfname, "%s%d", "bin/laser-",v);

 	FILE* dataFile;
 	dataFile = fopen(lfname, "w");

  	sick.lockDevice();

  	readings = sick.getRawReadingsAsVector();

  	for (it = readings->begin(); it != readings->end(); it++)
  	{
   		if ((*it).getRange() < filter)
      		{
			counter = counter + 1;
      		}
  	}

  	fprintf ( dataFile, "%d 2\n",counter);
	
	vector<PointXY> laserPoints;

 	for (it = readings->begin(); it != readings->end(); it++)
  	{
    		if ((*it).getRange() < filter)
      		{
       		 // Correct coordinates right here so we don't have to do it later!
        		float x = -(*it).getLocalY();
        		float y = (*it).getLocalX();
        		fprintf( dataFile, "%f %f\n", x, y );
			laserPoints.push_back(PointXY(x, y));
      		}
  	}


  	sick.unlockDevice();
  	ArUtil::sleep(1500);

  	fclose( dataFile );

		
 	double clusterThreshold =600;// ct; //atof(argv[1]);
    	int surfaceSize = 200;//ss;// atoi(argv[2]);
    	int errorThreshold = 150;//et;//atoi(argv[3]);
    	//cout << "Looking for surfaces with" << endl;
    	//cout << "  clusterThreshold = " << clusterThreshold << endl;
    	//cout << "  surfaceSize      = " << surfaceSize << endl;
    	//cout << "  errorThreshold   = " << errorThreshold << endl << endl;	

    	// Read laser data and calculate surfaces
    	//vector<PointXY> laser;
    	//readASCII(laser, lfname);
    	vector<SurfaceT> surfaces = Laser2Surface(laserPoints, clusterThreshold, surfaceSize, errorThreshold);
	char sname[50];
	sprintf(sname, "%s%d", "bin/surfaces-",v);
	writeASCII(surfaces, sname);
	//PointXY a=surfaces[20].getP1();
	//cout<<"surface Length: "<< a.distanceFrom(surfaces[20].getP2())<<endl;
//	char png[50];
//        	sprintf(png, "%s%d%s","view-points-",v, ".png");
//    	// Plot results
//    	plotPointsAndSurfaces(png, laserPoints, surfaces);
	
	char filename[50];
	sprintf(filename, "%s%d", "bin/surfaces-", v);
	
	//cout<<"........Reading "<<filename<<endl;
  	vector <Object> view = readASCII(filename);
	cout<<"\n************************view # "<<v<<" saved**************************"<<endl<<endl;

	return view;
}
 */
// Finds surfaces from raw laser data.
vector<SurfaceT> Laser2Surface(const vector<PointXY> & laserPoints,
							  double clusterTh, double minSurfaceSize, double errorTh)
{
    /**********************************************************************
    1) Cluster the laser data
    2) Segment the clusters using modified Split without Merge Algorithm,
    **********************************************************************/

    /*****************************/
    // 1) Cluster the laser data
    /*****************************/
    vector<int> clusterIndex;

    for(unsigned int i = 0; i < laserPoints.size(); i++)
    {
        if(i == 0)
        {
            clusterIndex.push_back(i);
        }
        else
        {
            /* Determine distance between two consecutive readings.
             * Cluster data based on distance.
             */
            if (laserPoints[i-1].distFromSq(laserPoints[i]) >= clusterTh*clusterTh)
            {
                clusterIndex.push_back(i);
            }
        }
    }
    if(clusterIndex.size() <= 1 && laserPoints.size() > 0)
    {
        clusterIndex.push_back(laserPoints.size()-1);
    }

    // Form a surface for each cluster
    vector<SurfaceT> groupedSurfaces;

    for(unsigned int i = 0; i < clusterIndex.size(); i++)
    {
    	int currIndex = clusterIndex[i];
    	int nextIndex;
        if (i == clusterIndex.size()-1)
        {
        	nextIndex = laserPoints.size() - 1;       //point to last element
        }
        else
        {
            nextIndex = clusterIndex[i+1] - 1;
        }

        SurfaceT surface(laserPoints[currIndex], laserPoints[nextIndex]);
        groupedSurfaces.push_back(surface);
    }

    //cout << "Found " << groupedSurfaces.size() << " clusters (grouped surfaces)." << endl;



    /**********************************************************************/
    // 2) Segment the clusters using modified Split without Merge Algorithm.
    /**********************************************************************/

    vector<surfaceIndices> finalSurfaceIndices;

    // For each cluster (grouped surface), do:
    for (unsigned int i = 0; i < clusterIndex.size(); i++)
    {
    	int currIndex = clusterIndex[i];
    	int nextIndex;

        if (i == clusterIndex.size()-1)
        {
            nextIndex = laserPoints.size() - 1;       //point to last element
        }
        else
        {
            nextIndex = clusterIndex[i+1] - 1;
        }

        // Recursively split the surface until it can't/shouldn't be split anymore
        vector<surfaceIndices> indices;
        indices = splitSurface(laserPoints, currIndex, nextIndex, minSurfaceSize, errorTh);
        finalSurfaceIndices.insert(finalSurfaceIndices.end(), indices.begin(), indices.end());
    }

    // Generate the results from the indices
    vector<SurfaceT> finalSurfaces;
    for(unsigned int i = 0; i < finalSurfaceIndices.size(); i++)
    {
        SurfaceT surface(laserPoints[finalSurfaceIndices[i].startIndex],
                        laserPoints[finalSurfaceIndices[i].endIndex]);
        finalSurfaces.push_back(surface);
    }

    // Clean up
    removeBogusSurfaces(finalSurfaces);
   // markOccludingEdges(finalSurfaces, RobotSettings::SURFACE_MIN_OCCLUDING_DISTANCE);
    //markBoundarySurfs(finalSurfaces, false);

   // cout << "Found " << finalSurfaces.size() << " surfaces." << endl;

    //DEBUG
	//cout << "Plotting surfaces.png ..." << endl;
	//plotPointsAndSurfaces("surfaces.png", laserPoints, finalSurfaces);

    return finalSurfaces;
}



/*
 * Splits a surface recursively until it can/should be split no more
 * After split, both resulting surfaces are still connected
 */
vector<surfaceIndices> splitSurface(const vector<PointXY> & laserdata, int startIndex, int endIndex, double surfaceSize, double errorThreshold)
{
    vector<surfaceIndices> returnSurfaces;

    // Find a suitable splitting point for this surface.
    int splitIndex = findSplittingPoint(laserdata, startIndex, endIndex, surfaceSize, errorThreshold);
    if(splitIndex <= 0)
    {
        splitIndex = findSplittingPoint(laserdata, startIndex, endIndex, surfaceSize, errorThreshold/2);
    }

    // Found a splitting point?
    if(splitIndex > 0)
    {
        // Split the surface, and call the function again for both halves
        vector<surfaceIndices> before, after;
        before = splitSurface(laserdata, startIndex, splitIndex, surfaceSize, errorThreshold);
        after = splitSurface(laserdata, splitIndex, endIndex, surfaceSize, errorThreshold);
        returnSurfaces.insert(returnSurfaces.end(), before.begin(), before.end());
        returnSurfaces.insert(returnSurfaces.end(), after.begin(), after.end());
    }
    else
    {
        // Do not split, and return only this surface
        surfaceIndices indices = { startIndex, endIndex };
        returnSurfaces.push_back(indices);
    }
    return returnSurfaces;
}


/* Finds a splitting point for a surface.
 * First, the length is determined. If it is longer than the threshold, then find the point
 * farthest from the surface. It this distance exceeds the error threshold, then a split would occur
 * at that particular point.
 * Returns the index of the point where the surface should be split.
 */
int findSplittingPoint(const vector<PointXY> & laserPoints, int currInd, int nextInd, double surfaceSizeTh, double errorTh)
{
	SurfaceT surf(laserPoints[currInd], laserPoints[nextInd]);

    // Check to see if the current surface is significant enough to split. If not, then just leave it
    if((surf.getLength() > surfaceSizeTh) && ((nextInd - currInd) > 1))
    {
        int maxIndex = -1;
    	double maxError = 0.0;

    	// Find the point with the maximum distance from the surface (greatest error)
    	for (int counter = currInd; counter <= nextInd; counter++)
    	{
    		PointXY nearestPoint = nearestPointOnSurf(surf, laserPoints[counter], false);
    		double distance = nearestPoint.distFrom(laserPoints[counter]);

    		if (distance > maxError)
    		{
    			maxError = distance;
    			maxIndex = counter;
    		}
    	}

        if (maxError > errorTh)
        {
            return maxIndex;
        }
    }

    return -1;
}


// Remove surfaces that are too small to matter
void removeBogusSurfaces(vector<SurfaceT> & surfaces)
{
    for(vector<SurfaceT>::iterator it = surfaces.begin(); it != surfaces.end();)
    {
        if(it->getLength() < 0.8 * 200)
        {
            it = surfaces.erase(it);
        }
        else
        {
            ++it;
        }
    }
}


// Marks occluding edges (we are certain that a surface ends here, and is not occluded)
void markOccludingEdges(vector<SurfaceT> & surfaces, double threshold)
{
    PointXY robotPos(0, 0);

    for(unsigned int i = 0; i < surfaces.size() - 1; ++i)
    {
        const PointXY & pA = surfaces[i].getP2();
        const PointXY & pB = surfaces[i+1].getP1();

        // The gap between the surfaces is big enough
        if(pA.distFromSq(pB) > threshold * threshold)
        {
            // Calculate distance from robot (0,0)
            if(pA.distFromSq(robotPos) < pB.distFromSq(robotPos))
            {
                // pA is closer to the robot and occludes pB
                surfaces[i].setP2Occluding(true);
            }
            else
            {
                // pB is closer to the robot and occludes pA
                surfaces[i+1].setP1Occluding(true);
            }
        }
        else
        {
            // Surfaces (at least almost) connect, so set both points as occluding (=reliable)
            surfaces[i].setP2Occluding(true);
            surfaces[i+1].setP1Occluding(true);
        }
    }
}


/*
 * Marks boundary surfaces in a vector (single view).
 * The surfaces MUST be ordered by ID, and also geometrically (angle in respect to origin)!
 * => It works only with surfaces directly from the laser scan.
 */
void markBoundarySurfs(vector<SurfaceT> & surfaces, bool plotDebugImg)
{
	const PointXY robotPos(0, 0);

	if(surfaces.size() < 1)
	{
		cerr << "WARNING: No surfaces found!" << endl;
		return;
	}

	// Maintain and modify a linked list of boundary surfaces
	list<SurfaceT> bSurfs;
	bSurfs.insert(bSurfs.end(), surfaces.begin(), surfaces.end());

	/* This stack remembers "inward" movements, such as from occluded edges.
	 * Once we find an "outward" movement, we can shortcut it by connecting the outer surfaces
	 * (and removing the inner ones).
	 */
	stack<list<SurfaceT>::iterator> inwardsIndex;

	for(list<SurfaceT>::iterator it = ++bSurfs.begin(); it != bSurfs.end(); ++it)
	{
		list<SurfaceT>::iterator prevIt = it;
		--prevIt;

		// Moving inwards
		if(prevIt->getP2().distFrom(robotPos) > it->getP1().distFrom(robotPos) + 500)
		{
			inwardsIndex.push(it);
		}

		// Moving outwards
		if(prevIt->getP2().distFrom(robotPos) < it->getP1().distFrom(robotPos) - 500)
		{
			// Remove "inside" surfs
			while(inwardsIndex.size() > 0)
			{
				list<SurfaceT>::iterator eraseStart = inwardsIndex.top();

				/* Get the angle difference between the inward/outward connections.
				 * If it's too big, don't shortcut across the whole view!
				 */
				double angleDiff = SurfaceT(eraseStart->getP2(), robotPos).getAngleDiffTo(SurfaceT(it->getP1(), robotPos));
				if(normAngleDiff(angleDiff) > 45)
					break;

				// Angle is small, remove the inner surfaces
				inwardsIndex.pop();
				bSurfs.erase(eraseStart, it);
			}
		}
	}

	// We have the boundary surfaces, now mark them in the original vector
	vector<SurfaceT>::iterator surfIt = surfaces.begin();
	for(list<SurfaceT>::const_iterator bSurfIt = bSurfs.begin(); bSurfIt != bSurfs.end(); ++bSurfIt)
	{
		while(surfIt->getId() < bSurfIt->getId())
			++surfIt;

		if(surfIt->getId() != bSurfIt->getId())
		{
			cerr << "ERROR: could not mark boundary surfaces! There is a problem in surface ID order!" << endl;
			return;
		}
		surfIt->setBoundarySurf(true);
	}



	if(plotDebugImg)
	{
		// For plotting: Connect the surfaces endpoint-to-endpoint.
		vector<SurfaceT> connections;
		connections.push_back(SurfaceT(PointXY(0,0), bSurfs.front().getP1()));
		SurfaceT prevSurf = bSurfs.front();
		for(list<SurfaceT>::const_iterator it = ++bSurfs.begin(); it != bSurfs.end(); ++it)
		{
			SurfaceT connection(prevSurf.getP2(), it->getP1());
			if(connection.getLength() > 50)
				connections.push_back(connection);
			prevSurf = *it;
		}
		connections.push_back(SurfaceT(bSurfs.back().getP2(), PointXY(0,0)));

		vector<vector<SurfaceT> > plotVector;
		plotVector.push_back(surfaces);
		plotVector.push_back(connections);
		vector<SurfaceT> bSurfsV;
		bSurfsV.insert(bSurfsV.end(), bSurfs.begin(), bSurfs.end());
		plotVector.push_back(bSurfsV);

		cout << "Plotting boundary.png ..." << endl;
		//plotSurfaces("boundary.png", plotVector);
	}
}
