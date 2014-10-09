#include "Map.h"

Map::Map(int _sizeX, int _sizeY) :
		sizeX(_sizeX), sizeY(_sizeY) {
	sizeX = 2000;
	sizeY = 2000;
	M = 0;
}

Map::~Map() {

}

void Map::coordTransf(cv::Point3f *target, cv::Point3f newCenter, double hX,
		double hY) {

	// Homothetic transformation to adapt to axes
	target->x *= hX;
	target->y *= hY;
	// Change the center of the frame reference
	target->x += newCenter.x;
	target->y += newCenter.y;

}

void Map::rotate(cv::Point2f* target, cv::Point2f Center, float angle) {
	float r;
	float teta;

	target->x = target->x - Center.x;
	target->y = target->y - Center.y;
	target->y = -target->y;

	r = sqrt(target->x * target->x + target->y * target->y); // Getting distance from center

	if (target->x < 0) {
		if (target->y < 0) {
			teta = atan(target->y / target->x) - M_PI;
		} else {
			teta = atan(target->y / target->x) + M_PI;
		}
	} else
		teta = atan(target->y / target->x);

	// Set angle for rotated point
	angle *= M_PI;
	angle /= (float) 180;
	teta += angle;

	// Set new position
	target->x = r * cos(teta);
	target->y = r * sin(teta);

	target->x = target->x + Center.x;
	target->y = Center.y - target->y;

}

void Map::update(View newView) {
	// Robot
	cv::Point3f tmpPos;
	cv::Point3f O(sizeX / 2, sizeY / 2, 0);
	//cv::Point3f O(sizeX/2, sizeY-15, 0);

	// Comparing surface variables
	vector<Surface> tmpObst, tmpObst1; // Temporary surfaces vector to update Obst
	int counter;
	unsigned int Osize, Osize1; // Original size of Obst before adding new Surfaces to the map

	cout << surfaces.size() << " Initial Surfaces in the map" << endl;
	/* Adapt newView to compare Surfaces to previous ones */
	newView.setSurfaces();
	newView.rotate();
	newView.translate();

	if (surfaces.size() == 0) { // If the map is empty, place the robot in initial position and add surfaces
		// Set robot position on the map and add it to rbtPos
		cout << surfaces.size() << "No Surface in the map" << endl;
		tmpPos = newView.getRobotPos();
		coordTransf(&tmpPos, O, (double) 1 / 10, (double) -1 / 10);
		newView.setRobotPos(tmpPos.x, tmpPos.y, tmpPos.z);
		rbtPos.push_back(tmpPos);

		// Add the Surfaces to the map
		surfaces = newView.getSurfaces();

		newView.clearView();
		cout << surfaces.size() << " start Surface in the map" << endl;
	} else                            // Else compare surfaces
	{

		// Move the robot
		tmpPos = newView.getRobotPos();

		coordTransf(&tmpPos, O, (double) 1 / 10, (double) -1 / 10);
		newView.setRobotPos(tmpPos.x, tmpPos.y, tmpPos.z);
		rbtPos.push_back(tmpPos);

		Osize = surfaces.size();
		Osize1 = surfaces.size();
		//old and then new
		for (unsigned int j = 0; j < newView.getSurfaces().size(); j++) { // For each surface of the new View

			counter = 0;

			for (unsigned int i = 0; i < Osize; i++) { // For every surface of the map

				if (isBehind(surfaces[i], newView.getSurfaces()[j],
						rbtPos[rbtPos.size()])
						|| isBehind(newView.getSurfaces()[j], surfaces[i],
								rbtPos[rbtPos.size()])) // Check if any Surfaces are concealing others
								{
					counter++;

				}
				if (j == 0) {

					tmpObst.push_back(surfaces[i]);  // Add it to the updated map

				}
			}

			if (counter == 0) { // If an old Surface isn't concealing nor concealed....
				tmpObst.push_back(newView.getSurfaces()[j]); // add all surfaces from new view     // Add the new Surfaces

			}
		}

// new and then old
		//for(unsigned int i=0; i < Osize; i++)           // For every surface of the map

//{
		//     counter=0;

		// for(unsigned int j=0; j<newView.getSurfaces().size(); j++)         // For each surface of the new View

		//      {

		//          if(isBehind(surfaces[i], newView.getSurfaces()[j], rbtPos[rbtPos.size()]) || isBehind(newView.getSurfaces()[j], surfaces[i], rbtPos[rbtPos.size()]))     // Check if any Surfaces are concealing others
		// {
		//          counter++;

		//          }
//if(i == 0 ){ 
//surfaces.push_back(newView.getSurfaces()[j]);

		//             }
		//             }
		//             cout << counter << "Counter" << endl;
		//               if(counter == 0 ){
//tmpObst.push_back(surfaces[i]);  // Add it to the updated map
		// If an old Surface isn't concealing nor concealed....
		//add all surfaces from new view     // Add the new Surfaces

		//           }
		//          }

// both new and old
//for(unsigned int i=0; i < Osize1; i++)           // For every surface of the map

//{tmpObst.push_back(surfaces[i]);
//}
//           for(unsigned int j=0; j<newView.getSurfaces().size(); j++)         // For each surface of the new View

		//          {

//tmpObst.push_back(newView.getSurfaces()[j]);

//                  }  

		surfaces.clear();
		surfaces = tmpObst;
		// replace the map by the updated one
		cout << tmpObst.size() << " Surfaces in the new map" << endl;
		tmpObst.clear();

	}

}

bool Map::isBehind(Surface Old, Surface New, cv::Point3f rbtPos) {
	bool ret = false;

	// Translate to compare as same view
	Old.setP1(Old.getP1().x - rbtPos.x, Old.getP1().y - rbtPos.y);
	Old.setP2(Old.getP2().x - rbtPos.x, Old.getP2().y - rbtPos.y);
	New.setP1(Old.getP1().x - rbtPos.x, New.getP1().y - rbtPos.y);
	New.setP2(Old.getP2().x - rbtPos.x, New.getP2().y - rbtPos.y);

	if (Old.getP1().y > 0 && Old.getP2().y > 0) {
		if (Old.getP1().x == 0) {
			if (New.getP1().x > 0
					&& New.getP1().y
							> (float) Old.getP2().y / Old.getP2().x
									* New.getP1().x && New.getP2().x > 0
					&& New.getP2().y
							> (float) Old.getP2().y / Old.getP2().x
									* New.getP2().x) {
				ret = true;
			}
		} else if (Old.getP1().x < 0 && Old.getP2().x == 0) {
			if ((New.getP1().y
					> (float) Old.getP1().y / Old.getP1().x * New.getP1().x
					&& New.getP1().x < 0)
					|| (New.getP2().y
							> (float) Old.getP1().y / Old.getP1().x
									* New.getP2().x && New.getP2().x < 0)) {
				ret = true;
			}
		} else if (Old.getP1().x < 0 && Old.getP2().x < 0) {
			if ((New.getP1().y
					> (float) Old.getP1().y / Old.getP1().x * New.getP1().x
					&& New.getP1().y
							< (float) Old.getP2().y / Old.getP2().x
									* New.getP1().x)
					|| (New.getP2().y
							> (float) Old.getP1().y / Old.getP1().x
									* New.getP2().x
							&& New.getP2().y
									< (float) Old.getP2().y / Old.getP2().x
											* New.getP2().x)) {
				ret = true;
			}
		} else if (Old.getP1().x < 0 && Old.getP2().x > 0) {
			if ((New.getP1().y
					> (float) Old.getP1().y / Old.getP1().x * New.getP1().x
					&& New.getP1().y
							> (float) Old.getP2().y / Old.getP2().x
									* New.getP1().x)
					|| (New.getP2().y
							> (float) Old.getP1().y / Old.getP1().x
									* New.getP2().x
							&& New.getP2().y
									> (float) Old.getP2().y / Old.getP2().x
											* New.getP2().x)) {
				ret = true;
			}
		} else if (Old.getP1().x > 0 && Old.getP2().x > 0) {
			if ((New.getP1().y
					< (float) Old.getP1().y / Old.getP1().x * New.getP1().x
					&& New.getP1().y
							> (float) Old.getP2().y / Old.getP2().x
									* New.getP1().x)
					|| (New.getP2().y
							< (float) Old.getP1().y / Old.getP1().x
									* New.getP2().x
							&& New.getP2().y
									> (float) Old.getP2().y / Old.getP2().x
											* New.getP2().x)) {
				ret = true;
			}
		}
	}

	// Set them back to original position
	Old.setP1(Old.getP1().x + rbtPos.x, Old.getP1().y + rbtPos.y);
	Old.setP2(Old.getP2().x + rbtPos.x, Old.getP2().y + rbtPos.y);
	New.setP1(Old.getP1().x + rbtPos.x, New.getP1().y + rbtPos.y);
	New.setP2(Old.getP2().x + rbtPos.x, New.getP2().y + rbtPos.y);

	return ret;
}

void Map::display() {

	drawing = cv::Mat::zeros(cv::Size(sizeX, sizeY), CV_8UC3);
	drawing.setTo(cv::Scalar(255, 255, 255));
	cv::Point2f P11, P21, P1, P2;
	cv::Point2f O(0, 0);
	Surface curObst1;

	// Draw robot positions
	for (unsigned int i = 0; i < rbtPos.size(); i++) {
		cv::Point2f rbt0(rbtPos[i].x, rbtPos[i].y);
		cv::Point2f rbt1(rbt0.x - 15, rbt0.y - 15);
		cv::Point2f rbt2(rbt0.x + 15, rbt0.y + 15);
		cv::Point2f rbt3(rbt0.x, rbt0.y - 30);
		rotate(&rbt3, rbt0, rbtPos[i].z); // Set the robot into the right direction
		cv::rectangle(drawing, rbt1, rbt2, cv::Scalar(0, 0, 255), 3, 8, 0);
		cv::line(drawing, rbt0, rbt3, cv::Scalar(0, 0, 0), 3, 8, 0);
	}

	// Draw Surfaces Surfaces

//cout << surfaces.size() << " Surfaces drawn in new map" << endl;
	for (unsigned int i = 0; i < surfaces.size(); i++) {
		// Adapt the point to the openCV Mat drawing
		P1 = cv::Point2f(rbtPos[0].x + surfaces[i].getP1().x,
				rbtPos[0].y - surfaces[i].getP1().y);
		P2 = cv::Point2f(rbtPos[0].x + surfaces[i].getP2().x,
				rbtPos[0].y - surfaces[i].getP2().y);
		cv::line(drawing, P1, P2, cv::Scalar(0, 0, 255), 3, 8, 0); // Draw the actual line
	}

//new codes for consistency  
//unsigned int numline1 = 0;
//for (unsigned int i = 0; i<surfaces.size(); i++)
	//  {
	// curObst1 = surfaces[i];                  // Transform the coordinates to have the right frame of reference

	//    if(curObst1.getPoints().size() > 4 ) // If there is enough points in 1 surface...
	//   {
	//            curObst1.setSurface();                                                     // Construct a line with the points
	// Adapt the point to the openCV Mat drawing
	//            curObst1.setP1(rbtPos[0].x + curObst1.getP1().x,rbtPos[0].y - curObst1.getP1().y );
	//            curObst1.setP2(rbtPos[0].x + curObst1.getP2().x,rbtPos[0].y - curObst1.getP2().y );
//
	//            cv::line(drawing, curObst1.getP1(), curObst1.getP2(), cv::Scalar(0,0,255), 3, 8, 0);    // Draw that line
	//   numline1++;
// }
//}
//cout << numline1<< " Number of lines in this map" << endl;
// new code completes

	// Display the map in a file
	char filename[50];
	sprintf(filename, "%s%d%s", "../outputs/Maps/Map", M, ".jpg");
	cv::imwrite(filename, drawing);

	M++;
}
