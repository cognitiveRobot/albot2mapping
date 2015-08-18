#include "Camera.h"
#include "Printer.h"
#include "ImageProcessing.h"

Camera::Camera() {

}

/*Camera::~Camera()
 {

 }*/

void Camera::setV(int newV) {
    v = newV;
}

int Camera::getV() {
    return v;
}

void Camera::incV() {
    v++;
}

void Camera::initialize() {

    // Find cameras on the 1394 buses
    d = dc1394_new();

    // Enumerate cameras connected to the PC
    err = dc1394_camera_enumerate(d, &list);
    if (err != DC1394_SUCCESS) {
        fprintf(stderr, "Unable to look for cameras\n\n"
                "Please check \n"
                "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' "
                "are loaded \n"
                "  - if you have read/write access to /dev/raw1394\n\n");
        exit(EXIT_FAILURE);
    }

    if (list->num == 0) {
        fprintf(stderr, "No cameras found!\n");
        exit(EXIT_FAILURE);
    }

    printf("There were %d camera(s) found attached to your PC\n", list->num);

    // Identify cameras. Use the first stereo camera that is found
    for (Id = 0; Id < list->num; Id++) {
        camera = dc1394_camera_new(d, list->ids[Id].guid);

        if (!camera) {
            printf("Failed to initialize camera with guid %llx",
                    list->ids[Id].guid);
            continue;
        }

        printf("Camera %d model = '%s'\n", Id, camera->model);

        if (isStereoCamera(camera)) {
            printf("Using this camera\n");
            break;
        }
        dc1394_camera_free(camera);
    }

    if (Id == list->num) {
        printf("No stereo cameras were detected\n");
        exit(EXIT_SUCCESS);
    }

    // Free memory used by the camera list
    dc1394_camera_free_list(list);

    // query information about this stereo camera
    err = queryStereoCamera(camera, &stereoCamera);
    if (err != DC1394_SUCCESS) {
        fprintf(stderr, "Cannot query all information from camera\n");
        cleanup_and_exit(camera);
    }

    if (stereoCamera.nBytesPerPixel != 2) {
        // can't handle XB3 3 bytes per pixel
        fprintf(stderr,
                "Example has not been updated to work with XB3 in 3 camera mode yet!\n");
        cleanup_and_exit(stereoCamera.camera);
    }

    // set the capture mode
    printf("Setting stereo video capture mode\n");
    err = setStereoVideoCapture(&stereoCamera);
    if (err != DC1394_SUCCESS) {
        fprintf(stderr, "Could not set up video capture mode\n");
        cleanup_and_exit(stereoCamera.camera);
    }

}

void Camera::getImage() {
    //Stop the capture et reset transmission
    dc1394_video_set_transmission(camera, DC1394_OFF);

    // have the camera start sending us data
    printf("Start transmission\n");
    err = startTransmission(&stereoCamera);
    if (err != DC1394_SUCCESS) {
        fprintf(stderr, "Unable to start camera iso transmission\n");
        cleanup_and_exit(stereoCamera.camera);
    }

    // give the auto-gain algorithms a chance to catch up
    printf("Giving auto-gain algorithm a chance to stabilize\n");

    printf("Getting TriclopsContext from camera (slowly)... \n");
    error = getTriclopsContextFromCamera(&stereoCamera, &triclops);
    if (error != TriclopsErrorOk) {
        fprintf(stderr, "Can't get context from camera\n");
        cleanup_and_exit(camera);
        exit(EXIT_FAILURE);
    }
    printf("...done\n");
    // set up some stereo parameters:
    // set to 320x240 output images;
    error = triclopsSetResolution(triclops, DISPARITY_HEIGHT, DISPARITY_WIDTH);
    _HANDLE_TRICLOPS_ERROR("triclopsSetResolution()", error);
    // set disparity range
    error = triclopsSetDisparity(triclops, 2, 50);
    _HANDLE_TRICLOPS_ERROR("triclopsSetDisparity()", error);

    // turn on sub-pixel interpolation
    triclopsSetSubpixelInterpolation(triclops, 1);
    // make sure strict subpixel validation is on
    triclopsSetStrictSubpixelValidation(triclops, 1);

    // turn on surface validation
    triclopsSetSurfaceValidation(triclops, 1);
    triclopsSetSurfaceValidationSize(triclops, 200);
    triclopsSetSurfaceValidationDifference(triclops, 0.5);

    //  for(int i=0; i<repeatScan; i++) {

    // size of buffer for all images at mono8
    unsigned int nBufferSize = stereoCamera.nRows * stereoCamera.nCols
            * stereoCamera.nBytesPerPixel;

    // allocate a buffer to hold the de-interleaved images
    unsigned char* pucDeInterlacedBuffer = new unsigned char[nBufferSize];

    pucRGBBuffer = new unsigned char[3 * nBufferSize];
    pucGreenBuffer = new unsigned char[nBufferSize];
    pucRightRGB = NULL;
    pucLeftRGB = NULL;
    pucCenterRGB = NULL;

    // allocate a color input for color image rectification
    colorInput.nrows = stereoCamera.nRows;
    colorInput.ncols = stereoCamera.nCols;
    colorInput.rowinc = stereoCamera.nCols * 4;
    colorInput.inputType = TriInp_RGB_32BIT_PACKED;
    colorInput.u.rgb32BitPacked.data =
            (void*) (new unsigned char[colorInput.nrows * colorInput.rowinc]);

    // get the images from the capture buffer and do all required processing
    // note: produces a TriclopsInput that can be used for stereo processing
    extractImagesColor(&stereoCamera, DC1394_BAYER_METHOD_NEAREST,
            pucDeInterlacedBuffer, pucRGBBuffer, pucGreenBuffer, &pucRightRGB,
            &pucLeftRGB, &pucCenterRGB, &input);

    convertColorTriclopsInput(&colorInput, pucRightRGB);

    // set to 320x240 output images
    error = triclopsSetResolution(triclops, DISPARITY_HEIGHT, DISPARITY_WIDTH);
    // Preprocessing the images
    error = triclopsPreprocess(triclops, &input);
    _HANDLE_TRICLOPS_ERROR("triclopsPreprocess()", error);

    // stereo processing
    error = triclopsStereo(triclops);
    _HANDLE_TRICLOPS_ERROR("triclopsStereo()", error);

    // retrieve the depth image from the context
    error = triclopsGetImage16(triclops, TriImg16_DISPARITY, TriCam_REFERENCE,
            &depthImage);
    _HANDLE_TRICLOPS_ERROR("triclopsGetImage()", error);

    error = triclopsGetImage16(triclops, TriImg16_DISPARITY, TriCam_REFERENCE,
            &depthImage);
    _HANDLE_TRICLOPS_ERROR("triclopsGetImage()", error);

    // save the depth image// turn on sub-pixel interpolation
    triclopsSetSubpixelInterpolation(triclops, 1);
    //sprintf(filenameDepth, "%s%d%s", "depth/depth-", v, ".pgm");

    if (sprintf(filenameDepth, "../inputs/depth/depth.pgm") == -1) {
        cout << "Impossible to write depth.pgm file" << endl;
        exit(EXIT_FAILURE);
    }
    error = triclopsSaveImage16(&depthImage, filenameDepth);
    _HANDLE_TRICLOPS_ERROR("triclopsSaveImage()", error);

    // retrieve the depth image from the context
    error = triclopsGetImage(triclops, TriImg_RECTIFIED, TriCam_LEFT,
            &rectifiedImage);
    _HANDLE_TRICLOPS_ERROR("triclopsGetImage()", error);

    // save the rectified image
    if (sprintf(filenameRectified, "%s%d%s", "../inputs/rectified/rectified-",
            v, ".pgm") == -1) {
        cout << "Impossible to write rectified.pgm file" << endl;
        exit(EXIT_FAILURE);
    }
    error = triclopsSaveImage(&rectifiedImage, filenameRectified);
    _HANDLE_TRICLOPS_ERROR("triclopsSaveImage()", error);
    printf("wrote 'depth.pgm'\n");

    // save the color image by Martin
    TriclopsColorImage rectifiedColor;
    error = triclopsRectifyColorImage(triclops, TriCam_REFERENCE, &colorInput,
            &rectifiedColor);
    _HANDLE_TRICLOPS_ERROR("triclopsRectifyColorImage()", error);

    if (sprintf(filenameColor, "%s%d%s", "../inputs/color/color-", v, ".ppm")
            == -1) {
        cout << "Impossible to write color file" << endl;
        exit(EXIT_FAILURE);
    }
    ppmWriteFromTriclopsColorImage(filenameColor, &rectifiedColor);
    printf("wrote color file\n");

    //save point cloud in .pcd file
    savePointCloud(triclops, depthImage);

    TriclopsImage leftImage, rightImage;
    //TriclopsColorImage leftImage, rightImage;
    //retrieve left image
    error = triclopsGetImage(triclops, TriImg_RECTIFIED, TriCam_LEFT, &leftImage);
    //               error = triclopsRectifyColorImage(triclops, TriCam_LEFT, &colorInput,
    //			&leftImage);
    _HANDLE_TRICLOPS_ERROR("triclopsLeftImage()", error);
    if (sprintf(filenameColor, "%s%d%s", "../inputs/raw/left-", v, ".pgm")
            == -1) {
        cout << "Impossible to write left image" << endl;
        exit(EXIT_FAILURE);
    }
    error = triclopsSaveImage(&leftImage, filenameColor);
    _HANDLE_TRICLOPS_ERROR("triclopsSaveLeftImage()", error);
    printf("wrote 'raw/left.pgm'\n");

    //retrieve right image
    error = triclopsGetImage(triclops, TriImg_RECTIFIED, TriCam_RIGHT, &rightImage);
    //              error = triclopsRectifyColorImage(triclops, TriCam_RIGHT, &colorInput,
    //			&rightImage);
    _HANDLE_TRICLOPS_ERROR("triclopsLeftImage()", error);
    if (sprintf(filenameColor, "%s%d%s", "../inputs/raw/right-", v, ".pgm")
            == -1) {
        cout << "Impossible to write right image" << endl;
        exit(EXIT_FAILURE);
    }
    error = triclopsSaveImage(&rightImage, filenameColor);
    _HANDLE_TRICLOPS_ERROR("triclopsSaveRightImage()", error);
    printf("wrote 'raw/right.pgm'\n");
}

void Camera::cleanup_and_exit(dc1394camera_t* camera) {

    dc1394_capture_stop(camera);
    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_camera_free(camera);
    exit(0);

}

void Camera::convertColorTriclopsInput(TriclopsInput* colorInput,
        unsigned char* pucRGB) {
    unsigned char* pucInputData =
            (unsigned char*) colorInput->u.rgb32BitPacked.data;
    for (int i = 0, j = 0; i < colorInput->nrows * colorInput->ncols * 3;) {

        fflush(stdout);

        // get R, G and B
        pucInputData[j + 2] = pucRGB[i++];
        pucInputData[j + 1] = pucRGB[i++];
        pucInputData[j] = pucRGB[i++];
        // increment the Input counter once more to skip the "U" byte
        j += 4;
    }
    return;
}

void Camera::deleteBuff() {
    delete[] pucDeInterlacedBuffer;
    if (pucRGBBuffer)
        delete[] pucRGBBuffer;
    if (pucGreenBuffer)
        delete[] pucGreenBuffer;
}

unsigned char* Camera::getDeInterlacedBuffer() {
    return pucDeInterlacedBuffer;
}

unsigned char* Camera::getRGBBuffer() {
    return pucRGBBuffer;
}

unsigned char* Camera::getGreenBuffer() {
    return pucGreenBuffer;
}

unsigned char* Camera::getRightRGB() {
    return pucRightRGB;
}

unsigned char* Camera::getLeftRGB() {
    return pucLeftRGB;
}

unsigned char* Camera::getCenterRGB() {
    return pucCenterRGB;
}

PGRStereoCamera_t Camera::getStereoCam() {
    return stereoCamera;
}

TriclopsError Camera::getError() {
    return error;
}

TriclopsContext Camera::getTriclops() {
    return triclops;
}

TriclopsInput Camera::getInput() {
    return input;
}

TriclopsInput Camera::getColorInput() {
    return colorInput;
}

TriclopsImage16 Camera::getDepthImage() {
    return depthImage;
}

TriclopsImage Camera::getRectifiedImage() {
    return rectifiedImage;
}

TriclopsColorImage Camera::getRectifiedColor() {
    return rectifiedColor;
}

void Camera::savePointCloud(TriclopsContext triclops, TriclopsImage16 depthImage) {
    float x, y, z;
    int pixelinc;
    int i, j;
    unsigned short * row;
    unsigned short disparity;


    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = depthImage.nrows * depthImage.ncols;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);


    //error = triclopsSaveImage16(&depthImage, "depthImage.ppm");

    // The format for the output file is:
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // ...

    vector<PointXY> points;
    int lastI;

    vector<vector<PointXY> > pointsAllRow;
    vector<PointXY> pointsOneRow;

    int savedPoints = 0;
    // Determine the number of pixels spacing per row
    pixelinc = depthImage.rowinc / 2;
    for (i = 115; i < 180; i++) {
        pointsOneRow.clear();
        row = depthImage.data + i * pixelinc;
        for (j = 0; j < depthImage.ncols; j++) {
            disparity = row[j];

            // do not save invalid points
            if (disparity < 0xFF00) {
                // convert the 16 bit disparity value to floating point x,y,z
                triclopsRCD16ToXYZ(triclops, i, j, disparity, &x, &y, &z);


                /* (Source - Point Grey) By default, all Triclops library XYZ results are in the coordinate system of the reference (right) camera which is defined below:

The origin of the system is the optical center of the lens of the reference camera.
The X axis points to the right of the camera (from the camera's point of view - i.e. looking in the camera view direction).
The Y axis points towards the ground.
The Z axis points forward from the camera.
                 */


                // look at points within a range
                if (y > -0.001 && y < 0.38 && z > 0.1) {
                    //  cout<<i<<endl;
                    //waitHere();
                    cloud.points[savedPoints].x = x*CONVERT_M_TO_MM;
                    cloud.points[savedPoints].y = z*CONVERT_M_TO_MM;
                    cloud.points[savedPoints].z = -y*CONVERT_M_TO_MM;
                    savedPoints++;

                    //points in 2d
                    points.push_back(PointXY(x*CONVERT_M_TO_MM, z * CONVERT_M_TO_MM));
                    pointsOneRow.push_back(PointXY(x*CONVERT_M_TO_MM, z * CONVERT_M_TO_MM));

                    // lastI = i;
                } else {
                    pointsOneRow.push_back(PointXY(-1, -1));
                }
            } else {
                pointsOneRow.push_back(PointXY(-1, -1));
            }
        }
        pointsAllRow.push_back(pointsOneRow);
    }

    vector<PointXY> avgPoints;
    double sumX, sumY, avgX, avgY;
    int totalPointsInAColumn = 0;
    for (unsigned int j = 0; j < 319; j++) {
        sumX = 0;
        sumY = 0;
        totalPointsInAColumn = 0;
        for (unsigned int i = 0; i < pointsAllRow.size(); i++) {
            //cout<<pointsAllRow[i][j].size()<<" ";
            if (pointsAllRow[i][j].getX() != -1 && pointsAllRow[i][j].getY() != -1) {
                sumX += pointsAllRow[i][j].getX();
                sumY += pointsAllRow[i][j].getY();
                totalPointsInAColumn++;
                cout << "i " << i << " sX " << sumX << " sY " << sumY << " ";

            }
        }

        if (totalPointsInAColumn > 5) {
            avgX = sumX / totalPointsInAColumn;
            avgY = sumY / totalPointsInAColumn;
            cout << "sumX " << sumX << " sumY " << sumY << endl;
            cout << "avgX " << avgX << " avgY " << avgY << endl;
            cout << "total points " << totalPointsInAColumn << endl;
            //waitHere();
            avgPoints.push_back(PointXY(avgX, avgY));
        }
    }
    //   waitHere();

    char fileName[50];
    sprintf(fileName, "%s%d%s", "../inputs/pointCloud/pointCloud-", this->getV(), ".pcd");

    pcl::io::savePCDFileASCII(fileName, cloud);
    std::cerr << "Saved " << savedPoints << " data points @pointCloud-" << this->getV() << std::endl;
    //std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

    // plotPointsGNU("../outputs/Maps/points2D-",points);
    //sprintf(fileName, "%s%d%s", "../outputs/pointCloud/points2D-", this->getV(),".png");
    // plotPointsGNU(fileName,points);
    sprintf(fileName, "%s%d", "../inputs/pointCloud/points2D-", this->getV());
    writeASCIIPoints2D(fileName, avgPoints);
    cout << "Points are plotted using GNU." << endl;
}
