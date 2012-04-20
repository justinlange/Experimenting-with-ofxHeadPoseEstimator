#pragma once

//#include "ofMain.h"
#include "ofxKinect.h"
//#include "time.h"

    
class MeshMaker {    
        
public:
    
    MeshMaker();
    
    void recordTime( ofxKinect* kinect );
    void updateMesh();
    void setupMesh();
    void exit();
    void timeControl();
	
//    ofxKinect *kinect;
    ofImage presentImg;
    ofImage pastImg;
    ofMesh mesh;
    
    int lastTime;
    int recordInterval;
    int currentTime;
    int  numberOfFramesToRecord;
    bool recordingOn;
    bool kinectDisplayEnabled;
    int mostRecentFrame;  //will make the first frame recorded "2"
    ostringstream fileNameToSave;
    int picX; //short for pixelIndexCounterX
    int picY; //short for pixelIndexCounterY
    int numberOfFramesRecorded;
    bool recordReady;
    
    //---------showing the present or past-----------
    
    int timeOffsetFrames;
    int frameToShow;
    int previousFrame;
    string frameResult;
    ostringstream fileNameToLoad;
    int skip;	
    int width;
    int height;
    int startY;
    int startX;
    int endBufferY;
    int endBufferX;



};
