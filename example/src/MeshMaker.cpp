//
//  MeshMaker.cpp
//  ofxKinectExample
//
//  Created by Justin Lange on 4/16/12.
//  Copyright (c) 2012 Magic Hat Films. All rights reserved.
//

#include <iostream>
#include "MeshMaker.h"
#include "math.h"
#include <string>
#include <sstream>
#include <iostream> 
 

using namespace std; 

//---------creating a mesh out of the present or past-----------

void addFace(ofMesh& mesh, ofVec3f a, ofVec3f b, ofVec3f c) {
	mesh.addVertex(a);
	mesh.addVertex(b);
	mesh.addVertex(c);
}

void addFace(ofMesh& mesh, ofVec3f a, ofVec3f b, ofVec3f c, ofVec3f d) {
	addFace(mesh, a, b, c);
	addFace(mesh, a, c, d);
}

void addTexCoords(ofMesh& mesh, ofVec2f a, ofVec2f b, ofVec2f c) {
	mesh.addTexCoord(a);
	mesh.addTexCoord(b);
	mesh.addTexCoord(c);
}

void addTexCoords(ofMesh& mesh, ofVec2f a, ofVec2f b, ofVec2f c, ofVec2f d){
	addTexCoords(mesh, a, b, c);
	addTexCoords(mesh, a, c, d);
}

ofVec3f getVertexFromImg(ofImage& pastImg, int x, int y) {   
	ofColor color = pastImg.getColor(x, y);
	if(color.a > 0) {
		float z = ofMap(color.a, 0, 255, -480, 480);
        // this maps this to -480, 480
		return ofVec3f(x - pastImg.getWidth() / 2, y - pastImg.getHeight() / 2, z);
	} else {
		return ofVec3f(0, 0, 0);
	}
}



MeshMaker::MeshMaker() {
    
    /*
    //---------recording the present-----------
    
    int  numberOfFramesToRecord = 320;
    bool recordingOn = false;
    bool kinectDisplayEnabled = false;
    int mostRecentFrame = 2;  //will make the first frame recorded "2"
    ostringstream fileNameToSave;
    string path = "/Volumes/untitled/"; 
    int lastTime = 0;
    int recordInterval = 30;
    int currentTime = 0; 
    int picX=0; //short for pixelIndexCounterX
    int picY=0; //short for pixelIndexCounterY
    int numberOfFramesRecorded = 0;
    bool recordReady = false;
    
    //---------showing the present or past-----------
    
    int timeOffsetFrames = 0;
    int frameToShow = 2;
    int previousFrame = 0;
    string frameResult;
    ostringstream fileNameToLoad;
    int skip;	
    int width;
    int height;
    int startY = 0;
    int startX = 0;
    int endBufferY = 0;
    int endBufferX = 0;
     
     */
    
    
    //---------recording the present-----------
    
    numberOfFramesToRecord = 320;
    recordingOn = false;
    kinectDisplayEnabled = false;
    mostRecentFrame = 2;  //will make the first frame recorded "2"
    string path = "/Volumes/untitled/"; 
    lastTime = 0;
    recordInterval = 30;
    currentTime = 0; 
    picX=0; //short for pixelIndexCounterX
    picY=0; //short for pixelIndexCounterY
    numberOfFramesRecorded = 0;
    recordReady = false;
    
    //---------showing the present or past-----------
    
    timeOffsetFrames = 0;
    frameToShow = 2;
    previousFrame = 0;
    startY = 0;
    startX = 0;
    endBufferY = 0;
    endBufferX = 0;
    
    
    presentImg.allocate(640, 480, OF_IMAGE_COLOR_ALPHA);
    pastImg.allocate(640, 480, OF_IMAGE_COLOR_ALPHA);
    ofSetVerticalSync(true);
    pastImg.loadImage("1.png"); //this means we'll always have to have one image to start!
    mesh.setMode(OF_PRIMITIVE_TRIANGLES); //rather than points
    skip = 3;	
	width = pastImg.getWidth();
	height = pastImg.getHeight();
	ofVec3f zero(0, 0, 0);
    glEnable(GL_DEPTH_TEST);

    
}



void MeshMaker::updateMesh(int _smoothPZ, int _smoothDiffX, int _smoothDiffY) {
    
    if (ofGetKeyPressed('p') || ofGetKeyPressed('o')){
        printf("we are viewing frame number: %d\n", timeOffsetFrames);
        if(ofGetKeyPressed('p')){
            if(timeOffsetFrames < numberOfFramesRecorded-1){
                timeOffsetFrames = numberOfFramesRecorded;
            }else{
                timeOffsetFrames++;
            }
        }
        if(ofGetKeyPressed('o')){
            if (timeOffsetFrames > 1){
                timeOffsetFrames--;
            }else{
                timeOffsetFrames = 1;
            }
        }
        
        if(ofGetKeyPressed('i')){
            timeOffsetFrames = 0;
        }
    }    

    
    smoothPZ = _smoothPZ; 
    smoothDiffX = _smoothDiffX;
    smoothDiffY = _smoothDiffY;
    
    
    printf("smoothPZ: %d  smoothDiffX: %d  smoothDiffY: %d\n", smoothPZ, smoothDiffX, smoothDiffY);
    
    mesh.clear();

         
    
//    frameToShow = mostRecentFrame - timeOffsetFrames + 1;    
    frameToShow = numberOfFramesRecorded - 1;     
    ostringstream fileNameToLoad;     
    fileNameToLoad << "/Volumes/Untitled/" << frameToShow << ".png";     
    frameResult = fileNameToLoad.str(); 
    pastImg.loadImage(ofToString(frameResult));
    //                pastImg.loadImage(1.png);
    
    
    
    
    for(int y = startY; y < height - endBufferY - skip; y += skip) {        
        for(int x = startX; x < width - endBufferX - skip; x += skip) {
            
            /* 
             this is kind of like quadrants
             ofVec3f short for oF vector w/ 3 floats
             vector in c++ could be (1) coming from stl or (2)
             math -- a direction -- in either 2D or 3D space...
             */
            
            ofVec2f nwi (x,y);
            ofVec2f nei (x+skip, y);
            ofVec2f sei (x+skip, y+skip);
            ofVec2f swi (x, y+skip);
            
            ofVec3f nw = getVertexFromImg(pastImg, x, y);
            ofVec3f ne = getVertexFromImg(pastImg, x + skip, y);
            ofVec3f sw = getVertexFromImg(pastImg, x, y + skip);
            ofVec3f se = getVertexFromImg(pastImg, x + skip, y + skip);
            
            /*
             check for bad data i.e. making sure that nothing 
             is zero, otherwise vertices point to front of screen
             */
            
            if(nw != 0 && ne != 0 && sw != 0 && se != 0) {                 
                addTexCoords(mesh, nwi, nei, sei, swi);
                addFace(mesh, nw, ne, se, sw);                  
            }
        }
    }
    
    ofPushMatrix();   
    
    ofScale(1, 1, -1); // "make y point down" I still don't understand what this means
    ofTranslate(0,0,smoothPZ);
    ofRotateY(smoothDiffX/3*PI);
    ofRotateX(smoothDiffY/3*PI);
    ofTranslate(0,0,-smoothPZ); //translates back to the default
    pastImg.bind();
    mesh.draw();  //
    pastImg.unbind();
   
    ofPopMatrix();
    
}  
        

void MeshMaker::timeControl(){
    
    if(ofGetKeyPressed('q')) {
        
        printf("we are resetting the variables");

        mostRecentFrame = 2;  //will make the first frame recorded "2"
        numberOfFramesToRecord = 320;
        recordingOn = false;
        lastTime = 0;
        recordInterval = 30;
        currentTime = 0; 
        picX=0; //short for pixelIndexCounterX
        picY=0; //short for pixelIndexCounterY
        numberOfFramesRecorded = 0;
        recordReady = false;
    
    //---------showing the present or past-----------
    
        timeOffsetFrames = 0;
        frameToShow = 2;
        previousFrame = 0;
    
  } 
}
    
    




void MeshMaker::recordTime ( ofxKinect* kinect ) {
    
    
    if(ofGetKeyPressed('z')){
        printf("mostRecentFrame: %d numberOfFramesToRecord: %d frameToShow: %d numberOfFramesRecorded:  %d currentTime: %d lastTime: %d \n",mostRecentFrame,numberOfFramesToRecord, frameToShow, numberOfFramesRecorded, currentTime, lastTime);

    }
    
    currentTime = ofGetElapsedTimeMillis();
        
    if(ofGetKeyPressed(' ')) {
       recordingOn = true;
        printf("recordingOn has been set to true");
    }
    
    
    if(ofGetKeyPressed('s')){
        recordingOn = false;
        printf("recordingOn has been set to false");

    }
        
    if (recordingOn == true){
        if(currentTime > (lastTime + recordInterval)) {
            lastTime = currentTime;

            if (mostRecentFrame < numberOfFramesToRecord){
                mostRecentFrame = mostRecentFrame + 1;

                printf("we are in loop to record depth frames");

                ofPixels& depthPixels = kinect->getDepthPixelsRef();
                ofPixels& colorPixels = kinect->getPixelsRef();
                picX = 0;
                    
for(int x = 0; x < 640; x++) {
        picX++;
        picY=0;
    for(int y = 0; y < 480; y++) {
        picY++;                
        ofColor color = colorPixels.getColor(x, y);
        ofColor depth = depthPixels.getColor(x, y);
        presentImg.setColor(picX, picY, ofColor(color, depth.getBrightness()));
                }
            }
                    ostringstream fileNameToSave;
                    fileNameToSave << "/Volumes/Untitled/" << mostRecentFrame << ".png";
                    string result = fileNameToSave.str();
                    presentImg.saveImage(result);
                    numberOfFramesRecorded++;
                }
            }
        }         
    }    





