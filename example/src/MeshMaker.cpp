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
    
    //---------recording the present-----------
    
    int  numberOfFramesToRecord = 320;
    bool recordingOn = false;
    bool kinectDisplayEnabled = false;
    int mostRecentFrame = 2;  //will make the first frame recorded "2"
    ostringstream fileNameToSave;
    int lastTime = 0;
    int recordInterval = 100;
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
    
    
    presentImg.allocate(640, 320, OF_IMAGE_COLOR_ALPHA);
    pastImg.allocate(640, 320, OF_IMAGE_COLOR_ALPHA);
    ofSetVerticalSync(true);
    pastImg.loadImage("1.png"); //this means we'll always have to have one image to start!
    mesh.setMode(OF_PRIMITIVE_TRIANGLES); //rather than points
    skip = 3;	
	width = pastImg.getWidth();
	height = pastImg.getHeight();
	ofVec3f zero(0, 0, 0);
    glEnable(GL_DEPTH_TEST);

    
}



void MeshMaker::updateMesh() {
    
    ofScale(1, -1, 1); // "make y point down" I still don't understand what this means
    pastImg.bind();
    mesh.draw();  //
    pastImg.unbind();
    
    
}

void MeshMaker::timeControl(){
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
} 
    
    




void MeshMaker::recordTime ( ofxKinect* kinect ) {
    
    
    if(ofGetKeyPressed('z')){
        printf("mostRecentFrame: %d numberOfFramesToRecord: %d mostRecentFrame: %d frameToShow: %d numberOfFramesRecorded:  %d currentTime: %d lastTime: %d \n",mostRecentFrame,numberOfFramesToRecord, mostRecentFrame, frameToShow, numberOfFramesRecorded, currentTime, lastTime);

    }
    
    currentTime = ofGetElapsedTimeMillis();
        
    if(ofGetKeyPressed(' ')) {
       recordingOn = true;
                
        
    }
    if(ofGetKeyPressed('s')){
        recordingOn = false;
    }
        
    if (recordingOn == true){
        if(currentTime > lastTime + recordInterval) {
            lastTime = currentTime;
            if (mostRecentFrame < numberOfFramesToRecord){
                mostRecentFrame = mostRecentFrame + 1;

                ofPixels& depthPixels = kinect->getDepthPixelsRef();
                ofPixels& colorPixels = kinect->getPixelsRef();
                picX = 0;
                    
                    for(int x = 0; x < 640; x=x+2) {
                        picX++;
                        picY=0;
                        for(int y = 0; y < 480; y=y+2) {
                            picY++;                
                            ofColor color = colorPixels.getColor(x, y);
                            ofColor depth = depthPixels.getColor(x, y);
                            presentImg.setColor(picX, picY, ofColor(color, depth.getBrightness()));
                        }
                    }
                    ostringstream fileNameToSave;
                    fileNameToSave << mostRecentFrame << ".png";
                    string result = fileNameToSave.str();
                    presentImg.saveImage(result);
                    numberOfFramesRecorded++;
                }
                
                frameToShow = mostRecentFrame - timeOffsetFrames + 1;
                ostringstream fileNameToLoad;     
                fileNameToLoad << frameToShow << ".png";     
                frameResult = fileNameToLoad.str(); 
                pastImg.loadImage(ofToString(frameResult));
//                pastImg.loadImage(1.png);

                
                mesh.clear();

                
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
            }
        }         
    }    





