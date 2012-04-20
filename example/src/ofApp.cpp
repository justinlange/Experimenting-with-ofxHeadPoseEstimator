#include "ofApp.h"








void ofApp::setup() {
    
    // void setKinect(ofxKinect * mainKinect);
    // ofxKinect * kinect;
    // kinect->doSomthing();
    
    
    kinect->init();
	kinect->setRegistration(true);
	kinect->open();
  

    

    
//---------mesh stuff-----------
    



    /*
     
     Below, we'll set up our time/millis check
     so that we don't record too many frames. This 
     will become more important once we're recording to
     ssd media and aren't rate limited by the hdd
     
     */
    

}



void ofApp::update() {
    
   
//---------SHOWING the present or past-----------
    
    
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



void ofApp::draw() {
    
	ofBackground(0);
    
    cam.begin();
   
    cam.end();
    
    
    if(kinectDisplayEnabled == true){
        ofSetColor(255, 255, 255);
    	kinect.drawDepth(0, 0, 640, 480);
        kinect.draw(0, 480, 640, 480);
    }

}


//---------recording the present-----------




