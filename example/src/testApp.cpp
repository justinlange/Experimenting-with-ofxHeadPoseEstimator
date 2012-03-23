#include "testApp.h"

using namespace std;
using namespace cv;

// kinect frame width
#define KW 640
// kinect frame height
#define KH 480
// for the average fps calculation
#define FPS_MEAN 30

//-------------- Estimator parameters
// Path to trees
string g_treepath;
// Number of trees
int g_ntrees;
// Patch width
int g_p_width;
// Patch height
int g_p_height;
//maximum distance form the sensor - used to segment the person
int g_max_z;
//head threshold - to classify a cluster of votes as a head
int g_th;
//threshold for the probability of a patch to belong to a head
float g_prob_th;
//threshold on the variance of the leaves
float g_maxv;
//stride (how densely to sample test patches - increase for higher speed)
int g_stride;
//radius used for clustering votes into possible heads
float g_larger_radius_ratio;
//radius used for mean shift
float g_smaller_radius_ratio;
//pointer to the actual estimator
CRForestEstimator* g_Estimate;
//input 3D image
Mat g_im3D;
// estimator trees properly loaded
bool bTreesLoaded = false;
//-------------------------------------------------------
// I copied this code from the library demo code
// this kind of nomenclature is unknown to me
std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
std::vector< std::vector< Vote > > g_clusters; //full clusters of votes
std::vector< Vote > g_votes; //all votes returned by the forest

//-------------------------------------------------------
// kinect motor tilt angle
int kTilt = 0;
// stuff to calculate average fpg
float kFPS = 0;
float avgkFPS = 0;
int frameCount = 0;
int lastMillis = 0;
// draw / hide poincloud
bool bDrawCloud = true;

int rotateXvalue = 0;
int rotateYvalue = 0;
int rotateZvalue = 180;

float posX = 0;
float posY = 0;
float posZ = 0;
float dirX = 0;
float dirY = 0;
float dirZ = 0;

float angleDistanceFactor = 1;
int angleDistanceAdjuster = 1;
float translateDistanceFactor = 1;
int translateDistanceAdjuster = 10;
int translateDistanceAdjusterMin = 10;
int translateDistanceAdjusterMax = 10;



//--------------------------------------------------------------
void testApp::setup(){
    ofSetVerticalSync(true);
	glEnable(GL_DEPTH_TEST);
    // enable depth->rgb image calibration
	kinect.setRegistration(true);
    kinect.init();
    kinect.open();
    kinect.setDepthClipping(500,g_max_z);
    // setup the estimator
    setupEstimator();
}
//--------------------------------------------------------------
void testApp::setupEstimator() {
    // Number of trees
    g_ntrees = 10;
    //maximum distance form the sensor - used to segment the person
    g_max_z = 2000;
    //head threshold - to classify a cluster of votes as a head
    g_th = 500;
    //threshold for the probability of a patch to belong to a head
    g_prob_th = 1.0f;
    //threshold on the variance of the leaves
    g_maxv = 800.f;
    //stride (how densely to sample test patches - increase for higher speed)
    g_stride = 5;
    //radius used for clustering votes into possible heads
    g_larger_radius_ratio = 1.6f;
    //radius used for mean shift
    g_smaller_radius_ratio = 5.f;

    g_im3D.create(KH,KW,CV_32FC3);
    g_Estimate =  new CRForestEstimator();
    g_treepath = ofToDataPath("trees/") + "tree";
    
    if( !g_Estimate->loadForest(g_treepath.c_str(), g_ntrees) ){
		ofLog(OF_LOG_ERROR, "could not read forest!");
        bTreesLoaded = false;
	} else bTreesLoaded = true;
}

//--------------------------------------------------------------
void testApp::update(){
    kinect.update();
    
    printf("angleDistanceAdjuster: %d\n translateDistanceAdjuster: %d\n", angleDistanceAdjuster, translateDistanceAdjuster);
    
    if (kinect.isFrameNew()) {
        calcAvgFPS();
        updateCloud();

        g_means.clear();
        g_votes.clear();
        g_clusters.clear();

        //do the actual estimation
        g_Estimate->estimate( 	g_im3D,
                                g_means,
                                g_clusters,
                                g_votes,
                                g_stride,
                                g_maxv,
                                g_prob_th,
                                g_larger_radius_ratio,
                                g_smaller_radius_ratio,
                                false,
                                g_th
                            );
    }
}
//--------------------------------------------------------------
void testApp::calcAvgFPS() {
    int currMillis = ofGetElapsedTimeMillis();
    avgkFPS += (1000.0/(currMillis-lastMillis))/FPS_MEAN;
    lastMillis = currMillis;
    frameCount++;
    if (frameCount >= FPS_MEAN) {
        kFPS = avgkFPS;
        avgkFPS = frameCount =  0;
    }
}
//--------------------------------------------------------------
void testApp::updateCloud() {
    //generate 3D image
    // I copied part of this code from the library demo code
    // this kind of nomenclature is unknown to me: g_im3D.ptr<Vec3f>(y)
	for(int y = 0; y < g_im3D.rows; y++)
	{
		Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
		for(int x = 0; x < g_im3D.cols; x++){
			ofVec3f thePoint = kinect.getWorldCoordinateAt(x,y);
			
			if ( (thePoint.z < g_max_z) && (thePoint.z > 0) ){
				Mi[x][0] = thePoint.x;
				Mi[x][1] = thePoint.y;
				Mi[x][2] = thePoint.z;
			}
			else
				Mi[x] = 0;
		}
	}
}
//--------------------------------------------------------------
void testApp::drawPointCloud() {
	ofMesh mesh;

	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < KH; y += step) {
		for(int x = 0; x < KW; x += step) {
			if ((kinect.getDistanceAt(x, y) > 0) && (kinect.getDistanceAt(x, y) < g_max_z)) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	ofPushMatrix();
	glPointSize(3);
	// the projected points are 'upside down' and 'backwards'
	ofScale(1.5, -1.5, -1.5);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}
//--------------------------------------------------------------
void testApp::drawPoses() {
//    ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
//	ofScale(1, -1, -1);
//	ofTranslate(0, 0, -1000); // center the points a bit
//    ofSetColor(0,0,255);
//    glLineWidth(3);
    if(g_means.size()>0) {
            for(unsigned int i=0;i<g_means.size();++i){
                ofVec3f pos = ofVec3f(g_means[i][0], g_means[i][1], g_means[i][2]);
                ofVec3f dir = ofVec3f(0,0,-150);
                dir.rotate(g_means[i][3], g_means[i][4], g_means[i][5]);
                dir += pos;
//                ofLine(pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);
                posX = (int) pos.x;
                posY = (int) pos.y;
                posZ = (int) pos.z;
                dirX = (int) dir.x;
                dirY = (int) dir.y;
                dirZ = (int) dir.z;


                
   //               float dd = float(dd);
//               float dd =  ofNormalize(pos.x, -1, 1);
//                printf("posX%d  posY%d  posZ%d\n dirX%d  dirY%d dirZ%d\n", posX, posY, posZ, dirX, dirY, dirZ);
//                printf("diff x%d dif y %d  dif z%d\n", abs(pos.x)-abs(dir.x), abs(pos.y)-abs(dir.y), abs(pos.z)-abs(dir.z));
            }
        }
//	ofPopMatrix();
}
//--------------------------------------------------------------
void testApp::drawReport() {
    ofPushMatrix();
    ofSetColor(0);
    char reportStr[1024];
    sprintf(reportStr, "framecount: %i   FPS: %.2f \n angleDistanceAdjuster: %d \n translateDistanceFactorMin: %d \n translateDistanceFactorMax: %d \n ", frameCount, kFPS, angleDistanceAdjuster, translateDistanceAdjusterMin, translateDistanceAdjusterMax);
    ofDrawBitmapString(reportStr, 10, 10);
    ofPopMatrix();
}
//--------------------------------------------------------------
void testApp::draw(){
    
    // translate(variables)
    // rotate(variables
   
    
    ofPushMatrix();    
    angleDistanceFactor = int(abs(posZ)/angleDistanceAdjuster); // should yield values between 40 and 200
//    translateDistanceFactor = int(posZ/translateDistanceAdjuster);
    translateDistanceFactor =  ofMap(posZ, 400, 3000, (1/translateDistanceAdjusterMin), (1*translateDistanceAdjusterMax));

   
    
    //for a big distance, we want less translation....
    //that means, we want posX to have less of an effect.
    //so bigger posZ must make posX smaller
    
    ofTranslate((ofGetWindowWidth()/2-(posX*translateDistanceFactor)), (ofGetWindowHeight()/2));

    
//    ofTranslate((ofGetWindowWidth()/2-(posX*translateDistanceFactor)), (ofGetWindowHeight()/2-(posY/translateDistanceFactor)));
    ofRotateX(rotateXvalue);
    ofRotateY(posX/angleDistanceFactor);
    ofRotateZ(rotateZvalue);
   
    
//    easyCam.begin();
    if (bDrawCloud) {
        drawPointCloud();
        drawPoses();
    }
//    easyCam.end();

    ofPopMatrix();
    drawReport();


}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    switch (key) {
		case OF_KEY_UP: kTilt += 1; if (kTilt > 30) kTilt = 30; kinect.setCameraTiltAngle(kTilt); break;
        case OF_KEY_DOWN: kTilt -= 1; if (kTilt < -30) kTilt = -30; kinect.setCameraTiltAngle(kTilt); break;
        case 'm': bDrawCloud = !bDrawCloud; break;
                        
        case 'x': rotateXvalue+=10; break;
            
        case 'y': rotateYvalue+=10; break;
            
        case 'z': rotateZvalue+=10; break;
            
        case 't': translateDistanceAdjusterMin++; break;
            
        case 'g': translateDistanceAdjusterMin--; break;
            
        case 'r': translateDistanceAdjusterMax++; break;
       
        case 'f': translateDistanceAdjusterMax--; break;



            


    }
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){

}
