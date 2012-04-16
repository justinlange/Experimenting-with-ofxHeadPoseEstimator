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

int PX;
int PY;
int PZ;
int DX;
int DY;
int DZ;
int diffX;
int diffY;

float smoothPZ;

float smoothDiffX;
float smoothDiffY;

float smoothValue = 0.05;


//float posX = 0;
//float posY = 0;
//float posZ = 0;
//float dirX = 0;
//float dirY = 0;
//float dirZ = 0;

float eyePoint = 500;
int defaultVar = 0;



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
	ofScale(1.5, 1.5, -1.5);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}
//--------------------------------------------------------------
void testApp::drawPoses() {
   ofPushMatrix();
// the projected points are 'upside down' and 'backwards'
	ofScale(1, 1, -1);	
    ofTranslate(0, 0, -1000); // center the points a bit
    ofSetColor(0,0,255);
    glLineWidth(3);
   
    if(g_means.size()>0) {
            for(unsigned int i=0;i<g_means.size();++i){
                ofVec3f pos = ofVec3f(g_means[i][0], g_means[i][1], g_means[i][2]);
                ofVec3f dir = ofVec3f(0,0,-150);
                dir.rotate(g_means[i][3], g_means[i][4], g_means[i][5]);
                dir += pos;
                ofLine(pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);
                int posX = int(pos.x);
                int posY = int(pos.y);
                int posZ = int(pos.z);
                int dirX = int(dir.x);
                int dirY = int(dir.y);
                int dirZ = int(dir.z);
                
                PX = posX;
                PY = posY;
                PZ = posZ;
                DX = dirX;
                DY = dirY;
                DZ = dirZ;
                
                
                smoothPZ = ((0.95 * smoothPZ) + (0.05 *PZ));

                
                diffX = PX-DX;
                diffY = PY-DY;
                
                
                smoothDiffX = ((0.85 * smoothDiffX) + (0.15 *diffX));
                smoothDiffY = ((0.85 * smoothDiffY) + (0.15 *diffY));
                
                
                //smoothValue = 0.95f * smoothValue + 0.05f * ard.getAnalog();

                


                
   //               float dd = float(dd);
//               float dd =  ofNormalize(pos.x, -1, 1);
//                printf("PX: %d  PY: %d  PZ: %d DX: %d  DY: %d DZ: %d diffX: %d diffY: %d\n", PX, PY, PZ, DX, DY, DZ, diffX, diffY);
                
                printf("PZ: %d diffX: %d diffY: %d smoothValue: %d\n", PZ, diffX, diffY, int(smoothValue*100));
//                printf("diff x%d dif y %d  dif z%d\n", abs(pos.x)-abs(dir.x), abs(pos.y)-abs(dir.y), abs(pos.z)-abs(dir.z));
            }
        }
	ofPopMatrix();
}
//--------------------------------------------------------------
void testApp::drawReport() {
    ofPushMatrix();
    ofSetColor(0);
    char reportStr[1024];
    ofDrawBitmapString(reportStr, 10, 10);
    ofPopMatrix();
}
//--------------------------------------------------------------
void testApp::draw(){
    
    
    
    ofTranslate(ofGetWindowWidth()/2,ofGetWindowHeight()/2, -PZ); //centers everything
    
    /* we basically need a container for the translation -- translate to the head origin point, and then inside, perform the perspective translations
    */ 
    
    int translateZ = mouseX;
    
//    ofTranslate(0,0,mouseX); //translates to the eyePoint -- needs to be positive
    
    
    ofTranslate(0,0,smoothPZ);
    
    ofRotateY(smoothDiffX/3*PI);
    ofRotateX(smoothDiffY/3*PI);
    
//    ofRotateY(mouseY/(0.5*PI));
//    ofScale(1,1,sqrt(float(ofGetWindowWidth()/2)/mouseX));
    
    ofTranslate(0,0,-smoothPZ); //translates back to the default
 
    
    /* angleDistanceFactor = int(abs(posZ)/angleDistanceAdjuster); 
    // should yield values between 40 and 200
    //  translateDistanceFactor = int(posZ/translateDistanceAdjuster);
    translateDistanceFactor =  ofMap(posZ, 400, 3000, (1/translateDistanceAdjusterMin), (1*translateDistanceAdjusterMax));

   
    
    //for a big distance, we want less translation....
    //that means, we want posX to have less of an effect.
    //so bigger posZ must make posX smaller
    
    ofTranslate((ofGetWindowWidth()/2-(posX*translateDistanceFactor)), (ofGetWindowHeight()/2));

    
//    ofTranslate((ofGetWindowWidth()/2-(posX*translateDistanceFactor)), (ofGetWindowHeight()/2-(posY/translateDistanceFactor)));
    ofRotateX(posY/angleDistanceFactor);
    ofRotateY(posX/angleDistanceFactor);
    ofRotateZ(rotateZvalue);
     */
   
    
    if (bDrawCloud) {
        drawPointCloud();
        drawPoses();
    }

    drawReport();
    //transltae back from eye perspective

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
            
        case 't': smoothValue+=0.01; break;
            
        case 'g': smoothValue-=0.01; break;
            
        case 'r': defaultVar++; break;
       
        case 'f': defaultVar--; break;
            
        case 'h': defaultVar--; break;
        
        case 'j': defaultVar++; break;

            




            


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
