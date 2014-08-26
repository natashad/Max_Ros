#include "ForceEstimator.h"
#define _USE_MATH_DEFINES

//Offset for the force sensor
#define g 0.98
/*
//LPF constants 4th order butterworth centered at 5Hz
#define a1 0.003495
#define b1 1.881765
#define b2 -0.885260
const double B_low[] = {0.0000312389769170710, 0.000124955907668284, 0.000187433861502426, 0.000124955907668284, 0.0000312389769170710};
const double A_low[] = {1.0,-3.58973388711218, 4.85127588251942, -2.92405265616246, 0.663010484385892};

//HPF constants 2nd order butterworth centered at 0.5Hz
#define c1 0.969309
#define c2 -1.938618
#define c3 0.969309
#define d1 1.969070
#define d2 -0.969309
const double B_hi[] = {0.9890, -1.9779, 0.9890};
const double A_hi[] = {1.0000, -1.9778, 0.9780};
*/

//Replace the above two and use one bandpass filter
//9th order bandpass filter centered at 0.43Hz to 8.72Hz
//Designed in matlab with the following command:
//TODO
//>> [n, Wn] = buttord([0.0006 0.06], [0.0003 0.12], 2, 20);
//>> [B, A] = butter(n, Wn);
/*
//This one is poorly chosen, change asap
//[n, Wn] = buttord([0.005 0.075], [0.0025 0.15], 2, 20)
//[B, A] = butter(n, Wn);


const double B[] = { 0.000209065324766031780624395142176297213 , 0.0, -0.000836261299064127122497580568705188853 , 0.0,
					 0.00125439194859619084637669672588344838  , 0.0, -0.000836261299064127122497580568705188853 , 0.0,
					  0.000209065324766031780624395142176297213 };
const double A[] = {1.0,  -7.30673311931724356327322311699390411377 ,  23.389135553881306606172074680216610431671, 
					-42.845005908620073853398935170844197273254,  49.128667204783610600316023919731378555298, -36.111509797980986036236572545021772384644,
					 16.616949988370382840230377041734755039215,  -4.376691321298828540875547332689166069031,   0.505187400318038215552007841324666514993};

*/
/*
//bessel filter now, much better

const double B_bh[] = { 0.996007776488598, -3.98403110595439, 5.97604665893159, -3.98403110595439, 0.996007776488599};
const double A_bh[] = { 1.0, -3.99200185984714, 5.97603299013906, -3.97606035204219, 0.992029221789179};

const double B_bl[] = { 0.000166579901978436, 0.000666319607910637, 0.000999479411877502, 0.000666319607907084, 0.000166579901980213};
const double A_bl[] = { 1.0, -3.25292187235359, 4.00122951546180, -2.20423571645940, 0.458593351782839};
*/
//optimizing for music, want lower delays, using lower order filters
//2nd order butterworth HP filter cutoff at 0.1Hz
const double HP_B[] = {0.978248460365240, -2.93474538109572, 2.93474538109572, -0.978248460365240};
const double HP_A[] = {1.0, -2.95601858871573, 2.91299904400092, -0.956970050205270};

//5th order bessel filters, using different parameters for linear (accelerometer) and angular (gyro)
const double LP_lin_B[] = {0.00377012937874832, 0.0188506468937377, 0.0377012937874897, 0.0377012937874777, 0.0188506468937436, 0.00377012937874834};
const double LP_lin_A[] = {1.0, -2.10700786449859, 2.03032719802362, -1.06633509781897, 0.298985442442492, -0.0353255380286102};

const double LP_rot_B[] = {0.00544654458527871, 0.0272327229263958, 0.0544654458527842, 0.0544654458527893, 0.0272327229263932, 0.00544654458527915};
const double LP_rot_A[] = {1.0, -1.85707495483803, 1.65667465828658, -0.819524444742693, 0.219100882740600, -0.0248867147175389};

double sgn(double x){
	if(x < 0.0){
		return -1.0;
	}else{
		return 1.0;
	}
}

double degToRad(double deg){
	return ((deg * M_PI)/180.0);
}

ForceEstimator::ForceEstimator()
{
	force_pub_channel = nh_.resolveName("ardrone/extForce");
	navdata_sub_channel = nh_.resolveName("ardrone/navdata");
	
	std::string val;
	float valFloat;

	ros::param::get("~minPublishFreq", val);
	if(val.size()>0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = Global::rate;
	minPublishFreq = valFloat;
	std::cout << "set minPublishFreq to " << valFloat << "ms"<< std::endl;

	// channels
	force_pub = nh_.advertise<geometry_msgs::Twist>(force_pub_channel,1);
	navdata_sub = nh_.subscribe(navdata_sub_channel, 10, &ForceEstimator::estimate, this);
	
}

ForceEstimator::~ForceEstimator(){
}


/**
* Every time we get a navdata update from the drone
*/
void ForceEstimator::estimate(const ardrone_autonomy::NavdataConstPtr navdata){
	//Get acceleration from IMU
	Vector3 a;
	a.x = navdata->ax;
	a.y = navdata->ay;
	a.z = navdata->az - g;
	
	//Get velocity, did they just integrate accel to get these?
	Vector3 v;
	v.x = navdata->vx;
	v.y = navdata->vy;
	v.z = navdata->vz;
	
	//Approximate angular accel
	rpy.x = navdata->rotX;
	rpy.y = navdata->rotY;
	rpy.z = navdata->rotZ;
	
	omega.x = min(min(abs(rpy.x - rpy_.x), abs((rpy.x + 360.0) - rpy_.x)), abs(rpy.x - (rpy_.x + 360.0))) * sgn(rpy.x - rpy_.x) * Global::rate;
	omega.y = min(min(abs(rpy.y - rpy_.y), abs((rpy.y + 360.0) - rpy_.y)), abs(rpy.y - (rpy_.y + 360.0))) * sgn(rpy.y - rpy_.y) * Global::rate;
	omega.z = min(min(abs(rpy.z - rpy_.z), abs((rpy.z + 360.0) - rpy_.z)), abs(rpy.z - (rpy_.z + 360.0))) * sgn(rpy.z - rpy_.z) * Global::rate;
	
	Vector3 alpha;
	alpha.x = (omega.x - omega_.x) * Global::rate;
	alpha.y = (omega.y - omega_.y) * Global::rate;
	alpha.z = (omega.z - omega_.z) * Global::rate;
	
	rpy_.x = rpy.x;
	rpy_.y = rpy.y;
	rpy_.z = rpy.z;
	omega_.x = omega.x;
	omega_.y = omega.y;
	omega_.z = omega.z;
	
	//Height measured by the sonars and stuff, do we need it for anything?
	double z = navdata->altd;
	
	//Next do some signal processing to make the data less noisy
	//Shift previous data one up
	for(int i = N-1; i > 0; i--){
		inputs[i] = inputs[i-1];
		outputs[i] = outputs[i-1];
		outputs2[i] = outputs2[i-1];
	}
	
	//The input at this time step
	inputs[0].linear = a;
	inputs[0].angular = alpha;
	
	
	outputs[0].linear.x = 0;
	outputs[0].linear.y = 0;
	outputs[0].linear.z = 0;
	outputs[0].angular.x = 0;
	outputs[0].angular.y = 0;
	outputs[0].angular.z = 0;
	
	//Lowpass
	for(int i = 0; i < 6; i++){
		if(i != 0){
			outputs[0].linear.x += -LP_lin_A[i]*outputs[i].linear.x + LP_lin_B[i]*inputs[i].linear.x;
			outputs[0].linear.y += -LP_lin_A[i]*outputs[i].linear.y + LP_lin_B[i]*inputs[i].linear.y;
			outputs[0].linear.z += -LP_lin_A[i]*outputs[i].linear.z + LP_lin_B[i]*inputs[i].linear.z;
			outputs[0].angular.x += -LP_rot_A[i]*outputs[i].angular.x + LP_rot_B[i]*inputs[i].angular.x;
			outputs[0].angular.y += -LP_rot_A[i]*outputs[i].angular.y + LP_rot_B[i]*inputs[i].angular.y;
			outputs[0].angular.z += -LP_rot_A[i]*outputs[i].angular.z + LP_rot_B[i]*inputs[i].angular.z;
		}else{
			outputs[0].linear.x += LP_lin_B[i]*inputs[i].linear.x;
			outputs[0].linear.y += LP_lin_B[i]*inputs[i].linear.y;
			outputs[0].linear.z += LP_lin_B[i]*inputs[i].linear.z;
			outputs[0].angular.x += LP_rot_B[i]*inputs[i].angular.x;
			outputs[0].angular.y += LP_rot_B[i]*inputs[i].angular.y;
			outputs[0].angular.z += LP_rot_B[i]*inputs[i].angular.z;
		}
	}
	
	//force = outputs[0];
	
	/*
	//Second order IIR lowpass filter with cutoff frequency at 5Hz
	outputs[0].linear.x = 0;
	outputs[0].linear.y = 0;
	outputs[0].linear.z = 0;
	outputs[0].angular.x = 0;
	outputs[0].angular.y = 0;
	outputs[0].angular.z = 0;
	
	for(int i = 0; i < 5; i++){
		if(i != 0){
			outputs[0].linear.x += -A_low[i]*outputs[i].linear.x + B_low[i]*inputs[i].linear.x;
			outputs[0].linear.y += -A_low[i]*outputs[i].linear.y + B_low[i]*inputs[i].linear.y;
			outputs[0].linear.z += -A_low[i]*outputs[i].linear.z + B_low[i]*inputs[i].linear.z;
			outputs[0].angular.x += -A_low[i]*outputs[i].angular.x + B_low[i]*inputs[i].angular.x;
			outputs[0].angular.y += -A_low[i]*outputs[i].angular.y + B_low[i]*inputs[i].angular.y;
			outputs[0].angular.z += -A_low[i]*outputs[i].angular.z + B_low[i]*inputs[i].angular.z;
		}else{
			outputs[0].linear.x += B_low[i]*inputs[i].linear.x;
			outputs[0].linear.y += B_low[i]*inputs[i].linear.y;
			outputs[0].linear.z += B_low[i]*inputs[i].linear.z;
			outputs[0].angular.x += B_low[i]*inputs[i].angular.x;
			outputs[0].angular.y += B_low[i]*inputs[i].angular.y;
			outputs[0].angular.z += B_low[i]*inputs[i].angular.z;
		}
	}
	*/
	
	//Highpass
	outputs2[0].linear.x = 0;
	outputs2[0].linear.y = 0;
	outputs2[0].linear.z = 0;
	outputs2[0].angular.x = 0;
	outputs2[0].angular.y = 0;
	outputs2[0].angular.z = 0;
	
	for(int i = 0; i < 4; i++){
		if(i != 0){
			outputs2[0].linear.x += -HP_A[i]*outputs2[i].linear.x + HP_B[i]*outputs[i].linear.x;
			outputs2[0].linear.y += -HP_A[i]*outputs2[i].linear.y + HP_B[i]*outputs[i].linear.y;
			outputs2[0].linear.z += -HP_A[i]*outputs2[i].linear.z + HP_B[i]*outputs[i].linear.z;
			outputs2[0].angular.x += -HP_A[i]*outputs2[i].angular.x + HP_B[i]*outputs[i].angular.x;
			outputs2[0].angular.y += -HP_A[i]*outputs2[i].angular.y + HP_B[i]*outputs[i].angular.y;
			outputs2[0].angular.z += -HP_A[i]*outputs2[i].angular.z + HP_B[i]*outputs[i].angular.z;
		}else{
			outputs2[0].linear.x += HP_B[i]*outputs[i].linear.x;
			outputs2[0].linear.y += HP_B[i]*outputs[i].linear.y;
			outputs2[0].linear.z += HP_B[i]*outputs[i].linear.z;
			outputs2[0].angular.x += HP_B[i]*outputs[i].angular.x;
			outputs2[0].angular.y += HP_B[i]*outputs[i].angular.y;
			outputs2[0].angular.z += HP_B[i]*outputs[i].angular.z;
		}
	}
	
	force = outputs2[0];
	
	
	
	//Now publish it
	publish();
}

/**
* publish the estimated force on a ros message
*/
void ForceEstimator::publish(){
	force_pub.publish(force);
}