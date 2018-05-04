#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <boost/thread/mutex.hpp>

#define SIZE_OF_FILTER 2

//Antropometric parameters
#define a000 0.1 //|
#define a111 0.2 //|-> Leg width (min-max)
#define b000 0   //  |
#define b111 0.4 //  |-> Free space between two legs (min-max)
#define c000 0.1 //    |
#define c111 0.4 //    |-> Two legs together width (min-max)

// Pattern Type
#define SEPARATED_LEGS 1 // Legs separated
#define DEPHASED_LEGS 2 // Legs dephased
#define TOGETHER_LEGS 3 // Legs together

static const std::string OPENCV_IMAGE = "Image window";

using namespace std;

bool sensor_on   = false;

vector < double > x_pose;
vector < double > y_pose;

sensor_msgs::ImageConstPtr& SensorMsg
boost::mutex mutex;

void ImageFilter_Mean( vector <double> *vector_r, unsigned size );
void FindPattern( string str, string pattern, list <int> *element_found );
void CheckPattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> camera_x, vector <double> camera_y);
double Calculate2D_distance( double x0, double y0, double x1, double y1 );
void Human_Position( vector <double> *r_x, vector <double> *r_y, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> camera_x, vector <double> camera_y );

int main(int argc, char **argv){

  ros::init(argc, argv, "leg_detector");
  ros::NodeHandle n;
  // Publish human found
  ros::Publisher  node_pub = n.advertise <geometry_msgs::PoseArray>("leg_detector", 2);

  // get param from launch file
  string image_scan = "/scan";
  ros::param::get("~image_scan", image_scan);
  ros::Subscriber node_sub = n.subscribe(image_scan, 2, ImageCallback);
  geometry_msgs::PoseArray msgx;
  ros::Rate loop_rate(15);


  int countS = 0;
  
  while( ros::ok() ){
    if( sensor_on == true ){
      // Copying to PoseArray data structure
      vector < geometry_msgs::Pose > PoseVector;
      for( int K = 0; K < x_pose.size(); K++ ){
	geometry_msgs::Point HumanPoint;
	geometry_msgs::Quaternion HumanQuaternion;
	
	HumanPoint.x = x_pose[ K ];
	HumanPoint.y = y_pose[ K ];
	HumanPoint.z = 0; 
	
	HumanQuaternion.x = 0;
	HumanQuaternion.y = 0;
	HumanQuaternion.z = 0;
	HumanQuaternion.w = 1;

	geometry_msgs::Pose Human_Position;
	Human_Position.position = HumanPoint;
	Human_Position.orientation= HumanQuaternion;
	PoseVector.push_back( Human_Position );
      }

      // Header config
      msgx.header.stamp = ros::Time::now();
      msgx.header.frame_id = SensorMsg.header.frame_id;
      msgx.header.seq = countS;
      msgx.poses = PoseVector;
      //------------------------------------------

      node_pub.publish( msgx );
    }
    ros::spinOnce();
    loop_rate.sleep();
    countS++;
  }

  return 0;
}


void ImageCallback (const sensor_msgs::ImageConstPtr& msg){

  SensorMsg = *msg;
  x_pose.clear(); 
  y_pose.clear(); 
  
  sensor_on = true;
  
  double px, py, pr, pt;
  vector < double >  camera_x;
  vector < double >  camera_y;
  vector < double >  camera_r;
  vector < double >  camera_t;
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    pr = msg->ranges[ i ];
    pt = msg->angle_min + ( i * msg->angle_increment);
    camera_r.push_back( pr );
    camera_t.push_back( pt );
  }
 
  string str_aux = "";
    
  // PATTERN RECOGNITION
  string LEGS_LA  = "BSBS";
  string LEGS_FS1 = "BBS";
  string LEGS_FS2 = "BSS";
  string LEGS_SL = "BS";
  
  list <int> Pattern_LA;
  list <int> Pattern_FS1;
  list <int> Pattern_FS2;
  list <int> Pattern_SL;
 
  FindPattern( flank_string, LEGS_LA,  &Pattern_LA  );
  FindPattern( flank_string, LEGS_FS1, &Pattern_FS1 );
  FindPattern( flank_string, LEGS_FS2, &Pattern_FS2 );
  FindPattern( flank_string, LEGS_SL,  &Pattern_SL  );  

  // ANTROPOMETRIC VALIDATION (the non antropometric patterns are erased from the list)
  CheckPattern( &Pattern_LA,  SEPARATED_LEGS, flank_id0, flank_id1,  camera_x, camera_y);
  CheckPattern( &Pattern_FS1, DEPHASED_LEGS, flank_id0, flank_id1,  camera_x, camera_y);
  CheckPattern( &Pattern_FS2, DEPHASED_LEGS, flank_id0, flank_id1,  camera_x, camera_y);
  CheckPattern( &Pattern_SL,  TOGETHER_LEGS, flank_id0, flank_id1,  camera_x, camera_y);

  boost::mutex::scoped_lock lock(mutex);
  //CENTROID PATTERN COMPUTATION & UNCERTAINTY
  x_pose.clear();
  y_pose.clear();
  
  Human_Position( &x_pose, &y_pose, Pattern_LA,  SEPARATED_LEGS,  flank_id0, flank_id1,  camera_x, camera_y);
  Human_Position( &x_pose, &y_pose, Pattern_FS1, DEPHASED_LEGS,  flank_id0, flank_id1,  camera_x, camera_y);
  Human_Position( &x_pose, &y_pose, Pattern_FS2, DEPHASED_LEGS,  flank_id0, flank_id1,  camera_x, camera_y);
  Human_Position( &x_pose, &y_pose, Pattern_SL,  TOGETHER_LEGS,  flank_id0, flank_id1,  camera_x, camera_y);
}


// Mean value of the 'size' adjacent values
void ImageFilter_Mean( vector <double> *vector_r, unsigned size ){
  for( unsigned i = 0; i < ( (*vector_r).size() - size ); i++ ){
      double mean = 0;
      for( unsigned k = 0; k < size; k++  ){
	mean += (*vector_r)[ i + k ];
      }
      (*vector_r)[ i ] = mean / size;
  }
}


// Reports a found string pattern in a list
void FindPattern( string str, string pattern, list <int> *element_found ){
  size_t found = 0;

  while( string::npos != ( found = str.find( pattern, found ) ) ){
    (*element_found).push_back( found ); 
    found++;
  }
  
} 

// Performs the antropometric validation of the leg patterns
void CheckPattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> camera_x, vector <double> camera_y){
  
  double ANTRO_a_1, ANTRO_a_2, ANTRO_b, ANTRO_c; // Antropometric values from patterns to compare with constants.
  bool SavePattern = true;
  bool cond_a = true, cond_b = true, cond_c = true;
  list<int>::iterator it;
  
  for( it = (*Pattern_list).begin(); it != (*Pattern_list).end(); it++ ){

    // Obtain antropometric values
    switch( TYPE ){
      case SEPARATED_LEGS: //BSBS
	ANTRO_a_1 = Calculate2D_distance( camera_x[ flank_id1[ *it ] ], camera_y[ flank_id1[ *it ] ], camera_x[ flank_id0[ *it + 1 ] ], camera_y[ flank_id0[ *it + 1 ] ]);
	ANTRO_a_2 = Calculate2D_distance( camera_x[ flank_id1[ *it + 2 ] ], camera_y[ flank_id1[ *it + 2 ] ], camera_x[ flank_id0[ *it + 3 ] ], camera_y[ flank_id0[ *it + 3 ] ]);
	ANTRO_b = Calculate2D_distance( camera_x[ flank_id0[ *it + 1 ] ], camera_y[ flank_id0[ *it + 1 ] ], camera_x[ flank_id1[ *it + 2 ] ], camera_y[ flank_id1[ *it + 2 ] ] );
	ANTRO_c = 0;
	cond_a = ( ( ANTRO_a_1 >= a000 ) && ( ANTRO_a_1 <= a111 ) ) && ( ( ANTRO_a_2 >= a000 ) && ( ANTRO_a_2 <= a111 ) );
	cond_b = ( ( ANTRO_b >= b000 ) && ( ANTRO_b <= b111 ) );
	cond_c = true;
        break;
      case DEPHASED_LEGS: // BBS & BSS
	ANTRO_a_1 = Calculate2D_distance( camera_x[ flank_id1[ *it ] ], camera_y[ flank_id1[ *it ] ], camera_x[ flank_id0[ *it + 1 ] ], camera_y[ flank_id0[ *it + 1 ] ]);
	ANTRO_a_2 = Calculate2D_distance( camera_x[ flank_id1[ *it + 1 ] ], camera_y[ flank_id1[ *it + 1 ] ], camera_x[ flank_id0[ *it + 2 ] ], camera_y[ flank_id0[ *it + 2 ] ]);
	ANTRO_b = Calculate2D_distance( camera_x[ flank_id0[ *it + 1 ] ], camera_y[ flank_id0[ *it + 1 ] ], camera_x[ flank_id1[ *it + 1 ] ], camera_y[ flank_id1[ *it + 1 ] ] );
	ANTRO_c = 0;
	cond_a = ( ( ANTRO_a_1 >= a000 ) && ( ANTRO_a_1 <= a111 ) ) && ( ( ANTRO_a_2 >= a000 ) && ( ANTRO_a_2 <= a111 ) );
	cond_b = ( ( ANTRO_b >= b000 ) && ( ANTRO_b <= b111 ) );
	cond_c = true;
        break;
    case TOGETHER_LEGS: // BS
      	ANTRO_a_1 = 0;
	ANTRO_a_2 = 0;
	ANTRO_b = 0;
	ANTRO_c = Calculate2D_distance( camera_x[ flank_id1[ *it ] ], camera_y[ flank_id1[ *it ] ], camera_x[ flank_id0[ *it + 1 ] ], camera_y[ flank_id0[ *it + 1 ] ]);
	cond_a = true;
	cond_b = true;	
	cond_c = ( ( ANTRO_c >= c000 ) && ( ANTRO_c <= c111 ) );
	break;
    }

    SavePattern = cond_a && cond_b && cond_c;
    
    if( !SavePattern ){
      it = (*Pattern_list).erase( it );
      it--;
    }
  }  
}


// Euclidean distance between two coordinate points
double Calculate2D_distance( double x0, double y0, double x1, double y1 ){
  return sqrt( pow( x0 - x1, 2 ) + pow( y0 - y1, 2 ) );
}


void Human_Position( vector <double> *r_x, vector <double> *r_y, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> camera_x, vector <double> camera_y ){
  
  double c_x, c_y;
  int l1, l2, l3, l4;
  int count; 
  list<int>::iterator it;

  for( it = Pattern_list.begin(); it != Pattern_list.end(); it++ ){
    c_x = 0;
    c_y = 0;
    count = 0;

    l1 = flank_id1[ *it ];
    l2 = flank_id0[ *it + 1 ];
    
    switch( TYPE ){
    case SEPARATED_LEGS:
      l3 = flank_id1[ *it + 2 ];
      l4 = flank_id0[ *it + 3 ];
      break;
    case DEPHASED_LEGS:
      l3 = flank_id1[ *it + 1 ];
      l4 = flank_id0[ *it + 2 ];
      break;
    case TOGETHER_LEGS:
      l3 = 1;
      l4 = 0;
      break;
    }

    for( int i = l1; i <= l2; i++ ){
      c_x += camera_x[ i ];
      c_y += camera_y[ i ];
      count++;
    }
    for( int i = l3; i <= l4; i++ ){
      c_x += camera_x[ i ];
      c_y += camera_y[ i ];
      count++;
    }
    
    c_x /= (double) count;
    c_y /= (double) count;
    
    (*r_x).push_back( c_x );
    (*r_y).push_back( c_y );
  }
}
