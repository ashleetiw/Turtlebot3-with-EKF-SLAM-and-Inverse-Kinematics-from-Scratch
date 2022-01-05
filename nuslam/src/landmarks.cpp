/// \file
/// \brief Interprets LaserScan data and detects and publishes attributes of discovered landmarks
///
/// PARAMETERS:
///   threshold (double): used to determine whether two points from LaserScan belong to one cluster
///   callback_flag (bool): specifies whether to publish landmarks based on callback trigger
///   pc (sensor_msgs::PointCloud): contains interpreted pointcloud which is published for debugging purposes
///   map (nuslam::TurtleMap): stores lists of x,y coordinates and radii of detected landmarks
///   frequency (double): frequency of control loop.
///   frame_id_ (string): frame ID of discovered landmarks (in this case, relative to base_scan)
///
/// PUBLISHES:
///   landmarks (nuslam::TurtleMap): publishes TurtleMap message containing landmark coordinates (x,y) and radii
///   pointcloud (sensor_msgs::PointCloud): publishes PointCloud for visualization in RViz for debugging purposees
///
/// SUBSCRIBES:
///   /scan (sensor_msgs::LaserScan), which contains data with which it is possible to extract range,bearing measurements
///
/// FUNCTIONS:
///   scan_callback (void): callback for /scan subscriber, which processes LaserScan data and detects landmarks

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "rigid2d/rigid2d.hpp"
#include <cmath>
#include <armadillo>
#include <visualization_msgs/MarkerArray.h>

std::vector<rigid2d::Vector2D> coords;
std::vector<double> radius;
std::vector<double> array;

rigid2d::Vector2D fit_circle(std::vector<rigid2d::Vector2D> arr,std::vector<rigid2d::Vector2D> coords,std::vector<double> radius){
//         std::vector<rigid2d::Vector2D> coords;
// std::vector<double> radius;
        
        double x=0;
        double y=0;
        double x_centroid=0;
        double y_centroid=0;
        int n = arr.size();
		// Step 1: Compute x,y coordinates of the centroid of n data points
		for (unsigned i=0;i<arr.size();i++){
            x=x+arr[i].x;
            y=y+arr[i].y;
        }
        
        x_centroid=x/n;
        y_centroid=y/n;

     
	    // Step 2: Shift the coordinates so the centroid is at the origin
        // Step 3: compute zi = xi^2 + yi^2
        std::vector<double> z;
        for (unsigned i=0;i<arr.size();i++){
            arr.at(i).x= arr[i].x-x_centroid;
            arr.at(i).y= arr[i].y-y_centroid;
            double zi = pow(arr[i].x, 2) + pow(arr[i].y, 2);
			z.push_back(zi);
        }

           
        // ROS_ERROR_STREAM(z.size());


		// Step 4: Compute mean of z
        double sum;
         for (unsigned i=0;i<z.size();i++){
            sum+=z.at(i);
         }
	    double mean_z = sum/n;
	    // std::cout << "z mean: \n" << mean_z << std::endl;

	    // Step 5: form data matrix from n data points we know the column no which is equal to 4
        arma::mat Z;
	    Z=arma::mat(n,4,arma::fill::zeros);
	    

		// Step 6: Data Matrix M = (1/n) Z.T Z
		auto M = Z.t() * Z /double(n);

		// Step 7: Constraint Matrix H for 'Hyperaccurate algebraic fit'
        arma::mat H;
		H = arma::mat(4,4,arma::fill::zeros);
		// 8.0 * mean_z, 0.0, 0.0, 2.0;
		    // {0.0, 1.0, 0.0, 0.0};
	        // {0.0, 0.0, 1.0, 0.0};
		    // {2.0, 0.0, 0.0, 0.0}};
        H(0,0)= 8.0 * mean_z;
        H(0,3)=2;
        H(1,1)=1;
        H(2,2)=1;
        H(2,0)=2;

		// // Step 8: H Inv
        arma::mat H_inv;
		H_inv = arma::mat(4,4,arma::fill::zeros);
        // H_inv={ {0.0, 0.0, 0.0, 0.5};
		//     {0.0, 1.0, 0.0, 0.0};
	    //     {0.0, 0.0, 1.0, 0.0};
		//     {0.5, 0, 0, - 2.0 * mean_z}};
        H_inv(0,3)=0.5;
        H_inv(1,1)=1;
        H_inv(2,2)=1;
        H_inv(3,0)=0.5;
        H_inv(3,3)=- 2.0 * mean_z;



	
		// // Step 9: Singular Value Decomposition of Z
        arma::mat U;
        arma::mat V;
        arma::vec S;
		arma::svd(U,S,V,Z);

        // ROS_ERROR_STREAM(U.size());
        
        
        arma::mat v=arma::conv_to<arma::mat>::from(V);
        arma::vec A;
        A=arma::vec(4,arma::fill::zeros);

        auto sigma=S(3);
        // Step 10:smallest singular value is less than 
        if (sigma<1e-12){
             A(0)=V(3,0);
             A(1)=V(3,1);
             A(2)=V(3,2);
             A(3)=V(3,3);
        }
        else{
            arma::mat Y;
            Y=V*diagmat(S)*V.t();

           
            
                //  find the eigenvalues and vectors of
            arma::mat Q;
            Q=Y*H_inv*Y;

           
            arma::vec eigval;
            arma::mat eigvec;
            arma::eig_sym(eigval, eigvec, Q);

            //  ROS_ERROR_STREAM(eigval.size());
             
             // find min
            int min=1000;
            int index=0;
            for (unsigned i=0;i<eigval.size();i++){
                if (eigval(i)<min ){
                    min=eigval(i);
                    index=i;
                }
            }
            

            arma::vec Astar;
            Astar=arma::vec(4,arma::fill::zeros);

            Astar(0)=eigvec(0,0); 
            Astar(0)=eigvec(0,1); 
            Astar(0)=eigvec(0,2); 
            Astar(0)=eigvec(0,3); 
            A=arma::solve(Y,Astar);
            
            // ROS_ERROR_STREAM(Y.size());
            // ROS_ERROR_STREAM(A);
		}


            /// if nan means  V(0) is 0 try looking into SVD
		// // Step 12: eqn of circle is (x - a)^2 + (y - b)^2 = R^2
		auto a = -A(1) / (2.0 )+0.05;
		auto b = -A(2) / (2.0)+0.05;
		auto R = sqrt((pow(A(1), 2) + pow(A(2), 2) - (4.0 * A(0) * A(3))) / (4.0 * pow(A(0), 2)));
        //  ROS_ERROR_STREAM("the vec is ");
        
        // ROS_ERROR_STREAM(array[0].y);
		// std::cout << "A MATRIX: \n" << A << std::endl;


//         ///////////////////////////////////////////////////////////////////////////////

		// Step 13: We shifted our coordinate system, so actual centroid is at
		// a + mean_x, b + mean_y
		// Store Cluster Parameters (coords(x,y) and radius)
        rigid2d::Vector2D centre;
		centre.x= a + x_centroid;
		centre.y = b +y_centroid;
	
        // coords.push_back(centre);
        // radius.push_back(R);

// 		// Step 14: Calculate Root-Mean-Squared-Error of the fit
// 	// 	auto sum = 0;
// 	// 	for (auto i = arr.begin(); i != arr.end(); i++)
// 	// 	{
// 	// 		sum += pow((pow(i->pose.x - a, 2) + pow(i->pose.y - b, 2) - pow(R, 2) ), 2);
// 	// 	}

// 	// 	auto rms_err = sqrt(sum / static_cast<double>(n));

// 	// 	return rms_err;
// 	// }
    return centre;
    }



bool detect_circle(std::vector<rigid2d::Vector2D> arr)
	{
		bool is_circle = false;
		// Store endpoints of cluster arc
		rigid2d::Vector2D P1 = arr.at(0);
		rigid2d::Vector2D P2 = arr.back();

		std::vector<double> angles;


         // Using Law of Cosines to find angle
        double L1= sqrt(pow(P1.x - P2.x, 2) + pow(P1.y - P2.y, 2));

        // First, find the three lengths

		for (auto i=1;i<arr.size();i++)
		{
			                       
			double L2 = sqrt(pow(P1.x - arr.at(i).x, 2) + pow(P1.y - arr.at(i).y, 2));
			double L3 = sqrt(pow(P2.x - arr.at(i).x, 2) + pow(P2.y - arr.at(i).y, 2));

			// Next, find angle and append to vector
			double angle = acos((pow(L2, 2) + pow(L3, 2) - pow(L1, 2)) / ( 2.0 * L3*L2));
			angles.push_back(angle);

		}

		// Step 2: compute the mean and standard deviation from angle(1)-angle(end-1)
        double sum=0;
        for (auto i=1;i<angles.size();i++){
            sum+=angles.at(i);
        }
		double mean_angle = sum/static_cast<double>(angles.size());
        
        double total=0;
		for (auto i=1;i<angles.size();i++){
		    total += pow(angles.at(i)- mean_angle, 2);
		};

		double std_dev = sqrt(total / static_cast<double>(angles.size()));

		// Step 3: If std_dev is below 0.15 radians, and the mean_angle is between 90 and 135 degrees, we have a circle
		mean_angle *= 180.0 / rigid2d::PI;

		if (std_dev < 0.5 && mean_angle >= 10.0 && mean_angle <= 170.0)
		{
			is_circle = true;
		}
        else{
            is_circle=false;
        }

		return is_circle;
	}



rigid2d::Vector2D polar_to_car(int angle,int radius){
    rigid2d::Vector2D pose;
	pose.x = radius * cos(angle);
	pose.y = radius * sin(angle);

	return pose;

}

void scan_callback(const sensor_msgs::LaserScan &laser)
{ 
  /// \brief forms clusters from LaserScan data and fits circles to
  /// them before assessing whether or not they are landmarks (or walls)
  /// \param LaserScan, which contains data with radius r [0-360 degree ]RSO_()

   
 
  bool newl=false;
  double threshold=2.9;
  std::vector<rigid2d::Vector2D> arr; 
  std::vector<std::vector<rigid2d::Vector2D>> landmarks;
  std::vector<std::vector<rigid2d::Vector2D>> clean_landmarks;
  std::vector<std::vector<rigid2d::Vector2D>> circle_landmarks;
  // check angle and distance range and only consider data within it 


  int count=0;
   for (long unsigned int i = 0; i < 360; i++)
  {
        if (laser.ranges.at(i) >= 0.2 && laser.ranges.at(i) <= 3.5){

            // convert polar to cartisan 
            rigid2d::Vector2D point;
            point=polar_to_car(i,laser.ranges.at(i));   

                 
        
        
    ///////////// // form clusters based on the distances between them 

            if (arr.empty())  // 1s t point in any cluster 
            {
                arr.push_back(point);
                // ROS_ERROR_STREAM(" in here ");
            }
            else{ 
                // find distance with prev point 
                double abs_x = pow(point.x - arr.back().x, 2);
                double abs_y = pow(point.y - arr.back().y, 2);
                double dist = sqrt(abs_x + abs_y);
                //  ROS_ERROR_STREAM("landmarks");
                // ROS_ERROR_STREAM(dist);
                // ROS_ERROR_STREAM("end");
                if (dist<=threshold){

                    // assign  to the same cluster
                    arr.push_back(point);                
                }
                else{
                    newl=true;
                    // ROS_ERROR_STREAM("dis was");
                    // ROS_ERROR_STREAM(dist);
                }
            }
            if (newl==true){
                // need to form new cluster 
                landmarks.push_back(arr);
                // ROS_ERROR_STREAM("cluster size");
                // ROS_ERROR_STREAM(arr.size());
                arr.clear();
                count=count+1   ;    
   
                newl=false;         
            }
            // std::cout << count << std::endl;
            
        }
    }


    // count=0;
    


    // since laser dtaa is circle check if 0 degee and 360 dregree will be in same cluster 
    std::vector<rigid2d::Vector2D> first_cluster = landmarks.at(0);
    std::vector<rigid2d::Vector2D> last_cluster = landmarks.back();
    // double abs_x = pow(first_cluster.x - last_cluster.x, 2);
    // double abs_y = pow(first_cluster.y - last_cluster.y, 2);
    // double abs_dist = sqrt(abs_x + abs_y);

    // if (dist<=threshold){
    // // Insert Points of last_cluster into first_cluster
    // landmarks.at(0).arr.insert(landmarks.at(0).arr.end(),
    //                                 landmarks.back().arr.begin(),
    //                                 landmarks.back().arr.end());
    // // Pop last_cluster from landmarks
    // landmarks.pop_back();
    // }

// remove small and very large clusters as they cannot be landmarks 
 for (long unsigned int i = 0; i <landmarks.size(); i++)
  {
    // Check if cluster contains less than 3 points
    if (landmarks[i].size() >= 3 && landmarks[i].size()<=50 )      
    {

         clean_landmarks.push_back(landmarks[i]);

  }
  }

//     ROS_ERROR_STREAM("lanmarks");
//    ROS_ERROR_STREAM(clean_landmarks.size());


// /////////////////////////////////////////////////////   circle detection /////////////////////////////////////////////////////////////////

rigid2d::Vector2D centre;
coords.clear();

 bool is_cirlce=false;
 for(unsigned int i = 0;i<array.size();i++)  {
    array[i]=array[i]-0.22;

 }
 // Finally, we perform circle detection for each cluster
  for (auto i = 0; i < clean_landmarks.size(); i++)
  {
    centre=fit_circle(clean_landmarks[i],coords,radius);  // for each landmark[i]
    coords.push_back(centre);
    // ROS_ERROR_STREAM("new cluster");
    // ROS_ERROR_STREAM(landmarks[i].size());
//     if (detect_circle(clean_landmarks[i])==true){
//         circle_landmarks.push_back(clean_landmarks[i]);
//     }
   
  }

// ROS_ERROR_STREAM(" size");
// ROS_ERROR_STREAM(coords[0]);
 



}




int main(int argc, char** argv){
 
  ros::init(argc, argv, "landmarks"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  std::vector<double> array;
  nh.getParam("location",array);


//   double frequency = 60.0;
//   std::string frame_id_ = "base_scan";


  // Init LaserScan Subscriber
  ros::Subscriber laser = nh.subscribe("scan", 1, scan_callback);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("landmarks", 100);

   
    visualization_msgs::MarkerArray marker_array;

  
  

  while (ros::ok())
  {
      ros::spinOnce();
//       ROS_ERROR_STREAM("radius size");
// ROS_ERROR_STREAM(coords;
    
        


        marker_array.markers.resize(coords.size());
        // // ROS_ERROR_STREAM(array.size());
        unsigned int n=coords.size();
        for(unsigned int i = 0;i<n;i++)  {
                    // ROS_ERROR_STREAM(array[i]);
                    marker_array.markers[i].header.frame_id ="world";
                    marker_array.markers[i].header.stamp = ros::Time::now();
                    marker_array.markers[i].lifetime = ros::Duration(1); 
                    marker_array.markers[i].ns = "landmarks";
                    marker_array.markers[i].id = i;

                    marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
                    marker_array.markers[i].action = visualization_msgs::Marker::ADD;

                    marker_array.markers[i].pose.position.x = array[i]-coords[i].x;
                    marker_array.markers[i].pose.position.y =array[i+1]-coords[i].y;
                    marker_array.markers[i].pose.position.z = 0.0;

                    marker_array.markers[i].pose.orientation.x = 0.0;
                    marker_array.markers[i].pose.orientation.y = 0.0;
                    marker_array.markers[i].pose.orientation.z = 0.0;
                    marker_array.markers[i].pose.orientation.w = 1.0;

                    marker_array.markers[i].scale.x = 0.05;
                    marker_array.markers[i].scale.y = 0.05;
                    marker_array.markers[i].scale.z = 1;

                    marker_array.markers[i].color.r = 0.0f;
                    marker_array.markers[i].color.g = 0.0f;
                    marker_array.markers[i].color.b = 1.0f;
                    marker_array.markers[i].color.a = 1.0f;

            }

      marker_pub.publish(marker_array);

  }

  return 0;


}