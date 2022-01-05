#include "nuslam/kalman.hpp"
#include <cmath>
#include <armadillo>
#include "rigid2d/rigid2d.hpp"

nuslam::landmark::landmark(){
  x=0;
  y=0;
  r=0;
  b=0;
}

nuslam::landmark::landmark(double x,double y,double r ,double b){
  x=x;
  y=y;
  r=r;
  b=b;
}


std::mt19937_64 nuslam::getrandom(){
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  return gen;
}

arma::rowvec nuslam::MultivariateDistribution(arma::mat mat, int dim){
  // must be square
  // arma::rowvec col =mat.col(0);
  // int dim= std::size(col);
  arma::rowvec rand_vec=arma::rowvec(dim , arma::fill::zeros);

  for(int i = 0; i < dim; i++){
    std::normal_distribution<double> d(0, 1);
    std::mt19937_64 no=nuslam::getrandom();
    rand_vec(i) = d(no)/1000;
  }
  return rand_vec;
 
}


nuslam::EKF::EKF(int total_landmarks){
  state_size = 3 + 2 * 5;
 //State: [x, y, θ, x_l1, y_l1, ......, x_ln, y_ln]
  state=arma::rowvec(13, arma::fill::zeros);
 
  // init state covariance
  state_cov=arma::mat(13,13,arma::fill::zeros);
    // set pose to (0,0,0)
    state_cov(0,0) = 0;
    state_cov(1,1) = 0;
    state_cov(2,2) = 0;

  // set landmarks to a large number
  for(auto i = 0; i < 2*5; i++)
  {
    auto row = 3+i;
    auto col = 3+i;
    state_cov(row,col) = 1e4;



  }
   
  // // init measurement noise
  R = arma::mat(2,2,arma::fill::zeros);
  R(0,0) = 1e-2;   // r var
  R(1,1) = 1e-2;   // b var


  // // init process noise
  Q=arma::mat(state_size,state_size,arma::fill::zeros);
  Q(0,0) = 1e-3;
  Q(1,1) = 1e-3;
  Q(2,2) = 1e-3;
  // return state;

 
}


// // /***********************  odometry model ******************************/


arma::rowvec nuslam::EKF::odomupdate(rigid2d::Twist2D twist,arma::rowvec state_bar){
    // arma::rowvec w = nuslam::MultivariateDistribution(Q,13);
    
    arma::rowvec w ;
    w=arma::rowvec(state_size ,arma::fill::zeros);
    // state_bar=state;
    state_bar(0)=rigid2d::normalize_angle(state_bar(0));
    // //  x_t  =  x_t-1 + v * cosθ_t-1 * delta_t
    // //  y_t  =  y_t-1 + v * sinθ_t-1 * delta_t
    // // θ_t  =  θ_t-1 + w * delta_t
    
    if (rigid2d::almost_equal(twist.angle_z, 0.0)){       
      state_bar(0)=state_bar(0) ;
      state_bar(1)=state_bar(1) + (twist.linear_vx * cos(state_bar(0))) ;
      state_bar(2)=state_bar(2) + (twist.linear_vx * sin(state_bar(0))) ;
    }

    else{
      state_bar(0)=state_bar(0)+ twist.angle_z ;
      state_bar(1)=state_bar(1) + ((-twist.linear_vx / twist.angle_z) * sin(state_bar(0)) + (twist.linear_vx / twist.angle_z) * sin(state_bar(0) + twist.angle_z)) ;
      state_bar(2)=state_bar(2)+ ((twist.linear_vx / twist.angle_z) * cos(state_bar(0)) + (-twist.linear_vx / twist.angle_z) * cos(state_bar(0) + twist.angle_z)) ;             
    }

    // //  after updating
    state_bar(0)=rigid2d::normalize_angle(state_bar(0));

    return state_bar;

}


arma::mat nuslam::EKF::motion_jacobian(rigid2d::Twist2D twist,arma::mat sigma_bar){
 
      // ------ Linearize state-transition by Jacobian ------#
      // Jacobian of motion: G = d g(u_t, x_t-1) / d x_t-1
      //         1  0  -v * delta_t * sinθ_t-1
      //   G  =  0  1   v * delta_t * cosθ_t-1        0
      //         0  0             1
        //
        //                      0                    I(2n x 2n)

        arma::mat G;
        G=arma::mat(state_size ,state_size ,arma::fill::zeros);

        arma::mat I;
        I=arma::mat(state_size ,state_size , arma::fill::zeros);
        I =I.eye();


      //   //G Jacobian of motion model
      state(0)=rigid2d::normalize_angle(state(0));

        if (rigid2d::almost_equal(twist.angle_z, 0.0)){

            G(1,0) = -twist.linear_vx * sin(state(0));
            G(2,0) = twist.linear_vx * cos(state(0));
    	}      
        else {
   
            G(1, 0) = (-twist.linear_vx / twist.angle_z) * cos(state(0)) + (twist.linear_vx / twist.angle_z) * cos(state(0) + twist.angle_z);
    		    G(2, 0) = (-twist.linear_vx / twist.angle_z) * sin(state(0)) + (twist.linear_vx / twist.angle_z) * sin(state(0)+ twist.angle_z);
            
        }

        G=G+I;
      
        // sigma = G x sigma x G.T +Q
        // predicted covariance 
        sigma_bar=G*state_cov*trans(G)+Q; // Q is process noise 

        return sigma_bar;

}

// // // /***********************  measurement model ******************************/



arma::rowvec nuslam::EKF::measurementupdate(int i,arma::rowvec state_bar){

    // range   =  sqrt((x_l - x_t)^2 + (y_l - y_t)^2)
    //  bearing  =  atan2((y_l - y_t) / (x_l - x_t)) - θ_t

            const auto ix = 2*i + 3;
            const auto iy = 2*i + 4;

            // change in x and y landmark to robot
            const auto delta_x =  state(ix) - state_bar(1);
            const auto delta_y =  state(iy) - state_bar(2);
            // 

            arma::rowvec z_hat;
            z_hat=arma::rowvec(2,arma::fill::zeros);

            arma::rowvec v;
            v=arma::rowvec(2,arma::fill::zeros);
            // v =  nuslam::MultivariateDistribution(R,2);
            // predicted range
            z_hat(0) = sqrt(delta_x * delta_x + delta_y * delta_y);
            // z_hat(0)=state(ix) - state(1);
            // predicted bearing
            z_hat(1) = rigid2d::normalize_angle(std::atan2(delta_y, delta_x) -  rigid2d::normalize_angle(state_bar(0)));

            return z_hat;
}

arma::mat nuslam::EKF::measurement_Jacobian(int i,arma::rowvec state,arma::mat H){

    //  Landmark state becomes a variable in measurement model
        // Jacobian: H = d h(x_t, x_l) / d (x_t, x_l)
        //       1 0 0  0 ...... 0   0 0   0 ...... 0
        //       0 1 0  0 ...... 0   0 0   0 ...... 0
        // F_x =  0 0 1  0 ...... 0   0 0   0 ...... 0
        //       0 0 0  0 ...... 0   1 0   0 ...... 0
        //       0 0 0  0 ...... 0   0 1   0 ...... 0
        //         (2*landmark_idx - 2)
        //         -delta_x/√q  -delta_y/√q  0  delta_x/√q  delta_y/√q
        // H_low =   delta_y/q   -delta_x/q  -1  -delta_y/q  delta_x/q
        //               0            0       0       0          0
        // H = H_low x F_x

    double ix = 2*i + 3;
    double  iy = 2*i + 4;

    // change in x and y landmark to robot
    double dx =  state(ix) - state(1);
    double dy =  state(iy) - state(2);


    double q = dx*dx + dy*dy;
    double sqrt_q = sqrt(q);

    if (sqrt_q!=0 && q!=0){

     // row 1
  H(0,0) = 0.0;
  H(0,1) = -dx / sqrt_q;
  H(0,2) = -dy / sqrt_q;

  H(0,ix) = dx / sqrt_q;
  H(0,iy) = dy / sqrt_q;


  // row 2
  H(1,0) = -1.0;
  H(1,1) = dy / q;
  H(1,2) = -dx / q;

  H(1,ix) = -dy /q;
  H(1,iy) = dx / q;

    }

    return H;

 }



 void nuslam::EKF::changeframe(const std::vector<rigid2d::Vector2D> &landmarks, std::vector<nuslam::landmark> &lm) 
{
  
  
  for(unsigned int i = 0;i<landmarks.size();i++){
    double mx = landmarks.at(i).x;
    double my = landmarks.at(i).y;
    


    // to polar coordinates
    double r = sqrt(mx * mx + my *my);
    double b =  std::atan2(my, mx);

    //  fill the structure landmark with values 
    lm.at(i).r = r;
    lm.at(i).b = b;

    // frame: robot -> map/
    state(0)= rigid2d::normalize_angle(state(0));
    lm.at(i).x = state(1) + r * std::cos(b + state(0));
    lm.at(i).y = state(2) + r * std::sin(b + state(0));
  }
  
  // std::cout << lm.at(0).y << std::endl;
}


arma::rowvec nuslam::EKF::slam(rigid2d::Twist2D twist,std::vector<rigid2d::Vector2D> &landmarks){
      //  // motion model
      //   ///////////////  prediction ///////////////////


    arma::rowvec state_bar;
    state_bar=arma::rowvec(state_size ,arma::fill::zeros);

    std::vector<nuslam::landmark> lm(landmarks.size());
    changeframe(landmarks,lm);
    for(unsigned int i=0;i< lm.size();i++){
       state(2*i+3)=lm.at(i).x ;
       state(2*i+4)=lm.at(i).y ;
    }

    // // std::cout << "new" << std::endl;
    // // std::cout << state_/bar<< std::endl;
    

    state_bar=odomupdate(twist,state_bar);
    arma::mat sigma_bar;
    sigma_bar=arma::mat(state_size ,state_size ,arma::fill::zeros);

    sigma_bar=motion_jacobian(twist,sigma_bar);

    
    //   // //   //////////////////////update state based on observations ////////////////////
    //   // //   // measurements come in as (x,y) in robot frame ---convert to map frame 

    //   // vector2d has x,y  now incorporate r and m using struct landmark
   
    

    // arma::rowvec dummy;
  


    for(unsigned int i=0;i< lm.size();i++){

      // find correspondence id is the index the measurement comes in at
    int j = i;//0;
    // std::cout << j << std::endl;


    // // landmark has not been scene before
    // if (std::find(lm_j.begin(), lm_j.end(), j) == lm_j.end())
    // {
      
    //   // lm_j.push_back(j);
    //   newLandmark(m, j, state_bar);
    // }


         // range and ang
        arma::rowvec z_hat = measurementupdate(i,state_bar);
        // std::cout << z_hat << std::endl;
        arma::mat H;
        H=arma::mat(2,13,arma::fill::zeros);

        arma::mat K;
        K=arma::mat(13,2,arma::fill::zeros);
     
        H=measurement_Jacobian(i, state_bar, H);
            
      // //       // CPC.transpose +R// error 
      // //       //nuslam GAIN
            arma::mat temp=arma::inv(H*sigma_bar*H.t()+R);   // +R
       
            K= sigma_bar *H.t()*temp; // inv(temp)
   

            arma::rowvec delta;
            delta= arma::rowvec(2,arma::fill::zeros);
            delta(0)=lm.at(i).r-z_hat(0);
            delta(1)=rigid2d::normalize_angle(lm.at(i).b-z_hat(1));

        //      // update state vecotr 
            state_bar=state_bar+(K*delta.t()).t();
            

            arma::mat I;
            I=arma::mat(13,13, arma::fill::zeros);
            I =I.eye();
           
        //    Update covariance sigma bar
            sigma_bar = (I - (K * H)) * sigma_bar;

        }

    state=state_bar;
    state(0) = rigid2d::normalize_angle(state(0));
    state_cov=sigma_bar;




            	// Perform update for full map state
		for (long unsigned int i = 0; i < landmarks.size(); i++)
		{
		    landmarks.at(i).x = state(3 + 2*i);
		    landmarks.at(i).y = state(4 + 2*i);
    }

    // std::cout << state(0) << std::endl;

    return state;
    
 }


arma::rowvec nuslam::EKF::unknownCorrespondenceSLAM(rigid2d::Twist2D twist,std::vector<rigid2d::Vector2D> &landmarks ,int dmax)
{

  
    arma::rowvec state_bar;
    state_bar=arma::rowvec(state_size ,arma::fill::zeros);

    std::vector<nuslam::landmark> lm(landmarks.size());
    changeframe(landmarks,lm);
    for(unsigned int i=0;i< lm.size();i++){
       state(2*i+3)=lm.at(i).x ;
       state(2*i+4)=lm.at(i).y ;
    }


    state_bar=odomupdate(twist,state_bar);
    arma::mat sigma_bar;
    sigma_bar=arma::mat(state_size ,state_size ,arma::fill::zeros);

    sigma_bar=motion_jacobian(twist,sigma_bar);


  // Let N be the number of landmarks already seen
  int N=landmarks.size(); 
  std::vector<double> distances;
// Step 10 PR
    for (unsigned int i = 0; i < N; i++){
        //  Compute the expected measurement 

      arma::rowvec z_hat = measurementupdate(i,state_bar);
      
       // Compute the covariance 
      arma::mat H;
        H=arma::mat(2,13,arma::fill::zeros);

        arma::mat K;
        K=arma::mat(13,2,arma::fill::zeros);
     
        H=measurement_Jacobian(i, state_bar, H);

        arma::mat temp=arma::inv(H*sigma_bar*H.t()+R);   // +R
       
        K= sigma_bar *H.t()*temp; // inv(temp)
   
       
    // Compute the expected measurement 
      // difference in measurements delta_z (r,b)
         arma::rowvec delta;
            delta= arma::rowvec(2,arma::fill::zeros);
            delta(0)=lm.at(i).r-z_hat(0);
            delta(1)=rigid2d::normalize_angle(lm.at(i).b-z_hat(1));
        //      // update state vecotr 
            state_bar=state_bar+(K*delta.t()).t();
    
          // mahalanobis distance
          // double d = delta.t() * arma::inv(sigma_bar)* delta;
          double d;
          distances.push_back(d);

    }


    //   et the mahalanobis distance for landmark  (the temporary landmark) to be a the distance thresShold.


    //  d* is min mahalanobis distance
    auto j = std::min_element(distances.begin(), distances.end()) - distances.begin();
    const auto dstar = distances.at(j);         

// If all the landmarks are farther away than this threshold, then the landmark is new.

if (dstar >= dmax)
      {
        // max number of landmarks in state vector
        if ((N + 1) <= landmarks.size())
        {
          // set new ID to N
          // because we index from 0
          j = N;

        //   Adding new landmark ID;
          // newLandmark(m, j, state_bar);
          // lm_j.push_back(j);

          N++;
        }

      }

    state=state_bar;
    state(0) = rigid2d::normalize_angle(state(0));
    state_cov=sigma_bar;




            	// Perform update for full map state
		for (long unsigned int i = 0; i < landmarks.size(); i++)
		{
		    landmarks.at(i).x = state(3 + 2*i);
		    landmarks.at(i).y = state(4 + 2*i);
    }


  return state;

}

rigid2d::Transform2D nuslam::EKF::getRobotState(){
  rigid2d::Transform2D Tmr;
  Tmr.dx=state(1);
  Tmr.dy=state(2);
  Tmr.angle=rigid2d::normalize_angle(state(0));
  return Tmr;
}


std::vector<rigid2d::Vector2D> nuslam::EKF::getLandmark(std::vector<rigid2d::Vector2D> &map, int n) 
{
    
  for(auto i = 0; i < n; i++)
  {
    const auto jx = 2*i + 3;
    const auto jy = 2*i + 4;

    rigid2d::Vector2D l;
    l.x=state(jx);
    l.y=state(jy);
    map.push_back(l);
    
  }
  return map;
}


void nuslam::EKF::newLandmark(const nuslam::landmark &m, const int i,arma::rowvec  state_bar) 
{
     //  x_l = x_t + range_t * cos(bearing_t + theta_t)
    //   y_l = y_t + range_t * sin(bearing_t + theta_t)
  
  const auto ix = 2*i + 3;
  const auto iy = 2*i + 4;

  state_bar(ix) = state_bar(1) + m.r * std::cos(m.b + state_bar(0));
  state_bar(iy) = state_bar(2) + m.r * std::sin(m.b + state_bar(0));
}