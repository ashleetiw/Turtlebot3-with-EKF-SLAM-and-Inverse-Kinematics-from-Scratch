


// Let N be the number of landmarks already seen
// Step 10 PR
    for (unsigned int j = 0; j < N; j++){
        //  Compute the expected measurement 

      rigid2d::Vector2D z_hat = predictedMeasurement(k, state_bar);
       
       // Compute the covariance 
      Marma::mat H = arma::mat(2 ,state_size , arma::fill::zeros);
      measurementJacobian(k, state_bar, H);

      // Psi
      arma::mat Psi = H * sigma_bar * H.transpose() + measurement_noise;

    // Compute the expected measurement 
      // difference in measurements delta_z (r,b)
      rigid2d::Vector2D delta_z;
      delta_z(0) = lm.at(i).r - z_hat(0);   // not j (remember)
      delta_z(1) = normalize_angle_PI(normalize_angle_PI(lm.at(i).b) - normalize_angle_PI(z_hat(1)));

      // mahalanobis distance
      double d = delta_z.transpose() * Psi.inverse() * delta_z;
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
        if ((N + 1) <= n)
        {
          // set new ID to N
          // because we index from 0
          j = N;

        //   Adding new landmark ID;
          newLandmark(m, j, state_bar);
          lm_j.push_back(j);

          N++;
        }

      }
// Let be the minimum mahalonbis distance and  be the landmark index corresponding to the minimimum distance.
//  is the landmark corresponding to . If  then we have a new landmark so increment .
