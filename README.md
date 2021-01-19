# Attitude-multi-IMU-filter
Extended Kalman filter for attitude estimation on a multi-IMU configuration


 Date of the last update Jan 19 2021

 Checklist:
 DONE:
          1. Input a motion of the body defined by the magnitudes of the angular
             velocities.
          2. Generate synthetic data (acc + magn) for the center of mass of the body.
          3. Number of IMUs: 4. Define the orientation of each of them 
             with respect to the center of mass by the variables roll0, pitch0, yaw0.
          4. Convert the synthetic gyro, accel and magn data of the center
             of mass into gyro, accel and magn data of the body reference.
             Add noise for the gyro, accel and magn data.
          5. Transform gyro, accel and magn data of each of the IMUs into
             the data of the center of mass of the body.
          6. Plot the data from the initial synthetic data to see if it
             coincides with the transformed data.
          7. Apply the EKF to each of the resultling center of mass data
             coming from each of the IMUs. Results: four different
             attitudes coming from each IMU data.
 TO DO:
          8. After step 5, combine/fuse the center of mass data obtained
          from each IMU and then apply the EKF with only one center of mass data

