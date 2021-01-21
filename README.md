# Attitude-multi-IMU-filter
Extended Kalman filter for attitude estimation on a multi-IMU configuration.

The file containing the simulation is multiple_imu. 
It contains many plots with all the info about the z-axis constant rotation simulation. 
There are many plots containing many info, for the goal of comparing the motion of the center of mass vs the motion of each IMU:

- Plots 6, 7, 8 for IMU1.

- Plots 14, 15, 16 for IMU2.

- Plots 22, 23, 24 for IMU3.

- Plots 30, 31, 32 for IMU4.

Additionally, the code includes a final part of verification in which the theoretical values are compared with the ones obtained during the simulation.


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

          7. Verify if the simulated motion for the IMUs coincides with
             their theoretical value for constant z-axis rotation.

          8. Apply the EKF to each of the resultling center of mass data
             coming from each of the IMUs. Results: four different
             attitudes coming from each IMU data.

 TO DO:

          9. After step 5, combine/fuse the center of mass data obtained
          from each IMU and then apply the EKF with only one center of mass data

