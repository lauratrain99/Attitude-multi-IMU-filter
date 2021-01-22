# Attitude-multi-IMU-filter
Date of the last update Jan 22 2021.

This repo contains the code development for the data fusion algorithm of a multi-IMU configuration to estimate attitude using an Extended Kalman filter.

The body whose attitude is to be analyzed, contains four IMUs located at each corner so in order to obtain an accurate estimation on the motion of the body, it is necessary to convert the readings obtained from each IMU into the center of mass motion, which is motion of interest.

The generation of synthetic data from a known physical motion allows to test the algorithms at the same time as the code development keeps on. This makes the verification method much successful as well as avoiding errors for future real measurement implementation.

For the purpose of verification of the kinematic principles, the file *multiple_imu.m*, available in the path *synthetic-data/kinematic_v&v*, was developed. 
The first check was to compare if the simulated motion of the center of mass coincides with the transformed motion (first from the simulated to the local IMU and then back from the local IMU to the center of mass). If these two coincide, it means that the method to convert from VIMU to IMU and the one to convert from IMU to VIMU apply the same principles.
The plots checking this condition, each IMU containing angular velocity, accelerations and local magnetic field:

- Plots 1, 2, 3 for IMU1.

- Plots 7, 8, 9 for IMU2.

- Plots 13, 14, 15 for IMU3.

- Plots 19, 20, 21 for IMU4.

However, it is also necessary to compare the motion of the center of mass with the motion of each of the IMUs. For the z-axis constant motion, the components of the acceleration in the body reference frame of each IMU are easy to compute analytically.
The plots checking this condition:

- Plots 4, 5, 6 for IMU1.

- Plots 10, 11, 12 for IMU2.

- Plots 16, 17, 18 for IMU3.

- Plots 22, 23, 24 for IMU4.

Additionally, the code includes a final part in which the numeric analytical values are compared with the ones obtained during the simulation.

The checklist followed for developing the verification of the kinematic principles:

          1. Input a motion of the body defined by the magnitudes of the angular
             velocities.

          2. Generate synthetic data (acc + magn) for the center of mass of the body.

          3. Number of IMUs: 4. Define the orientation of each of them 
             with respect to the center of mass by the variables roll0, pitch0, yaw0.

          4. Convert the synthetic gyro, accel and magn data of the center
             of mass into gyro, accel and magn data of the body reference.

          5. Transform gyro, accel and magn data of each of the IMUs into
             the data of the center of mass of the body.

          6. Plot the data from the initial synthetic data to see if it
             coincides with the transformed data.

          7. Verify if the simulated motion for the IMUs coincides with
             their theoretical value for constant z-axis rotation.


Once the kinematic transformations are verified, it is time to apply the data filter algorithm. Since there are many possibilities on the design architecture, the approach will be to build as many architectures as suggested and compare their performance to obtain a convenient design.

Possible design architectures:

1. Convert the data from each IMU to generate one CM (center of mass) data per IMU.
    Perform the sensor fusion by averaging all the readings at the center
    of mass. Perform the EKF only once with those readings. Available in the path *synthetic-data/data-fusion/architecture-1*, the file *EKF_arch1.m* contains the main program.

 2. Convert the data from each IMU to generate one CM (center of mass) data per IMU.
    Perform the EKF four times, one for each of the CM readings. Average the readings at the center of mass. Code not available yet.

 (To be continued with other alternatives)

 Checklist:

 DONE:

          1. Obtain the data readings for each IMU.
           
          2. Include error characterization and consequent noise in the
             IMU readings.

          3. Build the first design architecture. Steps:

               - Convert the data from each IMU to generate angular
               velocity, accelerations and local magnetic field at the
               center of mass.

               - Perform the sensor fusion to obtain an average for the
               angular velocity, acceleration and local magnetic field at
               the center of mass.

               - Apply the EKF to the averaged values.

TO BE DONE:

          4. Build the second design achitecture. Steps:

               - Convert the data from each IMU to generate angular
               velocity, accelerations and local magnetic field at the
               center of mass.

               - Apply the EKF to the each CM readings. Four EKFs in total

               - Average the four outputs of the EKF.