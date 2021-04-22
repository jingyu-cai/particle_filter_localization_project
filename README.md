# particle_filter_localization_project
![gif](./gif/particle_filter.gif)

## Implementation Plan
### Name of team members
* Jingyu Cai
* Tianle Liu
### Implementation
* `initialize_particle_cloud()`: We will initialize the particle cloud with random locations and orientations throughout the map and populate the `particle_cloud` array with `Particle` elements. To test it, we will physically display the poses of the elements in the `particle_cloud` array within the map to visualize the distribution of the particles.
* `update_particles_with_motion_model()`: We will calculate how much the robot has moved using its current odometry data and its old odometry data from its last motion update, and move all of the particles with the same amount. To test it, we will test a few individual examples by directly printing out the array to see if the values match our expectations as well as update the display of the `particle_cloud` array to visualize if the movement of the particles correspond to the movement of the robot.
* `update_particle_weights_with_measurement_model()`: We will first define the sensor readings to be the `ranges` tuple from the robot's laser scan data, we will then take in the robot's laser scan data and compare the sensor readings with each particle's hypothetical sensor readings and assign a weight to each `Particle` element in the `particle_cloud` array. For now, we will use a similar measurement model provided in class 5 to compute the importance weights for each particle, with the difference that our model will have 360 computations in the denominator for each weight. To test it, we will test a few individual examples by directly printing out the weights for a few particles after a well-defined movement to see if the values match our expectations.
* `normalize_particles()` and `resample_particles()`: We will first add up all the weights of the `Particle` elements and reassign the weights by dividing each particle's original weight by that sum. We will then resample (with replacement) by creating an array of the cumulative sum of the weights, randomly generating a number, finding the index of the range that the number belongs to in the cumulative array, preserve the particle at that index into the next iteration, and repeat the loop until enough particles are sampled. To test it, we will print out the weights in the normalized array to see if they add up to one, and we will also print out the array after resampling to see if more larger weights are being preserved for the next iteration and physically display the poses of the elements in the `particle_cloud` array within the map to see if they are converging to the robot's real location.
* `update_estimated_robot_pose()`: We will update the robot's estimated pose by taking an average of all the pose data in the resampled `particle_cloud` array. To test it, assuming that we have computed the above steps correctly, we will print out the estimated pose to see if it's getting closer to the robot's real location.
* Incorporating noise: For each particle movement, we will add a 0-15% error to the amount the robot has moved and allow the pose data to be updated to any of the values in that range. To test it, we will test a few individual examples by directly printing out the array to see if the values match our expectations as well as update the display of the `particle_cloud` array to visualize if the movement of the particles display noise when corresponded to the movement of the robot.
### Timeline
* 4/18: Complete `initialize_particle_cloud()`, `update_particles_with_motion_model()`, and `update_particle_weights_with_measurement_model()`.
* 4/21: Complete `normalize_particles()`, `resample_particles()`, and `update_estimated_robot_pose()`.
* 4/25: Add noise into the system and ensure the code works with rigorous testing.

## Writeup
### Objectives description
The goal of this project is to solve the problem of robot localization using the particle filter algorithm. Specifically, by implementing each component of the particle filter algorithm, we will be able to predict the location of the robot with the states of the particles as it moves through an environment.
### High-level description
We followed the steps for implementing the particle filter algorithm illustrated on the project page and in the lectures. There are seven main components to solve robot localization: first being randomly initializing the particle cloud within the specified environment, second being updating the poses of the particles correspondingly to the movement of the robot, third being computing the weights for each particle after movement using the likelihood field for range finders model, fourth being normalizing and resampling the particles based on their weights to converge to the likely locations of the robot, fifth being incorporating movement noise to add robustness to the model, sixth being updating the estimated robot pose, and seventh being optimizing the parameters to more efficiently solve robot localization.
### Code explanation
#### Initialization of particle cloud
##### Code location
For initializing the particle cloud, I first wrote a helper function `draw_random_sample()` to randomize the distribution of particles, and the main code is located in the function `initialize_particle_cloud()`.
##### Functions/code description
* `draw_random_sample()`: In this function, n elements with a specified probability are drawn with replacement from a list of choices with the help of `random_sample()` from numpy import. In the context of this project, this function randomly chooses coordinates inside the boundaries of the house from the map list to be populated with particles.
* `initialize_particle_cloud()`: In this function, particles are being initialized with random locations and orientations throughout the map. By using the `draw_random_sample()` function, I was able to randomly obtain coordinates that correspond to a value of 0, which is light gray color inside the house, from `self.map.data`. These random particles will be used to localize the robot.
#### Movement model
##### Code location
For the movement model, the code is located in the function `update_particles_with_motion_model()`. I also used the given the `get_yaw_from_pos()` function from the starter code as a helper function.
##### Functions/code description
* `update_particles_with_motion_model()`: This function calculates how much the robot has moved using odometry and updates the poses of all the particles accordingly by the same amount. Therefore, the updated particles can then be fed into the next step of the measurement model.
#### Measurement model
##### Code location
For the measurement model, the code is located in `update_particle_weights_with_measurement_model()`. I also used code given by the starter code and class 06, including `get_yaw_from_pose()`, `get_closest_obstacle_distance()`, and `compute_prob_zero_centered_gaussian()`, as helper functions.
##### Functions/code description
* `update_particle_weights_with_measurement_model()`: This function uses the likelihood field for range finders model discussed in class 06 to update the weights of the particles. By doing so, particles that better reflect the likely positions of the robot will be assigned with a greater weight to localize the robot.
#### Resampling
##### Code location
For resampling, the code is located in the function `resample_particles()`. 
##### Functions/code description
* `resample_particles()`: For this function, I used the `choice()` function from numpy to sample particles with probabilities proportionate to their weights. In other words, particles with greater weights will be more likely to be resampled into the next iteration. 
#### Incorporation of noise
##### Code location
Noise is incorporated in `update_particles_with_motion_model()`. An helper function `add_noise()` is added to the top of the script.
##### Functions/code description
* `add_noise(delta)`: This function takes in the calculated position/yaw change (`delta`) from odometry and randomly adds an error (±0~15%) to it. 
*  The noise is incorporated by updating the particle poses with the return values of `add_noise()`. 
#### Updating estimated robot pose
##### Code location
For updating the estimated robot pose, the code is located in the function `update_estimated_robot_pose()`.
##### Functions/code description
* `update_estimated_robot_pose()`: This function updates the robot's estimated pose by taking an average of all the positions and orientations of the particles. As a result, since particles that more likely represent the likely locations of the robot are preserved, the average would converge onto the robot's true location, thus solving the problem of robot localization.
#### Optimization of parameters
##### Code location
The parameters we optimized are for handling the position data for edge cases as well as the standard deviation input and estimated sensor errors for particle probability calculations, located in the function `update_particle_weights_with_measurement_model()`.
##### Functions/code description
* If the detection angle is out of range, then we skipped the calculation of that angle completely.
* If `(x_ztk, y_ztk)` is out of the map boundaries, then we assigned a small value of 0.00001 to `prob` for computing the weight of the particle.
* We used 0.1 for the standard deviation to ensure quick convergence of the particles. 
* We estimated that the sensor correctly measures a hit with 80% probability, generates random measures with 10% and fails to detect an object with 10%.
### Challenges
TODO
### Future work
TODO
### Takeaways
TODO
