# robot_companion
Robot accompanying human

We present an indoor robot service system which will guide the human to their destination in a safe way through crowded areas. We will integrate a Kalman Filter with a motion model based on the social force model (SFM) presented by Helbing and Molar (Helbing, D., and P. Molnar, “Social force model for pedestrian dynamics,” Physical
review E, vol. 51 (1995), p. 4282. 1, 3) to track humans about the robot. These forces model different aspects of motion behaviors such as the motivation of people to reach a goal, the repulsive effect of walls and other people as well as physical constraints. We present a method to learn the weights of the forces for each of the humans in the environment to be able to track them better. In addition, once we learn the preference of the escortee to follow the robot, we can deduce their intent to follow and give controls to the robot accordingly.
