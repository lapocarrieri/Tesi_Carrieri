# Thesis Project: Real-Time Robot Simulation and Force Application Analysis

This repository contains the MATLAB code developed for my thesis, focusing on real-time robot simulation, force application analysis, and dataset generation for LSTM neural network training. The project uses a set of MATLAB scripts to simulate robot behavior, calculate external forces, and generate a comprehensive dataset for neural network training.

## Scripts Description

### `CPF_script2.m`
This script is responsible for calculating the point of force application on the robot. It utilizes a contact particle filter to accurately determine the application points of external forces during the simulation.

### `LWR_initialization.m`
This script sets up the basic parameters and objects necessary for simulating the robot in MATLAB. It initializes the environment and defines essential variables and settings that are used across the simulation.

### `LWR_sim_realBody.m`
The main function of the project, `LWR_sim_realBody.m`, handles the real-time simulation of the robot. It performs the following key tasks:
- Manages real-time simulation of the robot's movements and interactions.
- Uses residual calculations to determine external torques applied to the robot.
- Calculates wrenches based on the robot's current state and external forces.

### `SimulationForNNDataset_link.m`
This script is designed to generate a comprehensive dataset for training an LSTM neural network. It operates the robot throughout its entire workspace to gather as much data as possible, ensuring a diverse and extensive dataset.

## Running the Code

To run this project effectively, follow these steps:
1. Open two separate MATLAB windows.
2. In the first MATLAB window, run `CPF_script2.m`. This script will continuously calculate the force application points and does not require waiting for the simulation to progress.
3. In the second MATLAB window, run `LWR_sim_realBody.m`. This script handles the real-time simulation of the robot.

These two scripts communicate with each other and operate independently. The `CPF_script2.m` script receives data every 0.1 second from the simulation running in `LWR_sim_realBody.m`, ensuring that the contact particle filter is consistently updated without delays.

## Acknowledgments

This work has been developed with the support and contributions from Emanuele Magrini, Massimo Cefalo, Claudio Roberto Gaz, Daniele Zurlo, and Alessandro De Luca. Their invaluable inputs and insights have been crucial in the development of this project.

## Prerequisites

Before running the scripts, ensure that you have the following prerequisites met:
- MATLAB (version XXX or later recommended).
- Necessary MATLAB toolboxes (list any specific toolboxes required).
- Any additional setup or configuration steps specific to your project.

## Contributing

Feel free to fork this repository and contribute to the development. Any contributions to enhance the simulation, extend the capabilities of the particle filter, or improve the dataset generation for LSTM training are welcome.

## License

This project is licensed under the [MIT License](LICENSE.md).

## Contact

For any inquiries or contributions, please contact Lapo Carrieri at lapo.carrieri@gmail.com or call +393347577098.
