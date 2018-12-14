# Quadrotor Simulator

main.m needs to run before running visualize.m

Run main.m to simulate a Quadrotor helicopter.


Choose between two different controllers:
-- controlQuad.m  implements a PD feedback linearization controller,
-- lee_position_controller.m  is a Matlab-quadrotor adaptation of the lee_position_controller.cpp
included in the RotorSimulator framework. https://github.com/ethz-asl/rotors_simulator

quadrotor_EQ.m contains the quadrotor's dynamical equations

visualize.m should be run after main.m ( a "uav_pid_results.mat" file has been created )
to gain a visual representation of the resulting flight.
