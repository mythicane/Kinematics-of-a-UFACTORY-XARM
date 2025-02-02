%% Main Executable Program for XARM Kinematics and Display

import forward_kinematics.*
import inverse_kinematics.*
import visualization.*
import workspace.*

%Note on things to improve: 
% - Workspace & Visualization... incorporate the sword...?
% - inverse kinematics... clarify integration of end effector matrix
% - forward kinematics... origin to end? or link i-1 to link i?

%% For Forward Kinematics...
%Specify your "i" thetas, for each "i" joint (from Joints 1, 2... 5, 6).
fk_theta_val = [0.2, 0.8, 3.0, 0.0, 0.2, 0.0]; %forward kinematics theta values

%CHOOSE YOUR WEAPON!!!! >:D
%(Choose the sword that you'd like the XARM to wield in honorable combat... sorted by length in METERS)
%(You can also choose "no sword," but where is the fun in that?)

no_sword = 0; %fist-fighting, of Paleolithic origin, I presume.

hudiedao = 0.30; %Qing Dynasty Era Chinese Origin

sica = 0.35; %Ancient Roman/Dacian Origin

machete = 0.45; %Colonial Central American Origin

katana = 0.6; %Feudal Japanese Origin

tachi = 0.75; %Koto period Japanese Origin

ZhanmaDao = 0.95; %Song Dynasty Era Chinese origin

rapier = 1.05; %Medieval Spanish Origin 

longsword = 1.15; %Rennaisance Era Germanic Origin

claymore = 1.25; %Late Medieval Scottish Origin

sword_of_choice = no_sword; %%INSERT the NAME of the sword of your choosing HERE! [i.e, machete]

disp("Executing Forward Kinematics, please stand by...")
trans_matrixies = forward_kinematics(fk_theta_val, sword_of_choice); %1-7 joint positions and orientation!
end_effector_matrix = cell2mat(trans_matrixies(7)); %grabs last trans. matrix for "tip of sword" position and orientation!
disp("Given the Thetas and the sword choice, the Final Transformation Matrix is...")
disp(end_effector_matrix) %displays the End-Effector Posititon and Orientation (origin to end-effector) Matrix.

%% For Inverse Kinematics...
%Specify the end effector position as x,y,z coordinates.
alpha = 0; %yaw
beta = 0; %pitch
gamma = 0; %roll
end_effector_info = [0.5,0.5,0.5, gamma, beta, alpha];

disp("Executing Inverse Kinematics, please stand by...")
%ik_theta_val = inverse_kinematics(end_effector_info, sword_of_choice); %inverse kinematics theta values
disp("Given the End Effector Coordinates, and the given Sword, the associated Thetas are...")
disp(ik_theta_val)

%% For Displaying Robot Positioning... without the sword.
%Specify your "i" thetas, for each "i" joint (from Joints 1, 2... 5, 6).
%Uncomment the visualization line if you'd like the visualization
%Uncomment the workspace display for the reachable/dexterous workspace
%Uncomment both if you'd like both at the same time! This is a great way to
%see visually where your end-effector is within the reachable workspace...

visualization(fk_theta_val, sword_of_choice) %visualize the XARM6 (with real dimensions from the URDF file)
%% For Displaying the XARM Workspace... [EXTRA CREDIT]
workspace(sword_of_choice) % Displays the reachable/dextrous workspace of an XARM as a Voxel-Point Map... without sword in hand.



