%% The Forward Kinematics of a UFACTORY X-Arm (6 DOF)... 

function end_effector_positions = forward_kinematics(theta_val, sword)
    %% Main: Calculate the forward kinematics of UFACTORY X-Arm (6 DOF)
    % Forward Kinematics: Returns the "end_effector_positions",  
    % which is an array of transformation matrixies from ORIGIN to joint
    % "i" for i = {1,2,3,4,5,6,7}, given the the joint angles as the input, 
    % and sword dimensions. 
    % Note: the Robot End-Effector transformation matrix is the last matrix
    %in the "end_effector_positions" array.

    %According to the XARM Documentation, there are offsets for Theta 2 and Theta 3 that
    %We should be aware of... (in radians)

    theta_val(2) = theta_val(2) - 1.3849179;
    theta_val(3) = theta_val(3) + 1.3849179;

    theta_val(7) = 0; %The last theta is ALWAYS zero!

    [a_val, d_val, alpha_val] = xarm_parameters(sword);
    transform_array = cell(1, length(theta_val));

    %Calculates the transformation matrices for each joint
    for i = 1:length(theta_val)
        transform_array{i} = transformation_matrix(alpha_val(i), a_val(i), theta_val(i), d_val(i));
    end

    %Calculates each joint's "end effector position" by chaining
    %transformations together....
    end_effector_positions = cell(1, length(theta_val));
    end_effector_positions{1} = transform_array{1};

    for i = 2:length(transform_array)
        end_effector_positions{i} = end_effector_positions{i-1} * transform_array{i};
    end

end

function matrix = transformation_matrix(alpha, a, theta, d)
    %% This is the traditional DH convention transformation matrix... 
    %Since the documentation has given us the parameters according to this convention,
    %This matrix will be used instead of the matrix taught in class. (Please forgive us Prof. Peng)
    matrix = [
        cos(theta), -sin(theta), 0, a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
        0, 0, 0, 1];
end

function [a_val, d_val, alpha_val] = xarm_parameters(sword)
    %% This function returns the UFACTORY X-Arm Parameters as 3 arrays

    %a_val : array of "a" parameters in METERS (length of linkage in x-axis)
    a_val = [0.0, 0.0, 0.28948866, 0.0775, 0.0, 0.076, sword];

    %d_val : array of "d" parameters in METERS (length of link in z-axis)
    d_val = [0.267, 0.0, 0.0, 0.3425, 0.0, 0.097, 0];

    %alpha_val : array of "alpha parameters in RADIANS (angle around x)
    alpha_val = [0, -pi/2, 0.0, -pi/2, pi/2, -pi/2, 0];
end
