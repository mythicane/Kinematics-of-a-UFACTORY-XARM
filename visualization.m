%% Visualizing the Kinematics of a UFACTORY X-Arm (6 DOF)... 

function visualization(theta_val, sword_length)
    %% Given an array of theta values in the form "theta_val", 
    % which is an array of "i" joint theta values for i = {1,2,3,4,5,6},
    %displays the URDF of an XARM6 in the specified position. 
    
    %imports the URDF of a XARM6
    robot = importrobot('xarm6_robot.urdf'); %how do I modify an xarm urf to extend the last joint?
      
    sword = rigidBody('sword');
    hilt = rigidBodyJoint('hilt','revolute');
    sword.Joint = hilt;

    matrix = [
        1, 0, 0, sword_length;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];

    setFixedTransform(hilt,matrix);
    addBody(robot,sword,"link6")

    config = homeConfiguration(robot); %Configures each joint...
    config(1).JointPosition = theta_val(1);
    config(2).JointPosition = theta_val(2);
    config(3).JointPosition = theta_val(3);
    config(4).JointPosition = theta_val(4);
    config(5).JointPosition = theta_val(5);
    config(6).JointPosition = theta_val(6);

    show(robot,config); %Displays a 3D visualization!
end
