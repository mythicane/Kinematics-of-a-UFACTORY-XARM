%% The Backwards Kinematics of a UFACTORY X-Arm (6 DOF)... 
%Not accounting for end orientation... update needed!

function theta_val = inverse_kinematics(end_effector_coord, sword)
    import forward_kinematics.*
    %% Main: Calculate the inverse kinematics of UFACTORY X-Arm (6 DOF)
    %Inverse Kinematics: Returns the "theta_val", which is an array of "i" joint
    %theta values for i = {1,2,3,4,5,6,7}, given the end-effector position.
    %NOTE That theta_7 is ALWAYS zero!!
    
    syms t1 t2 t3 t4 t5 t6
    theta_val = [t1, t2, t3, t4, t5, t6];
    end_effector_positions = forward_kinematics(theta_val, sword); %passes symbolic values into FK... 
    MegaMatrix = end_effector_positions{6}; %grabs symbolic end-effector transformation matrix!
    
    %this is where the x, y, and z values are set
    xval=MegaMatrix(1,4)==end_effector_coord(1); 
    yval=MegaMatrix(2,4)==end_effector_coord(2);
    zval=MegaMatrix(3,4)==end_effector_coord(3); 

    %this is where the roll, pitch, and yaw values are set
    pitch = atan2(-1*MegaMatrix(3,1), sqrt(MegaMatrix(1,1)^2 + MegaMatrix(2,1)^2)) == end_effector_coord(5);
    %check if cos(pitch) = zero, if so, roll and yaw are arbitrary:        
    cospitch = cos(atan2(-1*MegaMatrix(3,1), sqrt(MegaMatrix(1,1)^2 + MegaMatrix(2,1)^2)));
    if cospitch == 0
         yaw = end_effector_coord(4);
         roll = end_effector_coord(6);
     else
     yaw = atan2(MegaMatrix(2,1)/cospitch, MegaMatrix(1,1)/cospitch) == end_effector_coord(4); 
     roll = atan2(MegaMatrix(3,2)/cospitch, MegaMatrix(3,3)/cospitch) == end_effector_coord(6); 
     end

    %If roll, pitch, and yar are desired, add them to the [xval, yval, zval] array below.
    theta_val = vpasolve([xval, yval, zval],[t1,t2,t3,t4,t5,t6],[[-pi, pi];[-pi, pi];[-pi, pi];[-pi, pi];[-pi, pi];[-pi, pi]]);
      
    for i = 1:length(theta_val)
        if ~isa(theta_val(i), 'double')
            warning('Desired end effector position and orientation out of workspace of robot! No solutions found: Returning [0×1 sym].');
        end
    end
end

