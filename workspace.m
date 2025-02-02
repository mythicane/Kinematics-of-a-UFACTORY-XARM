%% Visualizing the Workspace of a UFACTORY X-Arm (6 DOF)... 
function workspace(sword_length)
    %% Displays the reachable/dextrous workspace as a Voxel-Map
    %where the more blue the voxel-point is, the more solutions are associated,
    %but the more red the voxel-point is, the less solutions are associated. 
    %Dark red indicates a single solution.
    %Bright Blue indicates infinite solutions. 
    
    robot = importrobot('xarm6_robot.urdf'); %rigid body tree...
    
    sword = rigidBody('sword');
    hilt = rigidBodyJoint('hilt','fixed');
    sword.Joint = hilt;

    matrix = [
        1, 0, 0, sword_length;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];

    setFixedTransform(hilt,matrix);
    addBody(robot,sword,"link6")
    
    ee = "sword";
    rng default
    [workspace,configs] = generateRobotWorkspace(robot,{},ee,IgnoreSelfCollision="on");
    mIdx = manipulabilityIndex(robot,configs,ee);
    hold on
    showWorkspaceAnalysis(workspace,mIdx,Voxelize=true)
    axis auto
    title("Voxelized Reachable/Dexterous Workspace")
    hold off
end