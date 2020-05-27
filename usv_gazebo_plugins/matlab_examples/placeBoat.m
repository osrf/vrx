function placeBoat(n, pitchAngle, svc)
    if nargin < 3
        svc = rossvcclient('gazebo/set_model_state');
    end
    % unfortunately, can't use get_model_state due to mismatched md5 (might
    % need to change to using rostopics or modify the version of ROS)
    modelname = ['shape_',num2str(n),'_boat'];
    msg = rosmessage(svc);
    D = 0.5;
    msg.ModelState.ModelName = modelname;
    quat = eul2quat([0 deg2rad(pitchAngle), 0]);
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);
    
    % these are tuned to the waterline around the AVS
    if n == 1
        posX = 2;
        posY = 2;
    elseif n == 2
        posX = 2;
        posY = -2;
    elseif n == 4
        posX = -2;
        posY = 2;
    elseif n == 8
        posX = -2;
        posY = -2;
    end
    % use the computed waterline (need to see if this matches since some
    % changes were made)
    L = 0.6; % m
    W = 1; % m (I don't think this matches the STLs that we used since
    % those had variable width, dending on n)
    D = 0.5; % m
    [~, waterLine] = getWaterLine(pitchAngle,L,n,D,W);
    guessZ = -waterLine;
    msg.ModelState.Pose.Position.Z = guessZ;
    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    
    ret = call(svc, msg);
end
