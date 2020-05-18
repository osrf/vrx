function findWaterline(n, pitchAngle,isMathematica)
    if nargin < 3
        isMathematica = false;
    end
    svc = rossvcclient('gazebo/pause_physics');
    if isMathematica % use Mathematica boat
        modelname = ['shape_',num2str(n),'_boat_mathematica'];
    else
        modelname = ['shape_',num2str(n),'_boat'];
    end
    msg = rosmessage(svc);
    call(svc, msg);

    % unfortunately, can't use get_model_state due to mismatched md5 (might
    % need to change to using rostopics or modify the version of ROS)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);
    D = 0.5;
    msg.ModelState.ModelName = modelname;
    quat = eul2quat([0 deg2rad(pitchAngle), 0]);
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);
    % You could also use the predicted waterline.
    % We are somewhat constrained since we can't use the
    % gazeo/get_model_state service to do something
    % fancier.  You could also put the boat the predicted waterline (note:
    % this is not as straightforward as I would have hoped since the
    % in John's code water is rotated whereas in Gazebo, the boat is rotated).
    % I'm sure there must be an easy way to convert, but I didn't try to
    % work through it.  John: maybe you can have a look?
    % msg.ModelState.Pose.Position.Z = getWaterLine(n, pitchAngle);
    % It does seem to matter how you set this.
    
    % these are tuned to the waterline around the AVS
    if n == 1
        guessZ = 0.44*D;
        posX = 2;
        posY = 2;
    elseif n == 2
        guessZ = 0.52*D;
        posX = 2;
        posY = -2;
    elseif n == 4
        guessZ = 0.68*D;
        posX = -2;
        posY = 2;
    elseif n == 8
        guessZ = 0.78*D;
        posX = -2;
        posY = -2;
    elseif n == 0 % mathematica boat
        guessZ = 0.55*D;
        posX = 4;
        posY = -2;
    end
    msg.ModelState.Pose.Position.Z = guessZ;
    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;

    ret = call(svc, msg);
    
    svc = rossvcclient('gazebo/unpause_physics');
    msg = rosmessage(svc);
    call(svc, msg);
end