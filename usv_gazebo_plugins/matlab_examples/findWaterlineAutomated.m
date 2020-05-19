function findWaterlineAutomated(n)
sub = rossubscriber('/gazebo/model_states');
svc = rossvcclient('gazebo/set_model_state');
for theta = linspace(80,120,500)
    findWaterline(n,theta,svc);
    pause(0.2);
    m = sub.LatestMessage;
    % should be consistent order, but just in case
    model_idx = 0;
    for i = 1:length(m.Name)
        if strcmp(m.Name{i},['shape_',num2str(n),'_boat']);
            model_idx = i;
            break;
        end
    end
    tiltingSpeed = m.Twist(model_idx).Angular.Y;
    [theta tiltingSpeed]
end
end