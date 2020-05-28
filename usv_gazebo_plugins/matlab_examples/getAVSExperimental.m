function [thetas, angular_velocities] = getAVSExperimental(n,useGreensTheorem)
sub = rossubscriber('/gazebo/model_states');
svc = rossvcclient('gazebo/set_model_state');
thetas = linspace(0,180,500);
angular_velocities = [];

for theta = thetas
    placeBoat(n,theta,svc);
    pause(0.1);
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
    angular_velocities(end+1) = -tiltingSpeed;
    [theta tiltingSpeed]
end
figure;
plot(thetas,angular_velocities);
xlabel('heel angle (degrees)');
ylabel('approximately proportional torque');
end
