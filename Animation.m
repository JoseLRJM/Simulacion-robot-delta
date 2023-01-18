function [] = Animation( angles,traj,param )
%ANIMATION Make Plot animation
%!!!Precalcualte trajectory with forward kinematics

%simulation step size
N=size(angles,2);
dt=0.001;

for i=1:N 
    tic
    PlotPosition(traj(:,i),angles(:,i),param);
    toc
    pause(dt);
end

end

