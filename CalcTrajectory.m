function [ traj ] = CalcTrajectory( r0,rGoal,m )
traj=[linspace(r0(1),rGoal(1),m);...
        linspace(r0(2),rGoal(2),m);...
        linspace(r0(3),rGoal(3),m)];

end

