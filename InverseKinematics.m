function [ t ] = InverseKinematics( r,param  )
%INVERSEKINEMATICS calculates theta t from pose r
x=r(1);
y=r(2);
z=r(3);
t1=0;
t2=0;
t3=0;

[t1,s]=CalcInverse(r,param);
if(s==0)
    [t2,s]=CalcInverse([x*cos(120*pi/180) + y*sin(120*pi/180), y*cos(120*pi/180)-x*sin(120*pi/180), z],param);
end

if(s==0)
    [t3,s]=CalcInverse([x*cos(120*pi/180) - y*sin(120*pi/180), y*cos(120*pi/180)+x*sin(120*pi/180), z],param);
end

if(s==0) 
    t=eval([t1,t2,t3]);
else
    t=[0,0,0];
end

end

function [ theta,s ] = CalcInverse( r,param )
%CALCINVERSE
theta=[0,0,0];
%% parameters
x=r(1);
y=r(2);
z=r(3);

%rod length:
r_f=param(1); %in m
r_e=param(2);

%triangular side length:
f=param(3); %in mm
e=param(4);

%%inverse kinematics
y1=-f/(2*sqrt(3));
k=e/(2*sqrt(3));
y=y-k; %shift

a=(x^2 + y^2 + z^2 +r_f^2 - r_e^2 - y1^2)/(2*z);
b=(y1-y)/z;

d=-(a+b*y1)^2+r_f*(b^2*r_f+r_f); 
if(d<0)
   disp('Pose not in range!!!!!!!!!!!!!!!!!!!! Choose other Pose that is not a singularity');
   s=1;
else
    yj = (y1 - a*b - sqrt(d))/(b^2 + 1); % choosing outer point
    zj = a + b*yj;
    theta = 180*atan(-zj/(y1 - yj))/pi;

    if (yj>y1)
        theta=theta+180;
    end
    s=0;
end


end


