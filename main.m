clc;
clear all;

%% Parametros
r_f=200e-3; %Base a codo
r_e=380e-3; %codo al efector final

%placas
f=526e-3; %base
e=121e-3; %efector final

%efector final en z
syms z real

param=[r_f,r_e,f,e,z];

%% cinematica
%angulos actuados:
m=15; %puntos de trayectorias
n=9; %posiciones
r0=[0,0,-0.1]; %posicion de inicio
rGoal=zeros(n+1,3);
%rGoal(1,:)=r0';
rGoal(1,:)=[0.1,0,-0.3];
rGoal(2,:)=[0,0.1,-0.35];
rGoal(3,:)=[0.1,0,-0.4];
rGoal(4,:)=[0,0,-0.5];
rGoal(5,:)=[0,0.1,-0.3];
rGoal(6,:)=[0.1,0,-0.35];
rGoal(7,:)=[0,0,-0.5];
rGoal(8,:)=[0,0,-0.2];
rGoal(9,:)=[0,0,-0.3];
rGoal(10,:)=[0,-0.15,-0.5];

trajectory=zeros(3,m,n);
angles=zeros(3,m,n);
for i=1:n
    trajectory(:,:,i)=CalcTrajectory(rGoal(i,:),rGoal(i+1,:),m);
    angles(:,:,i)=CalcTrajectoryAngles(trajectory(:,:,i),param)*pi/180;
end

%% Plot
for i=1:n
    Animation(angles(:,:,i),trajectory(:,:,i),param);
end





















