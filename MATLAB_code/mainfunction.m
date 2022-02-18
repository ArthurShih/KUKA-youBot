function finallist=mainfunction()
initialconfigac=[-0.2 -0.2 0.1 -0.1 -0.2 -0.5 -0.1 0.1 0 0 0 0]; % actual chassis initial configuration
initialconfigre=[0 0 0 -0.3 -1 -0.5 -0.5 -0.5 0 0 0 0]; % reference chassis initial configuration

phi=initialconfigre(1); % find Tse by given reference initial configuration
x=initialconfigre(2);
y=initialconfigre(3);
thetalist=initialconfigre(4:8)';
B1=[0; 0;1; 0;0.033;0];
B2=[0;-1;0;-0.5076;0;0];
B3=[0;-1;0;-0.3526;0;0];
B4=[0;-1;0;-0.2176;0;0];
B5=[0; 0;1;0;0;0];
Blist=[B1 B2 B3 B4 B5];
Tsb = [[ cos(phi) -sin(phi)    0        x    ];
       [ sin(phi)  cos(phi)    0        y    ];
       [   0         0         1     0.0963  ];
       [   0         0         0        1    ]];
   
Tb0 = [[   1         0         0     0.1662  ];
       [   0         1         0        0    ];
       [   0         0         1     0.0026  ];
       [   0         0         0        1    ]];
   
M0e = [[   1         0         0     0.033   ];
       [   0         1         0        0    ];
       [   0         0         1     0.6546  ];
       [   0         0         0        1    ]];
T0e=FKinBody(M0e, Blist, thetalist);
Tse=Tsb*Tb0*T0e;


Tscinitial=[[1 0 0   1  ]; % define cube's initial and final position
            [0 1 0   0  ];
            [0 0 1 0.025];
            [0 0 0   1  ]];
        
Tscfinal=[[ 0 1 0   0  ];
          [-1 0 0  -1  ];
          [ 0 0 1 0.025];
          [ 0 0 0   1  ]];

% Tscfinal=[[1 0 0   1  ]; % define cube's initial and final position for new task 
%           [0 1 0   0  ];
%           [0 0 1 0.025];
%           [0 0 0   1  ]];
%         
% Tscinitial=[[ 0 1 0   0  ];
%             [-1 0 0  -1  ];
%             [ 0 0 1 0.025];
%             [0 0 0   1  ]];
      
dt=0.01;
[Trajmatrix,st,ed]=TrajectoryGenerator(Tse,Tscinitial,Tscfinal,dt); % generate trajectory
[n,m]=size(Trajmatrix);

Kp=0.2*eye(6); % set Kp, Ki, dt
Ki=0*eye(6);

config=initialconfigac;
chasisconfig=initialconfigac(1:3);
armconfig=initialconfigac(4:8);
wheelangle=initialconfigac(9:12);

statelist=zeros(1,13); % make empty list for saving data 
Xerradd=zeros(6,1);
Xerrlist=zeros(6,1);

for i=1:m-1
    Xd=Trajmatrix{1,i};
    Xdnext=Trajmatrix{1,i+1};
    [u,thetadot,Xerradd,Xerr]=FeedbackControl(config,Xd,Xdnext,Kp,Ki,dt,Xerradd);
    wheelspeed = u';
    armspeed=thetadot';
    
    newstate=NextState(chasisconfig,armconfig,wheelangle,armspeed,wheelspeed,dt);
    
    chasisconfig=newstate(1:3);
    armconfig=newstate(4:8);
    wheelangle=newstate(9:12);
    config=newstate; % update configuration
    
    if mod(i,10)==0 % save data
        a=i/10;
        statelist(a,1:12)=newstate;
        Xerrlist(:,a)=Xerr;
    end
    
    [l,w]=size(statelist);
end

statelist(l+1,1:12)=newstate; % save last newstate, since it is 1399 rows

finallist=statelist;

for i=st+1:ed % closing gripper
    finallist(i,13)=1;
end

plot(Xerrlist');
legend('wbx','wby','wbz','vbx','vby','vbz');
csvwrite('finallist',finallist);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Trajmatrix,st,ed]=TrajectoryGenerator(Tse,Tscinitial,Tscfinal,dt)
g=150; % gripper angle
Tcegrasp=[[cos(g*pi/180)  0 sin(g*pi/180)   0  ];
          [      0        1      0          0  ];
          [-sin(g*pi/180) 0 cos(g*pi/180) -0.01]; % to grab cube tightly, I offset 0.01
          [      0        0      0          1  ]];
        
Tcestandoff=[[ cos(g*pi/180) 0 sin(g*pi/180)  0  ];
             [       0       1       0        0  ];
             [-sin(g*pi/180) 0 cos(g*pi/180) 0.09];
             [       0       0       0        1  ]];
         
T1=Tscinitial*Tcestandoff;
T2=Tscinitial*Tcegrasp;
T3=T1;
T4=Tscfinal*Tcestandoff;
T5=Tscfinal*Tcegrasp;
T6=T4;

k=10;
Tfl1=4;
Tfl2=4;
Tfs=1;
Tfc=1;
Nl1=Tfl1*k/dt;
Nl2=Tfl2*k/dt;
Ns=Tfs*k/dt;
Nc=Tfc*k/dt;
method=3;
traj1 = CartesianTrajectory(Tse, T1, Tfl1, Nl1, method);
traj2 = CartesianTrajectory(T1, T2, Tfs, Ns, method);
traj3 = CartesianTrajectory(T2, T2, Tfc, Nc, method);
traj4 = CartesianTrajectory(T2, T3, Tfs, Ns, method);
traj5 = CartesianTrajectory(T3, T4, Tfl2, Nl2, method);
traj6 = CartesianTrajectory(T4, T5, Tfs, Ns, method);
traj7 = CartesianTrajectory(T5, T5, Tfc, Nc, method);
traj8 = CartesianTrajectory(T5, T6, Tfs, Ns, method);
Trajmatrix=[traj1 traj2 traj3 traj4 traj5 traj6 traj7 traj8];

translist=zeros(1,13);
for i = 1:Nl1+Nl2+4*Ns+2*Nc
    translist(i,1:12)=[Trajmatrix{1,i}(1,1) Trajmatrix{1,i}(1,2) Trajmatrix{1,i}(1,3) Trajmatrix{1,i}(2,1) Trajmatrix{1,i}(2,2) Trajmatrix{1,i}(2,3) Trajmatrix{1,i}(3,1) Trajmatrix{1,i}(3,2) Trajmatrix{1,i}(3,3) Trajmatrix{1,i}(1,4) Trajmatrix{1,i}(2,4) Trajmatrix{1,i}(3,4)];
end

for i = Nl1+Ns+1:Nl1+Nl2+3*Ns+Nc
    translist(i,13)=1;
end
st=(Nl1+Ns)/k;
ed=(Nl1+Nl2+3*Ns+Nc)/k;
csvwrite('translist',translist);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function newstate=NextState(chasisconfig,armconfig,wheelangle,armspeed,wheelspeed,dt)
max=2; % maximum value of wheel speed & armspeed
for i=1:4 %limit speed of wheels and arms
    if wheelspeed(i)>max
        wheelspeed(i)=max;
    elseif wheelspeed(i)<-max
            wheelspeed(i)=-max;
    end
end
for i=1:5
    if armspeed(i)>max
        armspeed(i)=max;
    elseif armspeed(i)<-max
           armspeed(i)=-max;
    end
end

deltatheta=wheelspeed*dt;
l=0.47/2;
w=0.3/2;
r=0.0475;
F=(r/4)*[[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w)];
         [    1      1       1        1    ];
         [   -1      1      -1        1    ]];
Vb=F*deltatheta';
Vb6=[0;0;Vb;0];
Tbbp=MatrixExp6(VecTose3(Vb6)); % transfornation matrix from current b to next b'
phi=chasisconfig(1);
x=chasisconfig(2);
y=chasisconfig(3);
Tsb=[[cos(phi) -sin(phi) 0     x  ];
     [sin(phi) cos(phi)  0     y  ];
     [   0        0      1  0.0963];
     [   0        0      0     1  ]];
        
newwheel=wheelangle+wheelspeed*dt;
newarm=armconfig+armspeed*dt;
Tsbp=Tsb*Tbbp;
[Rsbp, p] = TransToRp(Tsbp);

if Rsbp==eye(3) 
    theta=0;
elseif trace(Rsbp)==1
    theta=pi;
else
    theta = asin(Rsbp(2,1));
end

newchasis=[theta p(1) p(2)];
newstate=[newchasis newarm newwheel];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [u,thetadot,Xerradd,Xerr]=FeedbackControl(config,Xd,Xdnext,Kp,Ki,dt,Xerradd)

B1=[0; 0;1; 0;0.033;0];
B2=[0;-1;0;-0.5076;0;0];
B3=[0;-1;0;-0.3526;0;0];
B4=[0;-1;0;-0.2176;0;0];
B5=[0; 0;1;0;0;0];
Blist=[B1 B2 B3 B4 B5];

phi=config(1);
x=config(2);
y=config(3);
thetalist=config(4:8)';

Tsb = [[ cos(phi) -sin(phi)    0        x    ];
       [ sin(phi)  cos(phi)    0        y    ];
       [   0         0         1     0.0963  ];
       [   0         0         0        1    ]];
Tb0 = [[   1         0         0     0.1662  ];
       [   0         1         0        0    ];
       [   0         0         1     0.0026  ];
       [   0         0         0        1    ]];
   
M0e = [[   1         0         0     0.033   ];
       [   0         1         0        0    ];
       [   0         0         1     0.6546  ];
       [   0         0         0        1    ]];
Tn=FKinBody(M0e, Blist, thetalist);
X=Tsb*Tb0*Tn;

Xerr=se3ToVec(MatrixLog6(X\Xd));
Vd=(1/dt)*se3ToVec(MatrixLog6(Xd\Xdnext));
Ad=Adjoint(X\Xd);
AdVd=Ad*Vd;
Xerradd=Xerradd+Xerr*dt;
V=AdVd+(Kp*Xerr)+(Ki*Xerradd);

thetalist=config(4:8)';
Jarm = JacobianBody(Blist, thetalist);
l=0.47/2;
w=0.3/2;
r=0.0475;
F=(r/4)*[[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w)];
         [    1      1       1        1    ];
         [   -1      1      -1        1    ]];
F6=[zeros(2,4);F;zeros(1,4)];

Jbase=Adjoint(inv(Tn)*inv(Tb0))*F6;   
Je=[Jbase Jarm];

violatejoint=testjointlimit(config,Je,V);

if isempty(violatejoint)
   Je=Je;
else
    n=size(violatejoint);
    
    for x=1:n
        j=violatejoint(x);
        Je(:,j+4)=zeros(6,1);
    end   
end

uthetadot=pinv(Je,0.01)*V;
u=uthetadot(1:4);
thetadot=uthetadot(5:9);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function violatejoint=testjointlimit(config,Je,V)
uthetadot=pinv(Je,0.01)*V;
thetadot=uthetadot(5:9);
violatejoint=[];
dt=0.01;
% arm1
if (thetadot(1)*dt)+config(4)>2.932
    violatejoint=[violatejoint 1];
elseif (thetadot(1)*dt)+config(4)<-2.932
    violatejoint=[violatejoint 1];
end  

%arm2  
if (thetadot(2)*dt)+config(5)>1.533
    violatejoint=[violatejoint 2];
elseif (thetadot(2)*dt)+config(5)<-1.117
    violatejoint=[violatejoint 2];
end   

%arm3 
if (thetadot(3)*dt)+config(6)>2.53
    violatejoint=[violatejoint 3];
elseif (thetadot(3)*dt)+config(6)<-2.62
    violatejoint=[violatejoint 3];
end    

%arm4 
if (thetadot(4)*dt)+config(7)>1.78
    violatejoint=[violatejoint 4];
elseif (thetadot(4)*dt)+config(7)<-2.932
    violatejoint=[violatejoint 4];
end 

%arm5   
if thetadot(5)*dt+config(8)>2.932
    violatejoint=[violatejoint 5];
elseif thetadot(5)*dt+config(8)<-2.932
    violatejoint=[violatejoint 5];
end    
end
