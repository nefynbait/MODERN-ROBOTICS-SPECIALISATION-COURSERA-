function thetafinal=main(Tsc1,Tsc2,iniconfig,refconfig,kp,ki)
  l=0.47/2;
  w=0.3/2;
  r=0.0475;
  Blist=[[0;0;1;0;0.1662;0],[0;0;0;1;0;0],[0;0;0;0;1;0],[0;0;1;0;0.033;0],[0;-1;0;-0.5076;0;0],[0;-1;0;-0.3526;0;0],[0;-1;0;-0.2176;0;0],[0;0;1;0;0;0]];
  thetalist=[0;0;0;0;0;0;0;0];
  ew=0.01;
  ev=0.01;
  M=[1,0,0,0.033;0,1,0,0;0,0,1,0.6546;0,0,0,1];
  
  %theta1=IKinBody(Blist,M,refconfig,thetalist,ew,ev);
  
  standoff=[0,0,1,0;0,1,0,0;-1,0,0,0.25;0,0,0,1];
  grip=[0,0,1,-0.015;0,1,0,0;-1,0,0,0.015;0,0,0,1];
  traj=TrajectoryGenerator(iniconfig,Tsc1,Tsc2,grip,standoff,1);
  
  %theta=[0.5 0 0.5 0 -1.57 0 0 0]';
  theta=[0 0 0 0 0 0 0 0]';
  %FKinBody(M,Blist,theta)
  
  [n1 n2]=size(traj);
  x=refconfig;
  chassi=theta([1:3],1);
  chassi=chassi';
  theta=theta([4:8],1);
  theta=theta';
  thetafinal=[chassi,theta,[0,0,0,0,0]];
  wheelpos=[0,0,0,0];
  err=0;
  Blist=[[0;0;1;0;0.033;0],[0;-1;0;-0.5076;0;0],[0;-1;0;-0.3526;0;0],[0;-1;0;-0.2176;0;0],[0;0;1;0;0;0]];
  f=r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1;-1 1 -1 1];
  Blist2=[[0;0;1;0;0.1662;0],[0;0;0;1;0;0],[0;0;0;0;1;0],[0;-1;0;-0.5076;0;0],[0;0;1;0;0;0]];
  i=1;
  delt=0.01;
  theta3=[0;0;0;0;0;0];
  for i = 1:(n1-1)
    fprintf("l")
    xdn=cell2mat(traj(i+1,1));
    xd=cell2mat(traj(i,1))
    [twist err]=FeedBackControl(x,xd,xdn,kp,ki,delt,err);
    twist';
    jacobianarm=JacobianBody(Blist,theta);
    jacobianwheel=[0,0,0,0;0,0,0,0;f;0,0,0,0];
    phi=chassi(1,1);
    cox=chassi(1,2);
    coy=chassi(1,3);
    tch=[cos(phi) -sin(phi) 0 cox;sin(phi) cos(phi) 0 coy;0 0 1 0;0 0 0 1];
    
    jacobianwheel=Adjoint(x^(-1)*tch)*jacobianwheel;
    
    jb=[jacobianwheel,jacobianarm];
    %jb=JacobianBody(Blist2,theta3);
    size(jb);
    %vel=jb\twist;
    vel=pinv(jb,1e-3)*twist;
    vel=round(vel*1000)/1000;
    vel';
    test=(jb*vel)';
    %vel=jb^(-1)*twist;
    
    vel';
    [chassi, theta, wheelpos]=NextState([chassi,theta,wheelpos],vel',delt,0);
    thetafinal=[thetafinal;[chassi,theta,wheelpos],0];
    thetafinal(i,:);
    thetafinal-1;
    thetafinal+0.99;
    %x=x*MatrixExp6(VecTose3(twist*0.01));
    
    phi=chassi(1,1);
    cox=chassi(1,2);
    coy=chassi(1,3);
    tch=[cos(phi) -sin(phi) 0 cox;sin(phi) cos(phi) 0 coy;0 0 1 0;0 0 0 1];
    
    tb0=[1 0 0 0.16662;0 1 0 0; 0 0 1 0.0026;0 0 0 1];
    e=FKinBody(M,Blist,theta');
    x=tch*tb0*e
  endfor  
  csvwrite('FINAL.csv',thetafinal);
endfunction