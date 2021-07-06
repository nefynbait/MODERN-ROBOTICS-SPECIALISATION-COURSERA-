function simulate=simu2(thetaini,Mlist,Glist,Slist,g)
  [row col]=size(thetaini);
  dtheta=zeros(row,1); 
  tau=zeros(row,1);
  ftip=zeros(6,1);
  ddtheta=[];
  simulate=zeros(501,row);
  simulate(1,:)=thetaini';
  r=2;
  for t=0:1/100:3
    ddtheta=ForwardDynamics(thetaini,dtheta,tau,g,ftip,Mlist,Glist,Slist);
    thetaini=thetaini+dtheta*(1/100)+(1/100)*(1/100)*ddtheta/2; %(finding the position after one time step)
    dtheta=dtheta+(1/100)*ddtheta;  %(finding the velocity after one time step)
    simulate(r,:)=thetaini'; %(saving the postion in simulate var)
    r=r+1;
  endfor
  csvwrite('output2.csv',simulate);
endfunction
