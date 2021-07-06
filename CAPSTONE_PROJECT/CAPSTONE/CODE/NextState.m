function [chassi,theta,wheelpos] = NextState(pos,vel,delt,maxlim)
  newarm=pos(1,[4:8]) + vel(1,[5:9])*delt;
  newwheel=pos(1,[9:12]) + vel(1,[1:4])*delt;
  for i=1:5
    if(newarm(1,i)>6.28)
      newarm(1,i)=mod(newarm(1,i),6.28);
    elseif(newarm(1,i)<-6.28)
      temp=-newarm(1,i);
      temp=mod(temp,6.28);
      newarm(1,i)=-temp;
    endif
    if i<=4
      if(newwheel(1,i)>6.28)
        newarm(1,i)=mod(newwheel(1,i),6.28);
      elseif(newwheel(1,i)<-6.28)
      temp=-newwheel(1,i);
      temp=mod(temp,6.28);        
      newwheel(1,i)=-temp;
      endif
    endif
  endfor
  l=0.47/2;
  w=0.3/2;
  r=0.0475;
  f=r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1;-1 1 -1 1];
  deltheta=vel(1,[1:4])*delt;
  twist=f*deltheta';
  twist6=[0; 0; twist; 0];
  T=MatrixExp6(VecTose3(twist6));
  %tsb=MatrixExp6(VecTose3([0;0;pos(1,1);0;0;0]));
  %T=tsb*T;
  newco=[T(1,4) T(2,4)];
  phi=pos(1,1)+acos(T(1,1));
  if phi>6.28
    phi=phi-6.28;
  elseif phi<-6.28
    phi=phi+6.28;
  endif
  chassi=[phi pos(1,[2:3])+newco];
  theta=newarm;
  wheelpos=newwheel;
  %newjoint=[phi pos(1,[2:3])+newco newarm newwheel];
endfunction
