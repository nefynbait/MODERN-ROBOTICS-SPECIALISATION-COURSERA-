function trajectory=TrajectoryGenerator(tse,tscini,tscf,tcegr,tcest,k)
  %tse=initial end effector congif
  %tscini=cube initial configuration
  %tscf=cube final configuration
  %tcegr=grasping configuration wrt cube
  %tcest=standoff configuration wrt cube
  %1st trajectory
  %Tse to tcest
  tsec=tscini*tcest;%configuration wrt to space frame;
  trajectory1=CartesianTrajectory(tse,tsec,1,1*k/0.01,3);
  %step=1;
  %2md trajectory
  tsg=tscini*tcegr;
  trajectory2=CartesianTrajectory(tsec,tsg,1,1*k/0.01,3);
  step=2;
  
  %3rd trajectory
  trajectory3=tsg;
  step=3;
  
  %4th trajectory
  trajectory4=CartesianTrajectory(tsg,tsec,1,1*k/0.01,3);
  
  
  %5th trajectory
  tsec2=tscf*tcest;%configuration wrt to space frame;
  trajectory5=CartesianTrajectory(tsec,tsec2,1,1*k/0.01,3);
  
  %6th trajectory
  tsg2=tscf*tcegr;
  trajectory6=CartesianTrajectory(tsec2,tsg2,1,1*k/0.01,3);
  
  %7th trajectory
  trajectory7=tsg;
  %8th trajectory
  trajectory8=CartesianTrajectory(tsg2,tsec2,1,1*k/0.01,3);
  
  %trajectory=trajectory1;
  trajectory=[trajectory1;trajectory2;trajectory3;trajectory4;trajectory5;trajectory6;trajectory7;trajectory8];
  size(trajectory)
  %csvwrite('output.csv',trajectory);
  
endfunction

