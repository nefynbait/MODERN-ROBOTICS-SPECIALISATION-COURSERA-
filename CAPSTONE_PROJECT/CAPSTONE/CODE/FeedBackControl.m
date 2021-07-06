function [twist err]=FeedBackControl(X,Xd,Xdn,kp,ki,delt,err)
  Xerr=se3ToVec(MatrixLog6(X^(-1)*Xd));
  Xd;
  Xdn;
  vd=se3ToVec(MatrixLog6(Xd^(-1)*Xdn));
  vd=Adjoint(X^(-1)*Xd)*vd/delt;
  err=err+Xerr*delt;
  twist=vd+kp*Xerr + ki*err;
endfunction
