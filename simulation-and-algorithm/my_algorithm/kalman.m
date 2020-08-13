function [qk,pk]=kalman(w,w1,z,Q)
global T qk pk I;
R=[0 -w(1) -w(2) -w(3);
     w(1) 0 w(3) -w(2);
     w(2) -w(3) 0 w(1);
     w(3) w(2) -w(1) 0];
R1=[0 -w1(1) -w1(2) -w1(3);
     w1(1) 0 w1(3) -w1(2);
     w1(2) -w1(3) 0 w1(1);
     w1(3) w1(2) -w1(1) 0];
R2=(R+R1)/2;  %平均
R3=R1*R;   %
fai=I+T*(3*R2+T*R2*R/4+T*R2^2/4+T^2*R2*R^2/32+T^2*R2*R3/32+T*R1*R2/4+T^2*R1*R2^2/16+T^2*R1*R2*R^2/16+T^2*R1*R2*R3/16)/6; %以上都是用来算状态更新方程的 
q2=fai*qk;  
q2=q2/norm(q2,2);
p2=fai*pk*fai';
k=p2/(p2+Q);
qk=q2+k*(z-q2);
qk=qk/norm(qk,2);
pk=(I-k)*p2;             %这些式子直接用的是kalman那5个式子
end
