function qmg=m_a_q(a,m,q_last,alpha,dqg)
global T;
a=a/norm(a,2);
m=m/norm(m,2);   %单位化
f=[2*(q_last(2)*q_last(4)-q_last(1)*q_last(3))-a(1);
   2*(q_last(1)*q_last(2)+q_last(3)*q_last(4))-a(2);
   1-2*(q_last(2)^2+q_last(3)^2)-a(3);
   0.5+2*(0.866*(q_last(2)*q_last(4)-q_last(1)*q_last(3))-0.5*(q_last(3)^2+q_last(4)^2))-m(1);
   2*0.5*(q_last(2)*q_last(3)-q_last(1)*q_last(4))+2*0.866*(q_last(1)*q_last(2)+q_last(2)*q_last(4))-m(2);
   2*0.5*(q_last(1)*q_last(3)+q_last(2)*q_last(4))-2*0.866*(q_last(2)^2+q_last(3)^2)+0.866-m(3)];  %测量与计算得到的之间的差值
j=[-2*q_last(3) 2*q_last(4) -2*q_last(1) 2*q_last(2);    
   2*q_last(2) 2*q_last(1) 2*q_last(4) 2*q_last(3);
   0 -4*q_last(2) -4*q_last(3) 0;
   -(433*q_last(3))/250 (433*q_last(4))/250 -(433*q_last(1))/250-2*q_last(3) (433*q_last(2))/250-2*q_last(4);
   (433*q_last(2))/250-q_last(4) (433*q_last(1))/250+q_last(3)+(433*q_last(4))/250 q_last(2) (433*q_last(2))/250 - q_last(1);
   q_last(3) q_last(4)-(433*q_last(2))/125 q_last(1)-(433*q_last(3))/125 q_last(2)];    %f的雅克比矩阵
f1=j'*f;  %?f
qmg=q_last-(f1/norm(f1,2))*alpha*T*norm(dqg,2);  %预测更新
qmg=qmg/norm(qmg,2);  %单位化
end

