function [q,q1,q2,q3]=jiesuan(a,w,m,Q,fs,m_local)
global T ea em qk pk I R alpha lamda;
I=[1 0 0 0; 0 1 0 0; 0 0 1 0;0 0 0 1];
p_temp=[1 0;0 0;0 0;0 0];
num=size(w,2);
q=zeros(4,num);
q(:,1)=[1;0;0;0];
alpha=1.1;
lamda=0.1;
T=1/fs;ea=0;em=0;   %qk初值不能这样设：q作为旋转四元数有关系的？
p_temp(:,2)=gy_upd_q(w(:,1),w(:,1),[1;0;0;0]);
qk=p_temp(:,2);
q(:,2)=qk;
%没用 p=zeros(4,num);
pk=cov(p_temp');
%此q为载体系到参考系的
 q1_2=q(1,2)^2;q2_2=q(2,2)^2;q3_2=q(3,2)^2;q4_2=q(4,2)^2;q12=q(1,2)*q(2,2);
    q13=q(1,2)*q(3,2);q14=q(1,2)*q(4,2);q23=q(2,2)*q(3,2);q24=q(2,2)*q(4,2);
    q34=q(3,2)*q(4,2);
    R=[(q1_2+q2_2-q3_2-q4_2) 2*(q23-q14) 2*(q24+q13);
       2*(q23+q14) (q1_2-q2_2+q3_2-q4_2) 2*(q34-q12);
       2*(q24-q13) 2*(q34+q12) (q1_2-q2_2-q3_2+q4_2)];  
q1=zeros(4,num);
q2=zeros(4,num);
q3=zeros(4,num);
%没用 p(:,1)=q(:,1);
%没用 p(:,2)=[q(1,2);-q(2,2);q(3,2);q(4,2)];
for i=3:num
    [qg,dq]=gy_upd_q(w(:,i-1),w(:,i),qk);
    qmg=m_a_q(a(:,i),m(:,i),qk,alpha,dq);
    q2(:,i)=qmg;
    q3(:,i)=qg;
    q_hubu=lamda*qg+(1-lamda)*qmg;
    q_hubu=q_hubu/norm(q_hubu,2);
    q1(:,i)=q_hubu;
    [qk,pk]=kalman(w(:,i-1),w(:,i),q_hubu,Q);
    q(:,i)=qk;
%R    
    q1_2=q(1,i)^2;q2_2=q(2,i)^2;q3_2=q(3,i)^2;q4_2=q(4,i)^2;q12=q(1,i)*q(2,i);
    q13=q(1,i)*q(3,i);q14=q(1,i)*q(4,i);q23=q(2,i)*q(3,i);q24=q(2,i)*q(4,i);
    q34=q(3,i)*q(4,i);
    R=[(q1_2+q2_2-q3_2-q4_2) 2*(q23-q14) 2*(q24+q13);
       2*(q23+q14) (q1_2-q2_2+q3_2-q4_2) 2*(q34-q12);
       2*(q24-q13) 2*(q34+q12) (q1_2-q2_2-q3_2+q4_2)];
   



  
end

end

