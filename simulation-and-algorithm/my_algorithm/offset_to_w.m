function w=offset_to_w(w,a,m,R,m_local)
global ea em;
a=a/norm(a);
m=m/norm(m);
a1=R'*[0;0;-1];          %Òª×¢ÒâÏÂ´ÅÆ«½Ç
m1=R'*m_local;

ea1=cross(a,a1);
em1=cross(m,m1);

ea1=ea1/norm(ea1);
ea1=ea1/norm(ea1);

ea=ea+ea1;
em=em+em1;
w=w+0.001*ea1+0.001*em1+0.0005*ea+0.0005*em;
end

