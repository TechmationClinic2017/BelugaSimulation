function intg = cdrag(xt,xt2,xn,xn2,d,a,a_offset,n,c,theta,l,lf,dx,xb);
%This function performs the integration for the first equation of 4.8
in
%the REMUS thesis
intg = 0;
xti = l;
for x = xt:dx:xt2
 %Need to adjust for the difference in origin definitions between
the
 %radius functions and the body-coord with the following transform:
 Rx = ( 1/2*d - ( 3*d/(2*c^2) - tan(theta)/c )*(xti-lf)^2 + ...
 ( d/(c^3) - tan(theta)/(c^2) )*(xti - lf)^3 );
 xti = xti-dx;

 intg = intg + 2*Rx*dx;
end
for x = xt2:dx:xn
 Rx = d/2;
 intg = intg + 2*Rx*dx;
end
for x = xn:dx:xn2
 %Need to adjust for the difference in origin definitions between
the
 %radius functions and the body-coord with the following transform:
 xi = -( x - abs(xb) + (a-a_offset) ) + a;
 Rx = ( 1/2*d*( 1 - ( (xi + a_offset-a)/a )^2 )^(1/n) );

 intg = intg + 2*Rx*dx;
end
%End of cdrag function