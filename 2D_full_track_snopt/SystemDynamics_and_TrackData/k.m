function curvature = k(x)
% L=100+20*3*pi/4+50.7107+20*pi+50.7107+20*3*pi/4;
% %x=mod(x,L);
% if x<=100
%     curvature=0;
% elseif x>100 && x<=100+20*3*pi/4
%     curvature=1/20;
% elseif x>100+20*3*pi/4 && x<=100+20*3*pi/4+50.7107
%     curvature=0;
% elseif x>100+20*3*pi/4+50.7107 && x<=100+20*3*pi/4+50.7107+20*pi
%     curvature=1/40;
% elseif x>100+20*3*pi/4+50.7107+20*pi && x<=100+20*3*pi/4+50.7107+20*pi+50.7107
%     curvature=0;
% elseif x>100+20*3*pi/4+50.7107+20*pi+50.7107 && x<=L
%     curvature=1/20;
% elseif x>L && x<=L+100
%     curvature=0;
% elseif x>L+100 && x<=L+100+20*3*pi/4
%     curvature=1/20;
% elseif x>L+100+20*3*pi/4 && x<=L+100+20*3*pi/4+50.7107
%     curvature=0;
% elseif x>L+100+20*3*pi/4+50.7107 && x<=L+100+20*3*pi/4+50.7107+20*pi
%     curvature=1/40;
% elseif x>L+100+20*3*pi/4+50.7107+20*pi && x<=L+100+20*3*pi/4+50.7107+20*pi+50.7107
%     curvature=0;
% elseif x>L+100+20*3*pi/4+50.7107+20*pi+50.7107 && x<=2*L
%     curvature=1/20;
% else 
%     curvature=0;
% end

%curvature=1/40;

curvature=(-1137/8000000000).*pi.*x.^2+(87/80000000000).*pi.*x.^3+( ...
  -4083/1600000000000000).*pi.*x.^4+(9639/4000000000000000000).*pi.* ...
x.^5+(-3213/4000000000000000000000).*pi.*x.^6;
end