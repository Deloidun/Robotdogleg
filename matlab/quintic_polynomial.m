function [output] = quintic_polynomial(q0,qf,tf,increment)
    t0 = 0; v0 = 0; a0 = 0;  vf = 0; af = 0;    % Setup
    
    % q0 = 0;     % Start angle
    % qf = 2*pi;  % Final angle/dist
    % tf = 10;    % Final time
    
    T = [1, t0, t0^2,   t0^3,    t0^4,    t0^5;...
         0,  1, 2*t0, 3*t0^2,  4*t0^3,  5*t0^4;...
         0,  0,    2,   6*t0, 12*t0^2, 20*t0^3;...
         1, tf, tf^2,   tf^3,    tf^4,    tf^5;...
         0,  1, 2*tf, 3*tf^2,  4*tf^3,  5*tf^4;...
         0,  0,    2,   6*tf, 12*tf^2, 20*tf^3];
    Q = [q0; v0; a0; qf; vf; af];
    C = T\Q;
    
    t = linspace(t0,tf,increment);
    % q = C(1) + C(2).*t + C(3).*t.^2 + C(4).*t.^3 + C(5).*t.^4 + C(6).*t.^5;
    q = polyval(flip(C),t);
    v = C(2) + C(3)*2.*t + C(4)*3.*t.^2 + C(5)*4.*t.^3 + C(6)*5.*t.^4;
    output = [q;v];
    % figure(2)
    % plot(t,q,'*');
    % hold on 
    % plot(t,v);
    % grid on
end

