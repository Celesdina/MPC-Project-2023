function [x_next] = RK4(X,U,h,f)

% Runge-Kutta 4 integration taken from exercise 7
   k1 = f(X,         U);
   k2 = f(X+h/2*k1, U);
   k3 = f(X+h/2*k2, U);
   k4 = f(X+h*k3,   U);
   x_next = X + h/6*(k1+2*k2+2*k3+k4);
end