function [x_next] = Euler(X,U,h,f)

% Euler integration taken from exercise 7
   x_next = X + h*f(X,U);