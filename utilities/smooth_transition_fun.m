% define a smooth transition function between two values



function  fun = smooth_transition_fun(x_range, y_range, ascend)

y_a = y_range(1);
y_b = y_range(2);
magnitude = y_range(2) - y_range(1);

syms f(t)
f(t) = piecewise(t <= 0, 0 ,t > 0, exp(-1/t));

syms fun(x)
if ascend 
    fun(x) = magnitude*(f(x-x_range(1)))/(f(x-x_range(1))+f(x_range(2)-x)) + y_a;
else
    fun(x) = magnitude*(f(x_range(2)-x))/(f(x-x_range(1))+f(x_range(2)-x)) + y_a;
end

end