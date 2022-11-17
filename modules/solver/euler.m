function yout =  euler(fun,dt,t0,y0)


f1 = fun(t0,y0);

yout = y0 + dt*f1;