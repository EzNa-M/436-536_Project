function [x] = static_state_feedback(sys,t,x0,K,xtilde,utilde)

Vmax = 6; %is it 6, nominal is 6, but max is what (maybe 8.6) 

u = @(x) max(-Vmax, min(Vmax, utilde - K*(x - xtilde)));



[~,x] = ode45(@(t,x) sys.dynamics(x,u(x)),t,x0);

end