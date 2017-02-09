function [tau, yss] = analyzeStep(t,v)
    dt = t(2)-t(1);
    vd = diff(v)/dt;
    lnV = log(vd);
    t= t(2:end);
    p = polyfit(t, lnV, 1);
    tau = -1/(p(1))
    yss = tau*max(vd)
    figure
    plot(t,vd)
end