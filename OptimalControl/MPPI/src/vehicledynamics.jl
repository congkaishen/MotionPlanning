function VehicleDynamics(states, ctrl)
    la          = 1.5521
    lb          = 2.715-la
    M           = 1.269e+03
    Izz         = 1.620e+03
    g           = 9.81
    mu          = 0.8
    KFZF        = 297.9598
    KFZR        = 391.1902
    KFZX        = 289.5
    Tire_par    = [-4.8543,1.4376,1.2205,0.1556,-0.0023,41.3705]

    B           = Tire_par[1]/mu; 
    C           = Tire_par[2] 
    E           = Tire_par[4] 
    Sh          = Tire_par[5] 
    Sv          = Tire_par[6]

    x           = states[1]
    y           = states[2]
    v           = states[3]
    r           = states[4]
    psi         = states[5]
    ux          = states[6]
    sa          = states[7]
    sr          = ctrl[1]
    ax          = ctrl[2]

    FZF         = 2*(KFZF * g - (ax - r * v) * KFZX)
    FZR         = 2*(KFZR * g + (ax- r * v) * KFZX)
    alpha_f     = atan((v + la * r) / (ux + 0.01)) - sa
    alpha_r     = atan((v - lb * r) / (ux + 0.01))

    X1_f        = min(max(B * (-alpha_f + Sh), -pi/2 + 0.001), pi/2 - 0.001)
    FY1         = -(mu * FZF * Tire_par[3] * sin(C * atan(X1_f - E * (X1_f - atan(X1_f)))) + Sv)
    X1_r        = min(max(B * (-alpha_r + Sh), -pi/2 + 0.001), pi/2 - 0.001)
    FY2         = -(mu * FZR * Tire_par[3] * sin(C * atan(X1_r - E * (X1_r - atan(X1_r)))) + Sv)

    dx          = ux * cos(psi) - v * sin(psi)
    dy          = ux * sin(psi) + v * cos(psi)
    dpsi        = r
    dv          = (FY1 + FY2) / M -  r * ux
    dr          = (FY1 * la - FY2 * lb) / Izz
    du          = ax
    dsa         = sr
    dstates     = [dx; dy; dv; dr; dpsi; du; dsa]
    constraint  = 0;
    cost        = v^2 * 1 + 1 * r^2 + 5 * ax^2 + 3 * sr^2 + 2 * sa^2 + 2 * y^2;
    return dstates, constraint, cost
end