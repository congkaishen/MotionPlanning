function dynamics(states, ctrl)
    la          = 1.56
    lb          = 1.64
    L           = la+lb

    x           = states[1]
    y           = states[2]
    psi         = states[3]
    ux          = states[4]
    sa          = states[5]
    sr          = ctrl[1]
    ax          = ctrl[2]

    dx          = ux * cos(psi) 
    dy          = ux * sin(psi)
    dpsi        = ux/L * tan(sa)
    dux         = ax
    dsa         = sr
    dstates     = [dx dy dpsi dux dsa]
    return dstates
end