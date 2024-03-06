#the code is prototyped based on the following link: https://atsushisakai.github.io/PythonRobotics/modules/path_planning/reeds_shepp_path/reeds_shepp_path.html
using Plots
function polar(bg_pt, ed_pt)
    vec = ed_pt - bg_pt
    r = sqrt(sum(vec.^2))
    θ = atan(vec[2], vec[1])
    return r, θ
end


function twoπmode(angle)
    while (angle < 0) | (angle > 2*pi) 
        if angle < 0
            angle = angle + 2*pi
        elseif angle > 2*pi
            angle = angle - 2*pi
        else
            return angle
        end
    end
    return angle
end

# function πmode(angle)
#     while (angle < -pi) | (angle >pi) 
#         if angle < -pi
#             angle = angle + 2*pi
#         elseif angle > pi
#             angle = angle - 2*pi
#         else
#             return angle
#         end
#     end
#     return angle
# end

function circleL(R, ox, oy, θ) 
# we assume that angle starts at negative y axis, ccwise
    return ox.+R*sin.(θ), oy.-R*cos.(θ)
end

function circleR(R, ox, oy, θ) 
# we assume that angle starts at positive y axis, ccwise
    return ox.-R*sin.(θ), oy.+R*cos.(θ)
end

function straight(ox, oy, θ, u)
    return ox + u*cos(θ), oy + u*sin(θ)
end



# TYPE 1
function LSL(minR, x, y, ψ)
    A_pos = [0, minR]
    C_pos = [x - minR*sin(ψ), y + minR*cos(ψ)]
    u,t = polar(A_pos, C_pos)
    v = ψ - t
    t = twoπmode(t)
    v = twoπmode(v)

    cost = minR*t+u+minR*v
    return t,u,v,cost
end

function plotLSL(minR, xg, yg, ψ, t, u)
    #plot L path
    xl1,yl1 = circleL(minR, 0, minR, collect(LinRange(0,t,20)))
    #plot S path
    xs,ys = straight(xl1[end], yl1[end], t, u)
    #plot L path
    xl2,yl2 = circleL(minR, xg - minR*sin(ψ), yg + minR*cos(ψ), collect(LinRange(t,ψ,20)))
    # print(vec([xl1,xs,xl2]))
    turn_x_pts = [xl1[end], xs]
    turn_y_pts = [yl1[end], ys]
    plot(vcat(xl1,xs,xl2), vcat(yl1,ys,yl2), aspect_ratio = :equal )
    scatter!(turn_x_pts, turn_y_pts, mc=:red, ms=2, ma=0.5, legend = false)

end


# TYPE 2
function LSR(minR, x,y,ψ)
    C_pos = [x+minR*sin(ψ), y-minR*cos(ψ)]
    A_pos = [0,minR]
    u1,t1 = polar(A_pos, C_pos)
    BF = minR
    if u1  <= 2*minR
        return 0, 0, 0, Inf
    end
    u = sqrt(u1^2-(2*minR)^2)
    FD = u/2
    θ = atan(BF/FD)
    t = t1 + θ
    v = t - ψ
    t = twoπmode(t)
    v = twoπmode(v)
    cost = minR*t+u+minR*v
    return t,u,v, cost
end

function plotLSR(minR, xg, yg, ψ, t, u)
    #plot L path
    xl1,yl1 = circleL(minR, 0, minR, collect(LinRange(0,t,20)))
    #plot S path
    xs,ys = straight(xl1[end], yl1[end], t, u)
    #plot L path
    xr,yr = circleR(minR, xg + minR*sin(ψ), yg - minR*cos(ψ), collect(LinRange(t,ψ,20)))
    # print(vec([xl1,xs,xl2]))
    turn_x_pts = [xl1[end], xs]
    turn_y_pts = [yl1[end], ys]
    plot!(vcat(xl1,xs,xr), vcat(yl1,ys,yr), aspect_ratio = :equal )
    scatter!(turn_x_pts, turn_y_pts, mc=:red, ms=2, ma=0.5, legend = false)
end


# TYPE 3
function LxRxL(minR, x,y,ψ)
    D_pos = [x - minR*sin(ψ), y + minR*cos(ψ)]
    B_pos = [0, minR]
    u1, θ = polar(B_pos, D_pos)
    if u1 > 4*minR
        return 0,0,0,Inf
    end
    A_ang = acos((u1/2)/(2*minR))
    t = pi/2 + θ + A_ang
    u = pi - 2*A_ang
    v = ψ - t - u

    t = twoπmode(t)
    u = twoπmode(u)
    v = twoπmode(v)

    cost = minR*t + minR*u+ minR*v
    return t,u,v, cost
end

function plotLxRxL(minR,xg,yg,ψ, t,u,v)
    #plot L path
    xl1,yl1 = circleL(minR, 0, minR, collect(LinRange(0,t,20)))
    #plot R path
    xr,yr = circleR(minR, 2*minR*sin(t), minR - 2*minR*cos(t), collect(LinRange(t,t+u,20)))
    #plot L path
    xl2,yl2 = circleL(minR, xg - minR*sin(ψ), yg + minR*cos(ψ), collect(LinRange(t+u,t+u+v,20)))
    
    turn_x_pts = [xl1[end], xr[end]]
    turn_y_pts = [yl1[end], yr[end]]
    plot!(vcat(xl1,xr,xl2), vcat(yl1,yr,yl2), aspect_ratio = :equal )
    scatter!(turn_x_pts, turn_y_pts, mc=:red, ms=2, ma=0.5, legend = false)
end

# TYPE 4
function LxRL(minR, x,y,ψ)
    D_pos = [x - minR*sin(ψ), y + minR*cos(ψ)]
    B_pos = [0, minR]
    u1, θ = polar(B_pos, D_pos)
    if u1 > 4*minR
        return 0,0,0,Inf
    end
    A_ang = acos((u1/2)/(2*minR))
    t = pi/2 + θ + A_ang
    u = pi - 2*A_ang
    v = -ψ + t + u

    t = twoπmode(t)
    u = twoπmode(u)
    v = twoπmode(v)
    cost = minR*t + minR*u + minR*v
    return t,u,v, cost
end


function plotLxRL(minR, xg,yg,ψ, t,u,v)
    #plot L path
    xl1,yl1 = circleL(minR, 0, minR, collect(LinRange(0,t,20)))
    #plot R path
    xr,yr = circleR(minR, 2*minR*sin(t), minR - 2*minR*cos(t), collect(LinRange(t,t+u,20)))
    #plot L path
    xl2,yl2 = circleL(minR, xg - minR*sin(ψ), yg + minR*cos(ψ), collect(LinRange(t+u,t+u-v,20)))
    turn_x_pts = [xl1[end], xr[end]]
    turn_y_pts = [yl1[end], yr[end]]
    plot!(vcat(xl1,xr,xl2), vcat(yl1,yr,yl2), aspect_ratio = :equal )
    scatter!(turn_x_pts, turn_y_pts, mc=:red, ms=2, ma=0.5, legend = false)
end

# TYPE 5
function LRxL(minR, x,y,ψ)
    D_pos = [x - minR*sin(ψ), y + minR*cos(ψ)]
    B_pos = [0, minR]
    u1, θ = polar(B_pos, D_pos)
    if u1 > 4*minR
        return 0,0,0,Inf
    end

    u = acos(1 - (u1^2)/(8*(minR^2)))
    A_ang = asin(2*minR*sin(u)/u1)

    t = pi/2 + θ - A_ang
    v =  t - u - ψ

    u = twoπmode(u)
    t = twoπmode(t)
    v = twoπmode(v)

    cost = minR*t + minR*u + minR*v
    return t,u,v, cost
end


function plotLRxL(minR, xg,yg,ψ, t,u,v)
    #plot L path
    xl1,yl1 = circleL(minR, 0, minR, collect(LinRange(0,t,20)))
    #plot R path
    xr,yr = circleR(minR, 2*minR*sin(t), minR - 2*minR*cos(t), collect(LinRange(t,t-u,20)))
    #plot L path
    xl2,yl2 = circleL(minR, xg - minR*sin(ψ), yg + minR*cos(ψ), collect(LinRange(t-u,t-u-v,20)))
    turn_x_pts = [xl1[end], xr[end]]
    turn_y_pts = [yl1[end], yr[end]]
    plot!(vcat(xl1,xr,xl2), vcat(yl1,yr,yl2), aspect_ratio = :equal )
    scatter!(turn_x_pts, turn_y_pts, mc=:red, ms=2, ma=0.5, legend = false)
end

# TYPE 6
function LRxLR(minR, x,y,ψ)
# hint: forms four circles, centers are B->C->L->G ==> BC = CL = LG = 2*R
# the trapezoid BGCL is iso-trapezoid ->∠LCG = ∠CLG = ∠CBG = u1
# then check case u1 < 2*minR or u1 >2*minR, use iso-trapezoid's proportion rules
    G_pos = [x + minR*sin(ψ), y - minR*cos(ψ)]
    B_pos = [0, minR]
    u1, θ = polar(B_pos, G_pos)
    if (u1 > 4*minR)
        return 0,0,0,Inf
    end

    if u1 <= 2*minR
        u = acos((u1 + 2*minR)/(4*minR))
        t = pi/2 + θ + u
        u = twoπmode(u)
        t = twoπmode(t)

    else
        u = acos((2*minR-u1)/(4*minR))
        t = -pi/2 + θ + u
        u = twoπmode(u)
        t = twoπmode(t)
    end
    v = ψ - t + 2*u    
    v = twoπmode(v)

    cost = minR*t + minR*u + minR*v
    return t,u,v, cost
end


function plotLRxLR(minR, xg,yg,ψ, t,u,v)
    #plot L1 path
    xl1,yl1 = circleL(minR, 0, minR, collect(LinRange(0,t,20)))
    #plot R1 path
    xr1c = 2*minR*sin(t)
    yr1c = minR - 2*minR*cos(t)
    xr1,yr1 = circleR(minR, xr1c, yr1c, collect(LinRange(t,t-u,20)))
    
    #plot L2 path
    xl2c = xr1c - 2*minR*sin(t-u)
    yl2c = yr1c + 2*minR*cos(t-u)
    xl2,yl2 = circleL(minR, xl2c, yl2c, collect(LinRange(t-u,t-2*u,20)))
    
    #plot L path
    xr2,yr2 = circleR(minR, xg + minR*sin(ψ), yg - minR*cos(ψ), collect(LinRange(t-2*u,t-2*u+v,20)))
    turn_x_pts = [xl1[end], xr1[end], xl2[end]]
    turn_y_pts = [yl1[end], yr1[end], yl2[end]]
    plot!(vcat(xl1,xr1,xl2, xr2), vcat(yl1,yr1,yl2,yr2), aspect_ratio = :equal )
    scatter!(turn_x_pts, turn_y_pts, mc=:red, ms=2, ma=0.5, legend = false)
end


# function 

minR =1
x = 1
y = 3
ψ = pi/2

h = plot()

# t,u,v, cost = LSL(minR, x, y, ψ)
# println("LSL cost is: ", cost)
# plotLSL(minR, x, y, ψ, t, u)

# t,u,v, cost = LSR(minR, x, y, ψ)
# println("LSR cost is: ", cost)
# plotLSR(minR, x, y, ψ, t, u)

# t,u,v,cost = LxRxL(minR, x, y, ψ)
# println("LxRxL cost is: ", cost)
# plotLxRxL(minR, x, y, ψ, t, u, v)


# t,u,v,cost = LxRL(minR, x, y, ψ)
# println("LxRL cost is: ", cost)
# plotLxRL(minR, x, y, ψ, t, u, v)

t,u,v,cost = LRxL(minR, x, y, ψ)
println("LRxL cost is: ", cost)
plotLRxL(minR, x, y, ψ, t, u, v)


# t,u,v,cost = LRxLR(minR, x, y, ψ)
# println("LRxLR cost is: ", cost)
# plotLRxLR(minR, x, y, ψ, t, u, v)
