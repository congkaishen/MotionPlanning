function plotTree(h, rrt::RRTSearcher)
    tree = getTree(rrt)
    for i in 1:2:size(tree, 2)
        h = plot!(h, tree[1, i:i+1], tree[2, i:i+1],color=:green, legend=false)
    end
    return current()
end

function ellipseShape(x, y, a, b)
	theta = LinRange(0, 2*pi,100)
	x .+ a*cos.(theta), y .+ b*sin.(theta)
end

function plotEnv(rrt::RRTSearcher)
    obs_info = rrt.s.obstacle_list
    obs_num = size(obs_info,1)
    theta = 0:0.01:2*pi
    # plot()
    for obs_idx in 1:obs_num
        ox = obs_info[obs_idx][1]
        oy = obs_info[obs_idx][2]
        r = obs_info[obs_idx][3]
        h = plot!(ellipseShape(ox,oy,r,r),
        seriestype = [:shape],
        lw = 0.5,
        c = :grey,
        linecolor = :black,
        fillalpha = 0.7,
        xlabel = "X[m]",
        ylabel = "Y[m]",
        title = string("RRT result")
        )
    end
    return current()
end

# function RRTplot(tpp::TPPSearcher)
#     rrt = tpp.rrt
#     h = plotEnv(rrt)
#     h = plotTree(h, rrt)

#     if rrt.r.status == :Solved
#         println("Solved")
#         plot!(h, rrt.r.Path[1,:], rrt.r.Path[2,:],
#         color=:blue,
#         lw = 2.
#         )
#         savefig("Results\\rrt_result.png")

#         xpath = rrt.p.states_collection[1,  rrt.r.resultIdxs]
#         ypath = rrt.p.states_collection[2,  rrt.r.resultIdxs]
#         zpath = rrt.p.states_collection[3,  rrt.r.resultIdxs]
#         upath = rrt.p.states_collection[5,  rrt.r.resultIdxs]

#         xpath = reverse(xpath)
#         ypath = reverse(ypath)
#         zpath = reverse(zpath)
#         upath = reverse(upath)
#         dis_path = sqrt.( (xpath[2:end]-xpath[1:end-1]).^2+(ypath[2:end]-ypath[1:end-1]).^2 + (zpath[2:end]-zpath[1:end-1]).^2)
#         uavg_path = (upath[2:end]+upath[1:end-1])/2
#         udif_path = (upath[2:end]-upath[1:end-1])
#         dt_path =  dis_path./uavg_path
#         acc_path = udif_path./dt_path

#         n = size(xpath, 1)
#         ################# ACC #######################
#         max_acc_list = Vector{Float64}()
#         min_acc_list = Vector{Float64}()

#         max_acc_torque_list = Vector{Float64}()
#         min_acc_torque_list = Vector{Float64}()

#         max_acc_fric_list = Vector{Float64}()
#         min_acc_fric_list = Vector{Float64}()


#         tol_dist = 0
#         tol_dists = deepcopy(dis_path)

#         for i in 1:n-1
#             tol_dists[i] = tol_dist
#             tol_dist = tol_dist + dis_path[i]
#             h1 = [xpath[i+1]; ypath[i+1]; zpath[i+1]]- [xpath[i]; ypath[i]; zpath[i]]
#             h1 = h1/norm(h1)

#             R1 = euler2Rot(rrt.p.nodes_collection[rrt.r.resultIdxs[n-i+1]].eulerang)
#             R2 = euler2Rot(rrt.p.nodes_collection[rrt.r.resultIdxs[n-i]].eulerang)

#             n1 = R1[:,3]
#             n2 = R2[:,3]
#             n_temp = n1+n2
#             n_temp = n_temp/norm(n_temp)


#             init_h = R1[:,1]
#             verti_ang_vec = cross(cross(init_h,  h1), init_h)
#             verti_ang = acos( round(dot(init_h,  h1)/(norm(init_h)*norm(h1)), digits = 2  ) )*sign(verti_ang_vec[3])

#             lat_vec = cross(n_temp, h1)
#             lat_vec = lat_vec/norm(lat_vec)
#             normal_vec = cross(h1, lat_vec)
#             normal_vec = normal_vec/norm(normal_vec)

#             centrifugal = -(upath[i]^2/dis_path[i])*getChildYaw(rrt.p.nodes_collection[rrt.r.resultIdxs[n-i+1]], rrt.p.nodes_collection[rrt.r.resultIdxs[n-i]])
#             y_friction = -lat_vec[3]*g + centrifugal
#             tol_friction = mu_f* (abs(normal_vec[3]*g) +  (upath[i]^2/dis_path[i])*verti_ang )


#             x_frinction = sqrt(tol_friction^2-y_friction^2)
#             x_acc = min(x_frinction, torque_acc)

#             max_acc = x_acc  -1* h1[3]*g
#             min_acc = -x_acc  -1* h1[3]*g

#             max_acc_torque = torque_acc  -1* h1[3]*g
#             min_acc_torque = -torque_acc  -1* h1[3]*g

#             max_acc_fric = x_frinction  -1* h1[3]*g
#             min_acc_fric = -x_frinction  -1* h1[3]*g

#             max_acc_list = [max_acc_list; max_acc]
#             min_acc_list = [min_acc_list; min_acc]
#             max_acc_torque_list = [max_acc_torque_list; max_acc_torque]
#             min_acc_torque_list = [min_acc_torque_list; min_acc_torque]
#             max_acc_fric_list = [max_acc_fric_list; max_acc_fric]
#             min_acc_fric_list = [min_acc_fric_list; min_acc_fric]
#         end

#         h = plot()
#         plot(tol_dists,acc_path, lw = 5, labels="rrt's acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))

#         plot!(tol_dists,max_acc_list, ls =:solid,color=:red, labels="max acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
#         plot!(tol_dists,min_acc_list, ls =:solid,color=:green, labels="min acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))

#         plot!(tol_dists,max_acc_torque_list, lw = 3, ls =:dash, color=:red, labels="max torque + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
#         plot!(tol_dists,min_acc_torque_list, lw = 3, ls =:dash,color=:green, labels="min torque + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
#         plot!(tol_dists,max_acc_fric_list, lw = 3, ls =:dot,color=:red, labels="max fric + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
#         plot!(tol_dists,min_acc_fric_list, lw = 3, ls =:dot,color=:green, legend=:outertopright, labels="min fric + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
#         savefig("Results\\rrt_acc.png")



#         h = plot()
#         plot!(tol_dists, upath[1:end-1], legend = false, color = :blue, lw = 2, xlabel = "travel distance[m]", ylabel = "longitudinal speed [m/s]", title = string("Speed Plot"))
#         savefig("Results\\rrt_speed.png")
#     else
#         println("Not Solved")
#         h
#         savefig("Results\\rrt_tree.png")
#     end

#     return h
# end

function TPPplot(tpp::TPPSearcher)
    rrt = tpp.rrt
    h = plotEnv(rrt)
    h = plotTree(h, rrt)

    if rrt.r.status == :Solved
        println("Solved")
        plot!(h, rrt.r.Path[1,:], rrt.r.Path[2,:],
        color=:blue,
        lw = 2.
        )
        savefig("Results\\rrt_result.png")
        plot!(h, tpp.r.Path[:, 1], tpp.r.Path[:, 2],
        color = :red,
        lw = 2)
        scatter!(h, tpp.rrt.p.sampling_bias_points[:, 1], tpp.rrt.p.sampling_bias_points[:, 2], markersize = 5, alpha = 0.8, legend = false, markershape = :circle)
        savefig("Results\\TPP_result.png")

        xpath = rrt.p.states_collection[1,  rrt.r.resultIdxs]
        ypath = rrt.p.states_collection[2,  rrt.r.resultIdxs]
        zpath = rrt.p.states_collection[3,  rrt.r.resultIdxs]
        upath = rrt.p.states_collection[5,  rrt.r.resultIdxs]

        xpath = reverse(xpath)
        ypath = reverse(ypath)
        zpath = reverse(zpath)
        upath = reverse(upath)
        dis_path = sqrt.( (xpath[2:end]-xpath[1:end-1]).^2+(ypath[2:end]-ypath[1:end-1]).^2 + (zpath[2:end]-zpath[1:end-1]).^2)
        uavg_path = (upath[2:end]+upath[1:end-1])/2
        udif_path = (upath[2:end]-upath[1:end-1])
        dt_path =  dis_path./uavg_path
        acc_path = udif_path./dt_path

        n = size(xpath, 1)
        ################# ACC #######################
        max_acc_list = Vector{Float64}()
        min_acc_list = Vector{Float64}()

        max_acc_torque_list = Vector{Float64}()
        min_acc_torque_list = Vector{Float64}()

        max_acc_fric_list = Vector{Float64}()
        min_acc_fric_list = Vector{Float64}()


        tol_dist = 0
        tol_dists = deepcopy(dis_path)

        for i in 1:n-1
            tol_dists[i] = tol_dist
            tol_dist = tol_dist + dis_path[i]
            h1 = [xpath[i+1]; ypath[i+1]; zpath[i+1]]- [xpath[i]; ypath[i]; zpath[i]]
            h1 = h1/norm(h1)

            R1 = euler2Rot(rrt.p.nodes_collection[rrt.r.resultIdxs[n-i+1]].eulerang)
            R2 = euler2Rot(rrt.p.nodes_collection[rrt.r.resultIdxs[n-i]].eulerang)

            n1 = R1[:,3]
            n2 = R2[:,3]
            n_temp = n1+n2
            n_temp = n_temp/norm(n_temp)

            init_h = R1[:,1]
            T₁ =  R1[:,1]
            T₂ =  R2[:,1]
            dTdS = (T₂ - T₁)/dis_path[i]

            lat_vec = cross(n_temp, h1)
            lat_vec = lat_vec/norm(lat_vec)
            normal_vec = cross(h1, lat_vec)
            normal_vec = normal_vec/norm(normal_vec)

            k_n = round( dot(normal_vec, dTdS), digits = 3) # positive means more normal force
            k_g = round( dot(lat_vec, dTdS), digits = 3) # positive means to left

            centrifugal = -( ( (upath[i] + upath[i+1])/2 )^2*k_g)
            y_friction = -lat_vec[3]*g + centrifugal
            tol_friction = mu_f* (abs(normal_vec[3]*g) +  (( (upath[i] + upath[i+1])/2 )^2)*k_n )


            x_frinction = sqrt(tol_friction^2-y_friction^2)
            x_acc = min(x_frinction, torque_acc)

            max_acc = x_acc  -1* h1[3]*g
            min_acc = -x_acc  -1* h1[3]*g

            max_acc_torque = torque_acc  -1* h1[3]*g
            min_acc_torque = -torque_acc  -1* h1[3]*g

            max_acc_fric = x_frinction  -1* h1[3]*g
            min_acc_fric = -x_frinction  -1* h1[3]*g

            max_acc_list = [max_acc_list; max_acc]
            min_acc_list = [min_acc_list; min_acc]
            max_acc_torque_list = [max_acc_torque_list; max_acc_torque]
            min_acc_torque_list = [min_acc_torque_list; min_acc_torque]
            max_acc_fric_list = [max_acc_fric_list; max_acc_fric]
            min_acc_fric_list = [min_acc_fric_list; min_acc_fric]
        end

        h = plot()
        plot(tol_dists,acc_path, lw = 5, labels="rrt's acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))

        plot!(tol_dists,max_acc_list, ls =:solid,color=:red, labels="max acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        plot!(tol_dists,min_acc_list, ls =:solid,color=:green, labels="min acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))

        plot!(tol_dists,max_acc_torque_list, lw = 3, ls =:dash, color=:red, labels="max torque + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        plot!(tol_dists,min_acc_torque_list, lw = 3, ls =:dash,color=:green, labels="min torque + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        plot!(tol_dists,max_acc_fric_list, lw = 3, ls =:dot,color=:red, labels="max fric + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        plot!(tol_dists,min_acc_fric_list, lw = 3, ls =:dot,color=:green, legend=:outertopright, labels="min fric + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        savefig("Results\\rrt_acc.png")



        h = plot()
        plot!(tol_dists, upath[1:end-1], legend = false, color = :blue, lw = 2, xlabel = "travel distance[m]", ylabel = "longitudinal speed [m/s]", title = string("Speed Plot"))
        savefig("Results\\rrt_speed.png")



        ################# ACC #######################
        max_acc_list = Vector{Float64}()
        min_acc_list = Vector{Float64}()

        max_acc_torque_list = Vector{Float64}()
        min_acc_torque_list = Vector{Float64}()

        max_acc_fric_list = Vector{Float64}()
        min_acc_fric_list = Vector{Float64}()

        xpath = tpp.r.Path[:, 1]
        ypath = tpp.r.Path[:, 2]
        zpath = tpp.r.Path[:, 3]
        upath = tpp.r.Path[:, 5]
        n = size(xpath, 1)
        dis_path = sqrt.( (xpath[2:end]-xpath[1:end-1]).^2+(ypath[2:end]-ypath[1:end-1]).^2 + (zpath[2:end]-zpath[1:end-1]).^2)
        uavg_path = (upath[2:end]+upath[1:end-1])/2
        udif_path = (upath[2:end]-upath[1:end-1])
        dt_path =  dis_path./uavg_path
        acc_path = udif_path./dt_path

        tol_dist = 0
        tol_dists = deepcopy(dis_path)
        for i in 1:n-1
            tol_dists[i] = tol_dist
            tol_dist = tol_dist + dis_path[i]
            h1 = [xpath[i+1]; ypath[i+1]; zpath[i+1]]- [xpath[i]; ypath[i]; zpath[i]]
            h1 = h1/norm(h1)

            R1 = euler2Rot(tpp.ltr.p.nodes_collection[i, 4:6])
            R2 = euler2Rot(tpp.ltr.p.nodes_collection[i+1, 4:6])

            n1 = R1[:,3]
            n2 = R2[:,3]
            n_temp = n1+n2
            n_temp = n_temp/norm(n_temp)

            init_h = R1[:,1]

            T₁ =  R1[:,1]
            T₂ =  R2[:,1]
            dTdS = (T₂ - T₁)/dis_path[i]

            lat_vec = cross(n_temp, h1)
            lat_vec = lat_vec/norm(lat_vec)
            normal_vec = cross(h1, lat_vec)
            normal_vec = normal_vec/norm(normal_vec)

            k_n = round( dot(normal_vec, dTdS), digits = 3) # positive means more normal force
            k_g = round( dot(lat_vec, dTdS), digits = 3) # positive means to left


            centrifugal = -( ( (upath[i] + upath[i+1])/2 )^2*k_g)
            y_friction = -lat_vec[3]*g + centrifugal
            tol_friction = mu_f* (abs(normal_vec[3]*g) +  (( (upath[i] + upath[i+1])/2 )^2)*k_n )


            x_frinction = sqrt(tol_friction^2-y_friction^2)
            x_acc = min(x_frinction, torque_acc)

            max_acc = x_acc  -1* h1[3]*g
            min_acc = -x_acc  -1* h1[3]*g

            max_acc_torque = torque_acc  -1* h1[3]*g
            min_acc_torque = -torque_acc  -1* h1[3]*g

            max_acc_fric = x_frinction  -1* h1[3]*g
            min_acc_fric = -x_frinction  -1* h1[3]*g

            max_acc_list = [max_acc_list; max_acc]
            min_acc_list = [min_acc_list; min_acc]
            max_acc_torque_list = [max_acc_torque_list; max_acc_torque]
            min_acc_torque_list = [min_acc_torque_list; min_acc_torque]
            max_acc_fric_list = [max_acc_fric_list; max_acc_fric]
            min_acc_fric_list = [min_acc_fric_list; min_acc_fric]
        end


        plot!(tol_dists, tpp.r.Path[1:end-1, 5], color = :red, lw = 2)
        savefig("Results\\speed.png")

        h = plot()
        plot(tol_dists,acc_path, lw = 5, labels="rrt's acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))

        plot!(tol_dists,max_acc_list, ls =:solid,color=:red, labels="max acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        plot!(tol_dists,min_acc_list, ls =:solid,color=:green, labels="min acc", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))

        plot!(tol_dists,max_acc_torque_list, lw = 3, ls =:dash, color=:red, labels="max torque + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        plot!(tol_dists,min_acc_torque_list, lw = 3, ls =:dash,color=:green, labels="min torque + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        plot!(tol_dists,max_acc_fric_list, lw = 3, ls =:dot,color=:red, labels="max fric + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        plot!(tol_dists,min_acc_fric_list, lw = 3, ls =:dot,color=:green, legend=:outertopright, labels="min fric + gravity", xlabel = "travel distance[m]", ylabel = "acc [m/s^2]", title = string("Acc Plot"))
        acc_result = [[tol_dists]; [acc_path];[max_acc_list];[min_acc_list]; [max_acc_torque_list]; [min_acc_torque_list]; [max_acc_fric_list]; [min_acc_fric_list]]
        file = matopen("acc_result.mat", "w")
        write(file, "acc_result", acc_result )
        close(file)
        savefig("Results\\acc.png")
    else
        println("Not Solved")
        h
    end

    return h
end
