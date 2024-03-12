using Plots
using DelimitedFiles
using ProgressBars
using Distributed
using LinearAlgebra
using Shuffle
using Parameters
using NearestNeighbors
using Statistics
using Distances
using MAT
using ProgressMeter
using Distributions
using StaticArrays

include("types.jl")
include("setup.jl")
include("ltr_utils.jl")
include("rrt_utils.jl")
include("astar_utils.jl")
include("plotting_utils.jl")



function planTPP(tpp::TPPSearcher)
    planAstar!(tpp.astar)
    total_length = sum( sqrt.( (tpp.astar.r.actualpath[1:end-1, 1] - tpp.astar.r.actualpath[2:end, 1]).^2 + (tpp.astar.r.actualpath[1:end-1, 2] - tpp.astar.r.actualpath[2:end, 2]).^2) )
    gap =  Int64(  maximum([floor(size(tpp.astar.r.actualpath, 1)/(floor(total_length/tpp.rrt.s.sample_max_dist))),  1])  )
    sampling_bias_points = tpp.astar.r.actualpath[1:gap:end, :]

    tpp.rrt.p.sampling_bias_points = sampling_bias_points
    planRRT!(tpp.rrt)
    if tpp.rrt.r.status != :Solved
        tpp.r.status = :NotSolved
        tpp.r.tSolve = tpp.rrt.r.tSolve
        println("RRT Failed!")
    else
        RRT2LTR(tpp)
        if !CheckInitialTraj(tpp.ltr)
            println("Initial Path Given is not Valid")

            return nothing
        end
        executeLTRArray(tpp.ltr)
        tpp.r.status = tpp.ltr.r.status
        tpp.r.Path = tpp.ltr.r.Path
        tpp.r.tSolve = tpp.ltr.r.tSolve + tpp.rrt.r.tSolve

        print("Cost RRT: ")
        CalculateRRTCost(tpp.rrt)
        print(tpp.rrt.r.cost)
        print("  ")
        print("LTR Cost: ")
        CalculateLTRCost(tpp.ltr)
        println(tpp.ltr.r.cost)

    end
    return nothing
end



function preparePath(tpp::TPPSearcher)
    rrt = tpp.rrt
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

    ################# ACC #######################
    max_acc_list = Vector{Float64}()
    min_acc_list = Vector{Float64}()

    max_acc_torque_list = Vector{Float64}()
    min_acc_torque_list = Vector{Float64}()

    max_acc_fric_list = Vector{Float64}()
    min_acc_fric_list = Vector{Float64}()

    max_limit_list = Vector{Float64}()

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

        if abs(max_acc - acc_path[i]) <= 0.1
            max_limit_list = [max_limit_list; 1.0]
        else
            max_limit_list = [max_limit_list; 0.0]
        end
    end
    tpp.r.TrajFollow = [tpp.r.Path [max_limit_list;0.0]]
end
