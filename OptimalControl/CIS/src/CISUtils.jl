
using JuMP
using MAT
using Interpolations
import Ipopt
# import HSL_jll
using LinearAlgebra
using Statistics
include("VehicleModel.jl")

function defineSlamonOCP(problem_setting)
    ########################   Preparing the Map Data ######################## 
    block_list = problem_setting["block_list"]
    ########################   Define Model Parameters ######################## 
    Horizon = problem_setting["Horizon"]
    n = problem_setting["n"]
    ######################## Initial States Settings ######################## 
    # states: x y v r ψ ux sa
    cur_states = problem_setting["X0"]
    # states: sr ax
    cur_ctrls = [0.0, 0.0]
    cur_var = vcat(cur_states, cur_ctrls)
    ######################## Prepare cost_to_go Parameters and Block for KS based on current states ######################## 
    Δt_list = Horizon/(n-1)*ones(n)
    ######################## IMPORTANT!!!! Setting for IPOPT ######################## 
    user_options = (
    "mu_strategy" => "adaptive",
    # "linear_solver" => "ma57",
    "max_iter" => 500,
    "tol" => 4e-2,
    "dual_inf_tol" => 2.,
    "constr_viol_tol" => 5e-1,
    "compl_inf_tol" => 5e-1,
    "acceptable_tol" => 1.5e-1,
    "acceptable_constr_viol_tol" => 0.02,
    "acceptable_dual_inf_tol" => 1e10,
    "acceptable_compl_inf_tol" => 0.02,
    "warm_start_init_point" => "yes",
    "fixed_variable_treatment" => "relax_bounds",
    "max_cpu_time" => 0.2,
    "print_level" => 1,
    )

    # Create JuMP model, using Ipopt as the solver
    model = Model(optimizer_with_attributes(Ipopt.Optimizer, user_options...))
    # states: x y v r ψ ux sa
    XL = [-10, -0.9, -5, -pi/2,  -pi/2, -pi/9]
    XU = [NaN, 4.5, 5, pi/2,  pi/2, pi/9]
    CL = [-1.22]
    CU = [1.22]

    numStates = size(XL, 1)
    @variables(model, begin
        XL[i] ≤ xst[j in 1:n, i in 1:6] ≤ XU[i]
        CL[i] ≤ u[j in 1:n, i in 1:1] ≤ CU[i] 
    end)

    # initial contidions
    x_s = cur_var[1]
    y_s = cur_var[2] 
    v_s = cur_var[3]
    r_s = cur_var[4]
    ψ_s = cur_var[5]
    sa_s = cur_var[6]
    sr_s = cur_var[7]

    fix(xst[1, 1], x_s; force = true)
    fix(xst[1, 2], y_s; force = true)
    fix(xst[1, 3], v_s; force = true)
    fix(xst[1, 4], r_s; force = true)
    fix(xst[1, 5], ψ_s; force = true)
    fix(xst[1, 6], sa_s; force = true)
    Δt = Δt_list


    states_s = [x_s, y_s, v_s, r_s, ψ_s, sa_s, sr_s]
    states_t = [x_s, y_s, v_s, r_s, ψ_s, sa_s, sr_s]
    interp_linear = Interpolations.LinearInterpolation([1, n], [states_s, states_t])
    initial_guess = mapreduce(transpose, vcat, interp_linear.(1:n))
    set_start_value.(all_variables(model), vec(initial_guess))

    x = xst[:, 1]
    y = xst[:, 2]
    v = xst[:, 3]
    r = xst[:, 4]
    ψ = xst[:, 5]
    sa = xst[:, 6]
    sr = u[:, 1]

    δxst = Matrix{Any}(undef, n, numStates)
    for i = 1:1:n
        δxst[i, :] = @expression(model, dynamics(xst[i, :], u[i, :]))
    end

    for j = 2:n
        for i = 1:numStates
            @constraint(model, xst[j, i] == xst[j - 1, i] +Δt[j - 1] * δxst[j, i])
        end
    end
    ux = 30

    smx = 0.5
    smy = 0.7 + 0.9
    # obs_softconstraint = @expression(model, sum((tanh(-1.4*((x[i] - block_list[j, 1])^2/(block_list[j,3] + smx)^2 +(y[i] - block_list[j, 2])^2/(block_list[j,3] + smy)^2)) + 1)/2 for i=1:n for j=1:size(block_list, 1) ))
    alphaf = @expression(model, [j=1:n], atan((v[j] + 1.56 * r[j]) / (ux+ 0.01)) - sa[j])
    alphar = @expression(model, [j=1:n], atan((v[j] - 1.64 * r[j]) / (ux+ 0.01)))

    obs_hardconstraint = @constraint(model, [i=1:n, j=1:size(block_list, 1)],1<=((x[i]-block_list[j,1])^2)/((block_list[j,3]+smx)^2)+((y[i]-block_list[j,2])^2)/((block_list[j,4]+smy)^2))
    v_cost = @expression( model, sum((v[j])^2  for j=1:1:n))
    sr_cost = @expression( model, sum((sr[j])^2  for j=1:1:n))
    # y_cost = @expression( model, sum( (y[j] - 3.6)^2  for j=1:1:n))
    y_cost = @expression( model, sum( (y[j])^2  for j=1:1:n))
    alpha_cost = @expression(model, sum( (alphaf[j])^2 + (alphar[j])^2  for j=1:1:n))

    # y_cost = @expression( model, sum( (y[j]- 10*sin(x[j]/15))^2  for j=1:1:n))

    sa_cost = @expression( model, sum((sa[j])^2  for j=1:1:n))

    @objective(model, Min, 0.01*y_cost + 2*sr_cost + 0.05*v_cost + 0.1*sa_cost + 0.5 * alpha_cost )
    set_silent(model)  # Hide solver's verbose output
    return model
end

function getInterpolatedCtrls(model, problem_setting, current_sim_time)
    optCtrls = value.(model[:u])
    Horizon = problem_setting["Horizon"]
    n = problem_setting["n"]
    time_list = collect(range(current_sim_time, current_sim_time + Horizon, length=n))
    InterpolateSr = interpolate((time_list ,), optCtrls[:,1], Gridded(Constant{Next}()))
    return InterpolateSr
end

function setWarmStart(model, warm_states, warm_ctrls, X0, C0)
    warm_states[1,:] = X0
    warm_ctrls[1,:] = C0
    set_start_value.(model[:xst], warm_states)
    set_start_value.(model[:u], warm_ctrls)
end

function distance(pt1, pt2)
    return norm(pt1 - pt2)
end

function updateX0(model, cur_state, cur_ctrl)
    fix.(model[:xst][1,:], cur_state; force = true)
    fix.(model[:u][1,:], cur_ctrl; force = true)
end

function RetrieveSolveStatus(status::MOI.TerminationStatusCode)
    SolvingStatus = [:Optimal, :UserLimit, :InFeasible]
    OptimalList = [MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ALMOST_OPTIMAL]
    LimitList = [MOI.ITERATION_LIMIT, MOI.TIME_LIMIT, MOI.NODE_LIMIT, MOI.SOLUTION_LIMIT, MOI.MEMORY_LIMIT, MOI.OBJECTIVE_LIMIT, MOI.NORM_LIMIT, MOI.OTHER_LIMIT ]
    
    if status ∈ OptimalList
        return SolvingStatus[1]
    elseif status ∈ LimitList
        return SolvingStatus[2]
    else
        return SolvingStatus[3]
    end
end

function circleShape(h,k,rl, rw)
    θ = LinRange(0, 2*π, 500)
    h.+rl*sin.(θ), k.+rw*cos.(θ)
end


function plotRes(problem_setting, states_his, optStates)


    obs_setting = problem_setting["block_list"]

	h = plot(size = [800, 600])
	# h = plot()
    # h = plot!(h, circleShape(0,0, 1), seriestype = [:shape,], ;w = 0.5, aspect_ratio=:equal, c=:red, linecolor = :red, legend = false, fillalpha = 1.0)
    # for obs_idx = 1:1:size(obs_setting, 1)
    #     h = plot!(h, circleShape(obs_setting[obs_idx, 1], obs_setting[obs_idx, 2], obs_setting[obs_idx, 3] - 2.5, obs_setting[obs_idx, 4]-0.8), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
	# 	h = plot!(h, circleShape(obs_setting[obs_idx, 1], obs_setting[obs_idx, 2], obs_setting[obs_idx, 3], obs_setting[obs_idx, 4]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 0.2)
    # end


    h = plot!(h, optStates[:,1], optStates[:,2],lw = 2, aspect_ratio=:equal, lc=:red, legend=false)
    h = plot!(states_his[2,:], states_his[3,:], lw = 2, aspect_ratio=:equal, lc=:green, xlims = (-10, 130), ylims = (-10, 10))
    h = plot!(h, ObsShape( obs_setting[1],obs_setting[2],0, obs_setting[3], obs_setting[4]),
	seriestype = [:shape],
	lw = 0.5,
	c = :black,
	linecolor = :black,
	fillalpha = 1
	)
	h = plot!(h, vehicleShape(states_his[2, end],states_his[3, end],states_his[6, end]),
	seriestype = [:shape],
	lw = 0.5,
	c = :gold2,
	linecolor = :black,
	fillalpha = 1
	)
    return h

end
function vehicleShape(x,y,yaw)
	c = cos(yaw)
	s = sin(yaw)
	R = [c -s; s c]
	# nodes = [0 0 -3.61 -3.61 0 0; 0 0.78 0.78 -0.78 -0.78 0]
	nodes = [2.5 2.5 -2.5 -2.5 2.5 2.5; 0 0.9 0.9 -0.9 -0.9 0]
	rotatedNodes = R * nodes
	x .+ rotatedNodes[1,:], y .+ rotatedNodes[2,:]
end

function ObsShape(x,y,yaw, l, w)
	c = cos(yaw)
	s = sin(yaw)
	R = [c -s; s c]
	# nodes = [0 0 -3.61 -3.61 0 0; 0 0.78 0.78 -0.78 -0.78 0]
	nodes = [l l -l -l l l; 0 w w -w -w 0]
	rotatedNodes = R * nodes
	x .+ rotatedNodes[1,:], y .+ rotatedNodes[2,:]
end