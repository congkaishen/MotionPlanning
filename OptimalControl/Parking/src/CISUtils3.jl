
using JuMP
using MAT
using Interpolations
import Ipopt
# import HSL_jll
using LinearAlgebra
using Statistics
using Plots.PlotMeasures
include("VehicleModel2.jl")

function defineCISOCP(problem_setting)
    ########################   Preparing the Map Data ######################## 
    block_list = problem_setting["block_list"]
    vehLength = problem_setting["vehLength"]
    vehWidth = problem_setting["vehWidth"]
    vehDiagonal = problem_setting["vehDiagonal"]
    vehDiagonalAngle = problem_setting["vehDiagonalAngle"]
    vehSpace = problem_setting["vehSpace"]
    ########################   Define Model Parameters ######################## 
    Horizon = problem_setting["Horizon"]
    n = problem_setting["n"]
    ######################## Initial States Settings ######################## 
    # states: x y ψ ux sa
    cur_states = problem_setting["X0"]
    # controls: sr ax
    cur_ctrls = [0.0, 0.0]
    cur_var = vcat(cur_states, cur_ctrls)
    ######################## Prepare cost_to_go Parameters and Block for KS based on current states ######################## 
    Δt_list = Horizon/(n-1)*ones(n)
    ######################## IMPORTANT!!!! Setting for IPOPT ######################## 
    user_options = (
    "mu_strategy" => "adaptive",
    # "linear_solver" => "ma57",
    "max_iter" => 3000,
    "tol" => 5e-2,
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
    XL = problem_setting["XL"]
    XU = problem_setting["XU"]
    CL = problem_setting["CL"]
    CU = problem_setting["CU"]

    numStates = size(XL, 1)
    numControls = size(CL, 1)
    @variables(model, begin
        XL[i] ≤ xst[j in 1:n, i in 1:numStates] ≤ XU[i]
        CL[i] ≤ u[j in 1:n, i in 1:numControls] ≤ CU[i]
        0.2 <= tf <= 7.0 
    end)

    # initial contidions
    x_s = cur_var[1]
    y_s = cur_var[2] 
    ψ_s = cur_var[3]
    ux_s = cur_var[4]
    sa_s = cur_var[5]
    sr_s = cur_var[6]
    ax_s = cur_var[7]


    fix(xst[1, 1], x_s; force = true)
    fix(xst[1, 2], y_s; force = true)
    fix(xst[1, 3], ψ_s; force = true)
    fix(xst[1, 4], ux_s; force = true)
    fix(xst[1, 5], sa_s; force = true)
    Δt = Δt_list

    states_s = [x_s, y_s, ψ_s, ux_s, sa_s, sr_s, ax_s]
    states_t = [x_s, y_s, ψ_s, ux_s, sa_s, sr_s, ax_s]
    interp_linear = Interpolations.LinearInterpolation([1, n], [states_s, states_t])
    initial_guess = mapreduce(transpose, vcat, interp_linear.(1:n))
    set_start_value.(all_variables(model), push!(vec(initial_guess), 5.0))


    x = xst[:, 1]
    y = xst[:, 2]
    ψ = xst[:, 3]
    ux = xst[:, 4]
    sa = xst[:, 5]
    sr = u[:, 1]
    ax = u[:, 2]

    δxst = Matrix{Any}(undef, n, numStates)
    for i = 1:1:n
        δxst[i, :] = @expression(model, dynamics(xst[i, :], u[i, :]))
    end

    for j = 2:n
        for i = 1:numStates
            @constraint(model, xst[j, i] == xst[j - 1, i] +Δt[j - 1] * δxst[j, i])
        end
    end

    safetyMargin=0.2;

    # constraints for middle of rear axle
    @constraint(model, [j = 1:n], ((x[j]-(1.5*vehLength+vehSpace))^10)/(vehLength/2+safetyMargin)^10+((y[j]-0)^10)/(vehWidth/2.0+safetyMargin)^10 >= 1)
    @constraint(model, [j = 1:n], ((x[j]+(0.5*vehLength+vehSpace))^10)/(vehLength/2+safetyMargin)^10+((y[j]+0)^10)/(vehWidth/2.0+safetyMargin)^10 >= 1)
    # constraints for middle of front axle
    @constraint(model, [j = 1:n], (((x[j]+vehLength*cos(ψ[j]))-(1.5*vehLength+vehSpace))^10)/(vehLength/2.0+safetyMargin)^10+(((y[j]+vehLength*sin(ψ[j]))-0)^10)/(vehWidth/2.0+safetyMargin)^10 >= 1)
    @constraint(model, [j = 1:n], (((x[j]+vehLength*cos(ψ[j]))+(0.5*vehLength+vehSpace))^10)/(vehLength/2.0+safetyMargin)^10+(((y[j]+vehLength*sin(ψ[j]))+0)^10)/(vehWidth/2.0+safetyMargin)^10 >= 1)
    # constraints for right front corner
    @constraint(model, [j = 1:n], (((x[j]+vehLength*cos(ψ[j])+vehWidth/2.0*sin(ψ[j]))-(1.5*vehLength+vehSpace))^10)/(vehLength/2.0+safetyMargin)^10+(((y[j]-vehWidth/2.0*cos(ψ[j])+vehLength*sin(ψ[j]))-0)^10)/(vehWidth/2.0+safetyMargin)^10 >= 1)
    # constraints for right edge middle
    @constraint(model, [j = 1:n], (((x[j]+vehLength/2.0*cos(ψ[j])+vehWidth/2.0*sin(ψ[j]))-(1.5*vehLength+vehSpace))^10)/(vehLength/2.0+safetyMargin)^10+(((y[j]-vehWidth/2.0*cos(ψ[j])+vehLength/2.0*sin(ψ[j]))-0)^10)/(vehWidth/2.0+safetyMargin)^10 >= 1)

    sr_cost = @expression( model, sum((sr[j])^2  for j=1:1:n))
    ux_cost = @expression( model, sum((ux[j])^2  for j=1:1:n))
    ax_cost = @expression( model, sum((ax[j])^2  for j=1:1:n))
    sa_cost = @expression( model, sum((sa[j])^2  for j=1:1:n))

    # x_cost = @expression( model, sum( x[j]^2  for j=1:1:n))
    # y_cost = @expression( model, sum( y[j]^2  for j=1:1:n))
    # ψ_cost = @expression( model, sum( ψ[j]^2  for j=1:1:n))
    # @objective(model, Min, 10*y_cost + 10*x_cost + 10*ψ_cost + 0.01*sr_cost + 0.01*ax_cost  + 0.01*sa_cost )
    
    # x_cost = @expression( model, sum( log(x[j]^2+1)  for j=1:1:n))
    # y_cost = @expression( model, sum( log(y[j]^2+1)  for j=1:1:n))
    # ψ_cost = @expression( model, sum( log(ψ[j]^2+1)  for j=1:1:n))
    # @objective(model, Min, 1*y_cost + 1*x_cost + 1*ψ_cost + 0.01*sr_cost + 0.01*ax_cost  + 0.01*sa_cost )

    # x_cost = @expression( model, sum( x[j]^2  for j=1:1:n))
    # y_cost = @expression( model, sum( y[j]^2  for j=1:1:n))
    # ψ_cost = @expression( model, sum( ψ[j]^2  for j=1:1:n))
    # @objective(model, Min, 100*y_cost + 1*x_cost + 100*ψ_cost + 0.01*sr_cost + 0.01*ax_cost  + 0.01*sa_cost )

    # x_cost = @expression( model, sum( log(x[j]^2+1)  for j=1:1:n))
    # y_cost = @expression( model, sum( y[j]^2  for j=1:1:n))
    # ψ_cost = @expression( model, sum( ψ[j]^2  for j=1:1:n))
    # @objective(model, Min, 100*y_cost + 1*x_cost + 100*ψ_cost + 0.01*sr_cost + 0.01*ax_cost  + 0.01*sa_cost )

    # x_cost = @expression( model, sum( x[j]^2  for j=1:1:n))
    # y_cost = @expression( model, sum( y[j]^2  for j=1:1:n))
    # ψ_cost = @expression( model, sum( ψ[j]^2  for j=1:1:n))
    # @objective(model, Min, 100*y_cost + 1*x_cost/(y_cost+0.1) + 100*ψ_cost + 0.01*sr_cost + 0.01*ax_cost + 0.01*sa_cost )

    # x_cost = @expression( model, sum( x[j]^2  for j=1:1:n))
    # y_cost = @expression( model, sum( abs(y[j])^1.5  for j=1:1:n))
    # ψ_cost = @expression( model, sum( ψ[j]^2  for j=1:1:n))
    # @objective(model, Min, 10*y_cost + 1*x_cost + 100*ψ_cost + 0.01*sr_cost + 0.01*ax_cost + 0.01*sa_cost )

    x_cost = @expression( model, sum( x[j]^2  for j=1:1:n))
    y_cost = @expression( model, sum( y[j]^2  for j=1:1:n))
    ψ_cost = @expression( model, sum( ψ[j]^2  for j=1:1:n))
    terminalcost = @expression(model, x[end]^2 + 10 * y[end]^2 + ψ[end]^2 + ux[end]^2 )
    @objective(model, Min, 10*y_cost + 0.5*x_cost + 100 * terminalcost + 50*ψ_cost + 0.15 *ux_cost + 0.01*sr_cost + 0.01*ax_cost + 0.01*sa_cost + 10 * tf )

    set_silent(model)  # Hide solver's verbose output
    return model
end

function getInterpolatedCtrls(model, problem_setting, current_sim_time)
    optCtrls = value.(model[:u])
    Horizon = problem_setting["Horizon"]
    n = problem_setting["n"]
    time_list = collect(range(current_sim_time, current_sim_time + Horizon, length=n))
    InterpolateSr = interpolate((time_list ,), optCtrls[:,1], Gridded(Constant{Next}()))
    InterpolateAx = interpolate((time_list ,), optCtrls[:,2], Gridded(Constant{Next}()))
    return [InterpolateSr, InterpolateAx]
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

function updateX0(model, cur_state, cur_ctrl, problem_setting)
    cur_state_bounded = fill(NaN, length(cur_state))
    for i = 1:1:length(cur_state)
        cur_state_bounded[i] = min(max(cur_state[i], problem_setting["XL"][i]), problem_setting["XU"][i])
    end
    fix.(model[:u][1,:], cur_ctrl; force = true)
    fix.(model[:xst][1,:], cur_state_bounded; force = true)
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

    vehLength = problem_setting["vehLength"]
    vehWidth = problem_setting["vehWidth"]
    vehDiagonal = problem_setting["vehDiagonal"]
    vehDiagonalAngle = problem_setting["vehDiagonalAngle"]
    vehSpace = problem_setting["vehSpace"]

	h = plot(size = [2*600, 2*300])
	# h = plot()
    # h = plot!(h, circleShape(0,0, 1), seriestype = [:shape,], ;w = 0.5, aspect_ratio=:equal, c=:red, linecolor = :red, legend = false, fillalpha = 1.0)
    # for obs_idx = 1:1:size(obs_setting, 1)
    #     h = plot!(h, circleShape(obs_setting[obs_idx, 1], obs_setting[obs_idx, 2], obs_setting[obs_idx, 3] - 2.5, obs_setting[obs_idx, 4]-0.8), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
	# 	h = plot!(h, circleShape(obs_setting[obs_idx, 1], obs_setting[obs_idx, 2], obs_setting[obs_idx, 3], obs_setting[obs_idx, 4]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 0.2)
    # end

    h = plot!(h, ObsShape(vehLength+vehSpace, 0, 0),
    seriestype = [:shape],
    lw = 0.5,
    c = :black,
    linecolor = :black,
    fillalpha = 1
    )

    h = plot!(h, ObsShape(-vehLength-vehSpace, 0, 0),
    seriestype = [:shape],
    lw = 0.5,
    c = :black,
    linecolor = :black,
    fillalpha = 1
    )

	h = plot!(h, vehicleShape(states_his[2, end],states_his[3, end],states_his[4, end]),
	seriestype = [:shape],
	lw = 0.5,
	c = :gold2,
	linecolor = :black,
	fillalpha = 1
	)
    h = plot!(h, optStates[:,1], optStates[:,2],lw = 2, aspect_ratio=:equal, lc=:red, legend=false)
    h = plot!(states_his[2,:], states_his[3,:], lw = 2, aspect_ratio=:equal, lc=:green, xlims = (-10, 12), ylims = (-5, 7.5), fillalpha = 1.0, xlabel = "X (m)", ylabel = "Y (m)", xtickfontsize=16, ytickfontsize=16, xguidefontsize=18, yguidefontsize=18, bottom_margin = 10mm, left_margin = 10mm)
    return h

end
function vehicleShape(x,y,yaw)
	c = cos(yaw)
	s = sin(yaw)
	R = [c -s; s c]
	# nodes = [0 0 -3.61 -3.61 0 0; 0 0.78 0.78 -0.78 -0.78 0]
	# nodes = [2.5 2.5 -2.5 -2.5 2.5 2.5; 0 0.9 0.9 -0.9 -0.9 0]
    nodes = [0.0 0.0 3.4 3.4 0.0 0.0; 0 0.9 0.9 -0.9 -0.9 0]
	rotatedNodes = R * nodes
	x .+ rotatedNodes[1,:], y .+ rotatedNodes[2,:]
end

function ObsShape(x,y,yaw)
	c = cos(yaw)
	s = sin(yaw)
	R = [c -s; s c]
	# nodes = [0 0 -3.61 -3.61 0 0; 0 0.78 0.78 -0.78 -0.78 0]
	# nodes = [l l -l -l l l; 0 w w -w -w 0]
    nodes = [0.0 0.0 3.4 3.4 0.0 0.0; 0 0.9 0.9 -0.9 -0.9 0]
	rotatedNodes = R * nodes
	x .+ rotatedNodes[1,:], y .+ rotatedNodes[2,:]
end