# one way to tackle initial guess: https://jso-docs.github.io/solve-pdenlpmodels-with-jsosolvers/

using JuMP
using MAT
using Interpolations
import Ipopt
import HSL_jll
using LinearAlgebra
using Statistics
include("VehicleModel.jl")




function defineSlamonOCP(problem_setting)
    ########################   Preparing the Map Data ######################## 
    goal_pt = problem_setting["goal_pt"]
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
    "linear_solver" => "ma57",
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
    "max_cpu_time" => 2.0,
    "print_level" => 1,
    )

    # Create JuMP model, using Ipopt as the solver
    model = Model(optimizer_with_attributes(Ipopt.Optimizer, user_options...))
    # states: x y v r ψ ux sa
    XL = [-100, -5, -5, -2*pi, -pi, 1, -pi/9]
    XU = [100, 5, 5, 2*pi, pi, 20, pi/9]
    CL=[-pi/8, -6]
    CU=[ pi/8, 6]

    numStates = size(XL, 1)
    @variables(model, begin
        XL[i] ≤ xst[j in 1:n, i in 1:7] ≤ XU[i]
        CL[i] ≤ u[j in 1:n, i in 1:2] ≤ CU[i] 
    end)

    # initial contidions
    x_s = cur_var[1]
    y_s = cur_var[2] 
    v_s = cur_var[3]
    r_s = cur_var[4]
    ψ_s = cur_var[5]
    ux_s = cur_var[6]
    sa_s = cur_var[7]
    sr_s = cur_var[8]
    ax_s = cur_var[9]

    fix(xst[1, 1], x_s; force = true)
    fix(xst[1, 2], y_s; force = true)
    fix(xst[1, 3], v_s; force = true)
    fix(xst[1, 4], r_s; force = true)
    fix(xst[1, 5], ψ_s; force = true)
    fix(xst[1, 6], ux_s; force = true)
    fix(xst[1, 7], sa_s; force = true)
    Δt = Δt_list


    states_s = [x_s, y_s, v_s, r_s, ψ_s, ux_s, sa_s, sr_s, ax_s]
    states_t = [goal_pt[1], goal_pt[2], v_s, r_s, ψ_s, ux_s, sa_s, sr_s, ax_s]
    interp_linear = Interpolations.LinearInterpolation([1, n], [states_s, states_t])
    initial_guess = mapreduce(transpose, vcat, interp_linear.(1:n))
    set_start_value.(all_variables(model), vec(initial_guess))

    x = xst[:, 1]
    y = xst[:, 2]
    v = xst[:, 3]
    r = xst[:, 4]
    ψ = xst[:, 5]
    ux = xst[:, 6]
    sa = xst[:, 7]
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


    smx = 1.0
    smy = 1.0
    # obs_softconstraint = @expression(model, sum((tanh(-1.4*((x[i] - block_list[j, 1])^2/(block_list[j,3] + smx)^2 +(y[i] - block_list[j, 2])^2/(block_list[j,3] + smy)^2)) + 1)/2 for i=1:n for j=1:size(block_list, 1) ))
    obs_hardconstraint = @constraint(model, [i=1:n, j=1:size(block_list, 1)],1<=((x[i]-block_list[j,1])^2)/((block_list[j,3]+smx)^2)+((y[i]-block_list[j,2])^2)/((block_list[j,3]+smy)^2))


    k_cost = @expression( model, sum((r[j]/ux[j])^2  for j=1:1:n))
    v_cost = @expression( model, sum((v[j])^2  for j=1:1:n))
    sr_cost = @expression( model, sum((sr[j])^2  for j=1:1:n))
    y_cost = @expression( model, sum( sqrt((y[j])^2+0.1)  for j=1:1:n))
    ax_cost = @expression( model, sum((ax[j])^2  for j=1:1:n))
    sa_cost = @expression( model, sum((sa[j])^2  for j=1:1:n))


    obj_goal = @expression(model, ((x[end] - goal_pt[1])^2 + (y[end] - goal_pt[2])^2)/( (x[end] - x[1])^2 + (y[end] - y[1])^2  + 0.1 )  ) # 1 is added in the dominator to avoid singurality

    @objective(model, Min, 150*obj_goal + 5*y_cost + 0.1*ax_cost + 1.5*sr_cost + 0.1*k_cost + 0.05*v_cost + 1*sa_cost)
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
    return InterpolateSr, InterpolateAx
end

function setWarmStart(model, warm_states, warm_ctrls, X0, C0)
    # warm_states = value.(model[:xst])
    # warm_ctrls = value.(model[:u])
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


using Plots
plot()
# in this example, we assume that prediction and plant are the same model
t_sim = 0.0
Δt_sim = 1e-3
max_sim_time = 10.0
exec_horizon = 0.2


cur_states = [0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0]
goal_pt = [100.0, 0.0]
block_list = [40 -1 2.5; 70 1 2.5]
# define goal position, block position, prediction horizon, number of collocation, and initial states
# the block is treated as circle: [center_x center_y radius; ...]
problem_setting = Dict("goal_pt"=>goal_pt,
                        "block_list"=> block_list, 
                        "Horizon" => 3,
                        "n" => 30,
                        "X0"=> cur_states)

# Here we only provide backward euler propogationm, which means the interpolation is constant next
model = defineSlamonOCP(problem_setting)
optimize!(model)  # Here we solve it just for warm start

updateX0(model, [1.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0], [0.0, 0.0])
states_symbol = model[:xst]
ctrls_symbol = model[:u]


optimize!(model)  
optStates = value.(model[:xst])
optCtrls = value.(model[:u])
plot(optStates[:,1], optStates[:,2])

sr_act, ax_act = getInterpolatedCtrls(model, problem_setting, t_sim)


states_his = cur_states


for i in 1:Int32(floor(max_sim_time/Δt_sim))
    global max_sim_time, t_sim, Δt_sim, cur_states, model, goal_pt, sr_act, ax_act, states_his, optStates, optCtrls, states_symbol, ctrls_symbol
    if distance(cur_states[1:2], goal_pt) <= 3.6
        break
    end
    if mod(i, Int32(exec_horizon/Δt_sim)) == 1
        ctrls = [sr_act(t_sim), ax_act(t_sim)]
        updateX0(model, cur_states, ctrls) 
        setWarmStart(model, optStates, optCtrls, cur_states, ctrls)
        optimize!(model) 
        optStates = value.(model[:xst])
        optCtrls = value.(model[:u])
        sr_act, ax_act = getInterpolatedCtrls(model, problem_setting, t_sim)
        h = plot(states_his[1,:], states_his[2,:], color =:black, aspect_ratio =:equal)
        h = plot!(h, optStates[:,1],optStates[:,2], color =:red)
        display(h)
        sleep(0.01)
    end
    ctrls = [sr_act(t_sim), ax_act(t_sim)]
    δstates = dynamics(cur_states, ctrls)
    cur_states = cur_states .+ vec(δstates).*Δt_sim
    states_his = [states_his cur_states]
    t_sim = t_sim + Δt_sim
end




