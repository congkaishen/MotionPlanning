# one way to tackle initial guess: https://jso-docs.github.io/solve-pdenlpmodels-with-jsosolvers/

using JuMP
using MAT
using Interpolations
import Ipopt
import HSL_jll
using LinearAlgebra
using Statistics
include("VehicleModel.jl")
########################   Preparing the Map Data ######################## 

goal_pt = [100.0, 0.0]
# block_list = [30 -1 2]
block_list = [30 1 2.5; 50 -1 2.5]
########################   Define Model Parameters ######################## 
Horizon = 4
n = 25
integration_rule = "bkwEuler"
######################## Initial States Settings ######################## 
# states: x y v r ψ ux sa
cur_states = [0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0]
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
# "tol" => 4e-2,
# "dual_inf_tol" => 2.,
# "constr_viol_tol" => 5e-1,
# "compl_inf_tol" => 5e-1,
# "acceptable_tol" => 1.5e-1,
# "acceptable_constr_viol_tol" => 0.02,
# "acceptable_dual_inf_tol" => 1e10,
# "acceptable_compl_inf_tol" => 0.02,
# "warm_start_init_point" => "yes",
# "fixed_variable_treatment" => "relax_bounds",
"max_cpu_time" => 2.0,
"print_level" => 5,
)

# Create JuMP model, using Ipopt as the solver
model = Model(optimizer_with_attributes(Ipopt.Optimizer, user_options...))
# states: x y v r ψ ux sa

XL = [-100, -15, -5, -2*pi, -pi, 1, -pi/9]
XU = [100, 15, 5, 2*pi, pi, 10, pi/9]
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
states_t = [x_s, y_s, v_s, r_s, ψ_s, ux_s, sa_s, sr_s, ax_s]
interp_linear = Interpolations.LinearInterpolation([1, n], [states_s, states_t])


# initial_guess = mapreduce(transpose, vcat, interp_linear.(1:n))
# set_start_value.(all_variables(model), vec(initial_guess))

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
# obs_softconstraint = @expression(model, sum((tanh(-1.4*((x[i] - block_list[j, 1])^2/(block_list[j,3] + smx)^2 +(y[i] - block_list[j, 2])^2/(block_list[j,3] + smy)^2)) + 1)/2 for i=1:numStates for j=1:size(block_list, 1) ))
obs_hardconstraint = @constraint(model, [i=1:n, j=1:size(block_list, 1)],1<=((x[i]-block_list[j,1])^2)/((block_list[j,3]+smx)^2)+((y[i]-block_list[j,2])^2)/((block_list[j,3]+smy)^2))


# speed_cost = @expression( model, sum( (70 - ux[j])/70 for j=n-4:1:n))
k_cost = @expression( model, sum((r[j]/ux[j])^2  for j=1:1:n))
v_cost = @expression( model, sum((v[j])^2  for j=1:1:n))
sr_cost = @expression( model, sum((sr[j])^2  for j=1:1:n))
y_cost = @expression( model, sum((y[j])^2  for j=1:1:n))
ax_cost = @expression( model, sum((ax[j])^2  for j=1:1:n))
sa_cost = @expression( model, sum((sa[j])^2  for j=1:1:n))
obj_goal = @expression(model, ((x[end] - goal_pt[1])^2 + (y[end] - goal_pt[2])^2)/( (x[end] - x[1])^2 + (y[end] - y[1])^2  + 1 )  ) # 1 is added in the dominator to avoid singurality
@objective(model, Min,  + 100*obj_goal + 10*y_cost + 10*sr_cost + 10*sa_cost + 0.02*ax_cost + 5*k_cost + 5*v_cost)

set_silent(model)  # Hide solver's verbose output
optimize!(model)  # Solve for the control and state
# @assert termination_status(model) == LOCALLY_SOLVED

using Plots
plot()

states = value.(model[:xst])

plot(states[:,1], states[:,2], aspect_ratio =:equal)





