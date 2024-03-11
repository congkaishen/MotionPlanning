# MPPI = mppi;
# ctrl_list = mppi.r.Control
# NumCol = MPPI.s.N
# states_his = zeros(NumCol+1, size(MPPI.s.X0)[1])
# cost_his = zeros(NumCol + 1)
# constraint_his = zeros(NumCol+1)
# dt = MPPI.s.dt
# states = MPPI.s.X0
# states_his[1,:] = states
# for j = 1:1:NumCol
# 	global states
# 	Location = states[1:2]
# 	Index = DecodePosition(Location, MPPI)
# 	collision_info = 1 < j ? ObstacleEvaluation(MPPI, states) : (1, 0.0) 
# 	bounds_info = 1 < j ? BoundEvaluation(MPPI, states) : (1, 0.0) 
# 	dynamics_info = Rungekutta2(MPPI, states, ctrl_list[j,:],  dt)
# 	states = dynamics_info[1]
# 	constraint_his[j] = dynamics_info[2] * collision_info[1] * bounds_info[1]
# 	cost_his[j] = dynamics_info[3] + bounds_info[2] + collision_info[2]
# 	states_his[j+1,:] = states
# end

# Location = states[1:2]
# Index = DecodePosition(Location, MPPI)
# dynamics_info = Rungekutta2(MPPI, states, [0, 0], dt)
# collision_info = ObstacleEvaluation(MPPI, states)
# bounds_info = BoundEvaluation(MPPI, states)
# constraint_his[NumCol + 1] = dynamics_info[2] * collision_info[1] * bounds_info[1]
# cost_his[NumCol + 1] = dynamics_info[3] + bounds_info[2]
# cost_total = sum(cost_his) + TerminalCostEvaluation(MPPI, states)/((states_his[1, 1] - MPPI.s.goal[1])^2 + (states_his[1, 2] - MPPI.s.goal[2])^2) * 150.0
# constraint = constraint_his == ones(size(constraint_his))
# return states_his, ctrl_list, constraint, cost_total



using Plots

function getTRMatric(original_vec, x, y, θ)
    R = [cos(θ) -sin(θ); sin(θ) cos(θ)]
    
    return R*original_vec .+ [x;y]
end


original_vec = rand(2, 10000)
h = scatter(original_vec[1,:], original_vec[2,:], color =:black)

x = 1
y = 2
θ = pi/4
# TRMatrix = [cos(θ) -sin(θ) 0; sin(θ) cos(θ) 0; 0 0 1]
# new_vec = TRMatrix*[original_vec; ones(size(original_vec, 2))']


@time begin
    for i in 1:1000
        global new_vec = getTRMatric(original_vec, x, y, θ)
    end
end

scatter!(h, new_vec[1,:], new_vec[2,:], color =:black, aspect_ratio =:equal)