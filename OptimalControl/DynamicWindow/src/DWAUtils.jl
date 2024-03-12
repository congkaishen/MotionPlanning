include("types.jl")
include("setup.jl")
include("vehicledynamics.jl")





function ShiftInitialCondition(DWA::DWASearcher, X0)
	DWA.s.X0 = X0
	return nothing
end



function TrajectoryRollout(DWA::DWASearcher, ctrl_list)
	NumCol = DWA.s.N
	states_his = zeros(NumCol+1, size(DWA.s.X0)[1])
	cost_his = zeros(NumCol + 1)
	constraint_his = zeros(NumCol+1)
	dt = DWA.s.dt
	states = DWA.s.X0
	states_his[1,:] = states
	for j = 1:1:NumCol
		collision_info = 1 < j ? ObstacleEvaluation(DWA, states) : (1, 0.0) 
		bounds_info = 1 < j ? BoundEvaluation(DWA, states) : (1, 0.0) 
		dynamics_info = Rungekutta2(DWA, states, ctrl_list,  dt)
		states = dynamics_info[1]
		constraint_his[j] = dynamics_info[2] * collision_info[1] * bounds_info[1]
		cost_his[j] = dynamics_info[3] + bounds_info[2] + collision_info[2]
		states_his[j+1,:] = states
	end

	dynamics_info = Rungekutta2(DWA, states, [0, 0], dt)
	collision_info = ObstacleEvaluation(DWA, states)
	bounds_info = BoundEvaluation(DWA, states)
	constraint_his[NumCol + 1] = dynamics_info[2] * collision_info[1] * bounds_info[1]
	cost_his[NumCol + 1] = dynamics_info[3] + bounds_info[2] + collision_info[2]
	cost_total = sum(cost_his) + TerminalCostEvaluation(DWA, states) / ((states_his[1, 1] - DWA.s.goal[1])^2 + (states_his[1, 2] - DWA.s.goal[2])^2) * 10000.0
	constraint = constraint_his == ones(size(constraint_his))
	return states_his, ctrl_list, constraint, cost_total
end


function Rungekutta1(DWA::DWASearcher, x,c,dt)
	dynamics_info1 = VehicleDynamics(x,c)
	k1 = dynamics_info1[1]
	k2 = VehicleDynamics(x+k1*dt,c)[1]
	constraint_value = dynamics_info1[2]
	p_cost = dynamics_info1[3]

	if constraint_value <= 1
		cons = 1
	else
		cons = 0
		p_cost = (constraint_value - 1) * DWA.s.SlackPenalty * 10		
	end
	return (x + dt* k2, cons, p_cost)
end

function Rungekutta2(DWA::DWASearcher, x,c,dt)
	dynamics_info1 = VehicleDynamics(x,c)
	k1 = dynamics_info1[1]
	k2 = VehicleDynamics(x+k1*dt,c)[1]
	constraint_value = dynamics_info1[2]
	p_cost = dynamics_info1[3]

	if constraint_value <= 1
		cons = 1
	else
		cons = 0
		p_cost = (constraint_value - 1) * DWA.s.SlackPenalty * 10		
	end
	return (x + dt* (k1 + k2) / 2, cons, p_cost)
end


function Rungekutta4(DWA::DWASearcher, x,c,dt)
	dynamics_info1 = VehicleDynamics(x,c)
	k1 = dynamics_info1[1]
	k2 = VehicleDynamics(x+k1*dt/2,c)[1]
	k3 = VehicleDynamics(x+k2*dt/2,c)[1]
	k4 = VehicleDynamics(x+k3*dt,c)[1]


	constraint_value = dynamics_info1[2]
	p_cost = dynamics_info1[]

	if constraint_value <= 1
		cons = 1
	else
		cons = 0
		p_cost = (constraint_value - 1) * DWA.s.SlackPenalty * 10		
	end
	return (x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4), cons, p_cost)
end

function TerminalCostEvaluation(DWA::DWASearcher, states)
	cost_value =  (states[1] - DWA.s.goal[1])^2 + (states[2] - DWA.s.goal[2])^2
	return cost_value
end



function ObstacleEvaluation(DWA::DWASearcher, states)
	constraint = 1
	cost_value = 0
	obstacle_list = DWA.s.obstacle_list
    for obs_idx in 1:size(obstacle_list,1)
        if sum((states[1:2]-obstacle_list[obs_idx][1:2]).^2)<=obstacle_list[obs_idx][3]^2
            constraint = 0
			cost_value = cost_value + 10000.0*712.5
            # cost_value = cost_value + DWA.s.SlackPenalty * abs((sqrt(sum((states[1:2]-obstacle_list[obs_idx][1:2]).^2)) - obstacle_list[obs_idx][3]))
        end
    end

	return constraint, cost_value
end


function BoundEvaluation(DWA::DWASearcher, states)
	constraint = 1
	cost_value = 0
	for i = 1:1:DWA.s.numStates
		if states[i] < DWA.s.XL[i]
			constraint = 0
			# cost_value = cost_value + 100.0*712.5
			cost_value = cost_value + DWA.s.SlackPenalty * abs(states[i] - DWA.s.XL[i])
		end
		if states[i] > DWA.s.XU[i]
			constraint = 0
			# cost_value = cost_value + 100.0*712.5
			cost_value = cost_value + DWA.s.SlackPenalty * abs(states[i] - DWA.s.XU[i])
		end
	end
	return constraint, cost_value
end



function DWAPlan(DWA::DWASearcher)

	DWA.p.TrajectoryCollection = Vector{DWAHolder}(undef, size(DWA.s.ControlSamples, 1))
	t1 = time()
	t2 = time()
	for i = 1:size(DWA.s.ControlSamples, 1)
		RolloutInfo = TrajectoryRollout(DWA, DWA.s.ControlSamples[i, :])
		DWANode = DWAHolder(RolloutInfo[1], RolloutInfo[2], RolloutInfo[3], RolloutInfo[4])
		DWA.p.TrajectoryCollection[i] = DWANode
		t2 = time()
	end
	BestCand = minimum(DWA.p.TrajectoryCollection)

	# DWA.r.Control = DWACtrl
	DWA.r.Traj = BestCand.Trajectory
	DWA.r.Control = BestCand.Control
	DWA.r.Feasibility = BestCand.Feasibility == true ? :Feasible : :InFeasible
	DWA.r.cost = BestCand.cost
	t3 = time()
	DWA.r.time = t3 - t1

	return nothing
end

function circleShape(h,k,r)
    θ = LinRange(0, 2*π, 500)
    h.+r*sin.(θ), k.+r*cos.(θ)
end

function plotRes(DWA, states_his)
	obs_setting = DWA.s.obstacle_list
	h = plot(size = [1000, 600])

    h = plot!(states_his[2,:], states_his[3,:], aspect_ratio=:equal, lc=:green, xlims = (-10, 110), ylims = (-10, 10), title =  "ux = $(round(states[6]; digits = 2)) m/s")
        
    for obs_idx = 1:1:size(obs_setting, 1)
        h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
    end
	h = plot!(h, circleShape(DWA.s.goal[1], DWA.s.goal[2], 7.2), seriestype = [:shape,], ;w = 0.5, c=:green, linecolor = :green, legend = false, fillalpha = 1.0)

    for candi_path = 1:1:size(DWA.p.TrajectoryCollection,1)
		if DWA.p.TrajectoryCollection[candi_path].Feasibility == true
        	h = plot!(h, DWA.p.TrajectoryCollection[candi_path].Trajectory[:,1], DWA.p.TrajectoryCollection[candi_path].Trajectory[:,2],aspect_ratio=:equal, lc=:gray, legend=false, alpha = 0.1)
		end
	end
    h = plot!(h, DWA.r.Traj[:,1], DWA.r.Traj[:,2],aspect_ratio=:equal, lc=:red, legend=false)

    return h

end