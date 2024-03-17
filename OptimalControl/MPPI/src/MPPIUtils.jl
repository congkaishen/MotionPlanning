include("types.jl")
include("setup.jl")
include("vehicledynamics.jl")

function SampleMPPIControl(MPPI::MPPISearcher)
	CtrlNoise = transpose(rand(MPPI.s.MultivariantNormal, MPPI.s.N))
	CtrlBeforeCheck = CtrlNoise + MPPI.s.NominalControl
	Ctrl = PushInBounds(MPPI, CtrlBeforeCheck)
	# Ctrl = CtrlBeforeCheck
	return Ctrl
end

function PushInBounds(MPPI::MPPISearcher, CtrlBeforeCheck)
	for i = 1:1:MPPI.s.N
		for j = 1:MPPI.s.numControls
			CtrlBeforeCheck[i, j] = min(max(CtrlBeforeCheck[i, j], MPPI.s.CL[j]), MPPI.s.CU[j])
		end
	end
	return CtrlBeforeCheck
end



function ShiftInitialCondition(MPPI::MPPISearcher, X0)
	MPPI.s.X0 = X0
	return nothing
end



function TrajectoryRollout(MPPI::MPPISearcher, ctrl_list)
	NumCol = MPPI.s.N
	states_his = zeros(NumCol+1, size(MPPI.s.X0)[1])
	cost_his = zeros(NumCol + 1)
	constraint_his = zeros(NumCol+1)
	dt = MPPI.s.dt
	states = MPPI.s.X0
	states_his[1,:] = states
	for j = 1:1:NumCol
		collision_info = 1 < j ? ObstacleEvaluation(MPPI, states) : (1, 0.0) 
		bounds_info = 1 < j ? BoundEvaluation(MPPI, states) : (1, 0.0) 
		dynamics_info = Rungekutta2(MPPI, states, ctrl_list[j,:],  dt)
		states = dynamics_info[1]
		constraint_his[j] = dynamics_info[2] * collision_info[1] * bounds_info[1]
		cost_his[j] = dynamics_info[3] + bounds_info[2] + collision_info[2] + MPPI.s.lambda * MPPI.s.NominalControl[j, :]' * MPPI.s.Σ * (ctrl_list[j,:] - MPPI.s.NominalControl[j, :])
		states_his[j+1,:] = states
	end

	dynamics_info = Rungekutta2(MPPI, states, [0, 0], dt)
	collision_info = ObstacleEvaluation(MPPI, states)
	bounds_info = BoundEvaluation(MPPI, states)
	constraint_his[NumCol + 1] = dynamics_info[2] * collision_info[1] * bounds_info[1]
	cost_his[NumCol + 1] = dynamics_info[3] + bounds_info[2] + collision_info[2]
	cost_total = sum(cost_his) + TerminalCostEvaluation(MPPI, states) / ((states_his[1, 1] - MPPI.s.goal[1])^2 + (states_his[1, 2] - MPPI.s.goal[2])^2) * 10000.0
	constraint = constraint_his == ones(size(constraint_his))
	return states_his, ctrl_list, constraint, cost_total
end


function Rungekutta1(MPPI::MPPISearcher, x,c,dt)
	dynamics_info1 = VehicleDynamics(x,c)
	k1 = dynamics_info1[1]
	k2 = VehicleDynamics(x+k1*dt,c)[1]
	constraint_value = dynamics_info1[2]
	p_cost = dynamics_info1[3]

	if constraint_value <= 1
		cons = 1
	else
		cons = 0
		p_cost = (constraint_value - 1) * MPPI.s.SlackPenalty * 10		
	end
	return (x + dt* k2, cons, p_cost)
end

function Rungekutta2(MPPI::MPPISearcher, x,c,dt)
	dynamics_info1 = VehicleDynamics(x,c)
	k1 = dynamics_info1[1]
	k2 = VehicleDynamics(x+k1*dt,c)[1]
	constraint_value = dynamics_info1[2]
	p_cost = dynamics_info1[3]

	if constraint_value <= 1
		cons = 1
	else
		cons = 0
		p_cost = (constraint_value - 1) * MPPI.s.SlackPenalty * 10		
	end
	return (x + dt* (k1 + k2) / 2, cons, p_cost)
end


function Rungekutta4(MPPI::MPPISearcher, x,c,dt)
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
		p_cost = (constraint_value - 1) * MPPI.s.SlackPenalty * 10		
	end
	return (x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4), cons, p_cost)
end

function TerminalCostEvaluation(MPPI::MPPISearcher, states)
	cost_value =  (states[1] - MPPI.s.goal[1])^2 + (states[2] - MPPI.s.goal[2])^2
	return cost_value
end



function ObstacleEvaluation(MPPI::MPPISearcher, states)
	constraint = 1
	cost_value = 0
	obstacle_list = MPPI.s.obstacle_list
    for obs_idx in 1:size(obstacle_list,1)
        if sum((states[1:2]-obstacle_list[obs_idx][1:2]).^2)<=obstacle_list[obs_idx][3]^2
            constraint = 0
			cost_value = cost_value + 10000.0*712.5
        end
    end

	return constraint, cost_value
end


function BoundEvaluation(MPPI::MPPISearcher, states)
	constraint = 1
	cost_value = 0
	for i = 1:1:MPPI.s.numStates
		if states[i] < MPPI.s.XL[i]
			constraint = 0
			# cost_value = cost_value + 100.0*712.5
			cost_value = cost_value + MPPI.s.SlackPenalty * abs(states[i] - MPPI.s.XL[i])
		end
		if states[i] > MPPI.s.XU[i]
			constraint = 0
			# cost_value = cost_value + 100.0*712.5
			cost_value = cost_value + MPPI.s.SlackPenalty * abs(states[i] - MPPI.s.XU[i])
		end
	end
	return constraint, cost_value
end


function CalculateMPPIWeights(MPPI::MPPISearcher)
	cost_idxs = argmin(MPPI.p.TrajectoryCollection)
	ρ = MPPI.p.TrajectoryCollection[cost_idxs].cost
	η = 0
	λ = MPPI.s.lambda
	for i = 1:size(MPPI.p.TrajectoryCollection, 1)
		η = η + exp(- 1/(λ) * (MPPI.p.TrajectoryCollection[i].cost - ρ))
	end
	w = zeros(size(MPPI.p.TrajectoryCollection, 1))
	for i = 1:size(MPPI.p.TrajectoryCollection, 1)
		w[i] = 1/η * exp(- 1/(λ) * (MPPI.p.TrajectoryCollection[i].cost - ρ))
	end
	return w
end

function MPPIPlan(MPPI::MPPISearcher)
	FeasibilityCount = 0
	RolloutCount = 1
	MPPI.p.TrajectoryCollection = Vector{MPPIHolder}(undef, MPPI.s.SamplingNumber)
	t1 = time()
	t2 = time()
	while FeasibilityCount <= MPPI.s.FeasibilityCount && RolloutCount <= MPPI.s.SamplingNumber && t2 - t1 <= MPPI.s.tmax
		RolloutInfo = TrajectoryRollout(MPPI, SampleMPPIControl(MPPI))
		if RolloutInfo[3] == true
			FeasibilityCount = FeasibilityCount + 1
		end
		MPPINode = MPPIHolder(RolloutInfo[1], RolloutInfo[2], RolloutInfo[3], RolloutInfo[4])
		MPPI.p.TrajectoryCollection[RolloutCount] = MPPINode
		RolloutCount = RolloutCount + 1
		t2 = time()
	end
	MPPI.p.TrajectoryCollection = MPPI.p.TrajectoryCollection[1:RolloutCount-1]
	w = CalculateMPPIWeights(MPPI)
	MPPICtrl = zeros(MPPI.s.N, MPPI.s.numControls)
	for i = 1:1:size(MPPI.p.TrajectoryCollection, 1)
		MPPICtrl = MPPICtrl + w[i] * MPPI.p.TrajectoryCollection[i].Control
	end
	# MPPI.r.Control = MPPICtrl
	RolloutInfo = TrajectoryRollout(MPPI, MPPICtrl)
	MPPI.r.Traj = RolloutInfo[1]
	MPPI.r.Control = RolloutInfo[2]
	MPPI.r.Feasibility = RolloutInfo[3] == true ? :Feasible : :InFeasible
	MPPI.r.cost = RolloutInfo[4]
	MPPI.r.RolloutCount = RolloutCount
	MPPI.r.FeasibleTrajCount = FeasibilityCount
	t3 = time()
	MPPI.r.time = t3 - t1

	return nothing
end

function circleShape(h,k,r)
    θ = LinRange(0, 2*π, 500)
    h.+r*sin.(θ), k.+r*cos.(θ)
end

function plotRes(mppi, states_his)
	h = plot(size = [800, 600])
	# h = plot()
    h = plot!(h, circleShape(0,0, 1), seriestype = [:shape,], ;w = 0.5, aspect_ratio=:equal, c=:red, linecolor = :red, legend = false, fillalpha = 1.0)
	obs_setting = mppi.s.obstacle_list
    for obs_idx = 1:1:size(obs_setting, 1)
        h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3] - 1), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
		h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 0.2)
    end

	h = plot!(h, circleShape(mppi.s.goal[1], mppi.s.goal[2], 7.2), seriestype = [:shape,], ;w = 0.5, c=:green, linecolor = :green,framestyle = :box, legend = false,xlims = (-10, 110), ylims = (-10, 10), fillalpha = 1.0, xlabel = "X (m)", ylabel = "Y (m)", xtickfontsize=14,ytickfontsize=14,xguidefontsize=14,yguidefontsize=14)
	for candi_path = 1:1:size(mppi.p.TrajectoryCollection,1)
        h = plot!(h, mppi.p.TrajectoryCollection[candi_path].Trajectory[:,1], mppi.p.TrajectoryCollection[candi_path].Trajectory[:,2],aspect_ratio=:equal, lc=:gray, legend=false, alpha = 0.1)
    end
	h = plot!(states_his[2,:], states_his[3,:], aspect_ratio=:equal, lc=:green, lw = 2, xlims = (-10, 110), ylims = (-10, 10))
    h = plot!(h, mppi.r.Traj[:,1], mppi.r.Traj[:,2],aspect_ratio=:equal, lw = 2, lc=:red, legend=false)
	h = plot!(h, vehicleShape(states_his[2, end],states_his[3, end],states_his[6, end]),
	seriestype = [:shape],
	lw = 0.5,
	c = :gold2,
	linecolor = :black,
	fillalpha = 1
	)
    return h

end

function plotEnv(mppi)
	h = plot(size = [800, 600])
	# h = plot()
    h = plot!(h, circleShape(0,0, 1), seriestype = [:shape,], ;w = 0.5, aspect_ratio=:equal, c=:red, linecolor = :red, legend = false, fillalpha = 1.0)
	obs_setting = mppi.s.obstacle_list
    for obs_idx = 1:1:size(obs_setting, 1)
        h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3] - 1), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
		h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 0.2)
    end
	h = plot!(h, circleShape(mppi.s.goal[1], mppi.s.goal[2], 7.2), seriestype = [:shape,], ;w = 0.5, c=:green, linecolor = :green,framestyle = :box, legend = false,xlims = (-10, 110), ylims = (-10, 10), fillalpha = 1.0, xlabel = "X (m)", ylabel = "Y (m)", xtickfontsize=14,ytickfontsize=14,xguidefontsize=14,yguidefontsize=14)
	return h
end

function vehicleShape(x,y,yaw)
	c = cos(yaw)
	s = sin(yaw)
	R = [c -s; s c]
	# nodes = [0 0 -3.61 -3.61 0 0; 0 0.78 0.78 -0.78 -0.78 0]
	nodes = [0 0 -2.712 -2.712 0 0; 0 0.78 0.78 -0.78 -0.78 0]
	rotatedNodes = R * nodes
	x .+ rotatedNodes[1,:], y .+ rotatedNodes[2,:]
end