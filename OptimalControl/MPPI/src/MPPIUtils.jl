
function DecodePosition(Position, MPPI::MPPISearcher)
	if size(Position, 2) <= 1
		Position = Position'
	end 
    x = Position[:,1]
    y = Position[:,2]
    index_x = min.(max.(Int64.(round.((x  .- MPPI.s.XL[1]) ./ MPPI.s.GridSize[1])) .+ 1,  1), MPPI.s.GridNum[1] + 1)
    index_y = min.(max.(Int64.(round.((y  .- MPPI.s.XL[2]) ./ MPPI.s.GridSize[2])) .+ 1,  1), MPPI.s.GridNum[2] + 1)

    Index = [(index_x) (index_y)]
    return Index
end

function EncodePosition(Idxes, MPPI::MPPISearcher)
	index_x = Idxes[1]
	index_y = Idxes[2]

	x = ((index_x-1) * MPPI.s.GridSize[1]) + MPPI.s.XL[1]
	y = ((index_y-1) * MPPI.s.GridSize[2]) + MPPI.s.XL[2]

    return [x,y]
end

function TransMat(Cord, States)
	x = States[1]
	y = States[2]
	θ = States[5]
    R = [cos(θ) -sin(θ); sin(θ) cos(θ)]
    return (R*Cord' .+ [x;y])'
end

function UpdateTouchVisibleandObsMap(MPPI::MPPISearcher, States)
	TouchMapCurrent = sqrt.((MPPI.s.XGrid .- States[1]).^2 .+ (MPPI.s.YGrid .- States[2]).^2) .<= MPPI.s.lidar_range
	MPPI.s.TGrid = MPPI.s.TGrid .| TouchMapCurrent

	TransVisCord = TransMat(MPPI.s.VisCord, States)
	VisIdxes = DecodePosition(TransVisCord, MPPI)
	for i in 1:size(VisIdxes, 1)
		MPPI.s.VGrid[VisIdxes[i,1], VisIdxes[i,2]] = true
	end
	for obs_idx = 1:size(MPPI.s.obstacle_list, 1)
		ObsMapCurrent = (MPPI.s.XGrid .- MPPI.s.obstacle_list[obs_idx][1]).^2 + (MPPI.s.YGrid .- MPPI.s.obstacle_list[obs_idx][2]).^2 .>= MPPI.s.obstacle_list[obs_idx][3]^2
		MPPI.s.OGrid = MPPI.s.OGrid .& ObsMapCurrent
	end
	return nothing
end


function SampleControlUniformInputs(MPPI::MPPISearcher)
	N = MPPI.s.N
	ctrl = rand(N, MPPI.s.numControls)
	ctrl = ctrl .* (MPPI.s.CU - MPPI.s.CL)' .+ MPPI.s.CL'
	return ctrl
end


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
		Location = states[1:2]
		Index = DecodePosition(Location, MPPI)
		collision_info = 1 < j ? ObstacleEvaluation(MPPI, Index) : (1, 0.0) 
		bounds_info = 1 < j ? BoundEvaluation(MPPI, states) : (1, 0.0) 
		dynamics_info = Rungekutta2(MPPI, states, ctrl_list[j,:],  dt)
		states = dynamics_info[1]
		constraint_his[j] = dynamics_info[2] * collision_info[1] * bounds_info[1]
		VisCost = 0
		VisAreaCostValue = 0
		if j == 5
			#VisAreaCostValue = VisAreaCost(mppi, states) 
		end
		if j > 4
			VisCost = VisabilityEvaluation(MPPI, Index) / j^2
		end
		cost_his[j] = dynamics_info[3] + bounds_info[2] + collision_info[2] + VisCost + VisAreaCostValue
		states_his[j+1,:] = states
	end

	Location = states[1:2]
	Index = DecodePosition(Location, MPPI)
	dynamics_info = Rungekutta2(MPPI, states, [0, 0], dt)
	collision_info = ObstacleEvaluation(MPPI, Index)
	bounds_info = BoundEvaluation(MPPI, states)
	# VisCost = VisabilityEvaluation(MPPI, Index)
	constraint_his[NumCol + 1] = dynamics_info[2] * collision_info[1] * bounds_info[1]
	cost_his[NumCol + 1] = dynamics_info[3] + bounds_info[2] + collision_info[2]
	cost_total = sum(cost_his) + TerminalCostEvaluation(MPPI, states)
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

function Rungekutta3(MPPI::MPPISearcher, x,c,dt)
	dynamics_info1 = VehicleDynamics(x,c)
	k1 = dynamics_info1[1]
	dynamics_info2 = VehicleDynamics(x+k1*dt/2,c)
	k2 = dynamics_info2[1]
	dynamics_info3 = VehicleDynamics(x+k2*dt,c)
	k3 = dynamics_info3[1]



	constraint_value = dynamics_info1[2]
	p_cost = dynamics_info1[3]

	if constraint_value <= 1
		cons = 1
	else
		cons = 0
		p_cost = (constraint_value - 1) * MPPI.s.SlackPenalty * 10		
	end
	return (x + dt / 6 * (k1 + 4 * k2 + k3), cons, p_cost)
end


function Rungekutta4(MPPI::MPPISearcher, x,c,dt)
	dynamics_info1 = VehicleDynamics(x,c)
	k1 = dynamics_info1[1]
	dynamics_info2 = VehicleDynamics(x+k1*dt/2,c)
	k2 = dynamics_info2[1]
	dynamics_info3 = VehicleDynamics(x+k2*dt/2,c)
	k3 = dynamics_info3[1]
	dynamics_info4 = VehicleDynamics(x+k3*dt,c)
	k4 = dynamics_info4[1]


	constraint_value = dynamics_info1[2]
	p_cost = dynamics_info1[3]

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

function DecodeForFasterSpeed(MPPI::MPPISearcher, Idx)
	NewIndex = (Idx[:, 2] .- 1) * MPPI.s.GridNum[1] .+ (Idx[:, 1])
	return NewIndex
end

function VisAreaCost(MPPI::MPPISearcher, States) # 212 mu s
	TransVisCord = TransMat(MPPI.s.VisCord, States) # 53 mus
	VisIdxes = DecodePosition(TransVisCord, MPPI) # 77 mus
	CurrentVisMap = copy(MPPI.s.TempGrid)
	VisIdxes = CartesianIndex.(tuple.(eachcol(VisIdxes)...))
	# NewVisIdx = DecodeForFasterSpeed(MPPI, VisIdxes) # 58 mu s
	CurrentVisMap[VisIdxes] .= false # 7 mu s
	OverallVisMap = (.!CurrentVisMap .& .!MPPI.s.VGrid) .& MPPI.s.TGrid # 1.7 mu s
	return sum(OverallVisMap)

end


function ObstacleEvaluation(MPPI::MPPISearcher, Index)
	constraint = 1
	cost_value = 0
	if MPPI.s.OGrid[Index[1], Index[2]] == 0
		cost_value = cost_value + 10000 * 712.5
		constraint = 0
	end
	return constraint, cost_value
end


function VisabilityEvaluation(MPPI::MPPISearcher, Index)
	cost_value = 0
	if MPPI.s.VGrid[Index[1], Index[2]] == 0
		cost_value = cost_value + 100 * 712.5
	end
	return cost_value
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
	if MPPI.s.MultiThreadBoolean == true
		Threads.@threads for i = 1:1:MPPI.s.SamplingNumber
			RolloutInfo = TrajectoryRollout(MPPI, SampleMPPIControl(MPPI))
			MPPINode = MPPIHolder(RolloutInfo[1], RolloutInfo[2], RolloutInfo[3], RolloutInfo[4])
			MPPI.p.TrajectoryCollection[i] = MPPINode
			# t2 = time()
			# if t2 - t1 >= MPPI.s.tmax
			# 	break
			# end
		end
		MPPI.p.TrajectoryCollection = MPPI.p.TrajectoryCollection[1:MPPI.s.SamplingNumber-1]
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
	else

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
	end
	t3 = time()
	MPPI.r.time = t3 - t1
	return nothing
end

function circleShape(h,k,r)
    θ = LinRange(0, 2*π, 500)
    h.+r*sin.(θ), k.+r*cos.(θ)
end

function plotRes_withColorMap(mppi, states_his)
    obs_setting = mppi.s.obstacle_list
	x_coords = mppi.s.XL[1] : mppi.s.GridSize[1] : mppi.s.XU[1]
	y_coords = mppi.s.XL[2] : mppi.s.GridSize[2] : mppi.s.XU[2]
	h = plot(size = [1000, 600])
	h = heatmap!(x_coords, y_coords, mppi.s.VGrid', c=:greys, legend = false, colorbar=false)
    h = plot!(h, states_his[1,:], states_his[2,:],aspect_ratio=:equal, lc=:green, ylimits=(-60,60), xlimits=(-10,110),  title =  "ux = $(round(states[6]; digits = 2)) m/s")
    

    min_cost = mppi.p.TrajectoryCollection[1].cost
    max_cost = mppi.p.TrajectoryCollection[1].cost
    for candi_path = 1:1:size(mppi.p.TrajectoryCollection,1)
        if mppi.p.TrajectoryCollection[candi_path].cost < min_cost
            min_cost = mppi.p.TrajectoryCollection[candi_path].cost
        end

        if mppi.p.TrajectoryCollection[candi_path].cost > max_cost
            max_cost = mppi.p.TrajectoryCollection[candi_path].cost
        end
    end

    for obs_idx = 1:1:size(obs_setting, 1)
        h = plot!(h, circleShape(obs_setting[obs_idx][1], obs_setting[obs_idx][2], obs_setting[obs_idx][3]), seriestype = [:shape,], ;w = 0.5, c=:black, linecolor = :black, legend = false, fillalpha = 1.0)
    end
	h = plot!(h, circleShape(mppi.s.goal[1], mppi.s.goal[2], 2), seriestype = [:shape,], ;w = 0.5, c=:green, linecolor = :green, legend = false, fillalpha = 1.0)
    for candi_path = 1:1:size(mppi.p.TrajectoryCollection,1)
        num_traj = size(mppi.p.TrajectoryCollection[candi_path].Trajectory[:,1], 1)
        normed_cost = (mppi.p.TrajectoryCollection[candi_path].cost - min_cost)/(max_cost-min_cost)
        h = plot!(h, mppi.p.TrajectoryCollection[candi_path].Trajectory[:,1], mppi.p.TrajectoryCollection[candi_path].Trajectory[:,2],aspect_ratio=:equal, linez = normed_cost*ones(num_traj), legend=false, alpha = 0.1)
    end
    h = plot!(h, mppi.r.Traj[:,1], mppi.r.Traj[:,2],aspect_ratio=:equal, lc=:red, legend=false)
    return h

end