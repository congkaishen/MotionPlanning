# one way to tackle initial guess: https://jso-docs.github.io/solve-pdenlpmodels-with-jsosolvers/


include("src/CISUtils.jl")


using Tables,CSV
using Plots
plot()
# in this example, we assume that prediction and plant are the same model
########################   Settings on Simulation and MPC ######################## 
t_sim = 0.0
Δt_sim = 1e-3
max_sim_time = 6
exec_horizon = 0.05

########################   Settings on the Slalom Scenario ######################## 

cur_states = [-30.0, 0.0, 0.0, 0.0, 0.0, 0.0]
block_list = [48 0 10 1.8]
########################   Transcribe into mathematical prohramming ######################## 
XL = [-40, -0.9, -7, -pi/5,  -pi/2, -pi/12]
XU = [1000, 4.5, 7, pi/5,  pi/2, pi/12]
# define goal position, block position, prediction horizon, number of collocation, and initial states
# the block is treated as circle: [center_x center_y radius; ...]
problem_setting = Dict( "block_list"=> block_list, 
                        "Horizon" => 2.5,
                        "n" => 50,
                        "X0"=> cur_states,
                        "XL"=>XL,
                        "XU"=>XU)
Δt = problem_setting["Horizon"] / (problem_setting["n"] - 1) 

# Here we only provide backward euler propogationm, which means the interpolation is constant next
model = defineCISOCP(problem_setting)


########################   Solve the ocp for the first time  ######################## 
########################   restore the optimal states and controls which are used to propogate plant and to warm start ######################## 

optimize!(model)  # Here we solve it just for warm start
optimize!(model)  # Here we solve it just for warm start
optStates = value.(model[:xst])
optCtrls = value.(model[:u])
sr_act = getInterpolatedCtrls(model, problem_setting, t_sim)


########################   Start the similation, define some placeholders for later plot  ######################## 
states_his = [0; cur_states]
ctrls_his = sr_act(t_sim)
make_gif = true
if make_gif
    if isdir("./gifholder")
        println("Already Exists")
        foreach(rm, filter(endswith(".gif"), readdir("./gifholder",join=true)))
    else
        mkdir("./gifholder")
    end
end
if make_gif anim = Plots.Animation() end

TimeList = []
for i in 1:1Int32(floor(max_sim_time/Δt_sim))
    global max_sim_time, t_sim, Δt_sim, cur_states, model, goal_pt, sr_act, ax_act, states_his, optStates, optCtrls, Δt, TimeList
    
    ########################  solve optimal control problem for every exec_horizon  ######################## 
    if mod(i, Int32(exec_horizon/Δt_sim)) == 1
        ctrls = [sr_act(t_sim)]
        updateX0(model, cur_states, ctrls, problem_setting) 
        YL = determinePoints(cur_states[1], 30, problem_setting["block_list"],Δt, problem_setting["n"])
        setWarmStart(model, optStates, optCtrls, cur_states, ctrls)
        set_parameter_value.(model[:YL], YL)
        optimize!(model) 
        TimeList = [TimeList; solve_time(model)]
        println("Current time: ", round(t_sim; digits = 1), "  Status: ", RetrieveSolveStatus(termination_status(model))," (",round(1000 * solve_time(model); digits = 1)," ms)   ")
        optStates = value.(model[:xst])
        optCtrls = value.(model[:u])
        sr_act = getInterpolatedCtrls(model, problem_setting, t_sim)
    end

    ########################  simulate the plant by sending the interpolated optimal control  ######################## 
    ctrls = [sr_act(t_sim)]
    δstates = dynamics(cur_states, ctrls)
    cur_states = cur_states .+ vec(δstates).*Δt_sim
    states_his = [states_his [t_sim; cur_states]]
    t_sim = t_sim + Δt_sim

    if mod(i, Int32(0.1/Δt_sim)) == 1
        h = plotRes(problem_setting, states_his, optStates)
        display(h)
        if make_gif
            Plots.frame(anim)
        end
        sleep(0.001)
    end
end


if make_gif gif(anim, "./gifholder/MPC.gif", fps = 10) end

CSV.write("MPCTrajectory.csv", Tables.table(states_his'), writeheader=false)

CollisionStates = [states_his[1:6, 2:end]; 30 * ones(size(states_his[5, 2:end]))'; states_his[7, 2:end]']'
CSV.write("CollisionImminentManeuver.csv", Tables.table(CollisionStates), writeheader=false)
