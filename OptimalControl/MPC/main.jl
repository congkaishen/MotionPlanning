# one way to tackle initial guess: https://jso-docs.github.io/solve-pdenlpmodels-with-jsosolvers/


include("src/SlalomUtils.jl")


using Tables,CSV
using Plots
plot()
# in this example, we assume that prediction and plant are the same model
########################   Settings on Simulation and MPC ######################## 
t_sim = 0.0
Δt_sim = 1e-3
max_sim_time = 20.0
exec_horizon = 0.1

########################   Settings on the Slalom Scenario ######################## 

cur_states = [0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0]
goal_pt = [110.0, 0.0]
block_list = [50.0 1 2.5; 70.0 -1 2.5; 90.0 1 2.5]

########################   Transcribe into mathematical prohramming ######################## 

# define goal position, block position, prediction horizon, number of collocation, and initial states
# the block is treated as circle: [center_x center_y radius; ...]
problem_setting = Dict("goal_pt"=>goal_pt,
                        "block_list"=> block_list, 
                        "Horizon" => 3,
                        "n" => 20,
                        "X0"=> cur_states)
# Here we only provide backward euler propogationm, which means the interpolation is constant next
model = defineSlalomOCP(problem_setting)


########################   Solve the ocp for the first time  ######################## 
########################   restore the optimal states and controls which are used to propogate plant and to warm start ######################## 

optimize!(model)  # Here we solve it just for warm start
optStates = value.(model[:xst])
optCtrls = value.(model[:u])
sr_act, ax_act = getInterpolatedCtrls(model, problem_setting, t_sim)


########################   Start the similation, define some placeholders for later plot  ######################## 
states_his = [0; cur_states]
ctrls_his = [sr_act(t_sim), ax_act(t_sim)]
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


for i in 1:Int32(floor(max_sim_time/Δt_sim))
    global max_sim_time, t_sim, Δt_sim, cur_states, model, goal_pt, sr_act, ax_act, states_his, optStates, optCtrls
    
    ########################  if the position is within the goal point for 3.6m, the simulation will stop  ######################## 
    if distance(cur_states[1:2], goal_pt) <= 6
        break
    end

    ########################  solve optimal control problem for every exec_horizon  ######################## 
    if mod(i, Int32(exec_horizon/Δt_sim)) == 1
        ctrls = [sr_act(t_sim), ax_act(t_sim)]
        updateX0(model, cur_states, ctrls) 
        setWarmStart(model, optStates, optCtrls, cur_states, ctrls)
        optimize!(model) 

        println("Current time: ", round(t_sim; digits = 1), "  Status: ", RetrieveSolveStatus(termination_status(model))," (",round(1000 * solve_time(model); digits = 1)," ms)")
        optStates = value.(model[:xst])
        optCtrls = value.(model[:u])
        sr_act, ax_act = getInterpolatedCtrls(model, problem_setting, t_sim)
    end

    ########################  simulate the plant by sending the interpolated optimal control  ######################## 
    ctrls = [sr_act(t_sim), ax_act(t_sim)]
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