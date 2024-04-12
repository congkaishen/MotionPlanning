# one way to tackle initial guess: https://jso-docs.github.io/solve-pdenlpmodels-with-jsosolvers/


include("src/CISUtils2.jl")


using Tables,CSV
using Plots
plot()
# in this example, we assume that prediction and plant are the same model
########################   Settings on Simulation and MPC ######################## 
t_sim = 0.0
Δt_sim = 1e-3
max_sim_time = 10.0
exec_horizon = 0.05

########################   Settings on the Slalom Scenario ######################## 

cur_states = [5.0, -10.0, π/2, 2.0, 0.0]
block_list = [48 0 10 1.8]
########################   Transcribe into mathematical prohramming ######################## 
XL = [-100, -10, -pi/2, -5.0, -0.7]
XU = [100, 10, pi/2, 5.0, 0.7]
CL = [-1.0, -4.0]
CU = [1.0, 4.0]
# define goal position, block position, prediction horizon, number of collocation, and initial states
# the block is treated as circle: [center_x center_y radius; ...]
problem_setting = Dict( "block_list"=> block_list, 
                        "Horizon" => 3,
                        "n" => 50,
                        "X0"=> cur_states,
                        "XL"=>XL,
                        "XU"=>XU,
                        "CL"=>CL,
                        "CU"=>CU)
Δt = problem_setting["Horizon"] / (problem_setting["n"] - 1) 

# Here we only provide backward euler propogationm, which means the interpolation is constant next
model = defineCISOCP(problem_setting)


########################   Solve the ocp for the first time  ######################## 
########################   restore the optimal states and controls which are used to propogate plant and to warm start ######################## 

optimize!(model)  # Here we solve it just for warm start
optimize!(model)  # Here we solve it just for warm start
optStates = value.(model[:xst])
optCtrls = value.(model[:u])
sr_act = getInterpolatedCtrls(model, problem_setting, t_sim)[1]
ax_act = getInterpolatedCtrls(model, problem_setting, t_sim)[2]

########################   Start the simulation, define some placeholders for later plot  ######################## 
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

TimeList = []
for i in 1:Int32(floor(max_sim_time/Δt_sim))
    global max_sim_time, t_sim, Δt_sim, cur_states, model, goal_pt, sr_act, ax_act, states_his, ctrls_his, optStates, optCtrls, Δt, TimeList
    
    ########################  solve optimal control problem for every exec_horizon  ######################## 
    if mod(i, Int32(exec_horizon/Δt_sim)) == 1
        ctrls = [sr_act(t_sim), ax_act(t_sim)]
        updateX0(model, cur_states, ctrls, problem_setting) 
        setWarmStart(model, optStates, optCtrls, cur_states, ctrls)
        optimize!(model) 
        TimeList = [TimeList; solve_time(model)]
        println("Current time: ", round(t_sim; digits = 1), "  Status: ", RetrieveSolveStatus(termination_status(model))," (",round(1000 * solve_time(model); digits = 1)," ms)   ")
        optStates = value.(model[:xst])
        optCtrls = value.(model[:u])
        sr_act = getInterpolatedCtrls(model, problem_setting, t_sim)[1]
        ax_act = getInterpolatedCtrls(model, problem_setting, t_sim)[2]
    end

    ########################  simulate the plant by sending the interpolated optimal control  ######################## 
    ctrls = [sr_act(t_sim), ax_act(t_sim)]
    δstates = dynamics(cur_states, ctrls)
    cur_states = cur_states .+ vec(δstates).*Δt_sim
    states_his = [states_his [t_sim; cur_states]]
    ctrls_his = [ctrls_his ctrls]
    t_sim = t_sim + Δt_sim

    if mod(i, Int32(0.1/2/Δt_sim)) == 1
        h = plotRes(problem_setting, states_his, optStates)
        display(h)
        if make_gif
            Plots.frame(anim)
        end
        sleep(0.001)
    end
end


if make_gif gif(anim, "./Parking/gifholder/MPC.gif", fps = 20) end

CSV.write("MPCTrajectory.csv", Tables.table(states_his'), writeheader=false)

ParkingStates = [states_his[1:6, 2:end]' ctrls_his[1:2, 2:end]']
CSV.write("ParkingManeuver.csv", Tables.table(ParkingStates), writeheader=false)
