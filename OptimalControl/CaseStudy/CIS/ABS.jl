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
########################   Start the similation, define some placeholders for later plot  ######################## 
cur_states = [-30.0, 0.0, 0.0, 0.0, 0.0, 30, 0.0]
states_his = [0; cur_states]
TimeList = []

for i in 1:1Int32(floor(max_sim_time/Δt_sim))
    global t_sim, Δt_sim, max_sim_time, cur_states, ux, states_his
    ########################  simulate the plant by sending the interpolated optimal control  ######################## 

    x = cur_states[1]
    y = cur_states[2]
    v = cur_states[3]
    r = cur_states[4]
    ψ = cur_states[5]
    ux = cur_states[6]
    sa = cur_states[7]
    if x >= 0 && ux > 0
        δux = -8
    else
        δux = 0
    end
    δstates = [ux, 0, 0, 0, 0, δux, 0]
    cur_states = cur_states .+ vec(δstates).*Δt_sim
    states_his = [states_his [t_sim; cur_states]]
    t_sim = t_sim + Δt_sim
end

CSV.write("CollisionImminentManeuver.csv", Tables.table(states_his'), writeheader=false)
