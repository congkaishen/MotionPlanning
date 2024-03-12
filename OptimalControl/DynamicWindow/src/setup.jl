using NearestNeighbors
using Distributions
function defineDWA(
    numStates::Int = 0,
    numControls::Int = 0,
    X0 = fill(NaN,numStates),
    goal = fill(NaN,2), 
    SampleNumber = [10, 10],
    T::Float64 = 3.0,
    XL = fill(NaN,numStates),
    XU = fill(NaN,numStates),
    CL = fill(NaN,numControls),
    CU = fill(NaN,numControls),
    )::DWASearcher
    
    if numControls <= 0
        error("Controls ($numControls) must be > 0")
    end
    if numStates <= 0
        error("States ($numStates) must be > 0")
    end
    if length(X0) != numStates
        error("Length of X0 ($(length(X0))) must match number of states ($numStates)")
    end
    if length(XL) != numStates
        error("Length of XL ($(length(XL))) must match number of states ($numStates)")
    end
    if length(XU) != numStates
        error("Length of XU ($(length(XU))) must match number of states ($numStates)")
    end
    if length(CL) != numControls
        error("Length of CL ($(length(CL))) must match number of controls ($numControls)")
    end
    if length(CU) != numControls
        error("Length of CU ($(length(CU))) must match number of controls ($numControls)")
    end

    DWA = DWASearcher()
    DWA.s.goal = goal
    DWA.s.numStates = numStates
    DWA.s.numControls = numControls
    DWA.s.X0 = X0
    DWA.s.XL = XL
    DWA.s.XU = XU
    DWA.s.CL = CL
    DWA.s.CU = CU
    DWA.s.T = T
    DWA.s.dt = T/DWA.s.N
    DWA.s.SampleNumber = SampleNumber
    defineDWAcontrols!(DWA)
    return DWA
end

function defineDWAobs!(DWA::DWASearcher, obstacle_list)
    DWA.s.obstacle_list = obstacle_list
    return nothing
end

function defineDWAcontrols!(DWA::DWASearcher)
    NumberOfControlSamples = prod(DWA.s.SampleNumber)
    ControlSamples = Matrix{Float64}(undef, NumberOfControlSamples, DWA.s.numControls)
    CtlVector = Vector{Any}(undef, DWA.s.numControls)
    CtlDecode = Vector{Any}(undef, DWA.s.numControls)
    SampleNumberTemp = deepcopy(DWA.s.SampleNumber)
    for ctl in 1:DWA.s.numControls
        CtlVector[ctl] = LinRange(CL[ctl], CU[ctl], DWA.s.SampleNumber[ctl])
        queue =  deleteat!(SampleNumberTemp, 1)
        if length(queue) > 0
            CtlDecode[ctl] = prod(queue)
        else
            CtlDecode[ctl] = 1
        end
    end

    for i = 1:NumberOfControlSamples
        ControlSample = Vector{Float64}(undef, DWA.s.numControls)
        ControlSampleIdx = DecodeControl(DWA, i, CtlDecode)
        for j = 1:DWA.s.numControls
            ControlSample[j] = CtlVector[j][ControlSampleIdx[j]]
        end
        ControlSamples[i, :] = ControlSample
    end
    DWA.s.ControlSamples = ControlSamples
    return nothing
end

function DecodeControl(DWA::DWASearcher, number, CtlDecode)
    ControlSampleIdx = Vector{Int64}(undef, DWA.s.numControls) 
    for i = 1:1:DWA.s.numControls
        ControlSampleIdx[i] = Int64(ceil(number / CtlDecode[i]))
        number = number - (ControlSampleIdx[i] - 1) * CtlDecode[i]
    end

    return ControlSampleIdx
end