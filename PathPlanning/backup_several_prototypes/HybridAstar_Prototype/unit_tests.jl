include("HybridAstar.jl")

####### Test 1: encoder and decoder #########

idx = zeros(363609, 1)
states = zeros(363609, 3)
states_decoded = zeros(363609, 3)

count = 1
for i in -50:0.5:50
    for j in -50:0.5:50
        for k in -pi:pi/4:pi
            global st_bounds, st_reso, count
            idx[count] = encode_states([i,j,k], st_bounds, st_reso)
            states[count,:] = [i,j,k]
            count += 1
        end
    end
end

for i in 1:363609
    global st_bounds, st_reso, count
    states_decoded[i,:] = decode_states(idx[i], st_bounds, st_reso)
end

print(sum(states - states_decoded))
####### Test 1 end #########
