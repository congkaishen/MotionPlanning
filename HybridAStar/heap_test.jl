using DataStructures




# alternative solution:
# findall(vertices[:,1].== request_st_code). this is abandoned since it is slower.
function gethandler(st_index_list, st_index)
    # input : 1. the encoded states index(first col of vertices)
    #         2. requested encoded state idx
    # return : handler(Int32) ➡ 0 means doesn't exist, otherwise, return the handler(row idx)
    for i in 1:size(st_index_list,1)
        if Int32(st_index_list[i]) == 0
            return 0
        end
        if Int32(st_index_list[i]) == Int32(st_index)
            return i
        end
    end
    println("Abnormal situation in gethandler: The encoded state idx goes beyond the limit of memory storage")
end


minR = 1
gear_set =  [1, -1]
steer_set = [-1/minR, 0, 1/minR]
T = 1

num_steer = size(steer_set, 1)
num_gear = size(gear_set, 1)
num_neighbors = num_steer*num_gear
num_states = 3

maxNum = 10000

# [states_coder, x, y, ψ] here only store states, the cost are stored in a heap for fast min search.
# everytime create a new states, vertices will update one --> the handler of heap map is equal to row idx 
# states_coder == 0 means vertices doesn't exist
vertices = zeros(Int32(maxNum), Int32(num_states + 1)) 

# [parent_handler(means row_idx, not endocded states idx), neighbor_handlers(many, use 0 to fill space)]
# handler == 0 means undefined
edges = zeros(Int32(maxNum), Int32(2 + num_neighbors))

# (f, g), f = g+h, g is traversed cost, h is heuristic cost-to-go
costs_heap = MutableBinaryMinHeap{Tuple{Float64, Float64}}() 


@time begin
for count in 1:maxNum
    global vertices, edges, costs_heap_table, handler
    cost = (100*rand(1)[1],100*rand(1)[1])
    handler = push!(costs_heap, cost)
    vertices[count, 1] = Float64(handler)
    edges[count, 1] = Float64(handler)
end
end


# Let `h` be a heap, `v` be a value, and `n` be an integer size

# length(h)            # returns the number of elements

# isempty(h)           # returns whether the heap is empty

# push!(h, v)          # add a value to the heap

# first(h)             # return the first (top) value of a heap

# pop!(h)              # removes the first (top) value, and returns it

# extract_all!(h)      # removes all elements and returns sorted array

# extract_all_rev!(h)  # removes all elements and returns reverse sorted array

# sizehint!(h, n)      # reserve capacity for at least `n` elements

# Let `h` be a heap, `i` be a handle, and `v` be a value.

# i = push!(h, v)            # adds a value to the heap and and returns a handle to v

# update!(h, i, v)           # updates the value of an element (referred to by the handle i)

# delete!(h, i)              # deletes the node with handle i from the heap

# v, i = top_with_handle(h)  # returns the top value of a heap and its handle