using DataStructures


maxNum = 10000
handler_table = Matrix{Any}(undef, maxNum, 1)
heap_table = MutableBinaryMinHeap{Tuple{Float64, Float64}}()

# count = 1
# cost = 100*rand(1)[1]
# handler = push!(heap_table, cost)
# handler_table[count, 1] = handler

@time begin
for count in 1:maxNum
    global handler, handler_table, heap_table
    cost = (100*rand(1)[1],100*rand(1)[1])
    handler = push!(heap_table, cost)
    handler_table[count, 1] = handler
end
end