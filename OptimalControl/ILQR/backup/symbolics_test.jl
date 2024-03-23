using Symbolics
# @variables x,y,ψ, ux, sa

# #################### DEFINE DYNAMICS #################### 
# L = 3
# # states: x, y, ψ
# # ctrls: ux, sa
# δx = ux*cos(ψ)
# δy = ux*sin(ψ)
# δψ = ux/L*tan(sa)

# A = Symbolics.jacobian([δx, δy, δψ], [x, y, ψ])
# B = Symbolics.jacobian([δx, δy, δψ], [ux, sa])

@variables states[1:3] ctrls[1:2]

#################### DEFINE DYNAMICS #################### 
L = 3
# states: x, y, ψ
# ctrls: ux, sa
δx = ctrls[1]*cos(states[3])
δy = ctrls[1]*sin(states[3])
δψ = ctrls[1]/L*tan(ctrls[2])

A = Symbolics.jacobian([δx, δy, δψ], states)
B = Symbolics.jacobian([δx, δy, δψ], ctrls)


# Afunc = eval(build_function(A, u_m, v_m, w_m, b, c, parallel = Symbolics.MultithreadedForm())[2])


# AMat = similar(A, Float64)
# A_fc(AMat,[1,2,3],[1,2])
# # res = simplify.(substitute.(Jac, (Dict(x => rand(1)[1], y=>rand(1)[1]),)))
# Hes = simplify.(Symbolics.hessian(x + exp(sin(x^2*y)^2), [x,y]))
# res = simplify.(substitute.(Hes, (Dict(x => rand(1)[1], y=>rand(1)[1]),)))


# res = simplify.(substitute.(A, (Dict(states => rand(3,1)[:], ctrls=>rand(2,1)[:]),)))


using BenchmarkTools
include("utils.jl")
@benchmark res = simplify.(substitute.(A, (Dict(states => [1,2,3], ctrls=>[1,2]),)))

# @benchmark jacobian([1;2;3], [1;2])
@benchmark gradient_hessian(stage_cost, [1;2;3], [1;2])