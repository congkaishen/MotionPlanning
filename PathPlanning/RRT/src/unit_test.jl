using Plots
using DelimitedFiles
using ProgressBars
using Distributed
using LinearAlgebra
using Shuffle
using Parameters
using NearestNeighbors
using Statistics
using Distances
using MAT
using ProgressMeter
using Distributions


function get_cubic_par(Δx, Δy, Δψ)
	b_col = [tan(Δψ); Δy]
	A_mat = [[3*Δx^2 2*Δx]; [Δx^3 Δx^2]]
	return inv(A_mat)*b_col
end

function get_2Dcurvature(cubic_par_list, x)
	a = cubic_par_list[1]
	b = cubic_par_list[2]
	k = abs.(6*a*x .+ 2*b)./(1 .+ (3*a*x.^2 .+ 2*b*x).^2 ).^(3/2)
	return k
end

Δx = 3.0
Δy = 1.0
Δψ = 0.32

par_list = get_cubic_par(Δx, Δy, Δψ)
x = 0:0.01:Δx
k_list = get_2Dcurvature(par_list, x)
k_max_idx = argmax(k_list)

y = par_list[1]*x.^3 + par_list[2]*x.^2

plot(x, y)
scatter!([x[k_max_idx]], [y[k_max_idx]])

# plot(x, k_list)
