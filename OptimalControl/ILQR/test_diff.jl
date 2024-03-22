using ForwardDiff
using ForwardDiff: GradientConfig, Chunk, gradient!

function rosenbrock(x)
    a = one(eltype(x))
    b = 100 * a
    result = zero(eltype(x))
    for i in 1:length(x)-1
        result += (a - x[i])^2 + b*(x[i+1] - x[i]^2)^2
    end
    return result
end


x = rand(5);
out = similar(x)
cfg1 = ForwardDiff.GradientConfig(rosenbrock, x)
@time gradient!(out, rosenbrock, x, cfg1);