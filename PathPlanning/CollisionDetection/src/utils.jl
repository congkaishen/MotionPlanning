
function PlotVehicle(h, pts)
    h = plot!(h, pts[1,:], pts[2,:], linestyle=:dash, color =:green,  legend = false)
    return h
end

function PlotWall(h, pts)
    for i in 1:size(pts,3)
        h = plot!(h, pts[1,:,i], pts[2,:,i], seriestype = [:shape,], color =:black, legend = false, aspect_ratio =:equal)
    end
    return h
end

function GetRectanglePts(block)
    # block is 5 element vector: x,y,ψ,l,w
    # order is UpLeft, LowerLeft, LowerRight, UpperRight, back to UpLeft
    ox = block[1]
    oy = block[2]
    ψ  = block[3]
    length = block[4]
    width  = block[5]
    pts = [[-length,width] [-length,-width] [length, -width] [length, width] [-length,width]]
    R = [cos(ψ) -sin(ψ); sin(ψ) cos(ψ)]
    return R*pts .+ [ox; oy]
end


function Block2Pts(block_info)
    pts = zeros(2,5,size(block_info, 1))
    for block_idx = 1:1:size(block_info, 1)
        pts[:, :, block_idx] = GetRectanglePts(block_info[block_idx])
    end
    return pts
end


function SeparatingAxisTheorem(pts_base, pts_other)
    # false means collision
    for bg_idx = 1:1:(size(pts_base, 2) - 1)

        bg_pt = pts_base[:, bg_idx]
        ed_pt = pts_base[:, bg_idx+1]

        base_vec = ed_pt - bg_pt

        normal_vec = [-base_vec[2]; base_vec[1]]
        
        base_dps = transpose(pts_base .- bg_pt)*normal_vec
        other_dps = transpose(pts_other .- bg_pt)*normal_vec

        min_base = minimum(base_dps)
        max_base = maximum(base_dps)

        min_other = minimum(other_dps)
        max_other = maximum(other_dps)
        
        if ((max_other <= min_base) || (max_base <= min_other))
            return true
        end
    end
    return false
end

function ConvexCollision(pts1, pts2)
    if SeparatingAxisTheorem(pts1, pts2)
        if SeparatingAxisTheorem(pts2, pts1)
            return true
        else
            return false
        end
    else
        return false
    end
end