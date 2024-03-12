struct Point
    x::Int
    y::Int
end

function bfs(start::Point, goal::Point, obstacle::Point, radius::Int)
    queue = [start]
    parent = Dict{Point, Point}()
    visited = Set{Point}()
    push!(visited, start)

    while !isempty(queue)
        current = popfirst!(queue)

        if current == goal
            path = []
            while current != start
                pushfirst!(path, current)
                current = parent[current]
            end
            pushfirst!(path, start)
            return path
        end

        for dx in -1:1, dy in -1:1
            if dx == 0 && dy == 0
                continue
            end
            next_point = Point(current.x + dx, current.y + dy)

            # Check if next point is inside the obstacle
            if (next_point.x - obstacle.x)^2 + (next_point.y - obstacle.y)^2 >= radius^2
                if !(next_point in visited)
                    push!(queue, next_point)
                    push!(visited, next_point)
                    parent[next_point] = current
                end
            end


        end
    end

    return []
end

start = Point(0, 0)
goal = Point(100, 0)
obstacle = Point(25, 1)
radius = 3

@benchmark path = bfs(start, goal, obstacle, radius)
if isempty(path)
    println("No path found!")
else
    println("Path found:")
    for point in path
        println("(", point.x, ", ", point.y, ")")
    end
end