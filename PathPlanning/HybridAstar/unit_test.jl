cur_st  = [1,2,pi/2]
goal_st = [2,3,pi/2+pi/4]

path = cubic_fit(cur_st, goal_st)

plot(path[1,:], path[2,:])