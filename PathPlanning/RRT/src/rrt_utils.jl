
function sampleRRT(rrt::RRTSearcher)::TPPNode
    max_x = rrt.s.BoundPosition[2]
    min_x = rrt.s.BoundPosition[1]
    max_y = rrt.s.BoundPosition[4]
    min_y = rrt.s.BoundPosition[3]

    bias_rate = rrt.s.bias_rate
    rand_num = rand(1)[1]

    if rand_num < bias_rate[1]
        xpos = rrt.p.ending_node.loc[1]
        ypos = rrt.p.ending_node.loc[2]
        zpos = 0.0
        ψ = (rand(1)[1]-0.5)*2*pi
    elseif rand_num >= bias_rate[1] && rand_num <= bias_rate[2]
        biased_points_number = min(floor(Int, rrt.p.sample_idx / rrt.s.sampling_number * size(rrt.p.sampling_bias_points, 1)) + 1, size(rrt.p.sampling_bias_points, 1))
        biased_points = rrt.p.sampling_bias_points[biased_points_number, :]
        rand_rho = rand(1)[1] * rrt.s.sample_max_dist

        rand_angle = rand(1)[1] * 2 * pi - pi
        xpos = biased_points[1] + rand_rho * cos(rand_angle)
        ypos = biased_points[2] + rand_rho * sin(rand_angle)
        zpos = 0.0

        if biased_points_number < size(rrt.p.sampling_bias_points, 1)
            next_pt = rrt.p.sampling_bias_points[biased_points_number+1,:]
            cur_pt = rrt.p.sampling_bias_points[biased_points_number,:]
        else
            next_pt = rrt.p.sampling_bias_points[biased_points_number,:]
            cur_pt = rrt.p.sampling_bias_points[biased_points_number-1,:]
        end
        mean_yaw1 = atan( (next_pt[2]-cur_pt[2])/(next_pt[1]-cur_pt[1]) )

        biased_points_number = maximum([biased_points_number-1, 1 ])
        if biased_points_number < size(rrt.p.sampling_bias_points, 1)
            next_pt = rrt.p.sampling_bias_points[biased_points_number+1,:]
            cur_pt = rrt.p.sampling_bias_points[biased_points_number,:]
        else
            next_pt = rrt.p.sampling_bias_points[biased_points_number,:]
            cur_pt = rrt.p.sampling_bias_points[biased_points_number-1,:]
        end
        mean_yaw2 = atan( (next_pt[2]-cur_pt[2])/(next_pt[1]-cur_pt[1]) )

        mean_yaw = (mean_yaw1+mean_yaw2)/2

        # println(mean_yaw)

        yaw_model = Normal(mean_yaw, pi/4)
        ψ = rand(yaw_model, 1)[1]


    else
        xpos = rand(1).*(max_x-min_x).+min_x
        ypos = rand(1).*(max_y-min_y).+min_y
        zpos = 0.0
        ψ = (rand(1)[1]-0.5)*2*pi
    end
    idxs, dists = knn(rrt.s.terrain_kdtree, [xpos; ypos], 5, true)
    zpos = rrt.s.terrain_info[3, idxs[1]]

    newloc = [xpos;ypos;zpos]
    normalvec = getNormalVec(rrt, newloc, idxs)
    θ = asin(normalvec[1])
    φ = asin(-normalvec[2]/(cos(θ)))
    ux = (rand(1)[1])*7+1

    node = TPPNode(newloc, [φ;θ;ψ], ux, 0.0)
    return node
end


function euler2Rot(eulerang::Vector{Float64})::Matrix{Float64}
    φ = eulerang[1]
    θ = eulerang[2]
    ψ =eulerang[3]
    R = [cos(θ)*cos(ψ) -cos(θ)*sin(ψ) sin(θ);
    cos(φ)*sin(ψ)+cos(ψ)*sin(φ)*sin(θ) cos(φ)*cos(ψ)-sin(φ)*sin(θ)*sin(ψ) -cos(θ)*sin(φ);
    sin(φ)*sin(ψ)-cos(φ)*cos(ψ)*sin(θ) cos(ψ)*sin(φ)+cos(φ)*sin(θ)*sin(ψ) cos(φ)*cos(θ)]
    R = R*sign(R[3,3])
    return R
end


function getChildYaw(args...)::Float64
    if length(args) == 2
        startnode = args[1]
        new_node = args[2]
        pos1 = startnode.loc
        pos2 = new_node.loc
        R1 = euler2Rot(startnode.eulerang)
        R2 = euler2Rot(new_node.eulerang)
    elseif length(args) == 1
        new_node = args[1][:, 2]
        startnode = args[1][:, 1]
        pos1 = startnode[1:3]
        pos2 = new_node[1:3]
        R1 = euler2Rot(startnode[4:6])
        R2 = euler2Rot(new_node[4:6])
    end
    h1 = R1[:,1]
    n1 = R1[:,3]
    zpos = -(n1[1]*(pos2[1]-pos1[1])+n1[2]*(pos2[2]-pos1[2]))/n1[3]+pos1[3]
    proj_loc = [pos2[1]; pos2[2]; zpos]
    dis_vec = proj_loc-pos1


    if norm(dis_vec) >= 0.001
        dis_vec = dis_vec/norm(dis_vec)
        Δψ_sign = cross(h1, dis_vec)
        Δψ = acos( round(dot(dis_vec, h1)/(norm(dis_vec)*norm(h1)), digits = 2  ) )*sign(Δψ_sign[3])
    else
        Δψ = 0
    end
    return Δψ
end



function getNormalVec(rrt, pos2::Vector{Float64}, idxs::Vector{Int64})::Vector{Float64}
    p_list =  rrt.s.terrain_info[:,idxs]
    cov_matrix  =cov(p_list, dims=2)
    sconv_matrix = SMatrix{3,3}(cov_matrix)
    eigen_result = eigen(sconv_matrix)
    svecs = eigen_result.vectors
    normalvec = [svecs[1,1]; svecs[2,1]; svecs[3,1]]
    normalvec = normalvec*sign(normalvec[3])
    return normalvec
end


function setChildSpeed(startnode::TPPNode, new_node::TPPNode)::Bool # true means change, false means no change
    v2 = new_node.ux
    v1 = startnode.ux
    dist = sqrt(sum( (new_node.loc-startnode.loc).^2))

    h1 = new_node.loc - startnode.loc
    h1 = h1/norm(h1)
    R1 = euler2Rot(startnode.eulerang)
    R2 = euler2Rot(new_node.eulerang)
    n1 = R1[:,3]
    n2 = R2[:,3]
    n = n1+n2
    n = n/norm(n)
    lat_vec = cross(n, h1)
    lat_vec = lat_vec/norm(lat_vec)
    normal_vec = cross(h1, lat_vec)
    normal_vec = normal_vec/norm(normal_vec)

    init_h = R1[:,1]

    T₁ =  R1[:,1]
    T₂ =  R2[:,1]
    dTdS = (T₂ - T₁)/dist

    k_n = round( dot(normal_vec, dTdS), digits = 3) # positive means more normal force
    k_g = round( dot(lat_vec, dTdS), digits = 3) # positive means to left

    if k_n >= 0
        verti_v2 = Inf
    else
        verti_v2 = 2*sqrt(abs(normal_vec[3]*g)/(-k_n)  )-v1
    end

    if verti_v2 <=0
        new_node.ux = 0
        return true
    end

    centrifugal = -(v1^2*k_g)

    y_friction = -lat_vec[3]*g + centrifugal
    tol_friction = mu_f* (abs(normal_vec[3]*g) +  (v1^2)*k_n )
    x_frinction_sqaure = tol_friction^2-y_friction^2

    if x_frinction_sqaure < 0
        new_node.ux = 0
        return true
    end

    x_frinction = sqrt(x_frinction_sqaure)
    x_acc = min(x_frinction, torque_acc)
    max_acc = x_acc  -1* h1[3]*g
    min_acc = -x_acc -1* h1[3]*g

    max_acc = min(x_acc - 1* h1[3]*g, 6)
    min_acc = max(-x_acc -1* h1[3]*g, -6)

    ux_sq_max = v1^2+2*max_acc*dist
    ux_sq_min = v1^2+2*min_acc*dist
    ux_sq = min(max(v2^2, ux_sq_min), ux_sq_max)

    ux_sq = min(ux_sq, verti_v2^2)

    if ux_sq >=1
        new_node.ux = sqrt(ux_sq)
        if abs(new_node.ux^2 - (ux_sq_min))<=0.01 || abs(new_node.ux^2 - (ux_sq_max))<=0.01
            return true
        else
            return false
        end
    else
        new_node.ux = 0
        return true
    end
end




function checkChildSpeed(args...)::Bool
    if length(args) == 2
        new_node = args[2]
        startnode = args[1]

        v2 = new_node.ux
        v1 = startnode.ux
        dist = sqrt(sum( (new_node.loc-startnode.loc).^2))

        h1 = new_node.loc - startnode.loc
        h1 = h1/norm(h1)
        R1 = euler2Rot(startnode.eulerang)
        R2 = euler2Rot(new_node.eulerang)
        n1 = R1[:,3]
        n2 = R2[:,3]
        n = n1+n2
        n = n/norm(n)
        lat_vec = cross(n, h1)
        lat_vec = lat_vec/norm(lat_vec)
        normal_vec = cross(h1, lat_vec)
        normal_vec = normal_vec/norm(normal_vec)

        init_h = R1[:,1]

        T₁ =  R1[:,1]
        T₂ =  R2[:,1]
        dTdS = (T₂ - T₁)/dist

        k_n = round( dot(normal_vec, dTdS), digits = 3) # positive means more normal force
        k_g = round( dot(lat_vec, dTdS), digits = 3) # positive means to left

        if k_n >= 0
            verti_v2 = Inf
        else
            verti_v2 = 2*sqrt(abs(normal_vec[3]*g)/(-k_n)  )-v1
        end

        if verti_v2 <=0
            new_node.ux = 0
            return true
        end

        centrifugal = -(  ((v1+v2)/2)^2*k_g)


        y_friction = -lat_vec[3]*g + centrifugal
        tol_friction = mu_f* (abs(normal_vec[3]*g) +  (((v1+v2)/2)^2)*k_n )
        x_frinction_sqaure = tol_friction^2-y_friction^2

        if x_frinction_sqaure < 0
            return false
        end

        x_frinction = sqrt(x_frinction_sqaure)
        x_acc = min(x_frinction, torque_acc)
        max_acc = x_acc  -1* h1[3]*g
        min_acc = -x_acc -1* h1[3]*g

        ux_sq_max = v1^2+2*max_acc*dist
        ux_sq_min = v1^2+2*min_acc*dist

        if v2^2 >= ux_sq_min && v2^2 <= ux_sq_max  && v2 >= 1.1 && v2 <= verti_v2
            return true
        else
            return false
        end
    elseif length(args) == 1
        new_node = args[1][:, 2]
        startnode = args[1][:, 1]
        v2 = new_node[7]
        v1 = startnode[7]
        dist = sqrt(sum( (new_node[1:3]-startnode[1:3]).^2))

        h1 = new_node[1:3]-startnode[1:3]
        h1 = h1/norm(h1)
        R1 = euler2Rot(startnode[4:6])
        R2 = euler2Rot(new_node[4:6])
        n1 = R1[:,3]
        n2 = R2[:,3]
        n = n1+n2
        n = n/norm(n)
        lat_vec = cross(n, h1)
        lat_vec = lat_vec/norm(lat_vec)
        normal_vec = cross(h1, lat_vec)
        normal_vec = normal_vec/norm(normal_vec)

        init_h = R1[:,1]

        T₁ =  R1[:,1]
        T₂ =  R2[:,1]
        dTdS = (T₂ - T₁)/dist

        k_n = round( dot(normal_vec, dTdS), digits = 3) # positive means more normal force
        k_g = round( dot(lat_vec, dTdS), digits = 3) # positive means to left

        if k_n >= 0
            verti_v2 = Inf
        else
            verti_v2 = 2*sqrt(abs(normal_vec[3]*g)/(-k_n)  )-v1
        end

        if verti_v2 <=0
            new_node.ux = 0
            return true
        end

        centrifugal = -(((v1+v2)/2)^2*k_g)


        y_friction = -lat_vec[3]*g + centrifugal
        tol_friction = mu_f* (abs(normal_vec[3]*g) +  ((v1+v2)/2)^2*k_n )
        x_frinction_sqaure = tol_friction^2-y_friction^2

        if x_frinction_sqaure < 0
            return false
        end

        x_frinction = sqrt(x_frinction_sqaure)
        x_acc = min(x_frinction, torque_acc)
        max_acc = x_acc  -1* h1[3]*g
        min_acc = -x_acc -1* h1[3]*g

        ux_sq_max = v1^2+2*max_acc*dist
        ux_sq_min = v1^2+2*min_acc*dist

        if v2^2 >= ux_sq_min && v2^2 <= ux_sq_max  && v2 >= 1.1 && v2 <= verti_v2
            return true
        else
            return false
        end

    end
end

function neural_cost(states)
    if size(states, 1) >5
        states = states[1:5]
    end

    x1 = deepcopy(states)
    x1[5] = x1[5] - x1[4]

    sx1 = SVector{5,Float64}(x1)
	x1_step1_xoffset = SVector{5,Float64}([0.804829203090416;-1.49870829359481;-0.359903686035566;1.00004152413469;-4.99979550389721])
	x1_step1_gain = SVector{5,Float64}([0.625949244716259;0.667038470913877;2.77871393313456;0.250021436676509;0.200366924650514])
	x1_step1_ymin = -1;

    b1 = SVector{16,Float64}([-1.3210606389189227805;-2.3241855366279819961;-0.017850569891583338566;2.1353561146218491551;-1.1513175309742269548;0.89845851467673820689;-1.3216203669795762998;-0.2399047581396230866;0.31676765238701148908;-1.3183628191676637353;0.5534211939844099204;-0.28634901262047357529;2.8359734966947782731;-0.88464475029866751488;1.848772091869806955;-1.8660472953508098204])
   
    IW1_1 = SMatrix{16,5}([-0.0091062526150703284461 -0.30768297979016273613 0.26007812984004086143 -0.04126956003031921616 0.76563302681512490633;0.72830629868020990703 0.2614327907626754488 -2.6017855481109846139 -0.24455447561246249077 -0.29894302711338904821;1.6881524374173981773 -5.0505913056995481725 4.1772883203047985745 -0.40636395838178868134 -0.26375049162658137369;-0.57964814630536010132 0.95402500241241816603 -3.2595783148867849555 0.30478659843231453719 0.11274841805097894631;1.1667511452209717859 0.97106715558156797563 1.2539111328051459893 -0.67138780936304975899 -0.23999240326666143597;-1.6104895757466204387 -1.6667592426708419673 1.3611940453630877457 0.24955153642973978978 0.22670904223100482588;1.2093059381700470567 -0.56897349589254009317 1.4564286895050586601 0.31095544463273383506 0.048293926396569264448;-0.65636996269055281061 -1.0573156944592212447 1.8133129949240562873 -0.20836177551232223926 -0.2925993936556148256;-0.23714267439502856316 0.47391178439565168068 -0.43746550974703501957 0.52170284926598509134 0.39756046772378372811;-0.31617490246245455454 3.8034813073436120057 -3.9792169573877478328 -0.14031391511906296521 0.047055640328616962775;-0.033363258699510789373 7.9179990718196364696 -5.3801667185191632825 -0.2380097654938089502 -0.029787119296321790812;-0.81099064230636619399 -4.1399196350097069441 2.1450916429982078704 0.29357932455217794487 0.10153860309741261081;-1.0318289159928633048 0.35453609592839147036 -0.587315593222957677 3.2853867217401684364 3.3615862900445221406;-0.002739611326346755478 -0.97012820512329611677 0.8312357241715585543 -0.29200928237914125507 -0.042907187148370280749;0.012056899451385562003 -0.223137654965350124 0.21344480812797445024 -0.098687816175724266121 0.86162445015255806346;-1.4705190621366219794 -0.73557064539199545905 1.41697402176982723 -0.24939341761058131386 -0.20316848296781400696])

    b2 = SVector{12,Float64}([-0.66132943856004200445;2.2412981191521783586;1.9389122244583432142;1.5239959792805031213;-0.082274234093199505313;-0.33462914911236985027;-4.203315237468506993;0.26050721009610183776;0.45422211762846403937;1.3044648604597530728;0.016390805659142117234;-2.1365989817751334101])

    LW2_1 = SMatrix{12,16}([-0.18830492097153078701 -0.019775540729373618232 0.025874913898548797253 -0.10772682465279188879 -0.0087947824525977571358 0.0017970497663883317017 0.081071352522409642627 0.041863540559696293952 -0.26173923101769369648 0.057413842929785373348 0.028181684304930416235 0.44461618611657444111 -0.030797250283116941322 0.91726094764879240184 0.46068729195806246146 -0.72911282612642658307;-0.16750771489195118158 -0.37763758806754055275 0.68722531861057689628 -1.3458144738685762221 0.10426580006375338328 -0.2232622153338462978 0.88816351635500279116 0.19956268259942802623 2.1138081551069864972 -0.78582795031872521019 -0.34851990332024407326 -0.32297141284302716135 0.054273161163595075307 -1.1631121370494841294 0.29274987348745784566 0.016975342319065809399;0.10003587750112709343 -0.33215202175095553949 -0.99792909764180748322 4.024554681574466386 -0.20968241818635616203 -1.3519851389450103785 -1.9042862626661818393 0.81557048550173349977 3.1077158495421328688 1.5283607739267861536 1.2142829712746545656 -3.4982330728094348515 -0.15679483100367999637 -1.743281873243548219 1.1731334320595696585 -0.080526255348998543315;-0.84588649511936342407 -1.7415726378319471568 2.4843810590231361068 0.31144962967364547834 -0.88681631545118666526 -0.19616915926071867848 -0.21219971162809189713 0.52045502403997623286 2.827315980002499618 0.47907073983849635113 -0.69869282820753153551 -0.2436904988104134917 0.03403160024247882387 -0.45024793843753829758 -1.1450166537271566636 -0.018322228020038276025;0.12039284698034256882 0.074670267724201724868 -0.03066658136254944364 -0.091140882772030279435 -0.040341976326197209746 -0.040459349578516282109 -0.21835309006800374121 -0.2733689214186653671 0.028801482554327718921 -0.45042311456484501742 0.27523669812174877958 -0.22746601499863774998 0.033558984319804398277 -0.95723034371457671465 -0.26302963529968881318 0.8973564917834090382;-0.11186664264995900553 0.19120934900934485889 -0.1440121578211326081 -0.083625269769057153368 0.01219870627028076665 -0.21708953774637396172 -0.17094292475342026671 0.38329191101096449223 -0.49977547974443714551 0.25215841989889248298 -0.056542881423666382923 -0.13062614715874887317 -0.078532239436526041221 -0.86722926765210484046 0.9418186125925210872 0.19287680177443869423;1.6573710099884451097 -11.099353112193867332 12.683966647443448039 6.3439274212220198024 -3.0620698877391627768 3.6995482177088363507 7.3890581751814394806 -10.200105723032914185 -1.2745811233373094407 -8.7326084996067532273 11.993573357070733465 14.544394477787463416 -0.93046782352888979251 3.5566603500632401058 -1.8230290249104945666 3.1598213190964563601;0.32299915711721788325 0.034764223568413341048 -0.032048866319414504034 0.034763374176029822327 0.02930352794643800024 -0.032841229803743161864 0.015545972409070706044 0.21581335673365581451 -0.16213003105745704691 0.28435371069247861175 -0.16935283142815371615 -0.10756718672051422681 -0.01906911684798330564 -0.46252103430494112546 -0.22049296147628316689 -0.082158266895411682929;0.16727647038736473406 0.096698884927724451654 -0.50198439801742067434 -0.29409523309152157955 -0.032710152336071024459 0.064171693524105941075 -0.32201965724335174235 -0.2463907223804578106 -2.3233928547480782179 0.95745411761532850381 0.2092516474630625456 0.49321514338302618885 -0.055712050938217405616 1.3060547382444318565 -0.47362566090618918313 -0.11732339134324415442;-0.85101177421074758467 0.042213565087174734214 -0.48165887357338499353 4.3664921577526278895 -0.70184310949925099443 -0.52586099015142284685 -0.43331271730408216225 -0.44058340344112972842 1.9211824095849796556 -0.28289738650588081015 1.5483627770645873589 -4.1103850870862519429 -0.090193598573221914205 1.110665866652828182 0.49456295674628986125 0.59833744095853780287;0.9373461808903906789 2.5697240251087460905 -1.986259198488224742 -4.8800688184381719736 4.3223931050922193009 14.051456248181793285 5.6639175344415697566 -0.92108607461003899619 -11.998653051636582134 6.5543571624679408671 -4.5100783153614854015 9.4368090875991939015 0.88812459686222233746 0.74725919062149070982 3.4600320611304056229 -3.8378864036280151417;-0.026509938416816175577 -0.3346806638617791374 -0.70280870849739585093 0.60673146138490074986 0.060571099131546915495 -0.70323616008583589299 -0.13519905700700804441 -1.476584733432789065 -0.046011552492629545497 0.54877333472777323209 -1.0138670199934314287 0.36920387455534675603 -0.29121362630933161908 2.2249953588859923137 -0.43958582502733795083 0.078589179604717715466])
    
    b3 = 2.426565331689628735
    
    LW3_2 = SMatrix{1,12}([1.5665145507715152462 3.6880266307609597121 2.9302634898089898741 -3.5780231590766313765 1.5289997117823708184 -0.86038774868399081619 -0.65351166455708720182 3.6572230296979775233 2.866987982375344135 -2.5454921917393997788 0.95745026159772783636 -2.4044821013891990447])


    y1_step1_ymin = -1;
	y1_step1_gain = 0.204889890977298;
	y1_step1_xoffset = 0.112905128149589;
	xtemp = sx1 .- x1_step1_xoffset;
	xtemp = xtemp .* x1_step1_gain;
	xp1 = xtemp .+ x1_step1_ymin;
	a1 = 2 ./ (1 .+ exp.(-2*(b1 + IW1_1 * xp1))) .- 1;
	a2 = 2 ./ (1 .+ exp.(-2*(b2 + LW2_1 * a1))) .- 1;
	a3 = b3 .+ LW3_2*a2;
	ytemp = a3 .- y1_step1_ymin;
	ytemp = ytemp ./ y1_step1_gain;
	y1 = ytemp .+ y1_step1_xoffset;
    cost = y1[1] + 1*x1[2]^2 + 10*(x1[3])^2 + 0.1*(x1[5])^2
    # cost = y1[1] 
	return cost
end

function get_max_acc(dist)
    a = -0.5078
    return a*(dist - 2.4).^2 + (5-a*(0.8-2.4)^2)
end

function get_max_ang_ratio(dist)
    # if dist < 1
    #     max_ang = 0.005
    # else
    #     max_ang = 0.0155 * dist^2 + 0.092518 * dist - 0.07;
    # end
    max_y = 0.4
    min_y = 0.279
    mid_x = 2.1300
    slope = 10.0700
    max_ang = (max_y-min_y)/(1+ exp(-slope*(dist-mid_x)))+min_y
    # max_ang = 0.32
    return max_ang/dist
end

function get_max_yawangle(x::Float64, y::Float64)
    p00 =    -0.03911
    p10 =     0.05831
    p01 =      0.1148
    p20 =    -0.04344
    p11 =      0.4135
    p02 =    -0.05552
    p30 =    0.005515
    p21 =    -0.05701
    p12 =      0.1892
    p03 =     -0.1725
    z1 = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 + p21*x^2*y + p12*x*y^2 + p03*y^3;

    p00 =     0.03911
    p10 =    -0.05831
    p01 =      0.1148
    p20 =     0.04344
    p11 =      0.4135
    p02 =     0.05552
    p30 =   -0.005515
    p21 =    -0.05701
    p12 =     -0.1892
    p03 =     -0.1725
    z2 = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 + p21*x^2*y + p12*x*y^2 + p03*y^3;
    return [z1;z2]
end

function DistToDeltaXY(dist, max_angle_ratio)
    max_angle = max_angle_ratio * dist
    Δx = dist * cos(max_angle)
    Δy = dist * sin(max_angle)
    return [Δx, Δy]
end

function FeasibilityClassifier(x1, planner)
    if size(x1, 1) >5
        x1 = x1[1:5]
    end

    dist = sqrt(x1[1]^2 +x1[2]^2)

    θ = atan(x1[2]/x1[1])
    Δψ = x1[3]
    u1 = x1[4]
    u2 = x1[5]

    if θ > abs(get_max_ang_ratio(dist)*dist)
        return false
    end

    ψ_bounds = get_max_yawangle(dist, θ)
    if Δψ < ψ_bounds[1] || Δψ > ψ_bounds[2]
        return false
    end

    if abs((u2^2-u1^2)/(2*dist))> get_max_acc(dist)
        return false
    end

    return true
end

function getChildYaw(args...)::Float64
    if length(args) == 2
        startnode = args[1]
        new_node = args[2]
        pos1 = startnode.loc
        pos2 = new_node.loc
        R1 = euler2Rot(startnode.eulerang)
        R2 = euler2Rot(new_node.eulerang)
    elseif length(args) == 1
        new_node = args[1][:, 2]
        startnode = args[1][:, 1]
        pos1 = startnode[1:3]
        pos2 = new_node[1:3]
        R1 = euler2Rot(startnode[4:6])
        R2 = euler2Rot(new_node[4:6])
    end
    h1 = R1[:,1]
    dis_vec = pos2-pos1
    dis_vec = dis_vec/norm(dis_vec)
    if norm(dis_vec) >= 0.001
        rho = norm(dis_vec)
        dis_vec = dis_vec/norm(dis_vec)
        Δψ_sign = cross(h1, dis_vec)
        Δψ = acos( round(dot(dis_vec, h1)/(norm(dis_vec)*norm(h1)), digits = 2  ) )*sign(Δψ_sign[3])
    else
        rho = 0
        Δψ = 0
    end
    return Δψ
end

function ProjectionStates(args...)
    if length(args) == 2
        node1 = args[1]
        node2 = args[2]
        pos1 = node1.loc
        pos2 = node2.loc
        eulerang1 = node1.eulerang
        eulerang2 = node2.eulerang
        R1 = euler2Rot(eulerang1)
        R2 = euler2Rot(eulerang2)
        u1 = node1.ux
        u2 = node2.ux
    elseif length(args) == 1
        node1 = args[1][:, 1]
        node2 = args[1][:, 2]
        pos1 = node1[1:3]
        pos2 = node2[1:3]
        eulerang1 = node1[4:6]
        eulerang2 = node2[4:6]
        R1 = euler2Rot(eulerang1)
        R2 = euler2Rot(eulerang2)
        u1 = node1[7]
        u2 = node2[7]
    end

    h1 = R1[:,1]
    n1 = R1[:,3]

    zpos = -(n1[1]*(pos2[1]-pos1[1])+n1[2]*(pos2[2]-pos1[2]))/n1[3]+pos1[3]
    proj_loc = [pos2[1]; pos2[2]; zpos]
    dis_vec = proj_loc-pos1

    if norm(dis_vec) >= 0.001
        rho = norm(dis_vec)
        dis_vec = dis_vec/norm(dis_vec)
        Δψ_sign = cross(h1, dis_vec)
        Δψ = acos( round(dot(dis_vec, h1)/(norm(dis_vec)*norm(h1)), digits = 2  ) )*sign(Δψ_sign[3])
    else
        rho = 0
        Δψ = 0
    end

    Δx = round(rho*cos(Δψ), digits = 2)
    Δy = round(rho*sin(Δψ), digits = 2)
    Δψ_head = round(eulerang2[3]- eulerang1[3], digits = 3)

    states = [Δx;Δy;Δψ_head;u1;u2]
    return states
end

function InitialCheck(states)
    Δx = states[1]
    Δy = states[2]
    Δψ_head = states[3]
    u1 = states[4]
    u2 = states[5]

    if Δx<1.0 || Δx>3.0
        return false
    end

    if Δy<-1.0 || Δy>1.0
        return false
    end

    if Δψ_head<-0.35 || Δψ_head>0.35
        return false
    end

    if u1<1 || u1>9
        return false
    end

    if u2<1 || u2>9 || abs(u2-u1) >5
        return false
    end

    return true
end


function FromProj2Real(node1, node2, proj_states,  rrt::RRTSearcher)
    pos1 = node1.loc
    R1 = euler2Rot(node1.eulerang)
    h1 = R1[:,1]
    n1 = R1[:,3]
    l1 = cross(n1, h1)
    xy_loc = pos1[1:2]
    xy_loc = xy_loc + h1[1:2]*proj_states[1]
    xy_loc = xy_loc + l1[1:2]*proj_states[2]

    xpos = xy_loc[1]
    ypos = xy_loc[2]

    idxs, dists = knn(rrt.s.terrain_kdtree, [xpos; ypos], 5, true)
    zpos = rrt.s.terrain_info[3, idxs[1]]

    p_list =  rrt.s.terrain_info[:,idxs]
    p_m = mean!([1., 1., 1.], p_list)
    cov_matrix  =cov(p_list, dims=2)
    vecs = eigvecs(cov_matrix)
    normalvec = vecs[:,1]
    normalvec = normalvec*sign(normalvec[3])
    θ = asin(normalvec[1])
    φ = asin(-normalvec[2]/(cos(θ)))
    ψ = node1.eulerang[3]+proj_states[3]
    ux = proj_states[5]

    newloc = [xpos;ypos;zpos]
    neweulerang = [φ;θ;ψ]
    node2.loc = newloc
    node2.eulerang = neweulerang
    node2.ux = ux
end

function steer(rrt::RRTSearcher, startnode::TPPNode, targetnode::TPPNode)::TPPNode
    return steer_aggressive(rrt, startnode, targetnode)
end


function decide_proj(dist::Float64, ang_ratio::Float64, startnode::TPPNode, new_node::TPPNode)::Vector{Float64}

    ΔXY = DistToDeltaXY(dist, ang_ratio)
    acceptable_ψ_bounds = get_max_yawangle(dist, ang_ratio*dist)

    # search_num = 10
    # acceptable_ψ_list = range(acceptable_ψ_bounds[1], acceptable_ψ_bounds[2], search_num)
    # proj_states_list = zeros(search_num,5)
    # cost_list = zeros(search_num)
    # for proj_idx = 1:1:search_num
    #     proj_states_list[proj_idx, :] = [ΔXY[1]; ΔXY[2]; acceptable_ψ_list[proj_idx]; startnode.ux; new_node.ux]
    #     cost_list[proj_idx] = neural_cost(proj_states_list[proj_idx, :])
    # end
    # min_idx = argmin(cost_list)
    # proj_states = proj_states_list[min_idx,:]
    # acceptable_ψ_bounds = get_max_yawangle(dist, ang_ratio*dist)
    # acceptable_ψ = (acceptable_ψ_bounds[2]+acceptable_ψ_bounds[1])/2
    
    acceptable_ψ = rand(1)[1]*(acceptable_ψ_bounds[2]-acceptable_ψ_bounds[1])+acceptable_ψ_bounds[1]
    proj_states = [ΔXY[1]; ΔXY[2]; acceptable_ψ; startnode.ux; new_node.ux]
    return proj_states
end

function steer_aggressive(rrt::RRTSearcher, startnode::TPPNode, targetnode::TPPNode)::TPPNode
    RANDOM = true
    new_node = deepcopy(targetnode)
    min_dist = rrt.s.grow_dist[1]
    maxdist = rrt.s.grow_dist[2]
    mindist = rrt.s.grow_dist[1]
    proj_states = ProjectionStates(startnode, new_node)
    dist = sqrt(proj_states[1] ^2 + proj_states[2] ^2)
    # TODO Dist within range
    if dist <= maxdist && dist >= min_dist
        max_ang_ratio = get_max_ang_ratio(dist)

        if abs(proj_states[1])<= 0.01
            proj_states[1] = 0.01
        end
        Δψ = atan(proj_states[2]/proj_states[1])
        if abs(Δψ)/dist > max_ang_ratio
            if RANDOM
                ang_ratio = (2*rand(1)[1]-1)*max_ang_ratio
            else
                ang_ratio = max_ang_ratio*sign(Δψ)
            end
            proj_states = decide_proj(dist, ang_ratio, startnode, new_node)
            FromProj2Real(startnode, new_node, proj_states,  rrt)

        else
            ang_ratio = Δψ/dist
            proj_states = decide_proj(dist, ang_ratio, startnode, new_node)
            FromProj2Real(startnode, new_node, proj_states,  rrt)
        end
        setChildSpeed(startnode, new_node)
        proj_states = ProjectionStates(startnode, new_node)
        new_node.cost = startnode.cost +  neural_cost(proj_states)
        return new_node
    else
        # if RANDOM
        #     adjustdist = rand(1)[1]*(maxdist-mindist)+mindist
        # else
        #     adjustdist = maxdist * (dist > maxdist) + min_dist * (dist < min_dist)
        # end

        adjustdist = maxdist
        # adjustdist = mindist

        vec2 = [proj_states[1], proj_states[2], 0]
        if dist > 0.001
            tuned_vec2 = vec2.* (adjustdist/dist)
            tuned_proj_states = [tuned_vec2[1], tuned_vec2[2], proj_states[3], proj_states[4], proj_states[5]]
            FromProj2Real(startnode, new_node, tuned_proj_states,  rrt)
        end

        proj_states = ProjectionStates(startnode, new_node)
        max_ang_ratio = get_max_ang_ratio(adjustdist)

        if abs(proj_states[1])<= 0.01
            proj_states[1] = 0.01
        end
        Δψ = atan(proj_states[2]/proj_states[1])
        if abs(Δψ)/adjustdist > max_ang_ratio
            if RANDOM
                ang_ratio = (2*rand(1)[1]-1)*max_ang_ratio
            else
                ang_ratio = max_ang_ratio*sign(Δψ)
            end
            proj_states = decide_proj(adjustdist, ang_ratio, startnode, new_node)
            FromProj2Real(startnode, new_node, proj_states,  rrt)
        else
            ang_ratio = Δψ/adjustdist
            proj_states = decide_proj(adjustdist, ang_ratio, startnode, new_node)
            FromProj2Real(startnode, new_node, proj_states,  rrt)
        end
        setChildSpeed(startnode, new_node)
        proj_states = ProjectionStates(startnode, new_node)
        new_node.cost = startnode.cost +  neural_cost(proj_states)
        return new_node
    end
end



############################
function nearDisk(rrt::RRTSearcher)::Float64
    gamma = 6*(rrt.s.BoundPosition[2]-rrt.s.BoundPosition[1])*(rrt.s.BoundPosition[4]-rrt.s.BoundPosition[3])*5
    return minimum([gamma*(log(rrt.p.sample_idx)/rrt.p.sample_idx),  24])
end


function chooseParents(rrt::RRTSearcher, new_node::TPPNode, nearIdxs::Vector{Int64})::Int64
    mincost = Inf
    minIdx = -1
    for nearIdx in nearIdxs
        proj_states = ProjectionStates(rrt.p.nodes_collection[nearIdx], new_node)
        tempcost = rrt.p.costs_collection[nearIdx]+ neural_cost(proj_states)
        if tempcost <= mincost && !collision(rrt.s.obstacle_list, rrt.p.nodes_collection[nearIdx], new_node, rrt)
            minIdx = nearIdx
            mincost = tempcost
        end
    end
    new_node.cost = mincost
    return minIdx
end


function rewire(rrt::RRTSearcher, new_node::TPPNode, nearIdxs::Vector{Int64}, root_idx::Int64)
    updateIdx_list = Vector{Int64}()
    costchange_list = Vector{Float64}()
    for nearIdx in nearIdxs
        if nearIdx !== root_idx
            proj_states = ProjectionStates(rrt.p.nodes_collection[nearIdx], new_node)
            tempcost = rrt.p.costs_collection[rrt.p.sample_idx]+ neural_cost(proj_states)
            costchange = tempcost - rrt.p.costs_collection[nearIdx]
            if costchange < 0 && !collision(rrt.s.obstacle_list, new_node, rrt.p.nodes_collection[nearIdx], rrt)
                # costchange = tempcost
                parent_idx = rrt.p.parents_collection[nearIdx]
                delete!(rrt.p.children_collection[parent_idx], nearIdx)
                push!(rrt.p.children_collection[rrt.p.sample_idx], nearIdx)
                rrt.p.parents_collection[nearIdx] = rrt.p.sample_idx
                updateIdx_list = [updateIdx_list; nearIdx]
                costchange_list = [costchange_list; costchange]
            end
        end
    end
    if size(updateIdx_list,1)>0
        costPropogation(rrt, updateIdx_list, costchange_list)
    end
end

function costPropogation(rrt::RRTSearcher, updateIdx_list::Vector{Int64} ,costchange_list::Vector{Float64})
    list_num = size(updateIdx_list, 1)
    for list_idx in 1:list_num
        costchange = costchange_list[list_idx]
        parent_idx = updateIdx_list[list_idx]
        updateIdx = findAllUpdatedChildren(rrt.p.children_collection, parent_idx)
        updateIdx = [updateIdx;parent_idx]
        updateNum = size(updateIdx,1)
        if updateNum>0
            for i in 1:updateNum
                rrt.p.costs_collection[updateIdx[i]] = rrt.p.costs_collection[updateIdx[i]]+costchange
                rrt.p.nodes_collection[updateIdx[i]].cost = rrt.p.nodes_collection[updateIdx[i]].cost+costchange
            end
        end
    end
end

function findAllUpdatedChildren(children_collection::Dict{Int64, Set{Int64}}, parent_idx)::Vector{Int64}
    childrenIdxs = children_collection[parent_idx]
    desired_idxs = Vector{Int64}()
    if !isempty(childrenIdxs)
        for child_idx in childrenIdxs
            desired_idxs =  [desired_idxs;child_idx;findAllUpdatedChildren(children_collection, child_idx)]
        end
        return desired_idxs
    else
        return Vector{Int64}()
    end
end



##############################

function CheckWithinBoundary(loc::Vector{Float64}, BoundPosition)::Bool
    flag = false
    if loc[1]<BoundPosition[1]||loc[1]>BoundPosition[2]||loc[2]<BoundPosition[3]||loc[2]>BoundPosition[4]
        flag = true
    end
    return flag
end

function collision(obstacle_list::Vector{Vector{Float64}}, args...)::Bool
    if length(args) == 3
        node1 = args[1]
        node2 = args[2]
        planner = args[3]
        loc_info1 = node1.loc
        loc_info2 = node2.loc
        ψ2 = node2.eulerang[3]
        ψ1 = node1.eulerang[3]
    elseif length(args) == 2
        node1 = args[1][:, 1]
        node2 = args[1][:, 2]
        planner = args[2]
        loc_info1 = node1[1:3]
        loc_info2 = node2[1:3]
        ψ2 = node2[6]
        ψ1 = node1[6]
    else
        error("Wrong number of inputs in collision")
    end
    if CheckWithinBoundary(loc_info2, planner.s.BoundPosition)
        return true
    end

    if length(args) == 3
        checkspeed = checkChildSpeed(node1, node2)
        proj_states = ProjectionStates(node1, node2)
        if InitialCheck(proj_states) && FeasibilityClassifier(proj_states, planner)
            kinematic_feasibility = true
        else
            kinematic_feasibility = false
        end
        if !checkspeed || !kinematic_feasibility
            return true
        end

    elseif length(args) == 2
        checkspeed = checkChildSpeed([node1 node2])
        proj_states = ProjectionStates([node1 node2])
        if InitialCheckLTR(proj_states) && FeasibilityClassifier(proj_states, planner)
            kinematic_feasibility = true
        else
            kinematic_feasibility = false
        end

        if !checkspeed || !kinematic_feasibility
            return true
        end
    end

    if size(obstacle_list,1) == 0
        return false
    else
        intervals = 5
        start_pt = loc_info1[1:2]
        end_pt = loc_info2[1:2]
        x_list = range(start_pt[1], end_pt[1], intervals)
        y_list = range(start_pt[2], end_pt[2], intervals)
        for obs_idx in 1:size(obstacle_list,1)
            for j in 1:intervals
                if sum(([x_list[j];y_list[j]]-obstacle_list[obs_idx][1:2]).^2)<=obstacle_list[obs_idx][3]^2
                    return true
                end
            end
        end
        return false
    end
end


function CalculateRRTCost(rrt::RRTSearcher)
    final_idxs = reverse(rrt.r.resultIdxs)
    cost = 0;
    for i = 1:1:size(final_idxs, 1) - 1
        start_node = rrt.p.nodes_collection[final_idxs[i]]
        next_node = rrt.p.nodes_collection[final_idxs[i + 1]]
        proj_states = ProjectionStates(start_node, next_node)
        cost = cost + neural_cost(proj_states)
    end
    rrt.r.cost = cost
end

function registerNode(rrt::RRTSearcher, new_node::TPPNode, parent_idx::Int64)
    rrt.p.sample_idx = rrt.p.sample_idx + 1
    rrt.p.buffer_idx = rrt.p.buffer_idx + 1
    rrt.p.nodes_collection[rrt.p.sample_idx] = deepcopy(new_node)
    rrt.p.states_collection[:,rrt.p.sample_idx] = [new_node.loc; new_node.eulerang[3]; new_node.ux]
    rrt.p.parents_collection[rrt.p.sample_idx] = parent_idx
    rrt.p.children_collection[rrt.p.sample_idx] = Set{Int}()
    rrt.p.costs_collection[rrt.p.sample_idx] = new_node.cost
    push!(rrt.p.children_collection[parent_idx], rrt.p.sample_idx)
end





function planRRT!(rrt::RRTSearcher)
    t1 = time()
    sampleNum = rrt.s.sampling_number
    p = Progress(sampleNum-1, 0.5, "RRT Progress...", 60)

    while rrt.p.sample_idx <= sampleNum-1
        # println(rrt.p.sample_idx)
        sample_node = sampleRRT(rrt)
        idxs, min_dist = knn(rrt.p.balltree, [sample_node.loc;sample_node.eulerang[3];sample_node.ux], 1, true)
        min_idx = idxs[1]
        closest_node = rrt.p.nodes_collection[min_idx]
        new_node = steer(rrt, closest_node, sample_node)

        if !collision(rrt.s.obstacle_list, closest_node, new_node, rrt)
            next!(p)
            registerNode(rrt, new_node, min_idx)

            if rrt.s.rrt_star
                nearIdxs = inrange(rrt.p.balltree, [new_node.loc;new_node.eulerang[3];new_node.ux], nearDisk(rrt), true)
                if size(nearIdxs, 1) >= 30
                    nearIdxs,~ = knn(rrt.p.balltree, [new_node.loc;new_node.eulerang[3];new_node.ux], 30, true)
                end

                if size(nearIdxs,1) != 0
                    parent_idx = min_idx
                    # parent_idx =  chooseParents(rrt, new_node, nearIdxs)
                    rewire(rrt, new_node, nearIdxs, parent_idx)
                end
            end

            if rrt.p.buffer_idx == rrt.s.buffer_size
                rrt.p.balltree = BallTree(rrt.p.states_collection[:, 1:rrt.p.sample_idx])
                rrt.p.buffer_idx = 0
            end
        end
        t3 = time()
        if t3 - t1 > 100
            break
        end

    end
    rrt.p.kdtree = KDTree(rrt.p.states_collection[1:3, 1:rrt.p.sample_idx])
    ed_idx = findBestIdx(rrt)
    t2 = time()

    if ed_idx > 0
        rrt.r.status = :Solved
        rrt.r.Path = getBestPath(rrt, ed_idx)
        rrt.r.resultIdxs = getBestIdxs(rrt, ed_idx)
    else
        rrt.r.status = :NotSolved
    end
    rrt.r.tSolve = t2 - t1

    if rrt.s.draw_fig
        h = RRTplot(rrt)
    end
    return nothing;
end

function findBestIdx(rrt::RRTSearcher)
    idxs = inrange(rrt.p.kdtree, rrt.p.ending_node.loc, rrt.s.goal_tolerance, true)
    if size(idxs, 1) > 0
        cost_idxs = argmin(rrt.p.costs_collection[idxs])
        return idxs[cost_idxs]
    else
        return 0
    end
end

function getBestPath(rrt::RRTSearcher, idx::Int)::Matrix{Float64}
    path = rrt.p.nodes_collection[idx].loc
    while idx != 1
        idx = rrt.p.parents_collection[idx]
        path = [path rrt.p.nodes_collection[idx].loc]
    end
    return path
end

function getBestIdxs(rrt::RRTSearcher, idx::Int)::Vector{Int64}
    idxs = Vector{Int64}[]
    idxs = [idxs; idx]
    while idx != 1
        idx = rrt.p.parents_collection[idx]
        idxs = [idxs; idx]
    end
    return idxs
end

function getTree(rrt::RRTSearcher):: Matrix{Float64}
    first = true
    tree = nothing
    for idx in 1:rrt.p.sample_idx
        if (rrt.p.parents_collection[idx]!==-1)
            p_idx = rrt.p.parents_collection[idx]
            if first
                tree = [rrt.p.states_collection[1:3, idx]  rrt.p.states_collection[1:3, p_idx]]
                first = false
            else
                tree = [tree [rrt.p.states_collection[1:3, idx]  rrt.p.states_collection[1:3, p_idx]]]
            end
        end
    end
    return tree
end


function retrieveTree(rrt::RRTSearcher):: Matrix{Float64}
    first = true
    tree = nothing
    for idx in 1:rrt.p.sample_idx
        if (rrt.p.parents_collection[idx]!=-1)
            p_idx = rrt.p.parents_collection[idx]
            if first
                tree = [rrt.p.states_collection[:, idx]  rrt.p.states_collection[:, p_idx]]
                first = false
            else
                tree = [tree [rrt.p.states_collection[:, idx]  rrt.p.states_collection[:, p_idx]]]
            end
        end
    end
    return tree
end
