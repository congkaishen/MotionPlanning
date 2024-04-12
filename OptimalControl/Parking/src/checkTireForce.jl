la          = 1.56
lb          = 1.64
M           = 2020
Izz         = 4095
g           = 9.81
mu          = 0.8
KFZF        = 1018.28 / 2
KFZR        = 963.34 / 2
KFZX        = 186.22
Tire_par    = [-10.4,1.3,1.0,0.1556,-0.0023,41.3705]

B           = Tire_par[1]/mu; 
C           = Tire_par[2] 
E           = Tire_par[4] 
Sh          = Tire_par[5] 
Sv          = Tire_par[6]

x           = 0
y           = 0
v           = 0
r           = 0
psi         = 0
sa          = 0
sr          = 0

ux = 30

FZF         = 2* KFZF * g + r * v * KFZX
FZR         = 2* KFZR * g - r * v * KFZX
alpha_f     = atan((v + la * r) / (ux + 0.01)) - sa
alpha_r     = atan((v - lb * r) / (ux + 0.01))

X1_f        = B * alpha_f
FY1         = (mu * FZF * Tire_par[3] * sin(C * atan(X1_f - E * (X1_f - atan(X1_f)))))
X1_r        = B * alpha_r
FY2         = (mu * FZR * Tire_par[3] * sin(C * atan(X1_r - E * (X1_r - atan(X1_r)))))