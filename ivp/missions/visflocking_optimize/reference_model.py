"""Ground truth for this mission: a minimal, dependency-free transcription of the paper's own simulator
(ABM/abm/projects/visual_flocking): projection_field + VSWRM_flocking_state_variables
+ vf_agent.update_agent_position, on a torus.

Why it exists: when the MOOS reproduction disagrees with the paper, the first
question is always "what should this parameter pair actually produce?", and the
answer has to come from the code the figure came from, not from the printed
equations (they differ -- e.g. vf_agent always takes the non-verbose branch,
which uses sigmoid rather than cos/sin angular masks). Pure stdlib on purpose,
so it runs anywhere without installing the ABM's pygame/numpy stack.

    python3 reference_model.py <alpha0> <beta0> [steps]

Reference values used to verify this mission (10 agents, full FOV):
    alpha0=0,   beta0=0    -> P 0.35, D 62.5 R_A   (unordered)
    alpha0=0.5, beta0=0.1  -> P 0.83, D 13.6 R_A   (flocking)
    alpha0=0.5, beta0=2.0  -> P 0.28, D 12.8 R_A   (swarming)
"""
import math, random

RES = 320          # VISUAL_FIELD_RESOLUTION
R_A = 5.5          # RADIUS_AGENT (px)
W = H = 900.0      # ENV_WIDTH/HEIGHT
GAM, V0 = 0.1, 1.0
ALP1 = BET1 = 0.09

def sigmoid(x, s): return 2.0/(1.0+math.exp(-s*x)) - 1.0
def cos_sig(p):
    s = 3*math.pi
    return sigmoid(p+math.pi/2, s) if p < 0 else -sigmoid(p-math.pi/2, s)
def sin_sig(p):
    s = 3*math.pi
    if -math.pi/2 < p < math.pi/2: return sigmoid(p, s)
    if p <= -math.pi/2: return -sigmoid(p+math.pi, s)
    return -sigmoid(p-math.pi, s)

PHIS = [-math.pi + i*(2*math.pi)/(RES-1) for i in range(RES)]   # linspace(-pi,pi,RES)

def find_nearest(value):
    # index of nearest entry in PHIS
    i = int(round((value + math.pi)/((2*math.pi)/(RES-1))))
    return max(0, min(RES-1, i))

def projection_field(pos, ori, others):
    """others: list of (x,y). Returns binary field (list of 0/1), reference conventions."""
    v = [0]*RES
    ax, ay = pos
    # agent's own facing unit vector, reference convention (y inverted)
    v1 = (math.cos(ori), -math.sin(ori))
    for (ox, oy) in others:
        dx, dy = ox-ax, oy-ay
        # torus minimum image (reference: boundary_cond == "infinite")
        if abs(dx) > W/2: dx -= math.copysign(W, dx)
        if abs(dy) > H/2: dy -= math.copysign(H, dy)
        d = math.hypot(dx, dy)
        if d == 0: continue
        # signed angle between v1 and v2 (supcalc.angle_between + calculate_closed_angle)
        v2 = (dx, dy)
        dot = (v1[0]*v2[0]+v1[1]*v2[1])/d
        ang = math.acos(max(-1.0, min(1.0, dot)))
        if v1[0]*v2[1]-v1[1]*v2[0] < 0: ang = -ang
        ca = ang % (2*math.pi)
        ca = -ca if 0 <= ca <= math.pi else 2*math.pi-ca
        vis_angle = 2*math.atan(R_A/d)
        proj_size = (vis_angle/(2*math.pi))*RES
        centre = find_nearest(ca)
        start = int(centre - math.floor(proj_size/2))
        end   = int(centre + math.floor(proj_size/2))
        if start < 0:
            for i in range(RES+start, RES): v[i] = 1
            start = 0
        if end >= RES:
            for i in range(0, end-(RES-1)): v[i] = 1
            end = RES
        for i in range(start, end): v[i] = 1
    v.reverse()          # projection_field's np.flip ...
    return v

def dphi_v(V):
    pad = [V[-1]] + V + [V[0]]
    raw = [pad[i+1]-pad[i] for i in range(len(pad)-1)]
    return raw[:-1] if (raw[0] > 0 and raw[-1] > 0) else raw[1:]

def state_vars(vel, V, alp0, bet0):
    dV = dphi_v(V)
    dphi_step = PHIS[-1]-PHIS[-2]
    # trapz over PHIS
    def trapz(ys):
        return sum((ys[i]+ys[i+1])/2.0*dphi_step for i in range(len(ys)-1))
    g_vel = [-x for x in V]
    g_psi = [-x for x in V]
    spike = [d*d for d in dV]
    a_blob = alp0*trapz([cos_sig(PHIS[i])*g_vel[i] for i in range(RES)])
    a_edge = alp0*ALP1*sum(cos_sig(PHIS[i])*spike[i] for i in range(RES))
    b_blob = bet0*trapz([sin_sig(PHIS[i])*g_psi[i] for i in range(RES)])
    b_edge = bet0*BET1*sum(sin_sig(PHIS[i])*spike[i] for i in range(RES))
    return GAM*(V0-vel) + a_blob + a_edge, b_blob + b_edge

def run(alp0, bet0, n=10, steps=20000, seed=1, warmup_frac=0.3):
    rnd = random.Random(seed)
    third = W/3
    pos = [[W/2+rnd.uniform(-third/2, third/2), H/2+rnd.uniform(-third/2, third/2)] for _ in range(n)]
    ori = [rnd.uniform(-math.pi, math.pi) for _ in range(n)]
    vel = [V0]*n
    Psum = Dsum = cnt = 0.0
    for t in range(steps):
        fields = []
        for i in range(n):
            others = [tuple(pos[j]) for j in range(n) if j != i]
            V = projection_field(pos[i], ori[i], others)
            fields.append(V)
        for i in range(n):
            dv, dpsi = state_vars(vel[i], [x for x in reversed(fields[i])], alp0, bet0)  # vf_agent passes np.flip(field)
            ori[i] = (ori[i]+dpsi) % (2*math.pi)
            vel[i] += dv
            pos[i][0] = (pos[i][0] + vel[i]*math.cos(ori[i])) % W
            pos[i][1] = (pos[i][1] - vel[i]*math.sin(ori[i])) % H
        if t >= steps*warmup_frac:
            sx = sum(math.cos(o) for o in ori); sy = sum(math.sin(o) for o in ori)
            Psum += math.hypot(sx, sy)/n
            ds=[];
            for i in range(n):
                for j in range(i+1, n):
                    dx = abs(pos[i][0]-pos[j][0]); dy = abs(pos[i][1]-pos[j][1])
                    dx = min(dx, W-dx); dy = min(dy, H-dy)
                    ds.append(math.hypot(dx, dy))
            Dsum += sum(ds)/len(ds); cnt += 1
    return Psum/cnt, Dsum/cnt, [v for v in vel]

if __name__ == "__main__":
    import sys
    a0 = float(sys.argv[1]); b0 = float(sys.argv[2])
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 20000
    P, D, vel = run(a0, b0, steps=steps)
    print(f"alpha0={a0} beta0={b0} steps={steps}:  P={P:.3f}  D={D:.1f} px ({D/R_A:.1f} R_A)  mean|v|={sum(vel)/len(vel):.2f}")
