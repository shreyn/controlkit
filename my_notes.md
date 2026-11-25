# Controls

# Foundations
## State
Minimum set of numbers needed to predict a robot's future perfectly.

Ex: ball rolling
- knowing position isnt enough, also need veloctiy:
- $\Rightarrow x = $[position velocity]. 

## Control u
What are we allowed to change? 
- Inputs for the robot (motor torque, thrust)

Usually limited:
- motor has a maximum torque: $-3 <= u <= 3$.

## Dynamics
Given the state x, and control u, how is the state changing? 

$\dot{x} = f(x, u)$

- $\dot{x}$: rate of change of the state

## Continuous vs. Discrete time
Chop time into small slices $dt$. 

Euler's method: 
$x_{t+1} = x_t + dt*f(x_t, u_t)$

## Feedback
How do we choose u?
1. Open loop: decide whole sequence of pushes in advance, never look at robot again. 
- problem: messy in the real worl (no way to correct)
2. Closed loop (feedback): At every step, look at current state, and choose u based on it. 

$u_t = \pi(x_t)$
- $\pi$: controller
 - function: state in, control out

Closed loop:
1. Read state $x_t$.
2. Controller computes $u_t = \pi (x_t)$
3. Robot moves $x_{t+1} = x_t + dt *f(x_t, u_t)$
4. Repeat

## Stability
What does it mean for a controller to "work"?

Equilibrium: state where, with the right control, the robot stays forever
- $\dot{x}=0$

Suppose you nudge the robot away from equilibrium. What happens?
1. Stable: it comes back (pendulum hanging down)
2. Unstable: goes away (marble on top of bowl)


## Model
Def: Controller's copy of the robot's physics
1. How many numbers are in state?
2. How many numbers are in the control?
3. Given the state x and control u, how fast is the state changing? (dynamics)

### Integrators
Def: takes current state, control, step size, and returns state one step later

#### Euler
$$x_{k+1} = x_k + \Delta t \cdot f(x_k, u_k)$$
- $f$: derivative of state.

#### RK4
Still takes one step of full size $\Delta t$.
But now splits the derivative evaluation:
- Samples slope at 4 points, averages them. 
$$x_{k+1} = x_k + h \cdot \frac{k_1 + 2k_2 + 2k_3 + k_4}{6}$$

Error:
- Euler: halve step, error drops by 2
- RK4: halve step, error drops by 16


### Linearization
Linear Systems:
$$\delta x_{k+1} = A \delta x_k + B \delta u_k$$
- But real robots are nonlinear.
- Fix: linearization: at a point, a smooth function looks linear

next state $\approx F(x_0, u_0) + A \delta x + B \delta u$.
- This is value at the point, plus slope times delta
- A: how the next state changes when current state is nudged
 - $A_{11}$: if position goes up by 1, next position goes up by $A_{11}$.
- B: how the next state change when control is nudged


Two Functions:
1. $f(x,u)$
 - Input: state, control
 - Output: Derivative of state
2. $F(x,u)$ (Integrator)
 - Input: state, control
 - Output: next state $x_{k+1}$

We want to linearize F (rk4_step).
- Robot physics is nonlinear:
 - No closed form formula for solving
 



## LQR (Linear Quadratic Regulator)

1. Robot has a state (position, velocity). There is a current state, and a goal state.
2. Before running, we need to find K once.
 - This is done by first describing a Model
 - Then straighten it near the goal to get A, B.
    - **How to find A, B:**
        - Nudge F one input at a time, see how output moves.
        - Ex: Start (p,v) = (3,0). $F(p, v, u) = \big(\,p + 0.1\,v + 0.005\,u,\;\; v + 0.1\,u\,\big)$
            - $A = \frac{\partial F}{\partial x}$: *derivative w.r.t state*
            - $B = \frac{\partial F}{\partial u}$: *derivative w.r.t control*

 - **How error evolves:** $$\delta x_{next} = A \delta x + B \delta u$$
    - $\delta x = x - x_{goal}$: state error
    - same with $\delta u$: control error
 - **define what we care about (Q,R)**
    - We have a rule that says how error evolves. we need a way to say what a good controller is. That is done by **scoring with a cost**: 
    $$penalty = \delta x^T Q \delta x + \delta u^T R \delta u$$
    - based on current error (Q) and the current push (R). 
    - then add up penalty over every tick:
    $$J = \sum_{k=0}^{\infty} \left( \delta x_k^\top Q\, \delta x_k + \delta u_k^\top R\, \delta u_k \right)$$
    - J is a number that scores an entire run (smaller is better)
    
 - **Now run LQR to get K**
    - Input: A, B, Q, R
    - Returns: matrix K, where: 
        - among all possible ways of chosing $\delta u_k$ at each tick, the rule $\delta u_k = - K \delta x_k$ gives the smallest total cost J.
    - "Best" means best over whole future. Do we need to compare infinitely many futures?
    - No: score each place you could be. 
        - GPS: for each intersection, we know minutes remainign to destination. 
        - Therefore, decision is: minutes on this road + minutes remainnig from intersection, choose the smallest
    - **cost-to-go**: $V(\delta x):$ if i behave perfectly from this error $\delta x$ onward, my total future cost J will be this much.
        - in LQR: $V(\delta x) = \delta x^T P \delta x$
        - P: matrix that encodes which directions of error are expensive
    - Pipeline: A, B, Q, R -> P -> K
    - Errors and Costs:
        - Error: $\delta x = x - x_{goal}$. $\delta u$ similar.
        - Penalty now: $\delta x^T Q \delta x + \delta u^T R \delta u$. 
        - Next error: $A \delta x + B \delta u$.
        - V: total penalty, summed over future ticks, if i behave perfectly starting from given error
        - Total cost of a push $C(\delta u): $$C(\delta u) = penalty now + V(next error)$$.


3. Then we run the robot
 - each tick, read current state, compute $u_t = -K(x_t - x_{goal})$
 - robot moves one step: $x_{t+1} = F(x_t, u_t)$
    - recall: $F(x,u) = x + \Delta t f(x,u)$, or RK4.
    - f(x,u): derivative of state w.r.t time = (v, u)
         - ***u is in some sense the acceleration (deriv of velocity)***


### In code:
Let $e_j$ be vector with 1 in slot j, else 0. 
Then col j of A is:
$$A_{\cdot j} \approx \frac{F(x + \varepsilon e_j,\, u) - F(x - \varepsilon e_j,\, u)}{2\varepsilon}, \qquad \varepsilon = 10^{-6}$$

Col j of B is same, but with u nudged instead of x.
F ix rk4_step



## MPC (Model Predictive Control)
Why MPC:
- LQR can't see limits! real motors have limits
- Naive: compute $u = -K \delta x$, then clip to allowed range.
    - But this fails: controller needs to look ahead and realize that its "weaker brakes" can't stop a high speed

MPC Idea:
1. Look at current state.
2. Plan: using model, find best sequence of next N pushes, with limits built in
3. Act: apply only the first push of that plan
4. Repeate next tick from new state, planning again (rolling window)

Why only first push?
- reality doesn't match model exactly, so we should replan from every tick

### LQR vs. MPC
The tradeoff compared with LQR

| | LQR | MPC |
|---|---|---|
| Work before the run | find $K$ once | almost none |
| Work each tick | one matrix multiply | solve a small optimization problem |
| Limits on pushes or states | no | yes |
| Cost | tiny | much more computation per tick |

### Predicting the future
We need to predict where the ball is after N ticks.
Use straight rule from LQR: $$\delta x_{k+1} = A \delta x_k + b\delta u_k$$
- $\delta x_0$ is the current error, which we measure. It is known.
- $\delta u_0, \delta u_1, \dots, \delta u_{N-1}$ are the planned pushes, which we choose. They are the unknowns.
- $\delta x_1, \delta x_2, \dots, \delta x_N$ are the predicted errors, which follow from the other two.

Stack everything into two vectors:
$$U = \begin{pmatrix}\delta u_0\\ \delta u_1\\ \delta u_2\end{pmatrix}, \qquad X = \begin{pmatrix}\delta x_1\\ \delta x_2\\ \delta x_3\end{pmatrix}$$

The three equations above then collapse into one:
$$X = \underbrace{\begin{pmatrix}A\\ A^2\\ A^3\end{pmatrix}}_{\text{call it } \mathcal{A}} \delta x_0 \;+\; \underbrace{\begin{pmatrix}B & 0 & 0\\ AB & B & 0\\ A^2B & AB & B\end{pmatrix}}_{\text{call it } \mathcal{B}} U$$

Finally:
$$X = \mathcal{A}\,\delta x_0 + \mathcal{B}\,U$$

### Scoring a whole plan
For one planned sequence of pushes, we add up the same penalty as in LQR, but for all ticks over the horizon: $$\text{penalty at tick } k = \delta x_k^\top Q\, \delta x_k + \delta u_k^\top R\, \delta u_k$$

**Problem**: plan only looks N ticks ahead. 
- If we scored only those N ticks, plan could look great, but then leave ball in terrible spot

Fix: **terminal cost:**
- Add one last term at the end of the plan: $\delta x_N$
- Use cost-to-go matrix P from LQR (how much will it cost, from this error onward, if I behave perfectly)
$$J_{\text{plan}} = \sum_{k=0}^{N-1}\left(\delta x_k^\top Q\, \delta x_k + \delta u_k^\top R\, \delta u_k\right) + \underbrace{\delta x_N^\top P\, \delta x_N}_{\text{terminal cost}}$$
- Basically a way to add an infinite-horizon cost

**Theorem: MPC with terminal cost P and no limits is equivalent to LQR.**

### Limits
Two kinds:
1. Push limits: motor can only push so hard: $-1 \leq u \leq 1$.
2. State limits: state must stay in some range (speed limit): $|v| \leq 1$, or wall the ball cannot hit

LQR can't enforce these, but MPC can, since it plans over a window.

**Both kinds of limits are limits on U.**
- limit on predicted state becomes a limit on a straight-line combination of pushes

In general, all limits can be written in a stacked form:
- M matrix, l, u vectors of lower and upper bounds:
$$\ell \;\le\; M\,U \;\le\; \mu$$

### Quadratic Program
Putting Scoring and Limits together, every tick we need to solve:
$$\text{minimize } J_{\text{plan}}(U) \quad \text{subject to} \quad \ell \le M\,U \le \mu$$
- quadratic cost J, with linear constraints

Connection to Linear programming:
- system of linear inequalities
- convexity (local optimum = global)
- Difference: objective is quadratic
    - this means optimum isn't at a vertex of a polyhedron
    - Simplex doesn't apply
    