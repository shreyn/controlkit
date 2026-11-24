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
$$\delta x_{k+1} = A \delta x_k + b \delta u_k$$
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
### HIGH LEVEL:
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

3. Then we run the robot
 - each tick, read current state, compute $u_t = -K(x_t - x_{goal})$
 - robot moves one step: $x_{t+1} = F(x_t, u_t)$
    - recall: $F(x,u) = x + \Delta t f(x,u)$, or RK4.
    - f(x,u): derivative of state w.r.t time = (v, u)
         - ***u is in some sense the acceleration (deriv of velocity)***


