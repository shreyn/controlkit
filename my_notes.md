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
2. Closed loop (feedback): At every step, look at current state, and choose u base don it. 

$u_t = \pi(x_t)$
