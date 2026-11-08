cmd shift v

# 1. Pose
- Def: where a robot is, and which way it is facing.
- 3 numbers:
  - x: how far right
  - y: how far up
  - theta: direction (0 rad is +x)

# 2. Compose
- Def: a function that answers: "robot is at pose A, moves by B (measured by its own POV). Where is it now?"
- Why: wheel sensors report motion from the robot's pov.
## Two Coordinate Systems
1. Map (world) frame: fixed
- +x (east), +y (north)
- a (robot pose)
2. Robot's own frame: turns with robot
- +x (forward): whichever way the robot's nose points
- +y (left)
- b (motion)

# 3. Angle Wrapping
- Def: a function that forces any angle into range [-pi, pi]
- Why: Angles repeat every 2pi. 

# 4. Simulator
Two Paths:
1. True path: where robot actually went.
2. Dead-reckoning path: where the robot thinks it went (adding up noisy wheel readings). Drifts from truth over time. 
- Gaussian noise, seeded.

## Full Circle (apps/simulate.cpp)
- move 1 meter forward, turn 2pi/100 rad.
- after 100 steps, robot has made a full circle
- track true path and dead-reckoning path (noisy)

# Landmarks
Add fixed landmarks (trees, etc) to the world
Sensor measures: 
- 1. Range: how far away a landmark is (meters)
- 2. Bearing: angle to landmark relative to robots nose (rad). positive is left, negative is right.
## Formulas
Let robot be at (x, y, theta), landmark at (lx, ly):
Then:
- dx = lx - x.    dy = ly - y.
- range = sqrt(dx^2 + dy^2)
- bearing = wrap_angle(atan2(dy, dx) - theta)
  - atan2(dy,dx): compass direction to landmark
  - subtracting theta converts to "relative to nose"

## Additions to simulator:
Put N fixed landmarks in the world. 
At every step, record which ones the robot can see, and what the noisy sensor reports.
- Max range: real sensors can only see so far.
- Robot can see 360 around. 


# EKF (Extended Kalman Filter)
Robot has to answer "where am I?" using noisy wheels and sensor. 
EKF combines the two into one bet guess.
1. A best guess of the pose: (x, y, theta)
2. How unsure it is about that guess
## Two steps
1. Predict (wheels)
- Wheels say "i moved 1 meter forward"
- Uncertainty grows (walking with eyes closed makes you less sure each step)
2. Update (landmark sensor)
- Sensor says "I see landmark 3, 8 meters away, to my left"
- Compare that with what EKF expected to see from its current guess
- If they disagree, nudge the guesss forward toward what the sensor says
- Uncertainty shrinks (new info).

## Kalman gain
How much to trust each sensor.

## Eigen
Two types:
1. Eigen::Vector3  (3 doubles)
2. Eigen::Matrix3d  (3x3 of doubles)

## Covariance Matrix
- 3x3 matrix, diag has variance of each number (var = std squared)

## Step 1
Given current belief (best guess + uncertainty), and one wheel reading, return the new belief.
- We already know how to move the guess (compose)
- Now need to grow the uncertainty
  - Old uncertainty gets carried along
  - wheels add fresh noise
  P' = FPF^T + GQG^T
  - P: old cov matrix
  - F: "how does a the new pose change if the old pose is nudged (jacobian)"
    - derivate of compose w.r.t old pose
  - G: how does the new pose change if the wheel reading is nudged (jacobian)
    - derivate of compose w.r.t wheel reading
  - Q: wheel noise

## Step 2
Given a landmark reading, uses it to correct EKF's belief.
1. Expected: from my current guess, what would the sensor read for this landmark?
2. Actual: what did the sensor actually read?
3. Innovation: difference between them
  - Given actual z, and expected z_hat, v = z-z_hat
4. Correct: nudge the guess by (trust) * (innovation)
  - trust: Kalman gain
Ex:
- guess says landmark is 10 m away, but sensor says 9m. The innovation is -1m, so guess moves towards the landmark.

## Simulation
Each loop:
1. Move true robot
2. Move noisy wheels robot
3. EKF predict, using same noisy wheel readings
4. EKF update (once for each landmark the sensor saw)
5. Write a row to path.csv

## NEES (Normalized Estimation Error Squared)
Is the claim (I am unsure by this much) honest?
Compare the real error to the claimed uncertainty.
Let real error be e. 
NESS = e^2 / sigma^2.
- An honest filter has NEES near 1 on average
Matrix:
- NEES = e^T P^{-1}e
  - again, P^{-1} is "divide by variance"
- We estimate 3 things, each contributes 1, so an honest filter averages around 3.
  - more than 3: overconfident
  - less than 3: underconfident

# SLAM (Simultaneous Localization and Mapping)
Localization: "I have a map. Where am I on it?"
- landmark positions, identity of each landmark

## Chicken-and-egg problem
"Where am I?" and "what does the map look like?" depend on each other:
- To place landmark on map, need to know where I was when I saw it
- To know where I am, I need landmarks on the map.
SLAM solves both at the same time!

## Two kinds
1. Filter SLAM: EKF idea, with landmarks added to state. Fast, but can't fix old mistakes.
2. Graph: keeps whole history and resolves everything at once.

## Main Idea
- Robot's path is a chain of poses. Each wheel reading says "pose 5 is about here relative to pose 4". 
- When the robot returns to a place it has seen, we get a new fact: "pose 10 is at the same place as pose 3". These two disagree slightly, and the optimizer adjusts the whole path until everything agrees as much as possible.
- Problem with EKF: if pose 10 was off, that error is baked into everything after, and it has no way to say "wait, actually redo pose 10"
## Poses as dots, measurements as rubber bands
Draw robot's path as dots on a map.
Between each pair of neighboring dots, the wheels gave a reading. Picture a rubber band connecting dot 3 and 4.
- the band has a rest length (what the wheels measured)
- if the dots sit exactly where the wheel reading says, the band is relaxed
- if the dots are pushed closer or farther, the band is stretched and pulls back.
### Graph
- dot: node
- Rubber band: edge
- Stretch of a band: error


### Loop Closure
Suppose a robot's path is a square, returns to start. Dot 4 should be back at start, but is 1 meter drifted from dot 0.
Now the robot recognizes the place (sees the same landmark it saw at start)
- Concludes: "dot 4 is at the same place as dot 0"

This is a new rubber band (loop closure edge)
- connects dot 4 back to dot 0, with rest length 0.

NOW: the picture has a conflict:
- wheel bands say dot 4 is 1m away from dot 0. 
- loop band says dot 4 is equal to dot 0.

The loop band pulls dot 4 toward dot 0. But dot 4 is tied to dot 3 by a wheel band, so dot 3 also gets pulled, then dot 2, etc. 
- The whole chain bends a little, and the 1m error gets spread evenly throughout all bands

The dots settle on where the total stretch is as small as possible: SLAM answer.

### Pose-graph SLAM: move all dots until the total stretch of all bands is as small as possible.

## Math
Basically just least squares.

#### There are two residuals, one for wheel band, one for loop band.
Residual: what dots currently say - what a measurement says.
- Let measurement be z, dots currently at x_0, x_.1 
- Then r = (x_1 - x_0) - z
  - r = 0: band is perfectly relaxed


Loop Closure:
- If robot decided "pose 4 is same place as pos 0", then $z = 0 => r = (x_4-x_0)$ - 0.

Cost of one band: $r^2 / sigma^2$
- a precise band (small var) should resist stretching

Least Squares:
Total cost = $\sum_{bands} r^2/sigma^2$

Write residuals, find poses that minimize their weighted sum of squares.

### Gauss-Newton
1. Start with initial guess for all dots
2. Linearize: use Jacobian (how each residual reponds to nudging each dot)
3. Solve using Jacobian: $J^T J \Delta = -J^T r$
4. Apply $\Delta$, repeat.
5. Stop when threshold reached. 

### Sparsity
The matrix $J^TJ$ is mostly zeros, can exploit.

- With N poses, $J^TJ$ is a 3Nx3N table.
- a 30000x30000 matrix of doubles is about 7GB. A dense solve costs roughly (3N)^3.

Why are most entries zero?
- Each band only connects two poses. So each residual depends on just two dots.
- Entry (i,j) means "dot i and j are tied together by a band".
- Therefore, only diagonal (x_1 and x_1) and two neighbors are nonzero: tridiagonal
- For N poses, that is about 3N nonzeros, instead of N^2.
- Now adding a loop closer (x_1 and x_4). This just adds two new nonzero entries. 

Dense H is 3684 x 3684 (13.6 million entries). But each edge fills only 4 blocks of 9 numbers (so just 53,000 entries are nonzero). 

Now can use spare matrixes, spares solver (Cholesky factorization).

1. Store nonzero entries as a list of triple: (row, col, value). Duplicates add.
2. Build sparse matrix from the list once.
3. Use a sparse solver.


### Relative Pose
Description from robot's pov.

- Dots in pose graph are stored in map coordinates.
- Wheel readings are relative positions 

A function that goes from relative -> map: compose(A, u): start at map pose A, move by u (in A's pov), tell me new map pose.

A rubber band stores relative positions. But the optimizer only has the dots (map positions). 

Given map poses A and B, what is B from A's pov. 
- This is inverse of compose

Relative(A,B) = first undo A, then apply B.
- Undo turn first, then undo move

=> Relative(A,B) = compose(inverse(A), B)

relative_pose is B-A. 
measurement is B from A's pov (wheel reading).
=> residual is relative_pose - measurement.

### J
- One band connects dot A to B. 
- The residual has 3 numbers, each dot has 3 numbers.
- Therefore, 2 tables:
1. $J_A$: how the 3 stretches change when A's 3 numbers are nudged
2. $J_B$: same for B.

Each entry: slope of that row's residual wr.t. that column's uknown.

One band has:
- residual r (3 numbers)
- $J_A, J_B$ (3x3)
- trust weight $\Omega$ (diagonal 3x3, from std)

### H
We want the $\Delta$ that makes the total cost $|r + J\Delta|^2$ as small as possible.

Where slope is zero: $(J^TJ)\Delta = -(J^Tr)$.
- $H = J^TJ$
- $b = J^Tr$

$\Rightarrow H\Delta = -b$.
- H's entry at row i, col j = sum_bands (band's slope wr.t. dot i) * (band's slope w.r.t dot j).
- b's entry at position i = sum_bands (band's slope wr.t dot i) * (band's slope wr.t. residual).

#### $H_{ij} measures how strongly dots i and j are tied together. b_i measures how hard the bands are currently pulling dot i.

$H_{12}$ is a 3x3 block, with rows for dot 1's (x,y,theta), and cols for dot 2's (x,y,theta).


#### Trust weights
A precise band should count more (scale by 1/sigma^2)

$H = J^T\Omega J$. 

$b = J^T \Omega r$.

#### Optimize
Builds H and b, solves for delta, move dots, repeat.

Fix dot 0 (make it extremely expensive to move).


### Full pipeline
1. Stack all poses into a list of uknowns.
2. For each band, compute its residual and its slope tables ($J_A, J_B$).
3. Start H, b as all zeros.
4. For each band, add its contribution to the blocks of the two dots it touches.
5. Solve $H \Delta = -b$ for $\Delta$.
6. Add the nudges to the poses, repeat.

# Real data (Intel g2o)

Dot (pose guess):
- VERTEX_SE2 1 1.02 0.03 0.01
  - 1: dot id
  - x, y, theta

Rubber band:
- EDGE_SE2 0 1 1.0 0.0 0.05 500 0 0 500 0 800
  - 0 1: from dot 0, to dot 1
  - 1.0 0.0 0.05: measurement (x, y, theta)
  - 500 0 0 500 0 800: trust table (upper triangle, so only needs 6 of 9)