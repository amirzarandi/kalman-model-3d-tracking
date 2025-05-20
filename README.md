# Kalman Filtering: 3D Location Tracking

## 1. Problem Statement

Our goal is to track the location (and velocity) of a moving object, e.g. a ball, in a 3-dimensional space. We will allow gravity to act on the ball and the initial position and velocities are assumed to be known. We will be using noisy location estimates using a (simulated) sensor. The objective is to estimate the true location (and velocity) of the ball in 3D space.

## 2. State Space

We keep track of the following:

1. $x_{1}$ : location in the $x_{1}$ direction.
2. $x_{2}$ : location in the $x_{2}$ direction.
3. $x_{3}$ : location in the $x_{3}$ direction.
4. $v_{1}$ : velocity in the $v_{1}$ direction.
5. $v_{2}$ : velocity in the $v_{2}$ direction.
6. $v_{3}$ : velocity in the $v_{3}$ direction.
7. $a_{1}$ : velocity in the $a_{1}$ direction.
8. $a_{2}$ : velocity in the $a_{2}$ direction.
9. $a_{3}$ : velocity in the $a_{3}$ direction. Therefore, let the state vector be represented as:

$$
x=\left(\begin{array}{l}
x_{1} \\
x_{2} \\
x_{3} \\
v_{1} \\
v_{2} \\
v_{3} \\
a_{1} \\
a_{2} \\
a_{3}
\end{array}\right)
$$

## 3. Kalman Equations and Dynamic Model

### 3.1. Kalman Equations

Reproduced from Wikipedia (below) we have the systems of equations (in matrix format) that need to be solved as part of the Kalman Filtering algorithm. Our challenge is to adapt the problem setting to result in the model that satisfies the form of these equations which will allow us to use Kalman Filtering to track the ball's position and velocity.

The Kalman filter model assumes the true state at time $\mathrm{k}$ is evolved from the state at $(\mathrm{k}-1)$ according to:

```math
\mathbf{x}_{k}=\mathbf{A}_{k} \mathbf{x}_{k-1}+\mathbf{B}_{k} \mathbf{u}_{k}+\mathbf{w}_{k}
```

where,

- $A_{k}$ is the state transition model which is applied to the previous state $X_{k-1}$
- $B_{k}$ is the control-input model which is applied to the control vector $u_{k}$ (which is not applicable in our setting);
- $w_{k}$ is the process noise which is assumed to be drawn from a zero mean multivariate normal distribution with covariance $Q_{k}$ :

```math
\mathbf{w}_{k} \sim \mathcal{N}\left(0, \mathbf{Q}_{k}\right) \mathbf{w}_{k} \sim \mathcal{N}\left(0, \mathbf{Q}_{k}\right)
```

At time $k$ an observation (or measurement) $z_{k}$ of the true state $x_{k}$ is made according to:

```math
\mathbf{z}_{k}=\mathbf{H}_{k} \mathbf{x}_{k}+\mathbf{v}_{k} \mathbf{z}_{k}=\mathbf{H}_{k} \mathbf{x}_{k}+\mathbf{v}_{k}
```

where $H_{k}$ is the observation model which maps the true state space into the observed space and $v_{k}$ is the observation noise which is assumed to be zero mean Gaussian white noise with covariance $R_{k}$.

```math
\mathbf{v}_{k} \sim \mathcal{N}(0, \mathbf{R}_{k})
```

The initial state, and the noise vectors at each step $\{x_{0}, w_{1}, \ldots, w_{k}, v_{1} \ldots v_{k}\}$ are all assumed to be mutually independent.

## 4.3.2 \quad$ Filtering

The state of the Kalman filter is represented by two variables:

$\hat{\mathbf{x}}_{k \mid k}$, the a posteriori state estimate at time $\mathrm{k}$ given observations up to and including at time $\mathrm{k}$;

$\mathbf{P}_{k \mid k}$, the a posteriori error covariance matrix (a measure of the estimated accuracy of the state estimate).

The Kalman filter can be written as a single equation, however it is most often conceptualized as two distinct phases: "Predict" and "Update". The predict phase uses the state estimate from the previous timestep to produce an estimate of the state at the current timestep. This predicted state estimate is also known as the a priori state estimate because, although it is an estimate of the state at the current timestep, it does not include observation information from the current timestep. In the update phase, the current a priori prediction is combined with current observation information to refine the state estimate. This improved estimate is termed the a posteriori state estimate.

Typically, the two phases alternate, with the prediction advancing the state until the next scheduled observation, and the update incorporating the observation.

#### 4.0.1. Predict

1. Predicted (a priori) state estimate:
   ```math
   \hat{\mathbf{x}}_{k \mid k-1}=\mathbf{A}_{k} \hat{\mathbf{x}}_{k-1 \mid k-1}+\mathbf{B}_{k} \mathbf{u}_{k}
   ```
2. Predicted (a priori) estimate covariance:
   ```math
   \mathbf{P}_{k \mid k-1}=\mathbf{A}_{k} \mathbf{P}_{k-1 \mid k-1} \mathbf{A}_{k}^{\mathrm{T}}+\mathbf{Q}_{k}
   ```

#### 4.0.2. Update

1. Innovation or measurement residual:
   ```math
   \tilde{\mathbf{y}}_{k}=\mathbf{z}_{k}-\mathbf{H}_{k} \hat{\mathbf{x}}_{k \mid k-1}
   ```
2. Innovation (or residual) covariance:
   ```math
   \mathbf{S}_{k}=\mathbf{H}_{k} \mathbf{P}_{k \mid k-1} \mathbf{H}_{k}^{T}+\mathbf{R}_{k}
   ```math
3. Optimal Kalman gain:
   ```math
   \mathbf{K}_{k}=\mathbf{P}_{k \mid k-1} \mathbf{H}_{k}^{T} \mathbf{S}_{k}^{-1}
   ```
4. Updated (a posteriori) state estimate:
   ```math
   \hat{\mathbf{x}}_{k \mid k}=\hat{\mathbf{x}}_{k \mid k-1}+\mathbf{K}_{k} \tilde{\mathbf{y}}_{k}
   ```
5. Updated (a posteriori) estimate covariance:
   ```math
   \mathbf{P}_{k \mid k}=\left(I-\mathbf{K}_{k} \mathbf{H}_{k}\right) \mathbf{P}_{k \mid k-1}
   ```

### 4.1. D Motion: Model

In our setting, we have the following:

#### 4.1.1. Dynamic Matrix, $A_{k}$

The Dynamic Matrix, $A_{k}$, is calculated from the dynamics of the motion:

$$
\begin{gathered}
x_{1, k+1}=x_{1, k}+v_{1, k} \cdot \Delta t+\frac{1}{2} a_{1, k} \cdot \Delta t^{2} \\
x_{2, k+1}=x_{2, k}+v_{2, k} \cdot \Delta t+\frac{1}{2} a_{2, k} \cdot \Delta t^{2} \\
x_{3, k+1}=x_{3, k}+v_{3, k} \cdot \Delta t+\frac{1}{2} a_{3, k} \cdot \Delta t^{2} \\
v_{1, k+1}=v_{1, k}+a_{1, k} \cdot \Delta t \\
v_{2, k+1}=v_{2, k}+a_{2, k} \cdot \Delta t \\
v_{3, k+1}=v_{3, k}+a_{3, k} \cdot \Delta t \\
a_{1, k+1}=a_{1, k} \\
a_{2, k+1}=a_{2, k} \\
a_{3, k+1}=a_{3, k}
\end{gathered}
$$

Hence,

$$
A_{k}=\left(\begin{array}{ccccccccc}
1.0 & 0.0 & 0.0 & \Delta t & 0.0 & 0.0 & \frac{1}{2} \Delta t^{2} & 0.0 & 0.0 \\
0.0 & 1.0 & 0.0 & 0.0 & \Delta t & 0.0 & 0.0 & \frac{1}{2} \Delta t^{2} & 0.0 \\
0.0 & 0.0 & 1.0 & 0.0 & 0.0 & \Delta t & 0.0 & 0.0 & \frac{1}{2} \Delta t^{2} \\
0.0 & 0.0 & 0.0 & 1.0 & 0.0 & 0.0 & \Delta t & 0.0 & 0.0 \\
0.0 & 0.0 & 0.0 & 0.0 & 1.0 & 0.0 & 0.0 & \Delta t & 0.0 \\
0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 1.0 & 0.0 & 0.0 & \Delta t \\
0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 1.0 & 0.0 & 0.0 \\
0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 1.0 & 0.0 \\
0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 1.0
\end{array}\right)
$$

$\Delta t$ specifies the time between consecutive steps.

#### 4.1.2. Matrix $B_{k}$ and vector $u_{k}$

In this constant veocity model we have $B_{k}=0$ and $u_{k}=0$.

#### 4.1.3. Observation Matrix, $H_{k}$

We always observe the location, $x_{1}, x_{2}$ and $x_{3}$, which results in the following simple model for $H_{k}$ :

$$
H_{k}=\left(\begin{array}{ccccccccc}
1.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 \\
0.0 & 1.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 \\
0.0 & 0.0 & 1.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0 & 0.0
\end{array}\right)
$$


#### 4.1.4. Measurement Noise Covariance, $R_{k}$

We have the following model for measurement noise covariance matrix:

$$
R_{k}=\left(\begin{array}{ccc}
r & 0.0 & 0.0 \\
0.0 & r & 0.0 \\
0.0 & 0.0 & r
\end{array}\right)
$$

The constant $r$ denotes the belief about the variance of the measurement noise.

#### 4.1.5. Process Noise Covariance, $Q_{k}$

Process noise refers to the modeling of forces that could influence our model by creating noise in the form of acceleration disturbance. According to Wikipedia, for this constant velocity model we have:

$$
Q_{k}=G \cdot G^{T} \cdot \sigma_{v}^{2}
$$

where,

$$
G=\left[\begin{array}{lllllllll}
0.5 \Delta t^{2} & 0.5 \Delta t^{2} & 0.5 \Delta t^{2} & \Delta t & \Delta t & \Delta t & 1.0 & 1.0 & 1.0
\end{array}\right]
$$

and we can assume the acceleration process noise $\sigma_{v}^{2}=8.8 \mathrm{~m} / \mathrm{s}^{2}$, according to Schubert, R., Adam, C., Obst, M., Mattern, N., Leonhardt, V., Wanielik, G. (2011). Empirical evaluation of vehicular models for ego motion estimation. 2011 IEEE Intelligent Vehicles Symposium (IV), 534-539.

## 5. Source

Material for this case study was inspired by the following:

https://balzer82.github.io/Kalman/

For more details on Kalman Filtering equations used, please check: https://en.wikipedia.org/wiki/Kalman_filter

