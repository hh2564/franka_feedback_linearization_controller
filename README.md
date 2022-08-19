# franka_feedback_linearization_controller
This is a ROS package of PD feedback linearization controller to control Franka Emika Panda arm in Gazebo simulation environment.

# Getting Started 
This package was tested in simulation environment Gazebo with: 
* Ubuntu 20.04.4 LTS
* [Ros Noetic](http://wiki.ros.org/noetic)
* [Libfranka](https://frankaemika.github.io/docs/libfranka.html) 0.9.0
* [franka_ros](https://frankaemika.github.io/docs/franka_ros.html) 0.9.0 
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 2.6.8r1


You can go to the their official website to find more instruction on installing these packages. 

# Feedback Linearization Controller with Orientation 
Assume we have a planned trajectory $x_d$, planned velocity $\dot x_d$, and planned acceleration $\ddot x_d$. All of which are 6x1 matrices with upper three rows corresponding to position, and bottom three rows corresponding to orientation of the end effector. 

In order to find the desired joint torque $\tau$, three most important quantities that we need to solve for are desired joint acceleration $\ddot q_d$, joint position and velocity errors $\Delta q$ and $\Delta\dot q$  respectively. 

Whereas we can solve for them by the following steps: 

$$
\begin{align}
    \Delta q = J^+(q)(x_d-x)
\end{align}
$$

$$\begin{align}
    \Delta\dot q = J^+(q)(\dot x_d-\dot x)
\end{align}$$

$$\begin{align}
    \ddot q_d = J^+(q)(\ddot x_d - \dot J(q)\dot q) 
\end{align}$$

whereas for franka arm, the Jacobian matrix $J$ and its time derivative $\dot J$ are 6x7 matrices, and Jacobian psedoinverse $J^+$ is a 7x6 matrix. 

To calculate joint torque, we have: 

$$\begin{align}
    \tau = M(\ddot q_d + K_p\Delta q+K_d\Delta \dot q) + \tau_G
\end{align}$$


# Calculating Trajectory 
In order to obtain the desired Trajectory, we need to find the $b_{i}$ for each of the 6 axis, which are x,y,z,r,p,ya respectively. For each of the axis, we can plug in the previous polynomial to solve for the constants accordingly. 

Take x for example, we have the following six equations: 

$$\begin{align*}
    S_x(0) = S_{xinit}
\end{align*}$$

$$\begin{align*}
    S_x(T) = S_{xend}
\end{align*}$$

$$\begin{align*}
    \dot S_x(0) = \dot S_x(T) = \ddot S_x(0) = \ddot S_x(T) = 0 
\end{align*}$$

The subscript x stands for the direction of the polynomial. And we can also rewrite and solve for the constants in the following linear form: 

$$\begin{align}
    \begin{bmatrix}
t(0)^T\\
\dot t(0)^T\\
\ddot t(0)^T\\
t(T)^T\\
\dot t(T)^T\\
\ddot t(T)^T
\end{bmatrix}\begin{bmatrix}
b_{x0}\\
b_{x1}\\
b_{x2}\\
b_{x3}\\
b_{x4}\\
b_{x5}
\end{bmatrix} = \begin{bmatrix}
x_{init}\\
0\\
0\\
x_{end}\\
0\\
0
\end{bmatrix} 
\end{align}$$

whereas t(t) is a differentiable function in vector form 

$$\begin{align*}
    t(t) = \begin{bmatrix}
1\\
t\\
t^2\\
t^3\\
t^4\\
t^5
\end{bmatrix}
\end{align*}$$

we can further solve for the constant variables $b_i$ by left multiplying the block matrix inverse: 

$$\begin{align}
\begin{bmatrix}
t(0)^T\\
\dot t(0)^T\\
\ddot t(0)^T\\
t(T)^T\\
\dot t(T)^T\\
\ddot t(T)^T
\end{bmatrix}^{-1}
\begin{bmatrix}
t(0)^T\\
\dot t(0)^T\\
\ddot t(0)^T\\
t(T)^T\\
\dot t(T)^T\\
\ddot t(T)^T
\end{bmatrix}\begin{bmatrix}
b_{x0}\\
b_{x1}\\
b_{x2}\\
b_{x3}\\
b_{x4}\\
b_{x5}
\end{bmatrix} = \begin{bmatrix}
t(0)^T\\
\dot t(0)^T\\
\ddot t(0)^T\\
t(T)^T\\
\dot t(T)^T\\
\ddot t(T)^T
\end{bmatrix}^{-1}\begin{bmatrix}
x_{init}\\
0\\
0\\
x_{end}\\
0\\
0
\end{bmatrix} 
\end{align}$$

and thus we can get $b_i$ by plugging in T, $x_{init}$, and $x_{end}$ respectively. 

$$\begin{align}
\begin{bmatrix}
b_{x0}\\
b_{x1}\\
b_{x2}\\
b_{x3}\\
b_{x4}\\
b_{x5}
\end{bmatrix} = \begin{bmatrix}
t(0)^T\\
\dot t(0)^T\\
\ddot t(0)^T\\
t(T)^T\\
\dot t(T)^T\\
\ddot t(T)^T
\end{bmatrix}^{-1}\begin{bmatrix}
x_{init}\\
0\\
0\\
x_{end}\\
0\\
0
\end{bmatrix} 
\end{align}$$

Repeat the previous steps and obtain the six constants for all six directions. And we can obtain the desired position, velocity, and acceleration from the polynomials and their time derivatives. We can also concentrate the result to obtain the position, velocity, and acceleration vector $X(t),\dot X(t),\ddot X(t)$: 

$$\begin{align*}
    X(t) = \begin{bmatrix}
S_x(t)\\
S_y(t)\\
S_z(t)\\
S_r(t)\\
S_p(t)\\
S_{ya}(t)
\end{bmatrix}
\end{align*}$$


# Changing Desired End effector Pose 
If we take a look at the parameter part of the launch file: 
```
  <param name="x_target" type="double" value="0.4"/>
  <param name="y_target" type="double" value="0.4"/>
  <param name="z_target" type="double" value="0.4"/>
  <param name="r_target" type="double" value="2.8"/>
  <param name="p_target" type="double" value="0.3"/>
  <param name="ya_target" type="double" value="0.3"/>
  <param name="terminal_time" type="double" value="10.0"/>
```
We are specifing the desired position and orientation as well as controlled period here.

If you want to replace the trajectory, you can change the source file in the update part in franka_feedback_linearization_controller.cpp 

# Run Controller 
To run the feedback linearization controller in Gazebo simulation, run the following code in command line: 
```
roslaunch franka_feedback_linearization_controller franka.launch controller:=panda/franka_feedback_linearization_controller
```
