# franka_feedback_linearization_controller
This is a ROS package of PD feedback linearization controller to control Franka Emika Panda arm in Gazebo simulation environment.
# Feedback Linearization Controller with Orientation 
Assume we have a planned trajectory $x_d$, planned velocity $\dot x_d$, and planned acceleration $\ddot x_d$. All of which are 6x1 matrices with upper three rows corresponding to position, and bottom three rows corresponding to orientation of the end effector. 

In order to find the desired joint torque $\tau$, three most important quantities that we need to solve for are desired joint acceleration $\ddot q_d$, joint position and velocity errors $\Delta q$ and $\Delta\dot q$  respectively. 

Whereas we can solve for them by the following steps: 
$$\begin{align}
    \Delta q = J^+(q)(x_d-x)
\end{align}$$
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