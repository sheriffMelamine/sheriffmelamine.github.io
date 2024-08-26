---
title: "Mathematical Modelling of a UAV Gimbal Rig Testbench"
date: 2024-06-29
draft: false
tags: ["Newton-Euler Formulation","Transformation Matrix"]

---

The gimbal rig setup for a three degrees of freedom (3 DOF) system represents a cornerstone in the field of rotational mechanics and control systems. This configuration, often visualized through the classic example of a gyroscope, allows for precise manipulation and measurement of an object's orientation in three-dimensional space. By employing a series of nested gimbals—each capable of independent rotation around orthogonal axes—this setup provides a robust framework for exploring complex rotational dynamics. In our testbench, a drone is securely mounted within the innermost ring of the gimbal rig, with each joint equipped with sensors to measure angular displacement. This configuration not only facilitates the study of the drone's orientation and stability but also enhances our understanding of control strategies in dynamic environments. The adaptability and precision of the 3 DOF gimbal system make it a powerful tool for experimental research and development in modern engineering and technology. 

##	Coordinate Frame Transformations  

For mathematical modeling of the testbench, rigid body assumptions are considered. The testbench itself is considered fully symmetrical and centered, therefore, all the CGs of each different components of the testbench coincide, thus the CG of the testbench can be assumed to be the origin for convenience. However, clamping the drone to the inner shaft of the testbench to cause its CG to also coincide with the origin would not be plausible, therefore, a positional vector would be assumed for the drone’s CG in this mathematical modeling effort. Due to symmetricity, the components of the testbench would have diagonal MOI, however, the clamped drone would cause the equivalent MOI of inner shaft to deviate from its diagonal form, so as its symmetricity. 

The testbench involves three different coordinate body frames for the rings and drone and one earth frame or the global frame where the testbench base is established. The following figure shows the body frames and the earth frame with their coordinates shifted for convenience of demonstration. The actual origin of all body frames coincides with the shaft frame as shown, which is the CG of the testbench.  


![Figure 1](/posts/testbench/Testbench-1.png#center "The earth frame and the body frames representing outer, inner gimbals and inner shaft respectively")


##	Earth and Body Frames 

To transfer from one frame to another, the following relation as can be seen in figure x, is established to determine the dynamics relationship between the frames. 

 

![Figure 2](/posts/testbench/Testbench-2.png#center "Transfer matrices with their corresponding rotations and their relation with each frames") 

 

The symbols used here are explained in the next section. For deriving complete formulation of the testbench dynamics, knowledge of rotation matrices and their time derivatives are important. The following section briefly discusses the basic ideas of rotation matrices for the testbench modelling. 

##	Rotation Matrices and Euler Angles 

Rotation matrices transform coordinate systems by rotating coordinates around a specific axis. They reorient points or vectors without altering their relative distances, effectively changing the coordinate frame's orientation. Since the gimbal rig setup has multiple different frames with different independent orientations, using rotation matrices to transform from frame to frame is necessary. Rotation matrices for three axes could be defined by the following expressions. 

$$ \textbf T_{BO}= \begin{bmatrix} \cos\psi && \sin\psi && 0 \\\\ -\sin\psi && \cos\psi && 0 \\\\ 0 && 0 && 1 \end{bmatrix} $$

$$ \textbf T_{OI}=\begin{bmatrix}\cos\theta&&0&&\sin\theta\\\\0&&1&&0\\\\-\sin\theta&&0&&\cos\theta\end{bmatrix} $$ 

$$ \textbf T_{IS}=\begin{bmatrix}1&&0&&0\\\\0&&\cos\phi&&\sin\phi\\\\0&&-\sin\phi&&\cos\phi\end{bmatrix} $$ 

Here, $\phi$,$\theta$ and $\psi$ means roll, pitch and yaw motion of the testbench (not to confuse with similar motions of the quadcopter). The roll, pitch and yaw motions in earth frame essentially result in the relative angular displacement of the inner shaft, inner gimbal and outer gimbal, respectively. 

Using the transpose of a rotation matrix for transforming coordinates would result in the rotation done in reverse direction. Therefore, the following relations can also be established. 

$$\textbf T_{OB}=\textbf T_{BO}^T$$ 

$$\textbf T_{IO}=\textbf T_{OI}^T$$

$$\textbf T_{SI}=\textbf T_{IS}^T$$ 

Euler angles are defined by three consequent angular displacements along three different earth frame axis that would result in the body frames orientation. The angles depend on the order of the rotation, for instance, rotation around x, y and z-axis sequentially would result in completely different orientation compared to rotation around x, y and x-axis in the order mentioned. In this study, XYZ convention would be used since it is more convenient, as it would omit the necessity of transforming angular displacement data obtained from testbench and obtained data can be analyzed with ease. The relative angular displacements of the inner shaft, inner gimbal and outer gimbal then would be identical to roll, pitch and yaw, respectively. 

The time derivative of rotation matrices is also essential for determining the dynamics as each frame would keep rotating with variable angular frequency. The time derivative of rotation matrix is multiplication of the rotation matrix and a skew-symmetric matrix, which can be expanded as below for the outer gimbal. 

$$\dot{\textbf T}_{BO}=\textbf T _{BO}.[\omega _{BO}] _\times = \textbf T _{BO} . \begin{Bmatrix} 0 \\\\ 0 \\\\ \dot{\psi} \end{Bmatrix} _\times \\\\= \begin{bmatrix} \cos\psi && \sin\psi && 0\\\\ -\sin\psi && \cos\psi && 0 \\\\0 && 0 && 1 \end{bmatrix} . \begin{bmatrix} 0 && -\dot{\psi} && 0 \\\\ \dot{\psi} && 0 && 0\\\\ 0 && 0 && 0 \end{bmatrix}$$ 

Hence, by matrix multiplication, 

$$ \dot{\textbf T}_{BO} = \dot{\psi} \begin{bmatrix} -\sin\psi && \cos\psi && 0 \\\\ -\cos\psi && -\sin\psi && 0 \\\\ 0 && 0 && 0 \end{bmatrix}$$ 

By similar calculations, other time derivatives of the transfer matrix can also be derived. 

$$ \dot{\textbf T}_{OI} = \dot{\theta} \begin{bmatrix} -\sin\theta && 0 && \cos\theta \\\\ 0 && 0 && 0 \\\\ -\cos\theta && 0 && -\sin\theta \end{bmatrix} $$

$$ \dot{\textbf T}_{IS} = \dot{\phi} \begin{bmatrix} 0 && 0 && 0 \\\\ 0 && -\sin\phi && \cos\phi \\\\ 0 && -\cos\phi && -\sin\phi \end{bmatrix} $$ 

These equations will be utilized in determining the complete kinematics and dynamics of the testbench. 

##	Kinematics  

The kinematic analysis of the testbench involves determination of the angular velocity and acceleration along the body frame axes. Each of the body frames require discreet analysis to assess the effect of other body frames on it, which involves the rotational matrices and its time derivatives. The following sections provide the derivation of such for each of the body frames. 

###	Inner Shaft with Drone Clamped  

The rotation in the innermost body frame, which involves the modified ring in form of shaft where the drone would be clamped, requires three different components of rotation to be considered. The angular velocity of the innermost body frame relative to the earth frame represented in body frame could be derived as follows. 

$$\vec\omega_s=\dot{\vec{\phi}}+\textbf T_{IS}.\dot{\vec\theta}+\textbf T_{OI}.\textbf T_{IS}.\dot{\vec\psi}$$

The angular acceleration, which is the time derivative of the angular velocity, can also be represented accordingly as follows. 

$$\vec\alpha_s=\frac{\partial{\vec\omega_s}}{\partial{t}}$$ 

$$\vec\alpha_s=\ddot{\vec\phi}+\dot{\textbf T}_ {IS}.\dot{\vec\theta}+\textbf T_{IS}.\ddot{\vec\theta}+\dot{\textbf T}_{OI}.{\textbf T} _{IS} .\dot{\vec\psi}+{\textbf T} _{OI}.\dot{\textbf T} _{IS}.\dot{\vec\psi}+{\textbf T} _{OI}.{\textbf T} _{IS}.\ddot{\vec\psi}$$

 

###	Inner Gimbal  

The inner gimbal has two components of rotation. The angular velocity of the inner gimbal body frame relative to the earth frame represented in body frame could also be derived as follows. 

$$\vec\omega_i= \dot{\vec\theta}+{\textbf T} _{IS}.\dot{\vec\psi} $$ 


The angular acceleration can also be thus determined. 

$$ \vec\alpha_i=\ddot{\vec\theta}+\dot{\textbf T} _{IS}.\dot{\vec\psi}+{\textbf T} _{IS}.\ddot{\vec\psi} $$ 

###	Outer Gimbal  

Determination of the kinematic relations of the outer gimbal is the simplest, as it requires only component of rotation along its own body frame. Both the angular velocity and acceleration can hence be represented as follows. 

$$\vec\omega_o= \dot{\vec\psi}$$ 

$$\vec\alpha_o=\ddot{\vec\psi}$$ 
 
## Dynamics  

For dynamics study of the testbench, the relation between the applied torque and thrust by the drone to the testbench, the consequent dynamic and reaction forces, and the kinematics as previously estimated would be determined. For determining the dynamic relationship, knowledge about Newton-Euler formulation is necessary as it will be used in this study. 

### Newton-Euler Formulation  

The Newton-Euler formulation combines Newton's second law and Euler's equations to describe the dynamics of rigid bodies. It provides a systematic approach to calculate forces and torques, accounting for both linear and angular motion, and is vital in robotics and mechanical systems analysis. The general expression of Newton-Euler formulation is as follows. 

$$m\{\vec\alpha\times\vec r+\vec\omega\times(\vec\omega\times\vec r)\}=\vec F$$ 

$$\textbf J \vec\alpha + \vec\omega \times \textbf J \vec\omega=\vec N$$ 

### Inner Shaft with Drone Clamped  

The shaft with drone attached has distinctive features compared to other body frames, since it involves the asymmetric placement of the drone and the torques and forces directly applied from the drone attached. First, the position vector of the drone’s CG relative to the symmetric CG of the inner shaft which coincides with the origin of the testbench is determined. 

$$\vec r_{OD} = \begin{bmatrix}r_{x,od}&&r_{y,od}&& r_{z,od}\end{bmatrix}^T$$ 

The position vector of the CG of drone affects the symmetricity of the testbench and hench it results in dynamic inertia forces. The position vector is also used to determine the asymmetric MOI of the drone clamped with the shaft. The equivalent MOI of the shaft with attached drone can be expressed as follows. 

$$\textbf J_{D+S} =\textbf J_S+\textbf J_D+m_D\{(\vec r_{OG}\cdot\vec r_{OG})\textbf I_{3\times3}-\vec r_{OG}\otimes\vec r_{OG}\}$$

As can be seen, the combined MOI involves the individual MOI of the drone and the shaft along with the dot product and Kronecker product terms. The exerted thrust and torque by a quadcopter’s propeller rotation can be reduced to following expressions. 

$$\vec F_p=\begin{bmatrix}0&&0&&U_1\end{bmatrix}^T$$ 

$$\vec N_p =\begin{Bmatrix}U_2\\\\U_3\\\\U_4\end{Bmatrix}+\vec\omega_s\times\begin{Bmatrix}0\\\\0\\\\J_{TP}\Omega\end{Bmatrix}$$

The body force or the gravitational force is fixated to the earth frame, therefore, it would constantly keep transforming its orientation with the gimbals, and thus the gravitational acceleration vector must be defined in the reference body frame. Transforming this vector is achieved by the following expression. 

$$\vec g_s = \textbf T_{OI}.\textbf T_{IS}.\vec g$$

Thus, the Newton-Euler formulation of the innermost body frame would be as follows. 

$$m_d\{\vec\alpha_s\times\vec r_{OD}+\vec\omega_s\times(\vec\omega_s\times\vec r_{OD})\}=\vec\lambda_{SI}+\vec F_p+(m_s+m_d)\vec g_s$$

$$\textbf J_{D+S}\vec\alpha_s+\vec\omega_s\times\textbf J_{D+S}\vec\omega_s=\vec\tau_{SI}+\vec r_{OD}\times(\vec F_p+m_D\vec g_S)+\vec N_p$$ 

### Inner Gimbal  

Since frictions are assumed to be negligible in the joints of the gimbals, the dynamics relations of inner and outer gimbals are quite simplified, whereas the issue of using the reaction force and moments applied by the inner frames still involves somewhat complexity in the analysis. The gimbals’ CG remains fixed, thus there would be no inertia forces present in the decoupled gimbals. However, the reaction forces from the internal gimbal or shaft would apply, which would result in reaction forces exerted to the external gimbal or base. For inner gimbal, the balance of the reaction forces would render following equation. 

$$\textbf T_{SI}\vec\lambda_{SI}=m_i\vec g_i+\vec\lambda_{IO}$$ 

Here, the gravitational acceleration constant is transformed to the inner gimbal body frame by the equation shown. 

$$\vec g_s = \textbf T_{OI}.\vec g$$ 

The inertial moments would exist, on contrary. Transforming the reaction torques of the inner shaft with drone attached to the body frame accordingly and thus determining the Newton-Euler formulation would result in following expression. 

$$\textbf J_I \vec\alpha_i+\vec\omega_i\times\textbf J_I\vec\omega_i=\vec\tau_{IO}-\textbf T_{SI}\vec\tau_{SI}$$ 

### Outer Gimbal 

The outer gimbal dynamic relations involve the reaction forces from the inner gimbal and the base of the testbench. Determining this relationship would require the reaction forces in the base to be considered. Like the inner gimbal, the following expression shows the formulation for outer gimbal. 

$$\textbf T_{IO}\vec\lambda_{IO}=m_o\vec g+\vec\lambda_{OB}$$ 

$$\textbf J_O \vec\alpha_o+\vec\omega_o\times\textbf J_O\vec\omega_o=\vec\tau_{OB}-\textbf T_{IO}\vec\tau_{IO}$$ 

### Reaction force on the base 

The Base is susceptible to the reaction forces from the outer gimbal. The following expression shows the reaction forces and torques applied on the base.  

$$\vec\lambda_B=\textbf T_{OB}\vec\lambda_{OB}$$ 

$$\vec\tau_B=\textbf T_{OB}\vec\tau_{OB}$$

All these equations can be reduced to a single linear algebra equation, which is a tedious process, as multiple matrix multiplications would take place. Newton-Euler Formulation, however, is not useful for this setup as it requires reaction forces to be known for a definite solution. So until there would be a viable way that the reaction forces could be measured, the feasibilty of this model will have to wait.  
