import pinocchio as pin
from ainex_controller.ainex_model import AiNexModel
from ainex_controller.spline_trajectory import LinearSplineTrajectory
import numpy as np  
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class HandController():
    def __init__(self, node: Node, model: AiNexModel, arm_side: str,
                 Kp: np.ndarray = None,
                 Kd: np.ndarray = None):
        """Initialize the Ainex Arm Controller.
        
        Args:
            model: Pinocchio robot model.
            arm_side: 'left' or 'right' arm.
        """
        self.node = node
        self.br = TransformBroadcaster(node)
        self.robot_model = model
        self.arm_side = arm_side

        self.x_cur = pin.SE3.Identity()
        self.x_des = pin.SE3.Identity()
        
        self.x_init = pin.SE3.Identity()
        self.x_target = pin.SE3.Identity()

        # velocity
        self.v_cur = pin.Motion.Zero()

        self.spline = None
        self.spline_duration = 0.0
        self.start_time = None

        self.w_threshold = 5e-4  # manipulability threshold
        self.ignore_manipulability=False
        if Kp is not None:
            self.Kp = Kp
        else:
            self.Kp = np.array([5.0, 5.0, 5.0])
        
        if Kd is not None:
            self.Kd = Kd
        else:
            self.Kd = np.array([0.5, 0.5, 0.5])


    def set_target_pose(self, pose: pin.SE3, duration: float, type: str = 'abs'):
        """Set the desired target pose for the specified arm.
        
        Args:
            pose (pin.SE3): Desired end-effector pose.
            type (str): 'abs' or 'rel' pose setting.
        """
        self.x_cur = pin.SE3.Identity()
        if self.arm_side == 'left':
            self.x_cur = self.robot_model.left_hand_pose().copy()
        else:
            self.x_cur = self.robot_model.right_hand_pose().copy()
        self.x_init = self.x_cur.copy()

        # set target pose based on type 
        # abs: absolute pose w.r.t. base_link
        # rel: relative pose w.r.t. current pose
        if type == 'abs':
            self.x_target = pose.copy()
        elif type == 'rel':
            self.x_target = self.x_cur * pose
        else:
            raise ValueError("type must be 'abs' or 'rel'")
        
        if duration <= 0.0:
            raise ValueError("duration must be a positive number")
        
        # Plan a spline trajectory from current to target pose using the
        # class LinearSplineTrajectory defined in spline_trajectory.py
        self.spline = LinearSplineTrajectory(
            self.x_init.translation.copy(),
            self.x_target.translation.copy(),
            duration
        )
        self.spline_duration = duration

        # Save start time
        self.start_time = self.node.get_clock().now()

    def update(self, dt: float) -> np.ndarray:
        """Update the arm controller with new joint states.
        
        Args:
            dt (float): Controller update period (kept for API compatibility).

        Returns:
            np.ndarray: Desired joint velocities for the arm.(4,)
        """

        # Broadcast target pose as TF for visualization
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = f"{self.arm_side}_hand_target"
        t.transform.translation.x = self.x_target.translation[0]
        t.transform.translation.y = self.x_target.translation[1]
        t.transform.translation.z = self.x_target.translation[2]
        quat = pin.Quaternion(self.x_target.rotation).coeffs()  # (x, y, z, w)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.br.sendTransform(t)

        # Get current pose and velocity from the robot model
        # We assume the model is already updated with the latest joint states
        if self.arm_side == 'left':
            self.x_cur = self.robot_model.left_hand_pose()
            self.v_cur = self.robot_model.left_hand_velocity()
            J = self.robot_model.J_left
        else:
            self.x_cur = self.robot_model.right_hand_pose()
            self.v_cur = self.robot_model.right_hand_velocity()
            J = self.robot_model.J_right

        # Calculate the time alapsend since the start of the trajectory
        # Update the spline to get desired position at current time
        if self.spline is None or self.start_time is None:
            self.x_des = self.x_cur.copy()
            vel_des = np.zeros(3)
        else:
            elapsed_time = (self.node.get_clock().now() - self.start_time).nanoseconds * 1e-9
            elapsed_time = max(0.0, min(self.spline_duration, elapsed_time))
            pos_des, vel_des = self.spline.update(elapsed_time)
            self.x_des = pin.SE3(self.x_target.rotation.copy(), pos_des)

        # Implement the Cartesioan PD control law for end-effector POSITION only (no orientation part)
        # compute desired end-effector velocity using Cartesian PD on translation only
        pos_error = self.x_des.translation - self.x_cur.translation
        if isinstance(self.v_cur, pin.Motion):
            v_linear = self.v_cur.linear
        else:
            v_linear = np.asarray(self.v_cur).reshape(-1)[:3]
        vel_error = vel_des - v_linear
        xdot_des = self.Kp * pos_error + self.Kd * vel_error
        
        # Retrieve the end-effector Jacibian that relates 
        # the end-effector LINEAR velocity and the ARM JOINTS.
        # Hint: Extract the linear part of the full Jacobian by taking its first three rows, 
        # and keep only the columns corresponding to the arm joints.
        # You can obtain the arm joint indices using AinexModel.get_arm_ids().
        arm_ids = self.robot_model.get_arm_ids(self.arm_side)
        J_pos = J[:3, arm_ids]

        # Compute the control command (velocities for the arm joints)
        # by mapping the desired end-effector velocity to arm joint velocities 
        # using the Jacobian pseudoinverse
        JJt = J_pos @ J_pos.T
        try:
            u = J_pos.T @ np.linalg.solve(JJt, xdot_des)
        except np.linalg.LinAlgError:
            u = np.zeros(J_pos.shape[1])

        ## Check manipulability to prevent singularities
        # Calculate the manipulability index with the task Jacobian J_pos.
        # Hint: w = sqrt(det(J * J^T))
        # If the manipulability is below the threshold self.w_threshold,
        # stop the robot by setting u to zero.
        w = np.sqrt(max(np.linalg.det(JJt), 0.0))
        if self.ignore_manipulability:
            return u
        else:
            if w < self.w_threshold:
                u = np.zeros_like(u)

            return u

    def motion_complete(self, position_tolerance: float = 1e-3) -> bool:
        """Return True when the current motion finished and the target is reached."""
        if self.spline is None or self.start_time is None:
            return False

        elapsed_time = (self.node.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed_time < self.spline_duration:
            return False

        if self.arm_side == 'left':
            pose = self.robot_model.left_hand_pose()
        else:
            pose = self.robot_model.right_hand_pose()

        distance = np.linalg.norm(pose.translation - self.x_target.translation)
        return distance <= position_tolerance
