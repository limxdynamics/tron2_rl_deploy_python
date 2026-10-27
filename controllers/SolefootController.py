import copy
import os
import time
from functools import partial
from typing import ClassVar

import numpy as np
import onnxruntime as ort
import yaml
from limxsdk import datatypes
from limxsdk.robot import Rate
from scipy.spatial.transform import Rotation as R

try:
    # 无图形会话时 SDL 初始化视频子系统会段错误，手柄只需 joystick 子系统
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

class SolefootController:
    # 自研遥控器按 PS 式枚举：× / ○ / □ / △
    JOY_BTNS: ClassVar[dict] = {"CROSS": 0, "CIRCLE": 1, "SQUARE": 2, "TRIANGLE": 3, "L1": 4, "R1": 5}
    JOY_AXES: ClassVar[dict] = {"left_vertical": 1, "left_horizon": 0, "right_horizon": 2}
    PYGAME_AXES: ClassVar[dict] = {"left_vertical": 1, "left_horizon": 0, "right_horizon": 3}

    def __init__(self, model_dir, robot, robot_type, start_controller, use_pygame_joystick=False):
        # Initialize robot and type information
        self.robot = robot
        self.robot_type = robot_type

        # Load configuration and model file paths based on robot type
        self.config_file = f'{model_dir}/{self.robot_type}/params.yaml'
        self.model_policy = f'{model_dir}/{self.robot_type}/policy.onnx'
        self.model_encoder = f'{model_dir}/{self.robot_type}/encoder.onnx'

        # Load configuration settings from the YAML file
        self.load_config(self.config_file)
        
        # Load the ONNX model
        self.initialize_onnx_models()

        # Prepare robot command structure with default values for mode, q, dq, tau, Kp, Kd
        self.robot_cmd = datatypes.RobotCmd()
        self.robot_cmd.mode = [0. for x in range(self.joint_num)]
        self.robot_cmd.q = [0. for x in range(self.joint_num)]
        self.robot_cmd.dq = [0. for x in range(self.joint_num)]
        self.robot_cmd.tau = [0. for x in range(self.joint_num)]
        self.robot_cmd.Kp = [self.control_cfg.get('leg_joint_stiffness', self.control_cfg.get('stiffness')) for x in range(self.joint_num)]
        self.robot_cmd.Kd = [self.control_cfg.get('leg_joint_damping', self.control_cfg.get('damping')) for x in range(self.joint_num)]
        self.robot_cmd.motor_names = ["" for _ in range(self.joint_num)]

        # Prepare robot state structure
        self.robot_state = datatypes.RobotState()
        self.robot_state.tau = [0. for x in range(self.joint_num)]
        self.robot_state.q = [0. for x in range(self.joint_num)]
        self.robot_state.dq = [0. for x in range(self.joint_num)]
        self.robot_state_tmp = copy.deepcopy(self.robot_state)

        # Initialize IMU (Inertial Measurement Unit) data structure
        self.imu_data = datatypes.ImuData()
        self.imu_data.quat[0] = 0
        self.imu_data.quat[1] = 0
        self.imu_data.quat[2] = 0
        self.imu_data.quat[3] = 1
        self.imu_data_tmp = copy.deepcopy(self.imu_data)

        # Set up a callback to receive updated robot state data
        self.robot_state_callback_partial = partial(self.robot_state_callback)
        self.robot.subscribeRobotState(self.robot_state_callback_partial)

        # Set up a callback to receive updated IMU data
        self.imu_data_callback_partial = partial(self.imu_data_callback)
        self.robot.subscribeImuData(self.imu_data_callback_partial)

        # Set up a callback to receive updated SensorJoy
        self.sensor_joy_callback_partial = partial(self.sensor_joy_callback)
        self.robot.subscribeSensorJoy(self.sensor_joy_callback_partial)

        # Set up a callback to receive diagnostic data
        self.robot_diagnostic_callback_partial = partial(self.robot_diagnostic_callback)
        self.robot.subscribeDiagnosticValue(self.robot_diagnostic_callback_partial)

        # Initialize the calibration state to -1, indicating no calibration has occurred.
        self.calibration_state = -1

        # Flag to start the controller
        self.start_controller = start_controller

        # Gait index
        self.gait_index = 0

        # Flag indicating first received observation
        self.is_first_rec_obs = True

        # Flag indicating the first RobotState frame has arrived
        self.robot_state_received = False

        self._use_pygame_joystick = use_pygame_joystick
        self._pygame_enabled = False
        self._pygame_joystick = None
        self.joy_msg_count = 0
        self.last_joy_time = 0.0
        self._pygame_last_r1 = 0
        self._last_r1 = 0

        self._setup_pygame_joystick()
    
    def initialize_onnx_models(self):
        # Configure ONNX Runtime session options to optimize CPU usage
        session_options = ort.SessionOptions()
        # Limit the number of threads used for parallel computation within individual operators
        session_options.intra_op_num_threads = 1
        # Limit the number of threads used for parallel execution of different operators
        session_options.inter_op_num_threads = 1
        # Enable all possible graph optimizations to improve inference performance
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # Disable CPU memory arena to reduce memory fragmentation
        session_options.enable_cpu_mem_arena = False
        # Disable memory pattern optimization to have more control over memory allocation
        session_options.enable_mem_pattern = False

        # Define execution providers to use CPU only, ensuring no GPU inference
        cpu_providers = ['CPUExecutionProvider']
        
        # Load the ONNX model and set up input and output names
        self.policy_session = ort.InferenceSession(self.model_policy, sess_options=session_options, providers=cpu_providers)
        self.policy_input_names = [self.policy_session.get_inputs()[i].name for i in range(self.policy_session.get_inputs().__len__())]
        self.policy_output_names = [self.policy_session.get_outputs()[i].name for i in range(self.policy_session.get_outputs().__len__())]
        self.policy_input_shapes = [self.policy_session.get_inputs()[i].shape for i in range(self.policy_session.get_inputs().__len__())]
        self.policy_output_shapes = [self.policy_session.get_outputs()[i].shape for i in range(self.policy_session.get_outputs().__len__())]

        self.encoder_session = ort.InferenceSession(self.model_encoder, sess_options=session_options, providers=cpu_providers)
        self.encoder_input_names = [self.encoder_session.get_inputs()[i].name for i in range(self.encoder_session.get_inputs().__len__())]
        self.encoder_output_names = [self.encoder_session.get_outputs()[i].name for i in range(self.encoder_session.get_outputs().__len__())]
        self.encoder_input_shapes = [self.encoder_session.get_inputs()[i].shape for i in range(self.encoder_session.get_inputs().__len__())]
        self.encoder_output_shapes = [self.encoder_session.get_outputs()[i].shape for i in range(self.encoder_session.get_outputs().__len__())]

    # Load the configuration from a YAML file
    def load_config(self, config_file):
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)['PointfootCfg']

        # Assign configuration parameters to controller variables
        self.joint_names = config.get('joint_names', config.get('init_state', {}).get('joint_names'))
        self.init_state = config['init_state']['default_joint_angle']
        
        self.control_cfg = config['control']
        self.rl_cfg = config['normalization']
        
        # Default observation scales if not in config
        self.obs_scales = config['normalization'].get('obs_scales', {"ang_vel": 0.25, "dof_pos": 1.0, "dof_vel": 0.05})
        
        self.actions_size = config['size']['actions_size']
        self.commands_size = config['size']['commands_obs_size']
        self.observations_size = config['size']['policy_obs_size']
        self.obs_history_length = config['size']['obs_history_length']
        self.encoder_output_size = config['size'].get('encoder_output_size', 3) # Default to 3 if missing
        
        imu_orientation_offset = config.get('imu_orientation_offset', {'roll': 0, 'pitch': 0, 'yaw': 0})
        self.imu_orientation_offset = np.array([
            imu_orientation_offset.get('yaw'),
            imu_orientation_offset.get('pitch'),
            imu_orientation_offset.get('roll')
        ])
        
        # These might be missing in new config, providing defaults or using other keys
        self.stand_duration = config.get('stand_mode', {}).get('stand_duration')
        self.user_cmd_cfg = config.get('user_cmd_scales', {
            'lin_vel_x': config['commands']['max']['lin_vel_x'],
            'lin_vel_y': config['commands']['max']['lin_vel_y'],
            'ang_vel_yaw': config['commands']['max']['ang_vel_yaw']
        })
        self.loop_frequency = config.get('loop_frequency',500)
        self.decimation = config['control'].get('decimation')
        
        self.encoder_input_size = self.obs_history_length * self.observations_size

        # Initialize variables for actions, observations, and commands
        self.proprio_history_buffer = np.zeros(self.encoder_input_size)
        self.proprio_history_vector = np.zeros(self.obs_history_length * self.observations_size)
        self.encoder_out = np.zeros(self.encoder_output_size)
        self.actions = np.zeros(self.actions_size)
        self.observations = np.zeros(self.observations_size)
        self.last_actions = np.zeros(self.actions_size)
        self.commands = np.zeros(self.commands_size)  # command to the robot (e.g., velocity, rotation)
        self.scaled_commands = np.zeros(self.commands_size)
        self.base_lin_vel = np.zeros(3)  # base linear velocity
        self.base_position = np.zeros(3)  # robot base position
        self.loop_count = 0  # loop iteration count
        self.stand_percent = 0  # percentage of time the robot has spent in stand mode
        self.policy_session = None  # ONNX model session for policy inference
        self.joint_num = len(self.joint_names)  # number of joints

        # In TRON2, these might be named differently or structured differently
        self.ankle_joint_damping = config['control'].get('ankle_joint_damping')
        self.ankle_joint_torque_limit = config['control'].get('ankle_joint_torque_limit')

        self.gait_frequencies = config['gait'].get('frequency')
        self.gait_swing_height = config['gait'].get('swing_height')

        # Initialize joint angles based on the initial configuration
        self.init_joint_angles = np.zeros(len(self.joint_names))
        for i in range(len(self.joint_names)):
            self.init_joint_angles[i] = self.init_state[self.joint_names[i]]
        
        # 上电后停在 IDLE，必须按 L1+○ 才进入 prepare
        self.mode = "IDLE"

    def wait_for_robot_state(self, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.robot_state_received:
                return True
            time.sleep(0.01)
        return False

    # Main control loop
    def run(self):
        # 首帧到达前 default_joint_angles 会退化成全 0，且 SDK 会拒发指令并刷屏报错
        if not self.wait_for_robot_state():
            print("\033[31mERROR: 未在 5s 内收到 RobotState，放弃启动。\033[0m")
            return

        # Set the loop rate based on the frequency in the configuration
        rate = Rate(self.loop_frequency)
        
        # Initialize default joint angles for standing
        self.default_joint_angles = np.array(self.robot_state.q)
        
        print("=" * 60)
        print(f"TRON2 Policy Interface ready [{self.robot_type}]")
        print(f"Pygame Joystick: {'ENABLED' if self._pygame_enabled else 'DISABLED'}")
        print("Prepare(进零位): L1 + ○, Start: L1 + △, Stop: L1 + ×, clear history: R1")
        print(f"当前模式: {self.mode}（未按 L1 + ○ 前不会自动进零位）")
        print("=" * 60)

        while True:
            self.update()
            rate.sleep()

    # Handle the stand mode for smoothly transitioning the robot into standing
    def handle_stand_mode(self):
        kp = self.control_cfg.get('leg_joint_stiffness', self.control_cfg.get('stiffness'))
        kd = self.control_cfg.get('leg_joint_damping', self.control_cfg.get('damping'))
        for j in range(len(self.joint_names)):
            # Interpolate between initial and default joint angles during stand mode
            pos_des = self.default_joint_angles[j] * (1 - self.stand_percent) + self.init_state[self.joint_names[j]] * self.stand_percent
            self.set_joint_command(j, pos_des, 0, 0, kp, kd)
            
        if self.stand_percent < 1:
            # Increment the stand percentage over time
            self.stand_percent += 1 / (self.stand_duration * self.loop_frequency)

    # Handle the walk mode where the robot moves based on computed actions
    def handle_walk_mode(self):
        # Update the temporary robot state and IMU data
        self.robot_state_tmp = copy.deepcopy(self.robot_state)
        self.imu_data_tmp = copy.deepcopy(self.imu_data)

        # Execute actions every 'decimation' iterations
        if self.loop_count % self.control_cfg['decimation'] == 0:
            self.compute_observation()
            self.compute_encoder()
            self.compute_actions()
            # Clip the actions within predefined limits
            action_min = -self.rl_cfg['clip_scales']['clip_actions']
            action_max = self.rl_cfg['clip_scales']['clip_actions']
            self.actions = np.clip(self.actions, action_min, action_max)

        # Iterate over the joints and set commands based on actions
        joint_pos = np.array(self.robot_state_tmp.q)
        joint_vel = np.array(self.robot_state_tmp.dq)

        for i in range(self.joint_num):
            action = self.actions[i]
            self.last_actions[i] = action
            
            # Use TRON2 style joint control if possible, otherwise fallback to basic position control
            kp = self.control_cfg.get('leg_joint_stiffness', self.control_cfg.get('stiffness'))
            kd = self.control_cfg.get('leg_joint_damping', self.control_cfg.get('damping'))
            torque_limit = self.control_cfg.get('leg_joint_torque_limit', self.control_cfg.get('user_torque_limit'))
            
            # Check for specific joint types if they exist in config
            if (i + 1) % 5 == 3 or (i + 1) % 5 == 0: # Light joint (proximal yaw / ankle pitch)
                kp = self.control_cfg.get('light_joint_stiffness', kp)
                kd = self.control_cfg.get('light_joint_damping', kd)
                torque_limit = self.control_cfg.get('light_joint_torque_limit', torque_limit)

            action_min = (joint_pos[i] - self.init_joint_angles[i] +
                          (kd * joint_vel[i] - torque_limit) / kp) / self.control_cfg['action_scale_pos']
            action_max = (joint_pos[i] - self.init_joint_angles[i] +
                          (kd * joint_vel[i] + torque_limit) / kp) / self.control_cfg['action_scale_pos']
            
            action = np.clip(action, action_min, action_max)
            pos_des = action * self.control_cfg['action_scale_pos'] + self.init_joint_angles[i]
            self.set_joint_command(i, pos_des, 0, 0, kp, kd)

    def compute_observation(self):
        # Convert IMU orientation from quaternion to Euler angles (ZYX convention)
        imu_orientation = np.array(self.imu_data_tmp.quat)
        q_wi = R.from_quat(imu_orientation).as_euler('zyx')  # Quaternion to Euler ZYX conversion
        inverse_rot = R.from_euler('zyx', q_wi).inv().as_matrix()  # Get the inverse rotation matrix

        # Project the gravity vector (pointing downwards) into the body frame
        gravity_vector = np.array([0, 0, -1])  # Gravity in world frame (z-axis down)
        projected_gravity = np.dot(inverse_rot, gravity_vector)  # Transform gravity into body frame

        # Retrieve base angular velocity from the IMU data
        base_ang_vel = np.array(self.imu_data_tmp.gyro)
        # Apply IMU orientation offset correction (using Euler angles)
        rot = R.from_euler('zyx', self.imu_orientation_offset).as_matrix()  # Rotation matrix for offset correction
        base_ang_vel = np.dot(rot, base_ang_vel)  # Apply correction to angular velocity
        projected_gravity = np.dot(rot, projected_gravity)  # Apply correction to projected gravity

        # Retrieve joint positions and velocities from the robot state
        joint_positions = np.array(self.robot_state_tmp.q)
        joint_velocities = np.array(self.robot_state_tmp.dq)

        gait = np.array([self.gait_frequencies, 0.5, 0.5, self.gait_swing_height])
        has_velocity_cmd = np.linalg.norm(self.commands[:3]) > 1e-3
        if has_velocity_cmd:
            step_dt = self.decimation / self.loop_frequency
            self.gait_index = np.mod(self.gait_index + step_dt * gait[0], 1.0)
            gait_clock = np.array([
                np.sin(self.gait_index * 2 * np.pi),
                np.cos(self.gait_index * 2 * np.pi),
            ])
        else:
            # 无速度指令：相位归零，步态时钟不推进（sin=0, cos=1）
            self.gait_index = 0.0
            gait_clock = np.array([0.0, 1.0])
        # Retrieve the last actions that were applied to the robot
        actions = np.array(self.last_actions)

        # Create a command scaler matrix for linear and angular velocities
        command_scaler = np.diag([
            self.user_cmd_cfg['lin_vel_x'],  # Scale factor for linear velocity in x direction
            self.user_cmd_cfg['lin_vel_y'],  # Scale factor for linear velocity in y direction
            self.user_cmd_cfg['ang_vel_yaw'],  # Scale factor for yaw (angular velocity)
        ])

        # Apply scaling to the command inputs (velocity commands)
        self.scaled_commands = np.dot(command_scaler, self.commands[:3])

        # Populate observation vector
        joint_pos_input = (joint_positions - self.init_joint_angles) * self.obs_scales['dof_pos']

        # Create the observation vector by concatenating various state variables:
        # - Base angular velocity (scaled)
        # - Projected gravity vector
        # - Joint positions (difference from initial angles, scaled)
        # - Joint velocities (scaled)
        # - Last actions applied to the robot
        # - gait_clock: A clock signal related to the gait of the robot.
        # - gait: Information about the current gait of the robot.
        obs = np.concatenate([
            base_ang_vel * self.obs_scales['ang_vel'],  # Scaled base angular velocity
            projected_gravity,  # Projected gravity vector in body frame
            joint_pos_input,  # Scaled joint positions
            joint_velocities * self.obs_scales['dof_vel'],  # Scaled joint velocities
            actions,  # Last actions taken by the robot
            gait_clock,  # A clock signal related to the robot's gait
            gait  # Information about the current gait pattern of the robot
        ])

        # Check if this is the first recorded observation
        if self.is_first_rec_obs:
            # Fill the proprioceptive history buffer with the current observation for the entire history length
            for i in range(self.obs_history_length):
                self.proprio_history_buffer[i * self.observations_size:(i + 1) * self.observations_size] = obs
            
            # Update the flag to indicate that the first observation has been processed
            self.is_first_rec_obs = False
        
        # Shift the existing proprioceptive history buffer to the left
        self.proprio_history_buffer[:-self.observations_size] = self.proprio_history_buffer[self.observations_size:]

        # Add the current observation to the end of the proprioceptive history buffer
        self.proprio_history_buffer[-self.observations_size:] = obs

        # Convert the proprioceptive history buffer to a numpy array
        self.proprio_history_vector = np.array(self.proprio_history_buffer)

        # Clip the observation values to within the specified limits for stability
        self.observations = np.clip(
            obs, 
            -self.rl_cfg['clip_scales']['clip_observations'],  # Lower limit for clipping
            self.rl_cfg['clip_scales']['clip_observations']  # Upper limit for clipping
        )

    def compute_actions(self):
        """
        Computes the actions based on the current observations using the policy session.
        """
        # Concatenate observations into a single tensor and convert to float32
        input_tensor = np.concatenate([self.encoder_out, self.observations, self.scaled_commands], axis=0)
        input_tensor = input_tensor.astype(np.float32)
        
        # Create a dictionary of inputs for the policy session
        inputs = {self.policy_input_names[0]: input_tensor}
        
        # Run the policy session and get the output
        output = self.policy_session.run(self.policy_output_names, inputs)
        
        # Flatten the output and store it as actions
        self.actions = np.array(output).flatten()

    def compute_encoder(self):
        """
        Computes the encoder output based on the proprioceptive history buffer.

        This method first concatenates the proprioceptive history buffer into a single input tensor.
        Then it converts the input tensor to the float32 data type. After that, it creates a dictionary
        of inputs for the encoder session and runs the encoder session to get the output. Finally,
        it flattens the output and stores it as the encoder output.
        """
        # Concatenate the proprioceptive history buffer into a single tensor and convert to float32
        input_tensor = np.concatenate([self.proprio_history_buffer], axis=0)
        input_tensor = input_tensor.astype(np.float32)

        # Create a dictionary of inputs for the encoder session
        inputs = {self.encoder_input_names[0]: input_tensor}

        # Run the encoder session and get the output
        output = self.encoder_session.run(self.encoder_output_names, inputs)

        # Flatten the output and store it as the encoder output
        self.encoder_out = np.array(output).flatten()
 
    def set_joint_command(self, joint_index, q, dq, tau, kp, kd):
        """
        Sends a command to configure the state of a specific joint.
        This method updates the joint's desired position, velocity, torque, and control gains.
        Replace this implementation with the actual communication logic for your hardware.

        Parameters:
        joint_index (int): The index of the joint to be controlled.
        q (float): The desired joint position, typically in radians or degrees.
        dq (float): The desired joint velocity, typically in radians/second or degrees/second.
        tau (float): The desired joint torque, typically in Newton-meters (Nm).
        kp (float): The proportional gain for position control.
        kd (float): The derivative gain for velocity control.
        """
        self.robot_cmd.q[joint_index] = q
        self.robot_cmd.dq[joint_index] = dq
        self.robot_cmd.tau[joint_index] = tau
        self.robot_cmd.Kp[joint_index] = kp
        self.robot_cmd.Kd[joint_index] = kd

    def _setup_pygame_joystick(self):
        self._pygame_enabled = False
        if not self._use_pygame_joystick or not PYGAME_AVAILABLE:
            return
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() <= 0:
                print("pygame joystick: no device found, fallback to SDK joystick topic")
                return
            self._pygame_joystick = pygame.joystick.Joystick(0)
            self._pygame_joystick.init()
            self._pygame_enabled = True
            print(f"pygame joystick ready: {self._pygame_joystick.get_name()}")
        except Exception as exc:  # noqa: BLE001 - any pygame failure falls back to the SDK joystick
            self._pygame_enabled = False
            self._pygame_joystick = None
            print(f"pygame joystick init failed: {exc}")

    @staticmethod
    def _deadzone(value, threshold=0.08):
        return 0.0 if abs(value) < threshold else value

    def _clip_unit(self, value):
        return max(-1.0, min(1.0, float(value)))

    def _clear_history(self):
        self.last_actions[:] = 0.0
        self.actions[:] = 0.0
        self.observations[:] = 0.0
        self.encoder_out[:] = 0.0
        self.proprio_history_vector[:] = 0.0
        self.proprio_history_buffer[:] = 0.0
        self.is_first_rec_obs = True
        self.gait_index = 0.0

    def _clear_commands(self):
        self.commands[:] = 0.0
        self.scaled_commands[:] = 0.0

    def _process_joystick(self, buttons, axes, r1_state_attr):
        l1 = buttons[self.JOY_BTNS["L1"]] if len(buttons) > self.JOY_BTNS["L1"] else 0
        cross = buttons[self.JOY_BTNS["CROSS"]] if len(buttons) > self.JOY_BTNS["CROSS"] else 0
        circle = buttons[self.JOY_BTNS["CIRCLE"]] if len(buttons) > self.JOY_BTNS["CIRCLE"] else 0
        triangle = buttons[self.JOY_BTNS["TRIANGLE"]] if len(buttons) > self.JOY_BTNS["TRIANGLE"] else 0
        r1 = buttons[self.JOY_BTNS["R1"]] if len(buttons) > self.JOY_BTNS["R1"] else 0

        if l1 and circle and self.mode != "STAND":
            print("L1 + ○: prepare...")
            self.start_controller = False
            # 从按下时的实测姿态开始插值，而不是启动时的旧快照
            self.default_joint_angles = np.array(self.robot_state.q)
            self.stand_percent = 0.0
            self.mode = "STAND"
            self._clear_history()
            self._clear_commands()

        if not self.start_controller and l1 and triangle:
            print("L1 + △: start_controller...")
            self.start_controller = True
            self.mode = "WALK"
            self._clear_history()

        if l1 and cross and self.mode != "IDLE":
            print("L1 + ×: stop_controller...")
            self.start_controller = False
            self.mode = "IDLE"
            self._clear_history()
            self._clear_commands()

        if r1 and not getattr(self, r1_state_attr):
            self._clear_history()
            self._clear_commands()
        setattr(self, r1_state_attr, int(r1))

        if self.mode != "WALK":
            return

        linear_x = axes[0] if len(axes) > 0 else 0.0
        linear_y = axes[1] if len(axes) > 1 else 0.0
        angular_z = axes[2] if len(axes) > 2 else 0.0
        linear_x = self._clip_unit(linear_x)
        linear_y = self._clip_unit(linear_y)
        angular_z = self._clip_unit(angular_z)

        self.commands[0] = float(np.clip(linear_x * self.user_cmd_cfg['lin_vel_x'], -self.user_cmd_cfg['lin_vel_x'], self.user_cmd_cfg['lin_vel_x']))
        self.commands[1] = float(np.clip(linear_y * self.user_cmd_cfg['lin_vel_y'], -self.user_cmd_cfg['lin_vel_y'], self.user_cmd_cfg['lin_vel_y']))
        self.commands[2] = float(np.clip(angular_z * self.user_cmd_cfg['ang_vel_yaw'], -self.user_cmd_cfg['ang_vel_yaw'], self.user_cmd_cfg['ang_vel_yaw']))

    def _poll_pygame_joystick(self):
        if not self._pygame_enabled or self._pygame_joystick is None:
            return
        pygame.event.pump()
        joy = self._pygame_joystick
        num_axes = joy.get_numaxes()
        num_btns = joy.get_numbuttons()

        lx = joy.get_axis(self.PYGAME_AXES["left_horizon"]) if num_axes > 0 else 0.0
        ly = joy.get_axis(self.PYGAME_AXES["left_vertical"]) if num_axes > 1 else 0.0
        rx = joy.get_axis(self.PYGAME_AXES["right_horizon"]) if num_axes > 3 else 0.0
        buttons = [joy.get_button(i) if i < num_btns else 0 for i in range(6)]
        axes = [
            -self._deadzone(ly) * 1.0,
            -self._deadzone(lx) * 1.0,
            -self._deadzone(rx) * 1.0,
        ]

        self._process_joystick(buttons, axes, "_pygame_last_r1")
        self.joy_msg_count += 1
        self.last_joy_time = time.time()

    def joystick_alive(self, timeout_sec=2.0):
        if self.joy_msg_count <= 0:
            return False
        return (time.time() - self.last_joy_time) <= float(timeout_sec)

    def update(self):
        """
        Updates the robot's state based on the current mode and publishes the robot command.
        """
        self._poll_pygame_joystick()
        if self.mode == "STAND":
            self.handle_stand_mode()
        elif self.mode == "WALK":
            self.handle_walk_mode()
        elif self.mode == "IDLE":
            self.handle_idle_mode()
        
        # Increment the loop count
        self.loop_count += 1

        # Publish the robot command
        self.robot.publishRobotCmd(self.robot_cmd)

    def handle_idle_mode(self):
        for i in range(self.joint_num):
            self.set_joint_command(i, self.robot_state.q[i], 0, 0, 0, self.control_cfg.get('leg_joint_damping', self.control_cfg.get('damping', 1.0)))

    # Callback function for receiving robot command data
    def robot_state_callback(self, robot_state: datatypes.RobotState):
        """
        Callback function to update the robot state from incoming data.
        
        Parameters:
        robot_state (datatypes.RobotState): The current state of the robot.
        """
        self.robot_state = robot_state
        self.robot_state_received = True

    # Callback function for receiving imu data
    def imu_data_callback(self, imu_data: datatypes.ImuData):
        """
        Callback function to update IMU data from incoming data.
        
        Parameters:
        imu_data (datatypes.ImuData): The IMU data containing stamp, acceleration, gyro, and quaternion.
        """
        self.imu_data.stamp = imu_data.stamp
        self.imu_data.acc = imu_data.acc
        self.imu_data.gyro = imu_data.gyro
        
        # Rotate quaternion values
        self.imu_data.quat[0] = imu_data.quat[1]
        self.imu_data.quat[1] = imu_data.quat[2]
        self.imu_data.quat[2] = imu_data.quat[3]
        self.imu_data.quat[3] = imu_data.quat[0]

    # Callback function for receiving sensor joy data
    def sensor_joy_callback(self, sensor_joy: datatypes.SensorJoy):
        if self._pygame_enabled:
            return
            
        self.joy_msg_count += 1
        self.last_joy_time = time.time()
        
        axes = [
            sensor_joy.axes[self.JOY_AXES["left_vertical"]] if len(sensor_joy.axes) > self.JOY_AXES["left_vertical"] else 0.0,
            sensor_joy.axes[self.JOY_AXES["left_horizon"]] if len(sensor_joy.axes) > self.JOY_AXES["left_horizon"] else 0.0,
            sensor_joy.axes[self.JOY_AXES["right_horizon"]] if len(sensor_joy.axes) > self.JOY_AXES["right_horizon"] else 0.0,
        ]
        
        self._process_joystick(sensor_joy.buttons, axes, "_last_r1")

    # Callback function for receiving diagnostic data
    def robot_diagnostic_callback(self, diagnostic_value: datatypes.DiagnosticValue):
      # Check if the received diagnostic data is related to calibration.
      if diagnostic_value.name == "calibration":
        print(f"Calibration state: {diagnostic_value.code}")
        self.calibration_state = diagnostic_value.code
