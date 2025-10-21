import time
import mujoco
import mujoco.viewer
import numpy as np
import os
import sys
import os
import threading

if os.name == 'nt':
    import msvcrt
else:
    import select, termios, tty



current_dir = os.path.dirname(os.path.abspath(__file__))
if os.name == 'nt':
    target_path = os.path.join(current_dir, "build", "Release")
else:
    target_path = os.path.join(current_dir, "build")
sys.path.append(target_path)
from cRoboticsController_wrapper_cpp import cRoboticsController as cRoboticsControllerCPP
    

def precise_sleep(duration):
    start = time.perf_counter()
    while True:
        now = time.perf_counter()
        if (now - start) >= duration:
            break

class RobotController():
    def __init__(self, manipulator_control_mode: str):

        self.manipulator_control_mode = manipulator_control_mode

        # Load the model
        xml_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "franka_fr3", self.manipulator_control_mode+"_scene.xml")
        self.urdf_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "franka_fr3", "fr3.urdf")
        self.mujoco_model = mujoco.MjModel.from_xml_path(xml_file_path)
        
        self.dt = self.mujoco_model.opt.timestep

        self.setup_controller()
        
        self.mujoco_data = mujoco.MjData(self.mujoco_model)
        
        # Initial Joint space state
        self.q = np.zeros(len(self.mujoco_mani_joint_names))
        self.qdot = np.zeros(len(self.mujoco_mani_joint_names))
        self.tau = np.zeros(len(self.mujoco_mani_joint_names))
                
        # Manipulator desired positions initialized
        self.q_desired = np.zeros(len(self.mujoco_mani_joint_names))
        self.tau_desired = np.zeros(len(self.mujoco_mani_joint_names))

        self.viewer_fps = 60
        self.print_fps = 2

        self.paused = False
        self.quit = False
        
        if os.name != 'nt':
            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        
        keyThread = threading.Thread(target=self.keyCallback)
        keyThread.daemon = True 
        keyThread.start()
        
    def setup_controller(self):
        self.controller = cRoboticsControllerCPP(self.urdf_file_path, self.manipulator_control_mode, self.dt)
        joint_names = [self.mujoco_model.names[self.mujoco_model.name_jntadr[i]:].split(b'\x00', 1)[0].decode('utf-8') for i in range(self.mujoco_model.njnt) if self.mujoco_model.names[self.mujoco_model.name_jntadr[i]:].split(b'\x00', 1)[0].decode('utf-8')]
        self.q_names = joint_names
        self.v_names = joint_names
        self.actuator_names = [self.mujoco_model.names[self.mujoco_model.name_actuatoradr[i]:].split(b'\x00', 1)[0].decode('utf-8') for i in range(self.mujoco_model.nu)]
        self.mujoco_mani_joint_names = [name for name in self.actuator_names if 'joint' in name]
        
    def updateJointState(self):
        # Get manipulator angle positions, velocities and torques from mujoco
        for i, mani_joint_name in enumerate(self.mujoco_mani_joint_names):
            self.q[i] = self.mujoco_data.qpos[self.q_names.index(mani_joint_name)]
            self.qdot[i] = self.mujoco_data.qvel[self.v_names.index(mani_joint_name)]
            self.tau[i] = self.mujoco_data.qfrc_actuator[self.v_names.index(mani_joint_name)] + \
                        self.mujoco_data.qfrc_applied[self.v_names.index(mani_joint_name)] + \
                        self.mujoco_data.qfrc_constraint[self.v_names.index(mani_joint_name)]
            
    def updateModel(self, q:np.array, qdot:np.array, tau:np.array):
        self.controller.updateModel(q, qdot, tau)
    
    def keyCallback(self):
        poll_interval = 0.01
        while True:
            if os.name == 'nt':
                if msvcrt.kbhit():
                    keycode = msvcrt.getwch()
                else:
                    time.sleep(poll_interval)
                    continue
            else:
                if select.select([sys.stdin], [], [], 0)[0]:
                    keycode = sys.stdin.read(1)
                else:
                    time.sleep(poll_interval)
                    continue

            if keycode == ' ':
                self.paused = not self.paused
            elif keycode in ('q', 'Q'):
                self.quit = True
            else:
                self.controller.keyMapping(ord(keycode))
            
    def run(self):
        with mujoco.viewer.launch_passive(self.mujoco_model, self.mujoco_data, show_left_ui=False, show_right_ui=False) as viewer:
            last_viewr_update_time = 0
            last_print_update_time = time.perf_counter()
            while viewer.is_running() and not self.quit:
                if not self.paused:

                    #  ------------------------
                    step_start = time.perf_counter()
                    mujoco.mj_step(self.mujoco_model, self.mujoco_data)

                    self.updateJointState()
                    self.updateModel(self.q, self.qdot, self.tau)
                    self.play_time = self.mujoco_data.time

                    if time.perf_counter() - last_print_update_time > 1/self.print_fps:
                        self.controller.printState()
                        last_print_update_time = time.perf_counter()

                    self.controller.compute(self.play_time)
                    if self.manipulator_control_mode == "position":
                        self.q_desired = self.controller.getCtrlInput()
                    elif  self.manipulator_control_mode == "torque":
                        self.tau_desired = self.controller.getCtrlInput()

                    for i, mani_joint_name in enumerate(self.mujoco_mani_joint_names):
                        self.mujoco_data.ctrl[self.actuator_names.index(mani_joint_name)] = self.q_desired[i] if self.manipulator_control_mode == "position" \
                                                                                                              else self.tau_desired[i]


                    if self.mujoco_data.time - last_viewr_update_time > 1/self.viewer_fps:
                        viewer.sync()
                        last_viewr_update_time = self.mujoco_data.time

                    time_until_next_step = self.dt - (time.perf_counter() - step_start)
                    if time_until_next_step > 0:
                        precise_sleep(time_until_next_step)
