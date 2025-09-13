import mujoco
import mujoco.viewer
import time

def main():
    model = mujoco.MjModel.from_xml_path("./robot/scene.xml")
    data = mujoco.MjData(model)
    viewer = mujoco.viewer.launch_passive(model, data)
    
    # Get the free joint ID for the root joint
    root_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    print(f"Root joint ID: {root_joint_id}")
    
    # Set the robot position using qpos (free joint has 7 DOF: 3 pos + 4 quat)
    # Position: [x, y, z, qw, qx, qy, qz]
    data.qpos[root_joint_id:root_joint_id+3] = [0, 0, 1]  # Set position to (0, 0, 1)
    data.qpos[root_joint_id+3:root_joint_id+7] = [1, 0, 0, 0]  # Set quaternion to identity
    
    start_time = time.time()
    while viewer.is_running:
        if time.time() - start_time > 0.01:
            start_time = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    main()
