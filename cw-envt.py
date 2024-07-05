import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import creature
import math
from simulation import Simulation

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    def gaussian(x, y, sigma=arena_size/4):
        """Return the height of the mountain at position (x, y) using a Gaussian function."""
        return mountain_height * math.exp(-((x**2 + y**2) / (2 * sigma**2)))

    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = gaussian(x, y)  # Height determined by the Gaussian function

        # Adjust the size of the rocks based on height. Higher rocks (closer to the peak) will be smaller.
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor

        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)



def make_rocks(num_rocks=100, max_size=0.25, arena_size=20):
    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = 0.5  # Adjust based on your needs
        size = random.uniform(0.1,max_size)
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)


def make_arena(arena_size=20, wall_height=10):
    wall_thickness = 0.5
    floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness])
    floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[0, 1, 0, 1])
    floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 1, 1])  # pale blue walls

    # Create four walls
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 1, 1])  # pale blue

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2])


def create_environment():
    p.setGravity(0, 0, -10)
    arena_size = 20
    make_arena(arena_size=arena_size)

   # make_rocks(arena_size=arena_size)
    mountain_position = (0, 0, -1) 
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    p.setAdditionalSearchPath('shapes/')
    # mountain = p.loadURDF("mountain.urdf", mountain_position, mountain_orientation, useFixedBase=1)
    # mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position, mountain_orientation, useFixedBase=1)

    mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)

    return mountain_position

def run_creature_simulation(simulation, cr, iterations=2400):
    simulation.run_creature(cr, iterations)

def main():
    #initialize sim
    sim = Simulation()

    mountain_position = create_environment()
    #generate a random creature
    cr = creature.Creature(gene_count=3)

    # save it to XML
    with open('test.urdf', 'w') as f:
        f.write(cr.to_xml())
    # load it into the sim
    rob1 = p.loadURDF('test.urdf', (0, 8, 5))

    if rob1 < 0:
        print("Error loading robot")
        return

    p.setRealTimeSimulation(1)

    #delay time to let simu initialize
    time.sleep(1.0)

    while True:
        p.stepSimulation()

        #check if creature exist and reaches base of mountain
        if rob1 is not None:
            pos, _ = p.getBasePositionAndOrientation(rob1)
            
            if pos[2] <= mountain_position[2] + 1.0:
                update_motors_at_base(rob1, cr)

        apply_movement_logic(rob1, cr)

        time.sleep(1.0 / 240)

def update_motors_at_base(rob1, cr):
    for jid in range(p.getNumJoints(rob1)):
        # Example: Stop motors or change control strategy
        p.setJointMotorControl2(rob1, jid, controlMode=p.POSITION_CONTROL, targetVelocity=0.0, force=5)


def apply_movement_logic(rob1, cr):
    dt = 1.0 / 240
    timescale = 0.5
    del_time = dt / timescale
    if del_time > 1.0:
        del_time = 1.0

    # Get current and target velocities
    target_linear_velocity = np.array([0.0, 0.0, 0.5])  # Example target to move up
    linear_velocity = np.array(p.getBaseVelocity(rob1)[0])
    new_linear_velocity = (1.0 - del_time) * linear_velocity + del_time * target_linear_velocity


    #prevent creature from moving erratically
    max_speed = 1.0 
    speed = np.linalg.norm(new_linear_velocity)
    if speed > max_speed:
        new_linear_velocity = new_linear_velocity / speed * max_speed

    # Set new linear velocity
    p.resetBaseVelocity(rob1, linearVelocity=new_linear_velocity.tolist())

      # Calculate and set angular velocity
    ground_normal = np.array([0.0, 0.0, 1.0])  # Simplification, replace with actual surface normal
    tangential_velocity = new_linear_velocity - np.dot(new_linear_velocity, ground_normal) * ground_normal
    tangential_speed = np.linalg.norm(tangential_velocity)
    MIN_ROLLING_LINEAR_SPEED = 0.01
    new_angular_velocity = np.array([0.0, 0.0, 0.0])
    if tangential_speed > MIN_ROLLING_LINEAR_SPEED:
        roll_axis = np.cross(ground_normal, tangential_velocity)
        roll_axis = roll_axis / np.linalg.norm(roll_axis)
        angular_speed = tangential_speed / 0.1  # Example rolling radius
        new_angular_velocity = angular_speed * roll_axis

    #prevent from spinning too much
    max_angular_speed = 2.0 
    angular_speed = np.linalg.norm(new_angular_velocity)
    if angular_speed > max_angular_speed:
        new_angular_velocity = new_angular_velocity / angular_speed * max_angular_speed
    
    # Set new angular velocity
    p.resetBaseVelocity(rob1, angularVelocity=new_angular_velocity.tolist())


if __name__ == "__main__":
    main()