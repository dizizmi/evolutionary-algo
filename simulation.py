import pybullet as p
import time
import pybullet_data
import numpy as np
from multiprocessing import Pool
import math

class Simulation: 
    def __init__(self, sim_id=0):
        self.physicsClientId = p.connect(p.DIRECT)
        self.sim_id = sim_id

    def run_creature(self, cr, mountain_position, iterations=2400):
        pid = self.physicsClientId
        p.resetSimulation(physicsClientId=pid)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)

        p.setGravity(0, 0, -10, physicsClientId=pid)
        plane_shape = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=pid)
        floor = p.createMultiBody(plane_shape, plane_shape, physicsClientId=pid)

        xml_file = 'temp' + str(self.sim_id) + '.urdf'
        xml_str = cr.to_xml()
        with open(xml_file, 'w') as f:
            f.write(xml_str)
        
        cid = p.loadURDF(xml_file, physicsClientId=pid)

        # p.resetBasePositionAndOrientation(cid, [0, 0, 2.5], [0, 0, 0, 1], physicsClientId=pid

        for step in range(iterations):
            p.stepSimulation(physicsClientId=pid)
            if step % 24 == 0:
                self.update_motors(cid, mountain_position)

            self.apply_movement_logic(cid, mountain_position)

            pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
            cr.update_position(pos)
            #print(pos[2])
            #print(cr.get_distance_travelled())
        
    #when creature at base of mountain
    def update_motors(self, cid, mountain_position, base_of_mountain_radius=2.0):
        """
        cid is the id in the physics engine
        cr is a creature object
        """
        pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=self.physicsClientId)
        x, y, z = pos

        direction_to_mountain = np.array(mountain_position) - np.array(pos)  # Calculate direction to mountain
        mountain_center = (mountain_position[0], mountain_position[1])

        distance_to_center = np.linalg.norm(np.array([x, y]) - np.array(mountain_center))

        if distance_to_center <= base_of_mountain_radius and z < 0.5:  # Check if at the base of the mountain
            motor_speed = 0.1  # Reduced speed at the base of the mountain

        else:
            motor_speed = 0.5  # Normal speed

        if z > 1.0:  # If the creature is high above the ground
            motor_speed = 0.8

        if x > 0.5:  # If the creature is on the positive side of the x-axis
            motor_speed = 0.3

        for jid in range(p.getNumJoints(cid, physicsClientId=self.physicsClientId)):
            if jid % 2 == 0:
                target_position = motor_speed * np.sin(time.time() * 1.0)
            else:
                target_position = -motor_speed * np.sin(time.time() * 1.0)

            p.setJointMotorControl2(cid, jid,
                                controlMode=p.POSITION_CONTROL,
                                targetVelocity=target_position,
                                force=5,
                                physicsClientId=self.physicsClientId)
    
    def apply_movement_logic(self, cid, mountain_position):

        pos, _ = p.getBasePositionAndOrientation(cid)
        direction_to_mountain = np.array(mountain_position) - np.array(pos)

        dt = 1.0 / 240
        timescale = 0.5
        del_time = dt / timescale
        if del_time > 1.0:
            del_time = 1.0

        target_linear_velocity = direction_to_mountain / np.linalg.norm(direction_to_mountain) * 0.5  # Move towards the mountain
        linear_velocity = np.array(p.getBaseVelocity(cid)[0])
        new_linear_velocity = (1.0 - del_time) * linear_velocity + del_time * target_linear_velocity

        max_speed = 1.0
        speed = np.linalg.norm(new_linear_velocity)
        if speed > max_speed:
            new_linear_velocity = new_linear_velocity / speed * max_speed

        p.resetBaseVelocity(cid, linearVelocity=new_linear_velocity.tolist())

        ground_normal = np.array([0.0, 0.0, 1.0])
        tangential_velocity = new_linear_velocity - np.dot(new_linear_velocity, ground_normal) * ground_normal
        tangential_speed = np.linalg.norm(tangential_velocity)
        MIN_ROLLING_LINEAR_SPEED = 0.01
        new_angular_velocity = np.array([0.0, 0.0, 0.0])
        if tangential_speed > MIN_ROLLING_LINEAR_SPEED:
            roll_axis = np.cross(ground_normal, tangential_velocity)
            roll_axis = roll_axis / np.linalg.norm(roll_axis)
            angular_speed = tangential_speed / 0.1  # Example rolling radius
            new_angular_velocity = angular_speed * roll_axis

        max_angular_speed = 2.0
        angular_speed = np.linalg.norm(new_angular_velocity)
        if angular_speed > max_angular_speed:
            new_angular_velocity = new_angular_velocity / angular_speed * max_angular_speed

        p.resetBaseVelocity(cid, angularVelocity=new_angular_velocity.tolist())


    # You can add this to the Simulation class:
    def eval_population(self, pop, iterations):
        for cr in pop.creatures:
            self.run_creature(cr, iterations) 


class ThreadedSim():
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at 240fps
        """
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):# the end
                    break
                # work out the sim ind
                sim_ind = i % len(self.sims)
                this_pool_args.append([
                            self.sims[sim_ind], 
                            pop.creatures[i], 
                            iterations]   
                )
            pool_args.append(this_pool_args)
            start_ind = start_ind + pool_size

        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                # it works on a copy of the creatures, so receive them
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                # and now put those creatures back into the main 
                # self.creatures array
                new_creatures.extend(creatures)
        pop.creatures = new_creatures
