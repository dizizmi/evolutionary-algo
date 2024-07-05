import genome
import creature
import pybullet as p
import time 
import random
import numpy as np
from simulation import Simulation

## ... usual starter code to create a sim and floor
p.connect(p.GUI)
p.setPhysicsEngineParameter(enableFileCaching=0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
plane_shape = p.createCollisionShape(p.GEOM_PLANE)
floor = p.createMultiBody(plane_shape, plane_shape)
p.setGravity(0, 0, -10)
# commenting out for mac compatibility
p.setRealTimeSimulation(1)

# generate a random creature
cr = creature.Creature(gene_count=3)
# save it to XML
with open('test.urdf', 'w') as f:
    f.write(cr.to_xml())
# load it into the sim

initial_position = (0,0,0.5)
rob1 = p.loadURDF('test.urdf')
start_pos, orn = p.getBasePositionAndOrientation(rob1)

# iterate 
elapsed_time = 0
wait_time = 1.0 / 240 # seconds
total_time = 5 # seconds
step = 0
dist_moved = 0

#fitness function
intial_height = start_pos[2]
max_height_reached = intial_height



while True:
    p.stepSimulation()
    step += 1

    #update motors every 120 steps
    if step % 120 == 0:
        motors = cr.get_motors()

        assert len(motors) == p.getNumJoints(rob1), "Something went wrong"
        for jid in range(p.getNumJoints(rob1)):
            mode = p.POSITION_CONTROL
            vel = motors[jid].get_output()
            p.setJointMotorControl2(rob1, 
                jid,  
                controlMode=mode, 
                targetVelocity=vel)
        new_pos, orn = p.getBasePositionAndOrientation(rob1)
        #print(new_pos)
        dist_moved = np.linalg.norm(np.asarray(start_pos) - np.asarray(new_pos))

        #track max height reached
        if new_pos[2] > max_height_reached:
            max_height_reached = new_pos[2]

    #calculate fitness
    fitness_score = max_height_reached - intial_height
    cr.set_fitness_score(fitness_score) #for creature

    print("Fitness Score: ", fitness_score)

    time.sleep(wait_time)
    elapsed_time += wait_time
    if elapsed_time > total_time:
        break

print("TOTAL DISTANCE MOVED:", dist_moved)

