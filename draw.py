from controller import Supervisor, Motor, Robot,PositionSensor
import math
import numpy as np
# Initialize the robot and supervisor
robot=Supervisor()


# Get the timestep from the robot
timestep = int(robot.getBasicTimeStep())

# Write joint names for the Panda robot 
joint_names = [
    "panda_joint1", "panda_joint2", "panda_joint3", 
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
]


# Create a list to store the joint motors
joints=[]
sensors = []

# Set initial speed and position for joint motors
for name in joint_names:
    joint = robot.getDevice(name)  # Get the motor device for the joint
    joint.setPosition(float('inf'))  # Set joint to position control mode (infinite position control)
    joint.setVelocity(0.25)
    
    joints.append(joint)  # Add the joint motor to the list
    
    
    
    sensor=joint.getPositionSensor()
    sensor.enable(timestep)
    sensors.append(sensor)

# This function makes a list of necessary x and z values to draw 'CU'
def find_xz_list():
    x_1= np.linspace(0.6,0.6,5)
    z_1 = np.linspace(0.95,1.1,5)
    
    angles=np.linspace(np.pi/2,3*np.pi/2,50)
    angles=np.linspace(np.pi/2,3*np.pi/2,50)

    x_2=0.3*np.cos(angles)+0.6
    z_2=0.4*np.sin(angles)+0.7

    x_3= np.linspace(0.6,0.6,5)
    z_3 = np.linspace(0.3,0.45,5)
    
    x_4=0.2*np.cos(angles)+0.6
    z_4=-0.25*np.sin(angles)+0.7
    
    angles_u=np.linspace(np.pi,2*np.pi,50)
    ux1=np.linspace(0.8,0.7,5)
    ux2=np.linspace(0.7,0.7,5)
    ux3=0.2*np.cos(angles_u)+0.9
    ux4=np.linspace(1.1,1.1,5)
    ux5=np.linspace(1.1,1.0,5)
    ux6=np.linspace(1.0,1.0,5)
    ux7=-0.1*np.cos(angles_u)+0.9
    ux8=np.linspace(0.8,0.8,5)
    
    
    uz1=np.linspace(1.1,1.1,5)
    uz2=np.linspace(1.1,0.6,5)
    uz3=0.3*np.sin(angles_u)+0.6
    uz4=np.linspace(0.6,1.1,5)
    uz5=np.linspace(1.1,1.1,5)
    uz6=np.linspace(1.1,0.6,5)
    uz7=0.15*np.sin(angles_u)+0.6
    uz8=np.linspace(0.6,1.1,5)

    x=np.concatenate((x_1,x_2,x_3,x_4,[np.nan],ux1,ux2,ux3,ux4,ux5,ux6,ux7,ux8))
    z=np.concatenate((z_1,z_2,z_3,z_4,[np.nan],uz1,uz2,uz3,uz4,uz5,uz6,uz7,uz8))

#This function moves the joints into the correct position
def set_joints(target_positions,tolerance):
   
   #move the joints toward the target positions
   for joint, target in zip(joints, target_positions):
        joint.setPosition(target)  
   
   #check to see how close the joints are to target positions
   errors = [abs(sensor.getValue() - target) for sensor, target in zip(sensors, target_positions)]
   
        
    # Check if all errors are within tolerance. If yes return True 
   if all(error < tolerance for error in errors):
       print("All joints reached target positions!")
       return True
    
    
    
# Run the main function
def main():
    #Define tolerance for error in joint positions (used in setjoints function)
    tolerance=0.01
    while robot.step(timestep) != -1:
        
        target_positions = [0.5, -0.5, 0.3, -0.7, 0.1, 0.0, 0.0]  # Desired joint angles in radians(Just an example)
        
        if set_joints(target_positions,tolerance):
            for i,sensor in enumerate(sensors):
                print(i,sensor.getValue())
                #The next part we need to do is use inverse kinematics to find the necessary joint positions
                #Then we will update the neew joint positions and iterate again 
                
                #target_positions=[set new target positions here]
        
        
        
        pass  # Let the simulation run
      
main()
        
 
