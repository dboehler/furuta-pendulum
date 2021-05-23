%initialize API and connect to server (CoppeliaSim) in synchronous mode
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
%Set synchronous mode so that model will wait for MATLAB script
sim.simxSynchronous(clientID,true);

%get object handles for joint1 (rotor arm) and joint2 (pendulum arm).
[~, rot_joint] = sim.simxGetObjectHandle(clientID,'rotor_joint',sim.simx_opmode_blocking);
[~, rot_arm] = sim.simxGetObjectHandle(clientID,'rotor_arm',sim.simx_opmode_blocking);
[~, pend_joint] = sim.simxGetObjectHandle(clientID,'pendulum_joint',sim.simx_opmode_blocking);
[~, pend_arm] = sim.simxGetObjectHandle(clientID,'pendulum_arm',sim.simx_opmode_blocking);
sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
%This line is for debugging, to test the model's response
%returnCode = sim.simxSetJointMaxForce(clientID,rot_joint,50,sim.simx_opmode_oneshot);

%Run 100 steps of simulation in Coppelia
for i=1:100
    %Get position and velocity of the rotator joint and position and
    %velocity of the pendulum joint. These are the 4 state variables.
    [~,theta0] = sim.simxGetJointPosition(clientID,rot_joint,sim.simx_opmode_blocking);    
    [~,~,rot_arm_vel] = sim.simxGetObjectVelocity(clientID,rot_arm,sim.simx_opmode_blocking);
    [~,theta1] = sim.simxGetJointPosition(clientID,pend_joint,sim.simx_opmode_blocking);    
    [~,~,pend_arm_vel] = sim.simxGetObjectVelocity(clientID,pend_arm,sim.simx_opmode_blocking);
    %extract the meaningful velocity measurements (rot_arm_vel and 
    %pend_arm_vel are 1x3 arrays of 3D coordinates). 
    dtheta0 = rot_arm_vel(3);
    dtheta1 = pend_arm_vel(1);
    
    disp([theta0, dtheta0, theta1, dtheta1])
    torque = controller(theta0,dtheta0,theta1,dtheta1);
    %To implement torque control in Coppelia, an arbitrary large target
    %velocity is chosen and the maximum force allowed is the desired
    %torque. The direction of the large velocity must be changed to
    %simulate negate torque. This is the reason for the if/else logic.
    if(torque>0)
        returnCode = sim.simxSetJointTargetVelocity(clientID,rot_joint,1e5,sim.simx_opmode_blocking);
        returnCode = sim.simxSetJointMaxForce(clientID,rot_joint,torque,sim.simx_opmode_blocking);
        
    else
        returnCode = sim.simxSetJointTargetVelocity(clientID,rot_joint,-1e5,sim.simx_opmode_blocking);
        returnCode = sim.simxSetJointMaxForce(clientID,rot_joint,-torque,sim.simx_opmode_blocking);
    end
    %Trigger the next simulation step
    sim.simxSynchronousTrigger(clientID);
end
%After 100 steps, pause the simulation and disconnect from the API
sim.simxPauseSimulation(clientID,sim.simx_opmode_blocking);
sim.simxFinish(clientID);
sim.delete();