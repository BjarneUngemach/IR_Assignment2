%% Movement of Tools to UR3 File
clf;
clc;
clear;

swipebot = SwipeBot;


toolChangeTr = transl(0,-0.15,0.35)*troty(-pi)*trotx(pi/2) ; %Didnt recognise from the class for some reason

q_start = swipebot.meca500.getpos();

sponge_pickup = transl(0.1, -0.13, 0.05)*troty(-pi)*trotx(pi/2)

wiper_pickup = transl(-0.1, -0.13, 0.05)*troty(-pi)*trotx(pi/2);

waiting_position = toolChangeTr*transl(0,0,-0.05)

q_toolChange = swipebot.meca500.ikcon(toolChangeTr)
q_waiting_position = swipebot.meca500.ikcon(waiting_position) %Robot lets go of tool and waits here


steps = 50;
delay = 0;

%% Pick up Sponge and move to Exchange
qGuessSponge = [1.8230,   -1.1938,    0.1013,         0,    1.0036,         0]
q_sponge = swipebot.meca500.ikcon(sponge_pickup,qGuessSponge);
pickup_sponge = jtraj(q_start, q_sponge, steps);

for i=1:steps
    swipebot.meca500.animate(pickup_sponge(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    
    pause(0.01);
end

% for 1:10
%     finger1.base = 
%     finger1.animate
%     
%     finger2.base =
%     finger2.animate
% end
% Code for Gripper fingers closing needed here
sponge_exchange = jtraj(q_sponge, q_toolChange, steps);

for i=1:steps
    swipebot.meca500.animate(sponge_exchange(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    swipebot.sponge.base = ee_current*trotx(-pi/2)*trotz(pi/2)*transl(0,0,-0.05)
    swipebot.sponge.animate(0)
    pause(0.01);
    
end

% Code for Gripper fingers opening needed here 
waiting_traj = jtraj(q_toolChange, q_waiting_position, steps);

for i=1:steps
    swipebot.meca500.animate(waiting_traj(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    
    pause(0.01);
    
end

pause(5);

% delay = xx

grab_tool = jtraj(q_waiting_position, q_toolChange, steps);

for i=1:steps
    swipebot.meca500.animate(grab_tool(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    
    pause(0.01);
end

% Gripper grabs the sponge
sponge_dropoff = transl(0.2, -0.15, 0)*troty(-pi)*trotx(pi/2);
q_spongedropoff = swipebot.meca500.ikcon(sponge_dropoff)

return_sponge = jtraj(q_toolChange, q_sponge, steps);

for i=1:steps
    swipebot.meca500.animate(return_sponge(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    swipebot.sponge.base = ee_current*trotx(-pi/2)*trotz(pi/2)*transl(0,0,-0.05)
    swipebot.sponge.animate(0)
    pause(0.01);
    
end


%% Pick up Wiper
qGuessWiper = [1.2732,   -1.1938,    0.1013,         0,    1.0036,         0];
q_wiper = swipebot.meca500.ikcon(wiper_pickup, qGuessWiper);
pickup_wiper = jtraj(q_sponge, q_wiper, steps);

for i=1:steps
    swipebot.meca500.animate(pickup_wiper(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    % Transform tools to end effector here
    pause(0.01);
    
end
q_wiperexchange = swipebot.meca500.ikcon(toolChangeTr, qGuessWiper)
wiper_exchange = jtraj(q_wiper, q_toolChange, steps)

for i=1:steps
    swipebot.meca500.animate(wiper_exchange(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    swipebot.wiper.base = ee_current*trotx(-pi/2)*trotz(pi/2)*transl(0,0,-0.05)
    swipebot.wiper.animate(0)
    pause(0.01);
end

for i=1:steps
    swipebot.meca500.animate(waiting_traj(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    % Transform tools to end effector here
    pause(0.01);
    
end

pause(5);

grab_tool = jtraj(q_waiting_position, q_toolChange, steps);

for i=1:steps
    swipebot.meca500.animate(grab_tool(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    % Transform tools to end effector here
    pause(0.01);
end

return_wiper = jtraj(q_toolChange, q_wiper, steps);

for i=1:steps
    swipebot.meca500.animate(return_wiper(i,:))
    q_current = swipebot.meca500.getpos;
    ee_current = swipebot.meca500.fkine(q_current)
    
    % Transform Gripper to end effector here
    swipebot.wiper.base = ee_current*trotx(-pi/2)*trotz(pi/2)*transl(0,0,-0.05)
    swipebot.wiper.animate(0)
    pause(0.01);
    
end
    




    

