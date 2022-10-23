           % simulate meca 500 to bring the sign
            %Location of sign 
            swipebot = SwipeBot;
            steps = 100
            
            current_q = swipebot.meca500.getpos;
            
            sign_pickup = transl(0.1,0.15,0.155)*trotx(pi)
            
            sign_q_guess = [1.5718   -0.0107   -0.6698         0   -0.8884         0];
            
            sign_pickup_q = swipebot.meca500.ikcon(sign_pickup, sign_q_guess)
            
            pickup_sign = jtraj(current_q, sign_pickup_q, steps)
            
            for i=1:steps
                swipebot.meca500.animate(pickup_sign(i,:))
                swipebot.UpdateGripper2F
                 
                pause(0.01);
            end
             
            
       
            tr = swipebot.toolChangeTr * swipebot.gripperMeca500offset;
            qTr = swipebot.meca500.ikcon(tr,deg2rad([0 -18 18 0 0 0]));
            toolChange_traj = jtraj(sign_pickup_q, qTr, steps)
            
            for i=1:steps
                
                swipebot.meca500.animate(toolChange_traj(i,:))
                swipebot.UpdateGripper2F
                
                ee_current = swipebot.meca500.fkine(swipebot.meca500.getpos)
                swipebot.sign.base = ee_current*transl(0.03685, 0, 0.051)*troty(-pi/2)
                swipebot.sign.animate(0)
                pause(0.01)
                
            end
            
            
            % gain
            lambda = 1;
            
            % servoing speed
            fps = 25;
            
            % Image goal points
            goalPoints = [626, 398, 398, 626;        % joint state [-0.0000   -1.2852   -1.7299   -1.6972    1.5708   -0.0000]
                          389, 389, 617, 617];
%             PStar = [657, 399, 394, 655;        % joint state [0.8330   -1.3682   -2.2793   -1.5167    1.1626    0.9093]
%                      381, 340, 699, 668];

            % corner points of sign in local coordinate frame
            cornerPointsLocal = [ 0.05,-0.05,-0.05, 0.05;
                                 -0.05,-0.05, 0.05, 0.05;
                                  0   , 0   , 0   , 0   ];
           
            %camera
            cam = CentralCamera('focal', 0.10, 'pixel', 10e-5, ...
                                'resolution', [1024 1024], 'centre', [512 512],...
                                'name', 'IBVS Camera');   
%             cam.plot(goalPoints, '*');
            cam.hold;
            
            % initial values
            qUR3 = swipebot.ur3.getpos;
            qRef = zeros(1,6);
%             cornerPoints = zeros(4,3);
%             middlePoint = zeros(3,1);
            
%             while app.RunIBVSButton.Value == 1
%                 eePos = swipebot.ur3.fkine(qUR3);                % get current position
%                 cam.T = eePos;
%                 
%                 qMeca500 = swipebot.meca500.getpos;
%                 if any(abs(qRef - qMeca500) > 0.001)
%                     qRef = qMeca500;
%                     signPos = swipebot.meca500.fkine(qMeca500)/swipebot.gripperMeca500offset; 
%                     middlePoint = signPos(1:3,4);
%                     cornerPoints = [signPos * [cornerPointsLocal',ones(size(cornerPointsLocal',1),1)]']';
%                 end
% 
%                 picturePoints = cam.plot(cornerPoints(:,1:3)', 'Tcam', eePos, 'o');        % calculate error
%                 pointError = goalPoints - picturePoints;
%                 pointError = pointError(:);
%                 
%                 depth = sqrt(sum((middlePoint - eePos(1:3,4)).^2));        % calculate camera distance
%                 
%                 IJ = cam.visjac_p(picturePoints, depth);                % get Image Jacobian
%                 
%                 camVel = lambda * pinv(IJ) * pointError;                % solve for camera velocity
%                 
%                 J = swipebot.ur3.jacobn(qUR3);                                 % Robot Jacobian in tool frame
%                 
%                 qdot = pinv(J) * camVel;                                % solve RMRC
%                 qUR3 = qUR3 + (1/fps)*qdot';
%                 
%                 swipebot.ur3.animate(qUR3);                                    % update robot
%                 swipebot.UpdateGripper3F;
%                 
%                 drawnow;
%                 pause(1/fps);
%             end
            
            % simulate meca 500 to bring back the sign
            signreturn_traj = jtraj(qTr, sign_pickup_q, steps)
            
            for i=1:steps
                swipebot.meca500.animate(signreturn_traj(i,:))
                swipebot.UpdateGripper2F
                
                ee_current = swipebot.meca500.fkine(swipebot.meca500.getpos)
                swipebot.sign.base = ee_current*transl(0.03685, 0, 0.051)*troty(-pi/2)
                swipebot.sign.animate(0)
                pause(0.01)
                
            end
                
            swipebot.meca500.animate(qTr);
            swipebot.UpdateGripper2F;