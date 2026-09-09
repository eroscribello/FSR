
robotModel = importrobot("phantomx_description-master\urdf\phantomx.urdf");
robotModel.DataFormat = 'column';
robotModel.Gravity = [0 0 -9.81];

leg_robot = subtree(robotModel, 'c1_rr');
leg_robot.DataFormat = 'column';
leg_robot.Gravity = [0 0 -9.81];