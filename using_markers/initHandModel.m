addpath(fileparts(pwd));
addpath('rvctools/');
run startup_rvc.m
deg = pi/180;
% ignore 'qlim'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load handParameter.mat;


% set some default plot options, base and shadow are not useful for a multi-arm plot
plotopts = { 'noshadow' , 'nobase'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%% create each finger
 thumb = SerialLink( [
    Revolute('d', 0, 'alpha', -pi/2, 'a', 0, 'offset',  0)
    Revolute('d', 0, 'alpha', pi/2, 'a', th_boneL_2,   'offset',  30*deg)    
    Revolute('d', 0, 'alpha', -pi/2, 'a', 0)
    Revolute('d', 0, 'alpha', 0,  'a', th_boneL_3)
    Revolute('d', 0, 'alpha', 0, 'a', th_boneL_4)
    ], ...
    'base', transl(mean_fixed_pos(5,:))*trotz(-0*deg)*trotx(180*deg), ...
    'plotopt',plotopts, ...
    'qlim', [-35 90; -35 90; -30 30; -5 90; -5 90]*deg, ...
    'name', 'thumb', 'manufacturer', 'sli');
%KDL::Chain chain_thu1;
       %chain_thu1.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.0, -M_PI_2 , 0.0, 0.0)));
      % chain_thu1.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.061, M_PI_2, 0.0, -(M_PI_2)/3)));
      % chain_thu1.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.0, -M_PI_2, 0.0, 0)));
      % chain_thu1.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.0461,0.0,0.0,0.0)));
      %chain_thu1.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.0237, 0.0, 0.0, 0.0)));


%%% Index
index = SerialLink( [
    Revolute('d', 0, 'alpha', -pi/2,  'a', 0, 'offset', 180*deg)   
    Revolute('d', 0, 'alpha', pi/2,  'a', 0, 'offset', -90*deg)  
    Revolute('d', 0, 'alpha', 0, 'a', in_boneL_2)
    Revolute('d', 0, 'alpha', 0, 'a', in_boneL_3)
    Revolute('d', 0, 'alpha', 0, 'a', in_boneL_4)


    ], ...
    'base', transl(mean_fixed_pos(4,:))*trotz(fingers_angles(1))*trotx(0) , ...
    'plotopt',plotopts, ...
    'qlim', [-10 10; -50 50; -15 100; -5 100; -5 100]*deg, ...
    'name', 'index', 'manufacturer', 'sli');


%%% Middle
middle = SerialLink( [
    Revolute('d', 0, 'alpha', -pi/2,  'a', 0, 'offset', 180*deg)   
    Revolute('d', 0, 'alpha', pi/2,  'a', 0, 'offset', -90*deg)  
    Revolute('d', 0, 'alpha', 0, 'a', mi_boneL_2)
    Revolute('d', 0, 'alpha', 0, 'a', mi_boneL_3)
    Revolute('d', 0, 'alpha', 0, 'a', mi_boneL_4)
    ], ...
    'base', transl(mean_fixed_pos(3,:))*trotz(fingers_angles(2))*trotx(0) , ...
    'plotopt',plotopts, ...   
    'qlim', [-10 10; -50 50; -15 100; -5 100; -5 100]*deg, ...
    'name', 'middle', 'manufacturer', 'sli');



%%% Ring
ring = SerialLink( [
    Revolute('d', 0, 'alpha', -pi/2,  'a', 0, 'offset', 180*deg)   
    Revolute('d', 0, 'alpha', pi/2,  'a', 0, 'offset', -90*deg)  
    Revolute('d', 0, 'alpha', 0, 'a', ri_boneL_2)
    Revolute('d', 0, 'alpha', 0, 'a', ri_boneL_3)
    Revolute('d', 0, 'alpha', 0, 'a', ri_boneL_4)
    ], ...
    'base', transl(mean_fixed_pos(2,:))*trotz(fingers_angles(3))*trotx(0) , ...
    'plotopt',plotopts, ...
    'qlim', [-10 10; -50 50; -15 100; -5 100; -5 100]*deg, ...
    'name', 'ring', 'manufacturer', 'sli');



%%% Little
little = SerialLink( [
    Revolute('d', 0, 'alpha', -pi/2,  'a', 0, 'offset', 180*deg)   
    Revolute('d', 0, 'alpha', pi/2,  'a', 0, 'offset', -90*deg)  
    Revolute('d', 0, 'alpha', 0, 'a', li_boneL_2)
    Revolute('d', 0, 'alpha', 0, 'a', li_boneL_3)
    Revolute('d', 0, 'alpha', 0, 'a', li_boneL_4)
    ], ...
     'base', transl(mean_fixed_pos(1,:))*trotz(fingers_angles(4))*trotx(0) , ...
    'plotopt',plotopts, ...
    'qlim', [-10 10; -50 50; -15 100; -5 100; -5 100]*deg, ...
    'name', 'little', 'manufacturer', 'sli');

%visualization with all joint angles = 0 deg
clf;
plotrange = 0.3;
thumb.plot([0 0 0 0 0]*deg, 'nobase','noshadow', 'notiles');
                                  joint_lit1(0) = double(little_1);
                                  joint_lit1(1) = double(little_2);
hold on;
axis(plotrange*[-1 1 -1 1 -1 1]);
index.plot([0 0 0 0 0]*deg, 'nobase','noshadow', 'notiles');
middle.plot([0 0 0 0 0]*deg, 'nobase','noshadow', 'notiles');
ring.plot([0 0 0 0 0]*deg, 'nobase','noshadow', 'notiles');
little.plot([0 0 0 0 0]*deg, 'nobase','noshadow', 'notiles');



hand = {thumb, index, middle, ring, little};













