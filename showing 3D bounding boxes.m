myimg=imread('your 3D image needed to be labeling .png .jpg etc.');
myimg=imresize(myimg,[375,1242]);%accorind to the resolution of the camera
fig = figure(1);
set(fig,'position',[200,100,0.8*size(myimg,2),0.8*size(myimg,1)]);
h(1).axes = axes('position',[0,0,1,1]);
%h(2).axes = axes('position',[0,0,1,0.5]);
imshow(myimg,'parent',h(1).axes); 
axis(h(1).axes,'image','off'); 
hold(h(1).axes, 'on');
P=[721.54,       0,        609.5;% camera intrinsic matrix
     0,       721.54,     172.85;
     0,          0,        1    ];
  % load labels
  %objects = readLabels(label_dir,img_idx);

   objects(1).type='Car';
   objects(1).truncation=0;%
   objects(1).occlusion=0;%
   objects(1).alpha=1.9;%
   
   objects(1).h=1.50;%
   objects(1).w=1.48;%
   objects(1).l=3.90;%
   
%    objects(1).x1=299;%70;%804 1004
%    objects(1).y1=183;%191;%407 408
%    objects(1).x2=465;%387;%945 1194
%    objects(1).y2=296;%360;%499 497
  t_x=0;
  t_z=10;
  ry=3.14/2;
while(1)
  cla(h(1).axes); 
  %h(1).axes = axes('position',[0,0,1,1]);
  imshow(myimg,'parent',h(1).axes);
  axis(h(1).axes,'image','off'); 
  hold(h(1).axes, 'on');
  

  key = get(gcf,'CurrentCharacter');
  
  switch lower(key)                         
    case 'w',  t_z = t_z+0.01;       %box going foward
    case 's',  t_z = t_z-0.01;       % backward
    case 'a',  t_x = t_x-0.01;      %left
    case 'd',  t_x = t_x+0.01;      %right
    case 'q',  ry=ry+0.01;          %yaw is increased
    case 'e',  ry=ry-0.01;      
      otherwise
          t_x,t_z,ry
  end

   objects(1).t=[t_x,...
                 1.65,...%
                 t_z];%16.0000
   objects(1).ry=-ry;
    
    % compute 3D bounding box coordinate
   [corners_3D,corners,face_idx] = computeBox3D(objects(1),P);




    drawBox3D(h,objects(1),corners,face_idx,0);
      waitforbuttonpress; 
   % drawBox2D(h,objects(1));
end

 function drawBox3D(h,object,corners,face_idx,orientation)
  % set styles for occlusion and truncation
 occ_col    = {'b','y','r','w'};
 trun_style = {'-','--'};
 trc        = double(object.truncation>0.1)+1;
  
  % draw projected 3D bounding boxes
  %if ~isempty(corners)
    for f=1:4
      line([corners(1,face_idx(f,:)),corners(1,face_idx(f,1))]+1,...
           [corners(2,face_idx(f,:)),corners(2,face_idx(f,1))]+1,...
           'parent',h(1).axes, 'color',occ_col{object.occlusion+1},...
           'LineWidth',2,'LineStyle',trun_style{trc});
%       line([corners(1,face_idx(f,:)),corners(1,face_idx(f,1))]+1,...
%            [corners(2,face_idx(f,:)),corners(2,face_idx(f,1))]+1,...
%            'parent',h(2).axes,'color','b','LineWidth',3);
    end
  %end
  
  % draw orientation vector
%   if ~isempty(orientation)
%     line([orientation(1,:),orientation(1,:)]+1,...
%          [orientation(2,:),orientation(2,:)]+1,...
%          'parent',h.axes,'color','w','LineWidth',4);
%     line([orientation(1,:),orientation(1,:)]+1,...
%          [orientation(2,:),orientation(2,:)]+1,...
%          'parent',h.axes,'color','k','LineWidth',2);
%   end
 end
    
function [corners_3D,corners_2D,face_idx] = computeBox3D(object,P)
% takes an object and a projection matrix (P) and projects the 3D
% bounding box into the image plane.

% index for 3D bounding box faces
face_idx = [ 1,2,6,5   % front face
             2,3,7,6   % left face
             3,4,8,7   % back face
             4,1,5,8]; % right face
%object.ry=1.55;
% compute rotational matrix around yaw axis
R = [+cos(object.ry), 0, +sin(object.ry);
                   0, 1,               0;
     -sin(object.ry), 0, +cos(object.ry)];

% 3D bounding box dimensions
l = object.l;
w = object.w;
h = object.h;
% ============added by WangWei car template(s)============

% 3D bounding box corners
x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2];
y_corners = [  0,   0,    0,    0,  -h,  -h,   -h,   -h];
z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2];

% rotate and translate 3D bounding box
corners_3D = R*[x_corners;y_corners;z_corners];
corners_3D(1,:) = corners_3D(1,:) + object.t(1);
corners_3D(2,:) = corners_3D(2,:) + object.t(2);
corners_3D(3,:) = corners_3D(3,:) + object.t(3);

% only draw 3D bounding box for objects in front of the camera
% if any(corners_3D(3,:)<0.1) 
%   corners_2D = [];
%   return;
% end

% project the 3D bounding box into the image plane
corners_2D = projectToImage(corners_3D, P);
end

function pts_2D = projectToImage(pts_3D, P)
% PROJECTTOIMAGE projects 3D points in given coordinate system in the image
% plane using the given projection matrix P.
%
% Usage: pts_2D = projectToImage(pts_3D, P)
%   input: pts_3D: 3xn matrix
%          P:      3x4 projection matrix
%   output: pts_2D: 2xn matrix
%
% last edited on: 2012-02-27
% Philip Lenz - lenz@kit.edu


  % project in image
  %pts_2D = P * [pts_3D; ones(1,size(pts_3D,2))];
  pts_2D = P * [pts_3D];
  % scale projected points
  pts_2D(1,:) = pts_2D(1,:)./pts_2D(3,:);
  pts_2D(2,:) = pts_2D(2,:)./pts_2D(3,:);
  pts_2D(3,:) = [];
end
