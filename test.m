r = LinearDobotMagician(SE3(0.25,0.25,0).T);
set(gcf, 'Windowstyle', 'docked');
axis equal;
r.model.teach(r.model.getpos())