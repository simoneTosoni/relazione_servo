function plot_SCARA_trajectory (Qi, Qf, S, L, fig, vista)
  
  % plot della traiettoria e del robot
  h1=figure(fig); % 3D plot of manipulator
  set(h1,'name','3D representation xyz')
  hold on
  grid on
  axis('equal')
  tit = title('Posa e traiettoria manipolatore');
  xlabel('x')
  ylabel('y')
  zlabel('z')
  PlotSCARA(Qi,L,'r',fig)     % Robot in pos. iniz.
  PlotSCARA(Qf,L,'b',fig)   % Robot in pos. fin.
  


  view(vista) %point of view azimut, elevation 
  plot3(S(1,:),S(2,:),S(3,:))
  legend('Robot in Si','Base frame','joints ','Robot in Sf','Base frame ','joints ','trajectory',"Location","best");
  hold off

end



function PlotSCARA(Q,L,colore,fig)

figure(fig) % fig figure number
q1 = Q(1);
q2 = Q(2);
q3 = Q(3);
q4 = Q(4);
h = L(1);
l1 = L(2);
l2 = L(3);
l3 = L(4);

% origine del sistema di rifermento alla base del robot
x1 = 0;
y1 = 0;
z1 = 0;


x2 = 0;
y2 = 0;
z2 = h;

% primo giunto rotoidale 
x3 = l1*cos(Q(1));
y3 = l1*sin(Q(1));
z3 = h;

% secondo giunto rotoidale 
x4 = l1*cos(Q(1))+l2*cos(Q(1)+Q(2));
y4 = l1*sin(Q(1))+l2*sin(Q(1)+Q(2));
z4 = h;

% giunto prismatico traslazione lungo -z 
x5 = l1*cos(Q(1))+l2*cos(Q(1)+Q(2));
y5 = l1*sin(Q(1))+l2*sin(Q(1)+Q(2));
z5 = h-q3;

% quarto giunto rotoidale gripper
x6 = l1*cos(Q(1))+l2*cos(Q(1)+Q(2));
y6 = l1*sin(Q(1))+l2*sin(Q(1)+Q(2));
z6 = h-q3;


hold on
plot3([x1 x2 x3 x4 x5 x6],[y1 y2 y3 y4 y5 y6],[z1 z2 z3 z4 z5 z6], 'LineWidth',2,'color',colore); %manipulator
plot3([0],[0],[0],'*','color','k'); %base
plot3([x2 x3 x4 x5 x6],[y2 y3 y4 y5 y6],[z2 z3 z4 z5 z6], 'o','color',colore); %joint position
end
