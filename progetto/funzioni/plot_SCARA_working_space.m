function plot_SCARA_working_space()
% 

figure(1)
plot3([-1.5 1.5],[0 0],[0 0],'k')
hold on
plot3([0 0],[-1.5 1.5],[0 0],'k')
Q=[pi/2 0 0.15 0];
geometria_SCARA

% plotto il centro dell'area di lavoro
S=SCARAdir(Q,L);
plot3(S(1),S(2),S(3),'r*')

PlotSCARA(Q,L,'b',1)
grid on
axis('equal')
view([0 90])
title("Area di lavoro SCARA (piano XY)")


% plot piano xy
% calcolo coordinate max ai giunti 
Q1max = pi/2 + 300/180*pi/2; 
Q1min = pi/2 - 300/180*pi/2; 

Q2max = + 300/180*pi/2;
Q2min = - 300/180*pi/2;

% PlotSCARA([Q1max Q2max Q(3) Q(4)],L,'b',1)
% PlotSCARA([Q1min Q2min Q(3) Q(4)],L,'b',1)

for i=1:101
    q1(i) = Q1min+(Q1max-Q1min)/100*(i-1);
    s1(:,i) = SCARAdir([q1(i) Q(2) Q(3) Q(4)],L)

    q2(i) = Q2min+(0-Q2min)/100*(i-1);
    s2(:,i) = SCARAdir([Q1min q2(i) Q(3) Q(4)],L)
    
    q3(i) = +(Q2max-0)/100*(i-1);
    s3(:,i) = SCARAdir([Q1max q3(i) Q(3) Q(4)],L)
   
end


plot3(s1(1,:),s1(2,:),s1(3,:),'r')
plot3(s2(1,:),s2(2,:),s2(3,:),'r')
plot3(s3(1,:),s3(2,:),s3(3,:),'r')

% plot dell'arco interno
for i=1:101
    q4(i) = Q1min+(Q1max-Q1min)/100*(i-1);
    s4(:,i) = SCARAdir([q4(i) Q2min Q(3) Q(4)],L);

    q5(i) = Q1min+(Q1max-Q1min)/100*(i-1);
    s5(:,i) = SCARAdir([q5(i) Q2max Q(3) Q(4)],L);
end

plot3(s4(1,:),s4(2,:),s4(3,:),'r')
plot3(s5(1,:),s5(2,:),s5(3,:),'r')

legend("","","centro area di lavoro","SCARA","","","working space","Location","best")

% plot piano zy
figure(2)
plot3([-2 2],[0 0],[0 0],'k')
hold on
plot3([0 0],[-2 2],[0 0],'k')
grid on
axis('equal')
view([90 0])
zlim([-0.2 0.5])

%plot del punto centrale dell'area di lavoro
S=SCARAdir(Q,L);
plot3(S(1),S(2),S(3),'r*')

PlotSCARA(Q,L,'b',2)

% Q3min = +0.05;
% Q3max = 0.40;

Q3min = +0.0;
Q3max = 0.3;

for i=1:101
    q6(i) = Q3min+(Q3max-Q3min)/100*(i-1);
    s6(:,i) = SCARAdir([Q(1) Q(2) q6(i) Q(4)],L);
end

plot3(s6(1,:),s6(2,:),s6(3,:),'r')

% ricavo i punti mancanti con la cinematica inversa

S1p = [0 0.33077 0.15 0]';
Q0 = [pi/10 8*pi/10 0.15 0]';

[Q7 val] = SCARAinv(S1p,L,Q0,1e-6,55)
% PlotSCARA(Q7,L,'b',1)
% PlotSCARA(Q7,L,'b',2)

S2p = [0 -0.9 0.15 0]';
Q0 = [Q1max 5*pi/10 0.15 0]';

[Q8 val] = SCARAinv(S2p,L,Q0,1e-6,55)
% PlotSCARA(Q8,L,'b',1)
% PlotSCARA(Q8,L,'b',2)

S3p = [0 -0.33 0.15 0]';
Q0 = [11*pi/10 8*pi/10 0.15 0]';

[Q9 val] = SCARAinv(S3p,L,Q0,1e-6,55)
% PlotSCARA(Q9,L,'b',1)
% PlotSCARA(Q9,L,'b',2)


for i=1:101
    s7(:,i) = SCARAdir([Q7(1) Q7(2) q6(i) Q7(4)],L);
    s8(:,i) = SCARAdir([Q8(1) Q8(2) q6(i) Q8(4)],L);
    s9(:,i) = SCARAdir([Q9(1) Q9(2) q6(i) Q9(4)],L);
end

plot3(s7(1,:),s7(2,:),s7(3,:),'r')
plot3(s8(1,:),s8(2,:),s8(3,:),'r')
plot3(s9(1,:),s9(2,:),s9(3,:),'r')
plot3([0 0],[s6(2,end) s7(2,end)],[s6(3,end) s6(3,end)],'r')
plot3([0 0],[s6(2,end) s7(2,end)],[s6(3,1) s6(3,1)],'r')
plot3([0 0],[s8(2,end) s9(2,end)],[s6(3,end) s6(3,end)],'r')
plot3([0 0],[s8(2,end) s9(2,end)],[s6(3,1) s6(3,1)],'r')

legend("","","centro area di lavoro","SCARA","","","working space")
title("Area di lavoro SCARA (piano YZ)")
ylabel("y")
zlabel("z")


end