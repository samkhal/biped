clc;
options.floating = true;
legs = Caminante('urdf/Legs.urdf',options);
nq = legs.getNumPositions();
pelvis = legs.findLinkId('pelvis');
l_foot = legs.findLinkId('l_foot');
r_foot = legs.findLinkId('r_foot');

kc1r = WorldEulerConstraint(legs,pelvis,[0;0;0],[0;0;0]);
kc2r = WorldEulerConstraint(legs,l_foot,[pi/2;0;nan],[pi/2;0;nan]);
kc3r = WorldEulerConstraint(legs,r_foot,[pi/2;0;nan],[pi/2;0;nan]);

q_nom = zeros(nq,1);
q_seed = q_nom+1e-2*randn(nq,1);
ikoptions = IKoptions(legs);
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMex(false);
ikoptions = ikoptions.setMajorIterationsLimit(5000);

x = -1:0.05:1;
y = -1:0.05:1;
% z = -5:1:5;
i = 1;
j = 1;
k = 1;
ii = 1;
% global foot;

for i = 1:length(x)
    for j = 1:length(y)
%         for k = 1:length(z)
%             pos_upper = [x(i)+0.00001;y(j)+0.01;z(k)+0.01];
%             pos_lower = [x(i)-0.00001;y(j)-0.01;z(k)-0.01];
            
            %kc1p = WorldPositionConstraint(legs,pelvis,[0;0;0],pos_lower,pos_upper);
            kc2p = WorldPositionConstraint(legs,l_foot,[0;0;0],[0;0;0],[0;0;0]);
            kc3p = WorldPositionConstraint(legs,r_foot,[0;0;0],[x(i);y(j);0],[x(i);y(j);0]);
            
            ikproblem = InverseKinematics(legs,q_nom,kc1r,kc2r,kc3r,kc3p,kc2p);%kc1p);
            ikproblem = ikproblem.setQ(ikoptions.Q);
            %ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);
            ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
            ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);
            
            tic;
            [qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_seed);
            toc
            
            if isempty(infeasible_cnstr_ik)
                disp('here')
                footrx(ii) = legs.feetPosition(qik).right(1);
                footry(ii) = legs.feetPosition(qik).right(2);
                ii = ii + 1;
            end
%         end
    end
end

for i = 1:length(x)
    for j = 1:length(y)
%         for k = 1:length(z)
%             pos_upper = [x(i)+0.00001;y(j)+0.01;z(k)+0.01];
%             pos_lower = [x(i)-0.00001;y(j)-0.01;z(k)-0.01];
            
            %kc1p = WorldPositionConstraint(legs,pelvis,[0;0;0],pos_lower,pos_upper);
            kc2p = WorldPositionConstraint(legs,r_foot,[0;0;0],[0;0;0],[0;0;0]);
            kc3p = WorldPositionConstraint(legs,l_foot,[0;0;0],[x(i);y(j);0],[x(i);y(j);0]);
            
            ikproblem = InverseKinematics(legs,q_nom,kc1r,kc2r,kc3r,kc3p,kc2p);%kc1p);
            ikproblem = ikproblem.setQ(ikoptions.Q);
            %ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);
            ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
            ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);
            
            tic;
            [qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_seed);
            toc
            
            if isempty(infeasible_cnstr_ik)
                disp('here')
                footlx(ii) = legs.feetPosition(qik).left(1);
                footly(ii) = legs.feetPosition(qik).left(2);
                ii = ii + 1;
            end
%         end
    end
end

% v = legs.constructVisualizer();
% v.inspector([qik;zeros(18,1)])