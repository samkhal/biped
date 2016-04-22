function zmp
    %% Load Caminante
    % Load the model with a floating base
    options.floating = true;
    options.dt = 0.001;
    options.terrain = RigidBodyFlatTerrain;

    r = Caminante('urdf/Legs.urdf',options);
    %r = r.removeCollisionGroupsExcept({'heel','toe','back','front','knee','butt'});
    r = r.removeCollisionGroupsExcept({'heel','midfoot','toe'});
    r = compile(r);
    
    q = zeros(18,1);
    [x,y] = getZMP(r,q)

    function [x_zmp, y_zmp] = getZMP(model, q)
        g = 9.81;
        tddot = 0;
        zddot = 0;
        x_com = 0;
        y_com = 0;
        I_x = 0;
        I_y = 0;
        p = 0;
        
        kinsol = doKinematics(model,q);   
        for i = 2:14 %length(model.getBody)
            com = forwardKin(model, kinsol,i,model.getBody(i).com);
            I = model.getBody(i).inertia;
            m = model.getBody(i).mass;
            x_com = x_com + m*(zddot + g)*com(1);
            y_com = y_com + m*(zddot + g)*com(2);
            I_x = I_x +I(1)*tddot;
            I_y = I_y +I(5)*tddot;
            p = p + m*(zddot + g);
        end
        x_zmp = (x_com - I_y)/p;
        y_zmp = (y_com + I_x)/p;
    end
end
    