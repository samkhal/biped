function [  ] = COW( contact_pts )
    %contact_pts = [x1 y1;x2 y2]
    %Reachable Workspace
    %Assuming reference point is in the center of the platform
    b = 0.2; %platform length
    lmin = 0.35; %min leg extension length
    lmax = 0.5; %max leg extension length
    d = sqrt((contact_pts(2,1)-contact_pts(1,1))^2 + (contact_pts(2,2)-contact_pts(1,2))^2)%distance between ground foot contact points
    e = b/2; %distance to midpoint of platform
    beta = 0; %angle between reference point and b
    
   %if d <= 2*sqrt(lmin^2 - ((d*l)/(2*k))^2)
    
        t3 = 0;%t3 = asin((contact_pts(1,2) - contact_pts(2,2))/d); %angle of platform
        k = 1/2*(d-b*cos(t3))
        if k==0
            k = b/2;
        end
        l = b/2*sin(t3)
        D = sqrt((2*k)^2 + (2*l)^2)

        p0 = [(2*l*sqrt(lmin^2-D^2/4))/D,(2*k*sqrt(lmin^2-D^2/4))/D]
        pt = [(2*l*sqrt(lmax^2-D^2/4))/D,(2*k*sqrt(lmax^2-D^2/4))/D]
        pr = [(k*(lmax^2-lmin^2)/D^2) + ((2*l)/D)*sqrt(lmin^2-((lmax^2-lmin^2-D^2)/(2*D))^2), ...
              (2*k)/D*sqrt(lmin^2-((lmax^2-lmin^2-D^2)/(2*D))^2 - (l*(lmax^2-lmin^2))/D^2)]
        pl = [(k*(lmin^2-lmax^2)/D^2) + ((2*l)/D)*sqrt(lmin^2-((lmax^2-lmin^2-D^2)/(2*D))^2), ...
              (2*k)/D*sqrt(lmin^2-((lmax^2-lmin^2-D^2)/(2*D))^2 - (l*(lmin^2-lmax^2))/D^2)]
          
        X_RLC = linspace(p0(1),pr(1),10);
        X_RUC = linspace(pt(1),pr(1),10);
        X_LLC = linspace(pl(1),p0(1),10);
        X_LUC = linspace(pl(1),pt(1),10);
        
        %lmin^2 = (X - k)^2 + (Y + l)^2; %RLC
        %lmax^2 = (X + k)^2 + (Y - l)^2; %RUC
        %lmin^2 = (X + k)^2 + (Y - l)^2; %LLC
        %lmax^2 = (X - k)^2 + (Y + l)^2; %LUC
        
        Y_RLC = sqrt(lmin^2 - (X_RLC-k).^2) - l;
        Y_RUC = sqrt(lmax^2 - (X_RUC+k).^2) + l;
        Y_LLC = sqrt(lmin^2 - (X_LLC+k).^2) + l;
        Y_LUC = sqrt(lmax^2 - (X_LUC-k).^2) - l; 
        
        figure('Name','2D COW');
        plot(X_RLC,Y_RLC,X_RUC,Y_RUC,X_LLC,Y_LLC,X_LUC,Y_LUC)
        hold on;
        plot(-1*k,l,'s',k,-1*l,'s');
        axis('equal')
        
%         %% 3D
%         %centers
%         phi = 0;
%         disp(k);
%         disp(l);
%         c1 = [k*cos(phi),k*sin(phi),-1*l];
%         kprime = -1*k;
%         c1prime = [kprime*cos(phi),kprime*sin(phi),l];
%        
%         i = 0;
%         j = 0;
%         for i = 1:length(X_RLC)
%             for j = 1:length(Y_RLC)
%                 Z_RLS(i,j) = sqrt(lmin^2 - (X_RLC(i)-c1(1))^2 - (Y_RLC(j)-c1(2))^2) + c1(3)
%             end
%         end
% %         Z_LUS = sqrt(lmax^2 - (X-c1(1)).^2 - (Y-c1(2)).^2) + c1(3);
% %         Z_LLS = sqrt(lmin^2 - (X-c1prime(1)).^2 - (Y-c1prime(2)).^2) + c1prime(3);
% %         Z_RUS = sqrt(lmax^2 - (X-c1prime(1)).^2 - (Y-c1prime(2)).^2) + c1prime(3);
%         
%         angles = 0:1:180;
%         Z_RLS = 
%         plot3(X_RLS,Y_RLS,Z_RLS);
%         figure
%         surf(Z_RLS)
    %end  
    
    
    
    
end

