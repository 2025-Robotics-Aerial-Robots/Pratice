classdef controller
   methods
       function [control, error] = geometric_tracking_ctrl(obj, iter, multirotor, Xd_enu, b1d)
           % f, M
           control = zeros(4, 1);
           
           % xd, vd, ad, b1d, Wd
           xd_enu = Xd_enu(1:3, 1);
           vd_enu = Xd_enu(4:6, 1);
           ad_enu = Xd_enu(7:9, 1);
           Wd = [0; 0; 0];
           
           % now states
           x_enu = multirotor.x(:, iter-1);
           v_enu = multirotor.v(:, iter-1);
           R = reshape(multirotor.R(:, iter-1), 3, 3);
           W = multirotor.W(:, iter-1);
           e3 = multirotor.e3;

%%　Checkpoint 2
% ---------- Force Controller ---------
           
           % control gains
           kx = diag([10.0*multirotor.m; 10.0*multirotor.m; 10.0*multirotor.m]);
           kv = diag([5.0*multirotor.m; 5.0*multirotor.m; 5.0*multirotor.m]);
           kR = 8.81;
           kW = 2.54;
           
           % convert position and velocity from enu to ned
           x_ned = vec_enu_to_ned(x_enu);
           v_ned = vec_enu_to_ned(v_enu);
           xd_ned = vec_enu_to_ned(xd_enu);
           vd_ned = vec_enu_to_ned(vd_enu);
           ad_ned = vec_enu_to_ned(ad_enu);
           
           % error
           ex_ned = ;
           ev_ned = ;
           
           % f
           A = ;
           b3 = ;
           f = vec_dot(,);

%% Chcekpoint 3 
           % ---------Find Rd----------
           % Given A and b1d, find Rd
           Rd = [b1d_proj b2d b3d];

%% Chcekpoint 4
           % eR and eW
           eR = ;
           eW = ;

           % M
           M = ;

           % f, M
           control(1) = f;
           control(2) = M(1);
           control(3) = M(2);
           control(4) = M(3);
           
           % ex, ev, eR, eW
           error(1:3) = ex_ned;
           error(4:6) = ev_ned;
           error(7:9) = eR;
           error(10:12) = eW;
       end
   end
end
