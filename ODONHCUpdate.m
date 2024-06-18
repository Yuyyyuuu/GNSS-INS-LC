function kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, dt)

    param = Param();
    
    % 启用ODO或NHC的标签 
    odouse=false;
    nhcuse=false;
    odonhc=true;

    pos = navstate.pos;
    vel = navstate.vel;
    omega_ib=thisimu(2:4, 1) / dt;
    wie_n = [param.wie*cos(pos(1)); 0; -param.wie*sin(pos(1))];
    wen_n = [vel(2)/(navstate.Rn+pos(3)); -vel(1)/(navstate.Rm+pos(3)); -vel(2)*tan(pos(1))/(navstate.Rn+pos(3))];
    win_n = wie_n + wen_n;
    Cnb=navstate.Cbn';
    wnb_b=omega_ib-Cnb*win_n;

    %% nhc update
    if (nhcuse)
        Z=[0,1,0;0,0,1]*cfg.Cbv*(Cnb*vel-cross(wnb_b,cfg.odolever));
        R = diag([ cfg.odonhc_measnoise(2)^2, cfg.odonhc_measnoise(3)^2]);
        H = zeros(2, kf.RANK);
        H(1:2, 4:6) = [0,1,0;0,0,1]*cfg.Cbv*Cnb;
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K*(Z - H*kf.x);
        kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
    end

    %% odo update
    if (odouse)
        Z=[1,0,0]*(cfg.Cbv*(Cnb*vel+cross(wnb_b,cfg.odolever))-odonhc_vel);
        R = diag(cfg.odonhc_measnoise(1)^2);
        H = zeros(1, kf.RANK);
        H(1:1, 4:6) = [1,0,0]*Cnb;
        H(1:1, 7:9) = -[1,0,0]*Cnb*skew(vel);
        H(1:1, 10:12) = -[1,0,0]*skew(cfg.odolever);
        H(1:1, 16:18) = -[1,0,0]*skew(cfg.odolever)*diag(omega_ib);
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K*(Z - H*kf.x);
        kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
    end

    %% odonhc update
    if (odonhc)
        %% measurement innovation
        Z=cfg.Cbv*(Cnb*vel+cross(wnb_b,cfg.odolever))-odonhc_vel;
        %% measurement equation and noise
        R = diag(power(cfg.odonhc_measnoise, 2));%(m/s)2 (m/s)2 (m/s)2
        H = zeros(3, kf.RANK);
        H(1:3, 4:6) = cfg.Cbv*Cnb;
        H(1:3, 7:9) = -cfg.Cbv*Cnb*skew(vel);
        H(1:3,10:12)= -cfg.Cbv*skew(cfg.odolever);
        H(1:3,16:18)= -cfg.Cbv*skew(cfg.odolever)*diag(omega_ib);
        %% update
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K*(Z - H*kf.x);
        kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
    end
end