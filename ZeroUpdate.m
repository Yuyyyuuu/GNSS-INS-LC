function kf = ZeroUpdate(navstate, kf, cfg,thisimu, dt)

     vel = navstate.vel;
     omega_ib=thisimu(2:4, 1) / dt;

     %% ZUV update
     Z=vel;
     R=diag(power(cfg.zerov_measnoise, 2));
     H = zeros(3, kf.RANK);
     H(1:3, 4:6) = eye(3);
     K = kf.P * H' / (H * kf.P * H' + R);
     kf.x = kf.x + K*(Z - H*kf.x);
     kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';

     %% ZARU update
     Z=[0,0,1]*omega_ib;
     R=diag(power(cfg.zeroa_measnoise(3), 2));
     H = zeros(3, kf.RANK);
     H(1:3, 10:12) = eye(3);
     H(1:3, 16:18) = diag(omega_ib);
     H =[0,0,1]*H;
     K = kf.P * H' / (H * kf.P * H' + R);
     kf.x = kf.x + K*(Z - H*kf.x);
     kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
     