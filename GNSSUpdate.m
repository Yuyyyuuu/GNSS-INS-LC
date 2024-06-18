function kf = GNSSUpdate(navstate, gnssdata, kf, antlever, usegnssvel, thisimu, dt)

    param = Param();

    %% GNSS position update
    % abandon gnss pos outlier 
    gnssposstd = gnssdata(5:7, 1);
    if gnssposstd(1, 1) > 5 || gnssposstd(2, 1) > 5 || gnssposstd(3, 1) > 5
        disp(['WARNING: Abandon gnss position measurement at: ', num2str(gnssdata(1, 1))]);
    else
        % measurement innovation
        DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
        Z = DR*(navstate.pos - gnssdata(2:4, 1))+navstate.Cbn*antlever;% N系下的NED
        
        % measurement matrix and noise matrix
        R = 100*diag(power(gnssdata(5:7, 1), 2));%m2 m2 m2
        H = zeros(3, kf.RANK);
        H(1:3, 1:3) = eye(3);
        H(1:3, 7:9) = skew(navstate.Cbn * antlever);
        
        % update covariance and state vector
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K*(Z - H*kf.x);
        kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
    end

    %% GNSS velocity update（本次大作业不做要求）
    if usegnssvel
        % abandon gnss vel outlier 
        gnssvelstd = gnssdata(11:13, 1);
        if gnssvelstd(1, 1) > 0.5 || gnssvelstd(2, 1) > 0.5 || gnssvelstd(3, 1) > 0.5
            disp(['WARNING: Abandon gnss velocity measurement at: ', num2str(gnssdata(1, 1))]);
        else
            % measurement innovation
            pos = navstate.pos;
            vel = navstate.vel;
            Cbn = navstate.Cbn;
            omega_ib=thisimu(2:4, 1) / dt;
            wie_n = [param.wie*cos(pos(1)); 0; -param.wie*sin(pos(1))];
            wen_n = [vel(2)/(navstate.Rn+pos(3)); -vel(1)/(navstate.Rm+pos(3)); -vel(2)*tan(pos(1))/(navstate.Rn+pos(3))];
            win_n = wie_n + wen_n;
            Z = navstate.vel-skew(win_n)*Cbn*antlever-Cbn*cross(antlever,omega_ib)-gnssdata(8:10, 1);

            % measurement matrix and noise matrix
            R = diag(power(gnssdata(11:13, 1), 2));%(m/s)2 (m/s)2 (m/s)2
            H = zeros(3, kf.RANK);
            H(1:3, 4:6) = eye(3);
            H(1:3, 7:9) = -skew(win_n)*skew(Cbn*antlever)-skew(Cbn*cross(antlever,omega_ib));
            H(1:3,10:12)= -skew(Cbn*antlever);
            H(1:3,16:18)= -Cbn*skew(antlever)*diag(omega_ib);

            % update covariance and state vector
            K = kf.P * H' / (H * kf.P * H' + R);
            kf.x = kf.x + K*(Z - H*kf.x);
            kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
        end
    end
end