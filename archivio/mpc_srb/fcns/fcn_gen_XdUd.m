function [Xd,Ud] = fcn_gen_XdUd(t,Xt,bool_inStance,p)
%% parameters
gait = p.gait;
acc_d = p.acc_d;
vel_d = p.vel_d;
yaw_d = p.yaw_d;

%% generate reference trajectory
% X = [pc dpc eta wb pf]'
lent = length(t);
% --- MODIFICA 1: Dimensioni estese per l'esapode (36 per Xd, 18 per Ud) ---
Xd = zeros(36,lent); 
Ud = zeros(18,lent);
Rground = p.Rground;           % ground slope

for ii = 1:lent
    if gait >= 0        % --- March forward and rotate ---
        %%%%%%%%%% linear motion %%%%%%%%%%%%%
        pc_d = [0;0;p.z0];
        dpc_d = [0;0;0];
        for jj = 1:2
            if t(ii) < (vel_d(jj) / acc_d)
                dpc_d(jj) = acc_d * t(ii);
                pc_d(jj) = 1/2 * acc_d * t(ii)^2;
            else
                dpc_d(jj) = vel_d(jj);
                pc_d(jj) = vel_d(jj) * t(ii) - 1/2 * vel_d(jj) * vel_d(jj)/acc_d;
            end
        end
        %%%%%%%%%% angular motion %%%%%%%%%%%%%
        if isempty(Xt)
            ea_d = [0;0;0];
        else
            ea_d = [0;0;yaw_d];
        end
        vR_d = reshape(expm(hatMap(ea_d)),[9,1]);
        wb_d = [0;0;0];
    end
    
    % --- MODIFICA 2: Reshape a 18 elementi per le posizioni dei 6 piedi ---
    pfd = reshape(Rground * p.pf36,[18,1]);
    Xd(:,ii) = [pc_d;dpc_d;vR_d;wb_d;pfd];
    
    %%%% force
    if (gait == -3)
        Ud(:,ii) = U_d; % Nota: se usi il backflip (gait -3), andrà configurato a 18
    else
        sum_inStance = sum(bool_inStance(:,ii));
        if sum_inStance == 0    % All legs in swing
            Ud(:,ii) = zeros(18,1);
        else
            % --- MODIFICA 3: Ripartizione della forza verticale desiderata (Fz) su tutti e 6 i piedi ---
            % Indici 3, 6, 9, 12, 15, 18 corrispondono alle componenti Fz dei piedi 1, 2, 3, 4, 5, 6
            Ud([3,6,9,12,15,18],ii) = bool_inStance(:,ii)*(p.mass*p.g/sum_inStance);
        end
    end
end
end