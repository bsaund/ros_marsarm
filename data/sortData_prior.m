close all
clear all
d = csvread('MeanCov_prior.txt');

lastPoints = d(:,1) == 10;

truth = [0,0,0,0,0,0];


ang_terms = [5:6];

err = d(:,2:7) - repmat(truth, size(d,1),1);
plot(d(:,1), err(:,3));

trans_err = sqrt(sum(err(:,1:3).^2,2));
ang_err = sqrt(sum(err(:,ang_terms).^2,2));

figure('Name', 'Trans_err')
plot(d(:,1), trans_err)
figure('Name', 'Ang_err')
plot(d(:,1), ang_err)

% mean_trans_err
% mean_ang_err

for i=0
    
    
    trial = i;
    ind = d(:,1) == trial;
    
    
    data = d(ind,2:7);
    err = data - repmat(truth, size(data,1),1);
    mean_trans_err(i+1) = mean(sqrt(sum(err(:,1:3).^2,2)))*100;
    var_trans_err(i+1) = std(sqrt(sum(err(:,1:3).^2,2)))*100;
    mean_ang_err(i+1) = mean(sqrt(sum(err(:,ang_terms).^2,2)));
    var_ang_err(i+1) = std(sqrt(sum(err(:,ang_terms).^2,2)));
end


mean_trans_err
var_trans_err
mean_ang_err
var_ang_err


figure('Name','Mean_Trans_err')

linespec = {'Color','black', 'LineWidth',2};
plot(0:20, mean_trans_err,linespec{:})
hold on
errorbar(1:20, mean_trans_err(2:end), var_trans_err(2:end), ...
         linespec{:})
xlim([0,20.2]);
saveas(gcf, 'On_Robot_Trans.png')



figure('Name','Mean_Ang_err')
plot(0:20, mean_ang_err,linespec{:})
hold on
errorbar(1:20, mean_ang_err(2:end), var_ang_err(2:end), linespec{:})
xlim([0,20.2]);
ylim([0, .1]);
saveas(gcf, 'On_Robot_Ang.png')

save('mean_err', 'mean_trans_err', 'var_trans_err', 'mean_ang_err', 'var_ang_err')
% plot(0:10, mean_ang_err)
