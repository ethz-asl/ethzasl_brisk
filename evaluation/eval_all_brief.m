clear all;
close all;
clc;

%% configure the job
datasets={...
    'wall','boat'};
numbers={...
    2,2};
detectors={...
    {'S-BRISKOLD63','S-BRISKOLD63','BRISKOLD67','S-BRISKOLD63','S-BRISKOLD63','S-BRISKOLD63','S-BRISKOLD63','BRISKOLD67'},...
    {'S-BRISKOLD94','S-BRISKOLD94','BRISKOLD115','S-BRISKOLD94','S-BRISKOLD94','S-BRISKOLD94','S-BRISKOLD94','BRISKOLD115'}};
descriptors={...
    {'SU-BRISK','S-BRISK','BRISK','BRIEF','ORB','SU-FREAK','S-FREAK','FREAK'},...
    {'SU-BRISK','S-BRISK','BRISK','BRIEF','ORB','SU-FREAK','S-FREAK','FREAK'}};
legendentries={...
    'SU-BRISK2','S-BRISK2','BRISK2','(SU-)BRIEF64','ORB','SU-FREAK','S-FREAK','FREAK'};
clrs={...
    [0.9 0.1 0.1],[0.9 0.1 0.1],[0.9 0.1 0.1],[0.1 0.1 0.1],[0.8 0.1 0.8],[0.8 0.6 0.1],[0.8 0.6 0.1],[0.8 0.6 0.1]};
stls={...
    '-.','--','-','-.','-.','-.','--','-'};

%% processing
for k=1:1%size(datasets,2)
    figure(k)
    hold on;
    % evaluate all
    legendentries2=legendentries;
    for i=1:size(detectors{1},2)
        [ oneMinusPrecision,recall,corresp ] = ...
            processFiles( 'sim', '/home/lestefan/datasets/mikolajczyk', ...
            datasets{k}, detectors{k}{i}, descriptors{k}{i},  numbers{k} );
        % plot
        plot(oneMinusPrecision,recall,'Color',clrs{i},'LineStyle',stls{i});
        legendentries2{i}=...
            [legendentries{i} ', corresp: ' num2str(corresp)];
    end
    % clean up figure  
    title([datasets{k} ' 1-' num2str(numbers{k})])
    xlabel('1-precision')
    ylabel('Recall')
    legend(legendentries2);
    grid on;
    axis([0 1 0 1]);
end
