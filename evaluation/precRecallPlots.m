% test - written by Stefan Leutenegger 02/2011
clear all;
close all;
clc;
strategy='sim'; % 'nn', 'sim', or 'rn': this will select matching


%% sets
sets{1}.name='graf';
sets{1}.folderName='graf/13';
sets{1}.threshold=67;
sets{1}.secondImageNumber=3;
sets{1}.SIFTthreshold=4.4;

sets{2}.name='wall';
sets{2}.folderName='wall/14';
sets{2}.threshold=68;
sets{2}.secondImageNumber=4;
sets{2}.SIFTthreshold=1.65;

sets{3}.name='rot-wall';
sets{3}.folderName='wall/0_60_degrees';
sets{3}.threshold=68;
sets{3}.secondImageNumber=60;
sets{3}.SIFTthreshold=1.52;

sets{4}.name='boat';
sets{4}.threshold=82;
sets{4}.folderName='boat/14';
sets{4}.secondImageNumber=4;
sets{4}.SIFTthreshold=2.6;

sets{5}.name='bikes';
sets{5}.threshold=45;
sets{5}.folderName='bikes/14';
sets{5}.secondImageNumber=4;
sets{5}.SIFTthreshold=2.7;

sets{6}.name='trees';
sets{6}.threshold=68;
sets{6}.folderName='trees/14';
sets{6}.secondImageNumber=4;
sets{6}.SIFTthreshold=1.94;

sets{7}.name='leuven';
sets{7}.threshold=69;
sets{7}.folderName='leuven/14';
sets{7}.secondImageNumber=4;
sets{7}.SIFTthreshold=2.05;

sets{8}.name='ubc';
sets{8}.threshold=88;
sets{8}.folderName='ubc/14';
sets{8}.secondImageNumber=4;
sets{8}.SIFTthreshold=2.0;

setSelector = 6;

%% run - do not edit
global regionScaler;
regionScaler = 1.0; % leave this at 1 for standard evaluation

disp('-------------------------------')

fh=figure;
appPath = '/home/lestefan/workspace/IBrief/';
clr=colormap('lines');
correspondences=zeros(3);
for i=1:3
    if i==2
        %SURF
        path='/home/lestefan/ibrief/EXPERIMENTS/22Jul_SURFdata/';
        file1=[path sets{setSelector}.folderName '/file1'];
        file2=[path sets{setSelector}.folderName '/file' num2str(min(sets{setSelector}.secondImageNumber,9))];
    else
        %SIFT/BRISK
        file1='file1.txt'; file2='file2.txt';
        system('rm info.m');
        clear H1to2 H1to3 H1to4 H1to5 H1to6;
        clear image1 image2 dataset detector descriptor;
        whereIAm=cd;
        cd(appPath);
        if i==3
            %BRISK
            system(['./mikolajczyk ' sets{setSelector}.name ' ' num2str(sets{setSelector}.secondImageNumber)...
                ' MULTIAGAST' num2str(sets{setSelector}.threshold) ' BRISK']);
        else
            %SIFT
            system(['./mikolajczyk ' sets{setSelector}.name ' ' num2str(sets{setSelector}.secondImageNumber)...
                ' SIFT' num2str(sets{setSelector}.SIFTthreshold) ' SIFT']);
        end
        cd(whereIAm);
        info;
    end

    % run repeatability to compute the correspondence matrix
    [v_overlap,v_repeatability,v_nb_of_corespondences,...
        matching_score,v_nb_of_matches,twi]=repeatability(...
        file1,file2,H1to2,image1,image2,0);


    % then run descperf to compute the matching score
    [cmatch_nn, tmatch_nn,cmatch_sim,tmatch_sim,cmatch_rn,tmatch_rn, corresp_nn, corresp_sim]=...
        descperf(file1,file2,H1to2,image1,image2,...
        v_nb_of_matches,twi);
    
    % recall-precision plot:
    if strcmp(strategy,'nn')==1
        recall=cmatch_nn/corresp_nn;
        oneMinusPrecision=(tmatch_nn-cmatch_nn)./tmatch_nn;
        corresp=corresp_nn;
    else
        if strcmp(strategy,'rn')==1
            recall=cmatch_rn/corresp_nn;
            oneMinusPrecision=(tmatch_rn-cmatch_rn)./tmatch_rn;
             corresp=corresp_nn;
        else
            recall=cmatch_sim/corresp_sim;
            oneMinusPrecision=(tmatch_sim-cmatch_sim)./tmatch_sim;
            corresp=corresp_sim;
        end
    end
    
    % plotting
    correspondences(i)=corresp;
    if i==1
        plot(oneMinusPrecision,recall,'-.','Color',clr(i,:)); hold on;
    else
        if i==2
            plot(oneMinusPrecision,recall,'--','Color',clr(i,:)); hold on;
        else
            plot(oneMinusPrecision,recall,'-','Color',clr(i,:)); hold on;
        end
    end
  
    % annotation
    xlabel('1-Precision [-]')
    ylabel('Recall [-]')
    axis([0 1 0 1]);
    
    % resizing
    set(gcf,'WindowStyle','normal')
    set(gcf,'Position',[0, 0, 280, 200])
    legend off
    title('')
    grid on
end

% annotation
xlabel('1-Precision [-]')
ylabel('Recall [-]')
axis([0 1 0 1]);

% resizing
set(gcf,'WindowStyle','normal')
set(gcf,'Position',[0, 0, 280, 200])
legend off
title('')
grid on

%legend({'SIFT';'SURF';'BRISK'})

% annotate correspondences
text(0.05,0.9,['SIFT(' num2str(correspondences(1)) '), SURF(' num2str(correspondences(2)) '), BRISK(' num2str(correspondences(3)) ')'],'FontSize',9)