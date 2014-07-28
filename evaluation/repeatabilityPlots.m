% repeatability
% written by lestefan 02/2011

clear all;
close all;
clc;

testOnlyDetection=1; % this will only consider the common area.
strategy='sim'; % 'nn', 'sim', or 'rn': this will select matching

global regionScaler;
regionScaler = 1.0; % leave this at 1 for standard evaluation

disp('-------------------------------')

path='/home/lestefan/ibrief/EXPERIMENTS/22Jul_SURFdata/';
%path='/home/lestefan/ibrief/SURFdata/';
appPath = '/home/lestefan/workspace/IBrief/';

sets{1}.name='graf';
sets{1}.threshold=67;
sets{1}.xlabel='Viewpoint change [deg]';
sets{1}.x=[20 30 40 50 60];
sets{1}.corresp=[1504 1284 792 187];

sets{2}.name='wall';
sets{2}.threshold=68;
sets{2}.xlabel='Viewpoint change [deg]';
sets{2}.x=[20 30 40 50 60];
sets{2}.corresp=[858 752 631 421 221];

sets{3}.name='boat';
sets{3}.threshold=82;
sets{3}.xlabel='scale change [-]';
sets{3}.x=[1.13 1.36 1.88 2.36 2.79];
sets{3}.corresp=[3352 2412 1148 698 591];

sets{4}.name='leuven';
sets{4}.threshold=69;
sets{4}.xlabel='Second image number [-]';
sets{4}.x=[2 3 4 5 6];
sets{4}.corresp=[818 622 467 356 228];

setSelector = 1;

datasetName=sets{setSelector}.name;
threshold=sets{setSelector}.threshold;
number1='1';

scoresBRISK=zeros(5,1);
scoresSURF=zeros(5,1);

for i=2:length(sets{setSelector}.corresp)+1
    number2=num2str(i);
    
    %BRISK:
    file1='file1.txt'; file2='file2.txt';
    system('rm info.m');
    clear H1to2;
    clear H1to3;
    clear H1to4;
    clear H1to5;
    clear H1to6;
    whereIAm=cd;
    cd(appPath);
    system(['./mikolajczyk ' datasetName ' ' num2str(i)...
        ' MULTIAGAST' num2str(threshold) ' BRISK']);
    cd(whereIAm);
    clear image1 image2 dataset detector descriptor
    info;
    [v_overlap,v_repeatability,v_nb_of_corespondences,...
        matching_score,v_nb_of_matches,twi]=repeatability(...
        file1,file2,H1to2,image1,image2,testOnlyDetection);
    scoresBRISK(i-1)=v_repeatability(5);
    %%%%%%
    
    %SURF:
    file1=[path datasetName '/1' num2str(i) '/file' number1];
    file2=[path datasetName '/1' num2str(i) '/file' number2];

    % run repeatability to compute the correspondence matrix
    [v_overlap,v_repeatability,v_nb_of_corespondences,...
        matching_score,v_nb_of_matches,twi]=repeatability(...
        file1,file2,H1to2,image1,image2,testOnlyDetection);
    scoresSURF(i-1)=v_repeatability(5);
    %%%%%%
end

figure(1);
%resize
set(gcf,'WindowStyle','normal');
set(gcf,'Position',[0, 0, 280, 200]);

x=sets{setSelector}.x;
bar(x,[scoresBRISK,scoresSURF],'grouped');
ylabel('Repeatability score [%]')
colormap summer
xlabel(sets{setSelector}.xlabel);
legend({'BRISK';'SURF'});
axis([x(1)-(x(5)-x(1))/8,x(5)+(x(5)-x(1))/8,0,100])

%numbers=[908 808 686 465 221]; %wall
%numbers=[1425 1302 805 191]; %graf
%numbers=[750 601 459 309 217]; %leuven
%numbers=[2978 2322 1205 766 630]; %boat

%old set:
%numbers=[1009 885 661 512 260]; %wall
%numbers=[1483 1292 777 171]; %graf
%numbers=[745 590 452 354 241]; %leuven
%numbers=[2882 1895 1151 771 565]; %boat

for i=1:length(sets{setSelector}.corresp)
    text(x(i),5+max(scoresBRISK(i),scoresSURF(i)),num2str(sets{setSelector}.corresp(i)),...
        'HorizontalAlignment','center');
end


