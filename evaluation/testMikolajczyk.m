% test - written by Stefan Leutenegger 02/2011
clear all;
testOnlyDetection=0; % this will only consider the common area.
strategy='sim'; % 'nn', 'sim', or 'rn': this will select matching

global regionScaler;
regionScaler = 1.0; % leave this at 1 for standard evaluation

disp('-------------------------------')
% run the info to determine what was used:
info;
file1='file1.txt'; file2='file2.txt';
%file1='boat1_a.surf'; file2='boat4_a.surf';
%path='/home/lestefan/ibrief/SURFdata/';
%dataset='trees/14';
%number1='1';
%number2='4';
%file1=[path dataset '/file' number1];
%file2=[path dataset '/file' number2];

% run repeatability to compute the correspondence matrix
[v_overlap,v_repeatability,v_nb_of_corespondences,...
    matching_score,v_nb_of_matches,twi]=repeatability(...
    file1,file2,H1to2,image1,image2,testOnlyDetection);

if testOnlyDetection~=1
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
    
    %plot(rn_oneMinusPrecision,rn_recall);
    plotPrecRecall(oneMinusPrecision,recall,...
        dataset,detector,descriptor,corresp,0);
end