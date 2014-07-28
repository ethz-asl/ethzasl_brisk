function [ oneMinusPrecision,recall,corresp ] = processFiles( strategy, path, dataset, detector, descriptor, no2)
%PROCESSFILES Summary of this function goes here
%   Detailed explanation goes here
% test - written by Stefan Leutenegger 02/2011
testOnlyDetection=0; % this will only consider the common area.

global regionScaler;
regionScaler = 1.0; % leave this at 1 for standard evaluation

disp('-------------------------------')
% run the info to determine what was used:

fname1=[path '/' dataset '/' detector '-' descriptor '-1.kpts'];
fname2=[path '/' dataset '/' detector '-' descriptor '-' num2str(no2) '.kpts'];
image1=[path '/' dataset '/img1.ppm'];
image2=[path '/' dataset '/img' num2str(no2) '.ppm'];
H1to2=load([path '/' dataset '/H1to' num2str(no2) 'p']);

% run repeatability to compute the correspondence matrix
[v_overlap,v_repeatability,v_nb_of_corespondences,...
    matching_score,v_nb_of_matches,twi]=repeatability(...
    fname1,fname2,H1to2,image1,image2,testOnlyDetection);

if testOnlyDetection~=1
    % then run descperf to compute the matching score
    [cmatch_nn, tmatch_nn,cmatch_sim,tmatch_sim,cmatch_rn,tmatch_rn, corresp_nn, corresp_sim]=...
        descperf(fname1,fname2,H1to2,image1,image2,...
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
    %plotPrecRecall(oneMinusPrecision,recall,...
    %    dataset,detector,descriptor,corresp,0);
end

end

