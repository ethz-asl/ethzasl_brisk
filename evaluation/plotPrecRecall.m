function h=plotPrecRecall(oneMinusPrec,recall,...
    dataset,detector,descriptor,correspondences,handle)
% Plot eval results
% Written by Stefan Leutenegger 02/2011
% usage:
% h=PLOTPRECRECALL(oneMinusPrec,recall,...
%   dataset,detector,descriptor,correspondences,handle)

%only delete everything if there is no figure:
if handle==0
    h=get(0,'CurrentFigure');
else
    h=handle;
end
if(isempty(h))
    legendentries=cell(0);
else
    handles=get(h,'Children');
    legendhandle=0;
    for i=1:length(handles)
        if strcmp(get(handles(i),'Tag'),'legend')==1
            legendhandle=handles(i);
            %disp('found legend');
            break;
        end
    end
    if legendhandle==0
        close h;
        legendentries=cell(0);
    else
        legendentries=get(legendhandle,'String');
    end
end

legendentries(size(legendentries,2)+1)={['detector: ' detector ', descriptor: ' descriptor ', corresp.: ' num2str(correspondences)]};

% if(strcmp(descriptor,'SURF')==1)
%     color = [1 0 0];
% else if (strcmp(descriptor,'BRIEF')==1)
%         color = [0 0 0];
%     else
%         color = [0 0 1];
%     end
% end

clr=colormap('lines');
hold on;
plot(oneMinusPrec,recall,'Color',clr(size(legendentries,2),:));
title(['Dataset: ' dataset])
xlabel('1-precision')
ylabel('recall')
legend(legendentries);
axis([0 1 0 1]);