figure(2)
clc;
clear varaiables;
disp('Welcome to the BRISK pattern generator!')
disp('---------------------------------------')

% draw pattern
sigma_min={1,2,4.8};
rList={[0,1.4,2.9,4.9,7.4,10.8]*0.85/0.6,[0,2.8,5.8,9.8]*0.85/0.6,[0,5.6]*1.2*0.85/0.6};
nList={[1,6,10,14,15,20],[1,6,10,14],[1,6]};
sigmas={[],[],[]};
levels=size(sigma_min,2);
levels=1;
for level=1:levels
    sigmas{level}=max(rList{level}.*sin(2*pi./nList{level}/2),sigma_min{level}); % stdev
end

N=32; % circle discretization
x=zeros(N+1);
y=zeros(N+1);

% plot some lines first
maxrad=0;
maxind=0;
maxlevel=0;
for level=1:levels
    [maxrad_tmp,maxind_tmp]=max(rList{level});
    if maxrad_tmp>maxrad
        maxrad=maxrad_tmp;
        maxind=maxind_tmp;
        maxlevel=level;
    end
end
maxint=ceil(maxrad+(sigmas{maxlevel}(maxind)))+0.5;
hold on
for i=-maxint:1:maxint
   plot([i,i],[-maxint,maxint],'-','Color',[0.7,0.7,0.7]);
   plot([-maxint,maxint],[i,i],'-','Color',[0.7,0.7,0.7]);
end
    
% the pattern
totalShortPairings=0;
totalLongPairings=0;
points=[];
pointSigmas=[];
longPairs=[];
shortPairs=[];
pointOffset=0;
for level=1:levels
    l=0;
    positions=[];
    allsigmas=[];
    pts=sum(nList{level});
    rings=length(rList{level});   % number of rings
    for i=1:rings
        for j=1:nList{level}(i)
            x0=rList{level}(i)*cos((j-1)/nList{level}(i)*2*pi);
            y0=rList{level}(i)*sin((j-1)/nList{level}(i)*2*pi);
            positions(l+1,:)=[x0,y0];
            points=[points;[x0,y0]];
            allsigmas(l+1,1)=sigmas{level}(i);
            pointSigmas=[pointSigmas;sigmas{level}(i)];
            plot(x0, y0,'o','LineWidth',2,'Color',[0,0,(levels-level+1)/levels]);
            for k=0:N
                x(k+1)=sigmas{level}(i)*cos(k/(N)*2*pi)+x0;
                y(k+1)=sigmas{level}(i)*sin(k/(N)*2*pi)+y0;
            end
            plot(x,y,'Color',[(levels-level+1)/levels,0,0],'LineWidth',2,'LineStyle','--');
            hold on;
            l=l+1;
        end
    end
    axis image;

    % short and long pairings
    shortPairings=0;
    longPairings=0;
    
    for p=2:l
        for q=1:p-1
            dist = (norm(positions(p,:)-positions(q,:))-allsigmas(p)-allsigmas(q))/((allsigmas(p)+allsigmas(q)));
            if dist < 0.7/0.6
                shortPairings=shortPairings+1;
                shortPairs=[shortPairs;[p+pointOffset,q+pointOffset]];
                %plot([positions(p,1),positions(q,1)],...
                    %[positions(p,2),positions(q,2)],'g','LineWidth',1);
            end
            dist2 = norm(positions(p,:)-positions(q,:));
%             if dist2 < 5.85/0.6
%                 shortPairings=shortPairings+1;
%                 shortPairs=[shortPairs;[p+pointOffset,q+pointOffset]];
%                 %plot([positions(p,1),positions(q,1)],...
%                 %[positions(p,2),positions(q,2)],'g','LineWidth',1);
%             end
%             if dist2 > 8.2/0.6
%                 longPairings=longPairings+1;
%                 longPairs=[longPairs;[p+pointOffset,q+pointOffset]];
%                 plot([positions(p,1),positions(q,1)],...
%                     [positions(p,2),positions(q,2)],'k','LineWidth',1);
%             end
             
            if dist < 1.3/0.6
                longPairings=longPairings+1;
                longPairs=[longPairs;[p+pointOffset,q+pointOffset]];
                %plot([positions(p,1),positions(q,1)],...
                    %[positions(p,2),positions(q,2)],'k','LineWidth',1);
            end
        end
    end

    % output
    totalShortPairings=totalShortPairings+shortPairings;
    totalLongPairings=totalLongPairings+longPairings;
    disp(['short pairings in level ' num2str(level) ': ' num2str(shortPairings)])
    disp(['long pairings in level ' num2str(level) ': ' num2str(longPairings)])
    
    pointOffset=pointOffset+(size(positions,1));
end
disp('---------------------------------------')
disp(['total short pairings: ' num2str(totalShortPairings)])
disp(['total long pairings: ' num2str(totalLongPairings)])

%% writing file
writePattern([points,pointSigmas],shortPairs,longPairs,'brisk.ptn');
% fid=fopen('brisk.ptn','w');
% fprintf(fid,'%i\n',size(points,1));
% for i=1:size(points,1)
%     fprintf(fid,'%i ',[points(i,:),pointSigmas(i,1)]*0.6);
%     fprintf(fid,'\n');
% end
% fprintf(fid,'%i\n',size(shortPairs,1));
% for i=1:size(shortPairs,1)
%     fprintf(fid,'%i ',shortPairs(i,:)-[1 1]);
%     fprintf(fid,'\n');
% end
% fprintf(fid,'%i\n',size(longPairs,1));
% for i=1:size(longPairs,1)
%     fprintf(fid,'%i ',longPairs(i,:)-[1 1]);
%     fprintf(fid,'\n');
% end
% fclose(fid);