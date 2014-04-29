function [points,shortPairs,longPairs]=readPattern(name)
fid=fopen(name);
dim=fscanf(fid, '%f',1);
points = fscanf(fid, '%f', [3, dim])';
points=points/0.6;
dim=fscanf(fid, '%f',1);
shortPairs = fscanf(fid, '%f', [2, dim])';
shortPairs=shortPairs+ones(size(shortPairs));
dim=fscanf(fid, '%f',1);
longPairs = fscanf(fid, '%f', [2, dim])';
longPairs=longPairs+ones(size(longPairs));
fclose(fid);