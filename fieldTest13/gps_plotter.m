% clear all;
% sizeA = [8544 10];
% f = fopen('data.csv','r');
% A = textscan(f,'%f');
% fclose(f);
data = readmatrix('G:\filteredpaths\test5filtered.csv');

Latitude = data(:,1);
% Latitude = Latitude/1e4
   
Longitude = data(:,2); 
% Longitude = Longitude/1e4
% viewExpansion = 11.5;
geoplot(Latitude,Longitude,'-+r');
% [latitudeLimits,longitudeLimits] = geolimits;
% latitudeLimits(1) = latitudeLimits(1)+viewExpansion;
% latitudeLimits(2) = latitudeLimits(2)-viewExpansion;
% longitudeLimits(1) = longitudeLimits(1)+1.5*viewExpansion;
% longitudeLimits(2) = longitudeLimits(2)-1.5*viewExpansion;
% geolimits(latitudeLimits,longitudeLimits);

geobasemap satellite;