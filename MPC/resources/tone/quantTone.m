clear all
close all
clc

% NOTE: The following data is for the elbow

%% Enter Numerical Data [Powers,1988]

% enter threshold
thresh = [1   0.26; 1   0.61; 1   1.00; 1   1.00; 1.5 0.45; 2   0.42;
          2   0.24; 2   0.39; 2   0.47; 2   0.46; 2   0.21; 3   0.08;
          3   0.05; 4   0.34];

%% Convert Graphical Data [McCrea,2003]

filenames = {'stiffness.png','damping.png'};

% initialize
n = 17;
stiff = zeros(n,2);
damp = zeros(n,2);

for i = 1:length{filenames}
    
    file = filename{i};
    img = imread(file);

end
xMin = 0; % MAS score, where 1.5 = 1+
xMax = 4;
if ~isempty(strfind(file,'stiff'))
    yMin = 0;
    yMax = 15;
else
    yMin = 0;
    yMax = 45;
end

figure()
imagesc([xMin xMax], [yMin yMax], flipud(img));

%% Convert from Graph to Numbers



for i = 1:n
    point = ginput(1);
    MAS = round(point(1),1);
    if ~isempty(strfind(filename,'stiff'))
        x = round(point(2),2)*10^-4;
    else
        x = round(point(2),2)*10^-5;
    end
    points(i,:) = [MAS x];
end
disp(points)
