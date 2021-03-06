% This function takes joint-wise bias data (joint angle vs. bias at that
% angle) and outputs the affine function fit (slope and intercept) for each
% joint.
function func = defineBiasFunc(data)

nJoints = size(data,3);
func.slope = zeros(nJoints,1);
func.inter = zeros(nJoints,1);

for i = 1:size(data,3)
    [func.slope(i), func.inter(i)] = fitData(data(:,1,i), data(:,2,i));
end

end