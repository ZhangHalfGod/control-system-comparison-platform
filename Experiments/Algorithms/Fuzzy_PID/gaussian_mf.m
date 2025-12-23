% 高斯隶属度函数实现
% 功能：计算输入值在高斯隶属度函数下的隶属度
% 输入：x - 输入值
%       c - 高斯函数中心点
%       sigma - 高斯函数标准差
% 输出：mu - 隶属度值（0~1）

function mu = gaussian_mf(x, c, sigma)
    mu = exp(-((x - c)^2) / (2 * sigma^2));
end