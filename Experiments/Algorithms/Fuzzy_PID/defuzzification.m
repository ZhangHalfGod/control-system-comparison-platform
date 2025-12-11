% 去模糊化方法实现（重心法）
% 功能：将模糊推理结果转换为精确值
% 输入：x - 输出变量论域
%       mu - 模糊隶属度函数值
% 输出：y - 去模糊化后的精确值

function y = defuzzification(x, mu)
    % 重心法去模糊化
    numerator = sum(x .* mu);
    denominator = sum(mu);
    
    if denominator == 0
        y = 0;
    else
        y = numerator / denominator;
    end
end