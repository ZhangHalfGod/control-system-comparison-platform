# MATLAB R2024a 离散PID控制器参数名差异文档

## 📋 文档概述

本文档记录 MATLAB R2024a 中 Simulink PID 控制器模块的参数名差异，特别针对离散时间控制器的配置参数。基于实际调试日志分析，为后续开发提供参考。

## 🔧 PID控制器参数名映射

### 1. 核心控制器参数

| 参数名         | 实际值          | 说明       | 备注                                           |
| :------------- | :-------------- | :--------- | :--------------------------------------------- |
| **Controller** | `PID`           | 控制器类型 | PID, PI, PD, P, I                              |
| **Form**       | `Parallel`      | 控制器结构 | Parallel（并联）, Ideal（串联）                |
| **TimeDomain** | `Discrete-time` | 时间域     | Discrete-time（离散）, Continuous-time（连续） |

### 2. 增益参数

| 参数名 | 实际值 | 说明       | 备注     |
| :----- | :----- | :--------- | :------- |
| **P**  | `1`    | 比例增益   | 数值类型 |
| **I**  | `1`    | 积分增益   | 数值类型 |
| **D**  | `0`    | 微分增益   | 数值类型 |
| **N**  | `100`  | 滤波器系数 | 数值类型 |

### 3. 采样时间相关参数（关键差异点）

| 参数名            | 实际值 | 说明            | ⚠️ 注意事项                            |
| :---------------- | :----- | :-------------- | :------------------------------------ |
| **SampleTime**    | `-1`   | 采样时间        | **不是 `Ts`**，这是最主要的参数名差异 |
| **UseExternalTs** | `off`  | 使用外部Ts信号  | 布尔值                                |
| **UseKiTs**       | `off`  | I参数是否包含Ts | 布尔值，影响积分增益的实际计算        |

### 4. 离散化方法参数

| 参数名               | 实际值          | 说明           | 选项                                       |
| :------------------- | :-------------- | :------------- | :----------------------------------------- |
| **IntegratorMethod** | `Forward Euler` | 积分器离散方法 | Forward Euler, Backward Euler, Trapezoidal |
| **FilterMethod**     | `Forward Euler` | 滤波器离散方法 | Forward Euler, Backward Euler, Trapezoidal |

## ⚠️ 重要差异与注意事项

### 1. 采样时间参数名

- **旧版本/常见误解**：寻找 `Ts` 参数

- **R2024a 实际参数**：`SampleTime`

- **默认值**：`-1` 表示继承采样时间

- **设置方法**：

  matlab

  ```
  % ❌ 错误（不存在的参数名）
  set_param(gcb, 'Ts', '0.01');
  
  % ✅ 正确
  set_param(gcb, 'SampleTime', '0.01');
  ```

  

### 2. 离散时间相关标志

- **必须设置**：`TimeDomain = 'Discrete-time'`
- **依赖关系**：只有在 `TimeDomain` 设置为离散时，`SampleTime` 才有效

### 3. 滤波器相关

- **UseFilter**：必须为 `on` 才能启用微分滤波器
- **N**：滤波器系数，仅在 `UseFilter = on` 时生效

## 🔄 参数设置顺序

正确的参数设置顺序对于避免配置错误至关重要：

1. **设置时间域**：

   matlab

   ```
   set_param(gcb, 'TimeDomain', 'Discrete-time');
   ```

   

2. **设置采样时间**：

   matlab

   ```
   set_param(gcb, 'SampleTime', '0.01'); % 例如：100Hz
   ```

   

3. **设置离散化方法**：

   matlab

   ```
   set_param(gcb, 'IntegratorMethod', 'Forward Euler');
   set_param(gcb, 'FilterMethod', 'Forward Euler');
   ```

   

4. **设置控制器参数**：

   matlab

   ```
   set_param(gcb, 'P', '1.5');
   set_param(gcb, 'I', '0.8');
   set_param(gcb, 'D', '0.2');
   set_param(gcb, 'N', '100');
   ```

   

5. **设置滤波器开关**：

   matlab

   ```
   set_param(gcb, 'UseFilter', 'on');
   ```

   

## 📊 完整配置示例

### 基本离散PID配置

matlab

```
% 创建离散PID控制器基本配置
pid_params = {
    'Controller', 'PID', ...
    'Form', 'Parallel', ...
    'TimeDomain', 'Discrete-time', ...
    'SampleTime', '0.01', ...          % 100Hz
    'IntegratorMethod', 'Forward Euler', ...
    'FilterMethod', 'Forward Euler', ...
    'P', '1.5', ...
    'I', '0.8', ...
    'D', '0.2', ...
    'N', '100', ...
    'UseFilter', 'on', ...
    'ControllerParametersSource', 'internal'
};

% 应用配置
for i = 1:2:length(pid_params)
    set_param(gcb, pid_params{i}, pid_params{i+1});
end
```



### 快速离散PID设置函数

matlab

```
function configureDiscretePID(blockPath, Ts, Kp, Ki, Kd, N)
    % 配置离散PID控制器
    % 输入：
    %   blockPath - PID块路径（如 'model/PID'）
    %   Ts - 采样时间（秒）
    %   Kp, Ki, Kd - PID增益
    %   N - 滤波器系数
    
    set_param(blockPath, 'TimeDomain', 'Discrete-time');
    set_param(blockPath, 'SampleTime', num2str(Ts));
    set_param(blockPath, 'IntegratorMethod', 'Forward Euler');
    set_param(blockPath, 'FilterMethod', 'Forward Euler');
    set_param(blockPath, 'P', num2str(Kp));
    set_param(blockPath, 'I', num2str(Ki));
    set_param(blockPath, 'D', num2str(Kd));
    set_param(blockPath, 'N', num2str(N));
    set_param(blockPath, 'UseFilter', 'on');
    
    fprintf('PID配置完成：Ts=%f, Kp=%f, Ki=%f, Kd=%f, N=%f\n', ...
            Ts, Kp, Ki, Kd, N);
end
```



## 🔍 调试技巧

### 1. 验证参数设置

matlab

```
% 获取当前参数值验证
pid_params = {'TimeDomain', 'SampleTime', 'P', 'I', 'D', 'N'};
for i = 1:length(pid_params)
    value = get_param(gcb, pid_params{i});
    fprintf('%s = %s\n', pid_params{i}, value);
end
```



### 2. 检查参数是否存在

matlab

```
% 检查参数名是否存在
param_name = 'SampleTime';
try
    value = get_param(gcb, param_name);
    fprintf('参数 %s 存在，值 = %s\n', param_name, value);
catch
    fprintf('参数 %s 不存在\n', param_name);
end
```



## 📝 常见问题与解决

### Q1: 采样时间设置无效

**问题**：设置 `SampleTime` 但控制器仍使用连续时间
**解决**：必须先设置 `TimeDomain = 'Discrete-time'`

### Q2: 滤波器不工作

**问题**：N参数设置但微分项未滤波
**解决**：确保 `UseFilter = 'on'`

### Q3: 继承采样时间配置

**问题**：需要PID继承系统采样时间
**解决**：设置 `SampleTime = '-1'`

### Q4: 参数名找不到

**问题**：使用错误参数名（如 `Ts`, `Ki`）
**解决**：参考本文档参数名表格

## 🗂️ 相关参考

1. **Simulink PID Controller Block Documentation**
   - MathWorks官方文档：[PID Controller Block](https://www.mathworks.com/help/simulink/slref/pidcontroller.html)
2. **离散PID实现方法**
   - 前向欧拉 vs 后向欧拉 vs 梯形积分
   - 滤波器系数的物理意义
3. **采样时间继承规则**
   - `-1`: 继承
   - `0`: 连续
   - `>0`: 离散采样时间

## 📅 版本记录

| 版本 | 日期       | 说明                                  |
| :--- | :--------- | :------------------------------------ |
| 1.0  | 2024-XX-XX | 初始版本，基于 MATLAB R2024a 调试日志 |
| 1.1  | 2024-XX-XX | 添加配置示例和调试函数                |

------

**重要提示**：本文档基于 MATLAB R2024a 版本，不同版本可能存在参数名差异。建议在重要配置前先验证参数名存在性。