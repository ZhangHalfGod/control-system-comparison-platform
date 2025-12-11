# MATLAB脚本使用指南

## 一、概述

本指南详细介绍了ODE数值求解方法实现中的各个MATLAB脚本的作用、启动方式和依赖关系，帮助用户正确执行和使用这些脚本。

## 二、目录结构

```
ODE_Solvers/
├── runge_kutta/          # 龙格-库塔方法
│   ├── rk4.m             # 四阶龙格-库塔方法实现
│   ├── rkf45.m           # RKF45自适应步长方法实现
│   ├── test_rk4.m        # RK4测试脚本
│   └── test_rkf45.m      # RKF45测试脚本
├── multi_step/           # 多步方法
│   ├── adams_bashforth.m     # Adams-Bashforth方法实现（2-4阶）
│   ├── adams_moulton.m       # Adams-Moulton方法实现（2-4阶）
│   └── test_multi_step_methods.m  # 多步方法测试脚本
├── stiff_solvers/        # 刚性方程求解
│   ├── backward_euler.m      # Backward Euler方法实现
│   └── test_stiff_solvers.m  # 刚性方程求解测试脚本
├── ode_solvers_theory.md  # ODE求解理论文档
└── script_usage_guide.md  # 脚本使用指南（本文档）
```

## 三、脚本说明与启动方式

### 1. 龙格-库塔方法

#### 1.1 rk4.m
- **作用**：实现四阶龙格-库塔方法，用于数值求解常微分方程
- **类型**：MATLAB函数文件
- **启动方式**：不能直接执行，需要在其他脚本或MATLAB命令窗口中调用
- **调用示例**：
  ```matlab
  f = @(t, y) -y + sin(t);  % 定义导数函数
  [t, y] = rk4(f, 0, 1, 10, 0.1);  % 调用RK4方法
  ```

#### 1.2 rkf45.m
- **作用**：实现Runge-Kutta-Fehlberg (RKF45)自适应步长方法
- **类型**：MATLAB函数文件
- **启动方式**：不能直接执行，需要在其他脚本或MATLAB命令窗口中调用
- **调用示例**：
  ```matlab
  f = @(t, y) -y + sin(t);  % 定义导数函数
  [t, y, h_used] = rkf45(f, 0, 1, 10, 0.1, 1e-5);  % 调用RKF45方法
  ```

#### 1.3 test_rk4.m
- **作用**：测试四阶龙格-库塔方法的性能和准确性
- **类型**：MATLAB脚本文件
- **启动方式**：
  - 在MATLAB命令窗口中输入：`run('test_rk4.m')`
  - 或直接点击MATLAB工具栏的"Run"按钮
- **依赖关系**：需要与`rk4.m`在同一目录下
- **输出**：控制台打印测试结果，生成多个图形窗口展示测试结果

#### 1.4 test_rkf45.m
- **作用**：测试RKF45自适应步长方法的性能和准确性
- **类型**：MATLAB脚本文件
- **启动方式**：
  - 在MATLAB命令窗口中输入：`run('test_rkf45.m')`
  - 或直接点击MATLAB工具栏的"Run"按钮
- **依赖关系**：需要与`rkf45.m`在同一目录下
- **输出**：控制台打印测试结果，生成图形窗口展示测试结果

### 2. 多步ODE方法

#### 2.1 adams_bashforth.m
- **作用**：实现Adams-Bashforth多步ODE方法（2-4阶）
- **类型**：MATLAB函数文件
- **启动方式**：不能直接执行，需要在其他脚本或MATLAB命令窗口中调用
- **调用示例**：
  ```matlab
  f = @(t, y) -y + sin(t);  % 定义导数函数
  [t, y] = adams_bashforth(f, 0, 1, 10, 0.1, 4);  % 调用4阶Adams-Bashforth方法
  ```

#### 2.2 adams_moulton.m
- **作用**：实现Adams-Moulton多步ODE方法（2-4阶）
- **类型**：MATLAB函数文件
- **启动方式**：不能直接执行，需要在其他脚本或MATLAB命令窗口中调用
- **调用示例**：
  ```matlab
  f = @(t, y) -y + sin(t);  % 定义导数函数
  [t, y] = adams_moulton(f, 0, 1, 10, 0.1, 4);  % 调用4阶Adams-Moulton方法
  ```

#### 2.3 test_multi_step_methods.m
- **作用**：测试Adams-Bashforth和Adams-Moulton多步方法的性能和准确性
- **类型**：MATLAB脚本文件
- **启动方式**：
  - 在MATLAB命令窗口中输入：`run('test_multi_step_methods.m')`
  - 或直接点击MATLAB工具栏的"Run"按钮
- **依赖关系**：
  - 需要与`adams_bashforth.m`和`adams_moulton.m`在同一目录下
  - 需要调用`rk4.m`，因此需要确保`rk4.m`所在目录在MATLAB路径中
- **输出**：控制台打印测试结果，生成图形窗口展示测试结果

### 3. 刚性方程求解

#### 3.1 backward_euler.m
- **作用**：实现Backward Euler方法，用于求解刚性常微分方程
- **类型**：MATLAB函数文件
- **启动方式**：不能直接执行，需要在其他脚本或MATLAB命令窗口中调用
- **调用示例**：
  ```matlab
  f = @(t, y) -1000*y + 1000*sin(t) + cos(t);  % 定义刚性方程
  [t, y] = backward_euler(f, 0, 0, 10, 0.01);  % 调用Backward Euler方法
  ```

#### 3.2 test_stiff_solvers.m
- **作用**：测试刚性方程求解方法的性能和准确性
- **类型**：MATLAB脚本文件
- **启动方式**：
  - 在MATLAB命令窗口中输入：`run('test_stiff_solvers.m')`
  - 或直接点击MATLAB工具栏的"Run"按钮
- **依赖关系**：
  - 需要与`backward_euler.m`在同一目录下
  - 需要调用`rk4.m`，因此需要确保`rk4.m`所在目录在MATLAB路径中
- **输出**：控制台打印测试结果，生成图形窗口展示测试结果

## 四、如何在MATLAB中执行脚本

### 1. 准备工作

1. **启动MATLAB**：打开MATLAB软件
2. **设置当前文件夹**：在MATLAB左侧的"当前文件夹"窗口中，导航到包含要执行脚本的目录
   - 例如，要执行`test_rkf45.m`，应将当前文件夹设置为`d:\trae_git\Simulink_test\Matlab_Experiment\Experiments\Math_Foundations\ODE_Solvers\runge_kutta`
3. **检查MATLAB路径**：确保所有依赖的函数文件都在MATLAB路径中
   - 可以使用`addpath`命令添加路径，例如：`addpath('d:\trae_git\Simulink_test\Matlab_Experiment\Experiments\Math_Foundations\ODE_Solvers\runge_kutta')`

### 2. 执行脚本的多种方式

#### 方式1：使用"Run"按钮
1. 在MATLAB编辑器中打开要执行的脚本文件
2. 点击MATLAB工具栏上的"Run"按钮

#### 方式2：在命令窗口中直接输入文件名
1. 确保当前文件夹已设置为脚本所在目录
2. 在命令窗口中直接输入脚本文件名（不带`.m`扩展名），例如：
   ```matlab
   test_rkf45
   ```

#### 方式3：使用`run`命令
1. 在命令窗口中使用`run`命令执行脚本，例如：
   ```matlab
   run('test_rkf45.m')
   ```
2. 也可以使用完整路径，例如：
   ```matlab
   run('d:\trae_git\Simulink_test\Matlab_Experiment\Experiments\Math_Foundations\ODE_Solvers\runge_kutta\test_rkf45.m')
   ```

### 3. 执行函数文件

函数文件不能直接执行，需要在其他脚本或命令窗口中调用：

1. 在命令窗口中定义必要的输入参数
2. 调用函数，例如：
   ```matlab
   f = @(t, y) -y + sin(t);  % 定义导数函数
   [t, y, h_used] = rkf45(f, 0, 1, 10, 0.1, 1e-5);  % 调用RKF45函数
   ```

## 五、常见问题及解决方案

### 1. 问题：无法找到函数文件
- **错误信息**：`Undefined function or variable 'rk4'`
- **解决方案**：确保函数文件与调用它的脚本在同一目录下，或已将函数文件所在目录添加到MATLAB路径中

### 2. 问题：输入参数不匹配
- **错误信息**：`Error using rkf45 (line 45). Not enough input arguments.`
- **解决方案**：检查函数调用时的输入参数数量和类型是否与函数定义匹配

### 3. 问题：脚本运行无输出
- **解决方案**：检查脚本中是否有`clear all; close all; clc;`命令，这些命令会清除工作区、关闭所有窗口并清除命令窗口

### 4. 问题：图形窗口不显示
- **解决方案**：检查脚本中是否有`figure`、`plot`等绘图命令，确保没有被注释掉

## 六、脚本执行示例

### 示例1：执行test_rkf45.m

1. 启动MATLAB
2. 在"当前文件夹"窗口中导航到`d:\trae_git\Simulink_test\Matlab_Experiment\Experiments\Math_Foundations\ODE_Solvers\runge_kutta`
3. 在命令窗口中输入：`test_rkf45`
4. 观察命令窗口输出和生成的图形窗口

### 示例2：执行test_multi_step_methods.m

1. 启动MATLAB
2. 在"当前文件夹"窗口中导航到`d:\trae_git\Simulink_test\Matlab_Experiment\Experiments\Math_Foundations\ODE_Solvers\multi_step`
3. 添加rk4.m所在目录到MATLAB路径：
   ```matlab
   addpath('d:\trae_git\Simulink_test\Matlab_Experiment\Experiments\Math_Foundations\ODE_Solvers\runge_kutta')
   ```
4. 在命令窗口中输入：`test_multi_step_methods`
5. 观察命令窗口输出和生成的图形窗口

## 七、随机系统分析工具使用指南

### 1. 目录结构

```
Probability/
├── stochastic_systems/  # 随机系统分析工具
│   ├── ito_solver.m           # 伊藤微分方程求解器
│   └── test_stochastic_systems.m  # 随机系统分析测试脚本
```

### 2. 脚本说明与启动方式

#### 2.1 ito_solver.m
- **作用**：实现伊藤随机微分方程求解器，包括Euler-Maruyama方法和Milstein方法
- **类型**：MATLAB函数文件
- **启动方式**：不能直接执行，需要在其他脚本或MATLAB命令窗口中调用
- **调用示例**：
  ```matlab
  f = @(t, y) 0.1*y;  % 漂移项函数
  g = @(t, y) 0.2*y;  % 扩散项函数
  [t, y] = ito_solver(f, g, 0, 100, 1, 0.001, 50);  % 调用伊藤微分方程求解器
  ```

#### 2.2 test_stochastic_systems.m
- **作用**：测试随机系统分析工具的性能和准确性
- **类型**：MATLAB脚本文件
- **启动方式**：
  - 在MATLAB命令窗口中输入：`run('test_stochastic_systems.m')`
  - 或直接点击MATLAB工具栏的"Run"按钮
- **依赖关系**：需要与`ito_solver.m`在同一目录下
- **输出**：控制台打印测试结果，生成图形窗口展示测试结果

### 3. 执行示例

1. 启动MATLAB
2. 在"当前文件夹"窗口中导航到`d:\trae_git\Simulink_test\Matlab_Experiment\Experiments\Math_Foundations\Probability\stochastic_systems`
3. 在命令窗口中输入：`test_stochastic_systems`
4. 观察命令窗口输出和生成的图形窗口

## 八、总结

本指南详细介绍了所有实现的MATLAB脚本的作用、启动方式和依赖关系，希望能帮助用户正确执行和使用这些脚本。如果在执行过程中遇到问题，可以参考"常见问题及解决方案"部分，或检查MATLAB命令窗口中的错误信息，根据错误信息进行排查。

## 九、更新日志

- **2025-12-11**：创建本指南，记录所有脚本的作用和启动方式
