# 最小发射测试单元实施计划

> **给代理开发者：** 必须按任务逐项执行本计划。每个任务都使用测试先行；状态机任务可使用 `superpowers:subagent-driven-development` 或 `superpowers:executing-plans`，本次建议在当前会话中分阶段执行。

**目标：** 在现有 STM32F407 云台主控板工程的副本 `E:\STM32Projects\Shoot` 中，实现一个只控制两侧摩擦轮和拨弹轮的最小发射测试单元。

**架构：** 用不依赖 STM32 外设的 `ShooterController` 管理 `SAFE → SPINUP → READY → FIRING → COOLDOWN` 状态机和目标值，再由发射任务把目标值映射到现有 `cMotor`、PID、滑模和 CAN 缓冲区。DT7 和未来串口入口只负责生成统一的 `ShooterCommand`，不进入控制器内部。

**技术栈：** C++17、STM32F407、HAL、FreeRTOS/CMSIS-RTOS、现有 `cMotor`、PID、滑模控制器、CMake、主机侧 C++ 测试。

**设计文档：** `docs/superpowers/specs/2026-09-21-minimal-shooter-test-design.md`

## 全局约束

- 第一版只包含摩擦轮加速、到速判断、单发、连发和急停。
- 不包含 Yaw、Pitch、底盘、视觉、裁判系统热量、UI、堵转恢复和摩擦轮掉速计数。
- 左摩擦轮使用 `0x201`，右摩擦轮使用 `0x202`，拨弹轮使用 `0x203`。
- 左右摩擦轮目标速度固定为 `-6200 rpm` 和 `+6200 rpm`。
- 每发拨弹轮目标增加 45°输出轴角度。
- 急停、撤销解锁和 DT7 断联必须进入 `SAFE` 并将三个电机电流写为 0。
- 串口接收和解析不在本次实现；只提供可被未来串口适配器填充的外部命令接口。
- 所有新增行为先写失败测试，再写最小实现。

---

### 任务 1：建立主机侧测试骨架和失败测试

**文件：**
- 创建：`tests/CMakeLists.txt`
- 创建：`tests/ShooterControllerTest.cpp`
- 创建：`User/Shooter/ShooterCommand.hpp`
- 创建：`User/Shooter/ShooterController.hpp`

**接口：**
- 消费：设计文档中的 `ShooterCommand` 字段和五态状态机。
- 产出：`ShooterCommand`、`ShooterState`、`ShooterFeedback`、`ShooterOutput` 和 `ShooterController` 的公共接口，供后续任务和测试使用。

- [ ] **步骤 1：定义测试构建入口和第一批失败测试**

`tests/CMakeLists.txt` 使用主机编译器构建测试，不包含 HAL：

```cmake
cmake_minimum_required(VERSION 3.20)
project(ShooterControllerTests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(shooter_controller_test
    ShooterControllerTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter/ShooterController.cpp)
target_include_directories(shooter_controller_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../User/Shooter)
```

测试先声明期望接口并覆盖最小行为：

```cpp
#include "ShooterController.hpp"
#include <cassert>

static ShooterFeedback feedback(float left, float right, float feeder)
{
    return ShooterFeedback{left, right, feeder};
}

int main()
{
    ShooterController controller;
    ShooterCommand idle{};
    controller.reset(feedback(0, 0, 100));
    controller.update(idle, feedback(0, 0, 100));
    assert(controller.state() == ShooterState::Safe);
    assert(controller.output().left_speed_target == 0);
    assert(controller.output().right_speed_target == 0);

    ShooterCommand spinup{true, true, false, false, false};
    controller.update(spinup, feedback(0, 0, 100));
    assert(controller.state() == ShooterState::Spinup);
    assert(controller.output().left_speed_target == -6200);
    assert(controller.output().right_speed_target == 6200);

    for (int i = 0; i < 19; ++i)
        controller.update(spinup, feedback(-6200, 6200, 100));
    assert(controller.state() == ShooterState::Spinup);
    controller.update(spinup, feedback(-6200, 6200, 100));
    assert(controller.state() == ShooterState::Ready);

    ShooterCommand fire_once{true, true, true, false, false};
    controller.update(fire_once, feedback(-6200, 6200, 100));
    assert(controller.state() == ShooterState::Firing);
    assert(controller.output().feeder_target == 145);

    return 0;
}
```

- [ ] **步骤 2：运行失败测试**

运行：

```powershell
cmake -S tests -B tests/build
cmake --build tests/build --config Debug
```

预期：失败，原因是 `ShooterController.hpp`、`ShooterController.cpp` 和相关类型尚未实现，而不是编译器或路径错误。

- [ ] **步骤 3：提交测试骨架**

```powershell
git -C E:\STM32Projects\Shoot add tests User/Shooter
git -C E:\STM32Projects\Shoot commit -m "test: define shooter controller behavior"
```

### 任务 2：实现纯状态机控制器并通过主机测试

**文件：**
- 修改：`User/Shooter/ShooterCommand.hpp`
- 修改：`User/Shooter/ShooterController.hpp`
- 创建：`User/Shooter/ShooterController.cpp`
- 修改：`tests/ShooterControllerTest.cpp`

**接口：**
- 消费：任务 1 的失败测试。
- 产出：
  - `enum class ShooterState { Safe, Spinup, Ready, Firing, Cooldown };`
  - `struct ShooterFeedback { float left_speed; float right_speed; float feeder_position; };`
  - `struct ShooterOutput { float left_speed_target; float right_speed_target; float feeder_target; bool safe_output; };`
  - `class ShooterController { void reset(const ShooterFeedback&); void update(const ShooterCommand&, const ShooterFeedback&); ShooterState state() const; const ShooterOutput& output() const; };`

- [ ] **步骤 1：补充失败测试**

在现有测试中补充以下行为，每个行为独立断言：

```cpp
// 到速必须连续20个周期，离开范围会清零计数。
// fire_once 的高电平保持不能重复产生第二个45°增量。
// 释放短暂 fire_once 后，FIRING 仍然能在反馈到位时进入 COOLDOWN。
// cooldown 完成后 fire_hold 才能产生下一发。
// emergency_stop、armed=false、friction_on=false 在任意状态都回到 SAFE。
// feeder 到位未满足时，动作超时进入 SAFE。
```

测试配置使用固定参数：到速连续计数 20、到位误差 1°、到位连续计数 2、发射间隔 10 ms、拨弹动作超时 1000 ms。

- [ ] **步骤 2：运行测试确认按预期失败**

运行：

```powershell
cmake --build tests/build --config Debug
```

预期：失败，失败点对应尚未实现的状态转换或输出值。

- [ ] **步骤 3：实现最小状态机**

实现要求：

```cpp
void ShooterController::update(const ShooterCommand& command,
                                const ShooterFeedback& feedback)
{
    if (command.emergency_stop || !command.armed || !command.friction_on) {
        enterSafe(feedback.feeder_position);
        return;
    }

    switch (_state) {
    case ShooterState::Safe:
        beginSpinup();
        break;
    case ShooterState::Spinup:
        updateSpinup(feedback);
        break;
    case ShooterState::Ready:
        startShotIfRequested(command, feedback);
        break;
    case ShooterState::Firing:
        updateFiring(feedback);
        break;
    case ShooterState::Cooldown:
        updateCooldown(command);
        break;
    }
}
```

控制器只保存状态、计数器、上次 `fire_once` 电平和拨弹目标，不包含 `cMotor`、HAL、FreeRTOS 或全局对象。

- [ ] **步骤 4：运行主机测试确认通过**

运行：

```powershell
cmake --build tests/build --config Debug
tests/build/Debug/shooter_controller_test.exe
```

预期：进程退出码为 0。

- [ ] **步骤 5：提交纯状态机实现**

```powershell
git -C E:\STM32Projects\Shoot add tests User/Shooter
git -C E:\STM32Projects\Shoot commit -m "feat: add minimal shooter state machine"
```

### 任务 3：接入现有 CAN 电机与控制器

**文件：**
- 创建：`User/Shooter/ShooterMotors.hpp`
- 创建：`User/Shooter/ShooterMotors.cpp`
- 修改：`User/Shooter/ShooterController.hpp`
- 修改：`User/Shooter/ShooterController.cpp`

**接口：**
- 消费：任务 2 的 `ShooterOutput`。
- 产出：
  - `extern cMotor shooter_fric_left;`
  - `extern cMotor shooter_fric_right;`
  - `extern cMotor shooter_feeder;`
  - `void ShooterMotors_Init();`
  - `void ShooterMotors_Update(const ShooterOutput& output);`

- [ ] **步骤 1：先写 CAN 映射测试/检查函数**

在 `tests/ShooterControllerTest.cpp` 中只测试控制器输出目标，不伪造 HAL；CAN 字节映射通过代码审查和固件构建验证。明确要求 `ShooterMotors_Update()` 调用：

```cpp
shooter_fric_left.MotorCtrl.c_PID.setSpdTar(output.left_speed_target);
shooter_fric_right.MotorCtrl.c_PID.setSpdTar(output.right_speed_target);
shooter_feeder.MotorCtrl.c_SMC.ErrorUpdate(
    output.feeder_target,
    shooter_feeder.getEcd().total_angle,
    shooter_feeder.getEcd().speed);
```

- [ ] **步骤 2：实现电机初始化和控制计算**

初始化严格使用：

```cpp
shooter_fric_left.canInit(0x201, &hcan1, cMotor::M_3508);
shooter_fric_right.canInit(0x202, &hcan1, cMotor::M_3508);
shooter_feeder.canInit(0x203, &hcan1, cMotor::M_2006_p36);
```

速度 PID 复用原参数 `20, 0, 0, 16384`；拨弹滑模复用原 `EISMC` 参数。`SAFE` 时必须先清零速度目标、清零滑模输出并写入零电流。

- [ ] **步骤 3：实现 CAN 缓冲区写入**

每次更新依次执行：

```cpp
shooter_fric_left.MotorCtrl.c_PID.spdLoop(shooter_fric_left.getEcd().speed);
shooter_fric_right.MotorCtrl.c_PID.spdLoop(shooter_fric_right.getEcd().speed);
shooter_feeder.MotorCtrl.c_SMC.SmcCalculate();
shooter_fric_left.canSend(cMotor::ePid);
shooter_fric_right.canSend(cMotor::ePid);
shooter_feeder.canSend(cMotor::eExternal,
                       static_cast<int16_t>(shooter_feeder.MotorCtrl.c_SMC.Out()));
```

实际发送仍由任务调用 `CanReg[1].sendBuff()`，对应 CAN1 标准帧 `0x200`。

- [ ] **步骤 4：编译工程确认接口正确**

运行：

```powershell
cmake -S E:\STM32Projects\Shoot -B E:\STM32Projects\Shoot\cmake-build-shooter -DCMAKE_BUILD_TYPE=Debug
cmake --build E:\STM32Projects\Shoot\cmake-build-shooter --config Debug
```

预期：当前工程能够完成交叉编译；若旧 Gimbal/INS/Vision 源码阻止链接，在下一任务的 CMake/任务裁剪中处理，不在本任务引入业务逻辑修复。

- [ ] **步骤 5：提交电机适配层**

```powershell
git -C E:\STM32Projects\Shoot add User/Shooter
git -C E:\STM32Projects\Shoot commit -m "feat: connect shooter controller to CAN motors"
```

### 任务 4：接入 DT7 和预留串口命令入口

**文件：**
- 创建：`User/Shooter/ShooterRcAdapter.hpp`
- 创建：`User/Shooter/ShooterRcAdapter.cpp`
- 创建：`User/Shooter/ShooterExternalCommand.hpp`
- 修改：`User/Framework/RemoteC/RemoteC.cpp`（仅在需要访问在线计时接口时）

**接口：**
- 消费：现有 `RC_GetDatas()`、`RC_GetNewData.live_time` 和任务 2 的 `ShooterCommand`。
- 产出：
  - `ShooterCommand ShooterCommand_FromDt7();`
  - `void ShooterCommand_SetExternal(const ShooterCommand&);`
  - `ShooterCommand ShooterCommand_GetExternal();`

- [ ] **步骤 1：写适配器测试表**

在 `tests` 中增加纯函数映射测试，固定约定：

```text
DT7 右拨杆中位 s[0] == 3 且遥控在线 → armed=true
DT7 左拨杆上位 s[1] == 1           → friction_on=true
拨盘 ch[4] 从 >= -100 变为 < -100    → fire_once=true
拨盘 ch[4] < -500                   → fire_hold=true
live_time <= 0                       → armed=false、friction_on=false、fire_* = false
```

这些阈值集中定义在适配器头文件中，后续调车只改常量。

- [ ] **步骤 2：运行失败测试**

运行：

```powershell
cmake --build tests/build --config Debug
```

预期：新增映射测试因适配器函数尚未实现而失败。

- [ ] **步骤 3：实现 DT7 适配器和外部命令存储**

DT7 适配器维护上一次拨盘阈值状态，只在本次低于 -100 且上次未低于 -100 时生成 `fire_once`。外部命令存储默认返回全零安全命令，不实现任何串口协议。

- [ ] **步骤 4：运行主机测试**

运行：

```powershell
cmake --build tests/build --config Debug
tests/build/Debug/shooter_controller_test.exe
```

预期：所有状态机和 DT7 映射测试通过。

- [ ] **步骤 5：提交输入适配层**

```powershell
git -C E:\STM32Projects\Shoot add tests User/Shooter User/Framework/RemoteC/RemoteC.cpp
git -C E:\STM32Projects\Shoot commit -m "feat: add DT7 shooter command adapter"
```

### 任务 5：替换运行时任务为最小发射任务

**文件：**
- 创建：`User/Tasks/ShooterTask/ShooterTask.hpp`
- 创建：`User/Tasks/ShooterTask/ShooterTask.cpp`
- 修改：`Core/Startup/mainpp.cpp`
- 修改：`Core/Src/freertos.c`
- 修改：`CMakeLists.txt`

**接口：**
- 消费：`ShooterController`、`ShooterMotors`、`ShooterCommand_FromDt7()` 和外部命令接口。
- 产出：唯一业务控制任务 `ShooterTask(void const*)`，周期固定为 1 ms。

- [ ] **步骤 1：实现发射任务入口**

任务每个周期执行：

```cpp
ShooterCommand command = ShooterCommand_FromDt7();
merge_external_command(command, ShooterCommand_GetExternal());
ShooterFeedback feedback{
    shooter_fric_left.getEcd().speed,
    shooter_fric_right.getEcd().speed,
    shooter_feeder.getEcd().total_angle};
controller.update(command, feedback);
ShooterMotors_Update(controller.output());
CanReg[1].sendBuff();
vTaskDelayUntil(&last_wake, 1 / portTICK_RATE_MS);
```

合并规则为：急停、撤销解锁和断联任一来源有效就安全；摩擦轮和开火请求采用 OR，但 `armed` 必须由 DT7 在线且解锁或外部命令明确解锁。

- [ ] **步骤 2：修改启动初始化**

`UserInit()` 只保留 CAN1、遥控器和发射电机初始化；移除 WIT IMU、图传/裁判、Gimbal 初始化和不属于最小发射单元的定时器启动。

- [ ] **步骤 3：修改 FreeRTOS 任务创建**

删除 Gimbal、Vision、INS 任务的创建，创建 `Shooter_Task`；保留必要的系统空闲/定时任务。不要调用原 `GimbalLoop()` 或 `Shootc::SetShootLoop()`。

- [ ] **步骤 4：裁剪 CMake 源文件依赖**

在 `CMakeLists.txt` 的递归源列表生成后排除原业务入口和不再运行的模块：

```cmake
list(FILTER SOURCES EXCLUDE REGEX "/User/Gimbal/")
list(FILTER SOURCES EXCLUDE REGEX "/User/Framework/ShootC/")
list(FILTER SOURCES EXCLUDE REGEX "/User/Tasks/GimbalTask/")
list(FILTER SOURCES EXCLUDE REGEX "/User/Tasks/visioncom_task/")
list(FILTER SOURCES EXCLUDE REGEX "/User/Tasks/INSTask/")
```

保留 `RemoteC`、`REMOTEC`、CAN、MotorC、PID、Sliding 和新增 Shooter 文件。

- [ ] **步骤 5：交叉编译和检查链接依赖**

运行：

```powershell
Remove-Item -LiteralPath E:\STM32Projects\Shoot\cmake-build-shooter -Recurse -Force
cmake -S E:\STM32Projects\Shoot -B E:\STM32Projects\Shoot\cmake-build-shooter -DCMAKE_BUILD_TYPE=Debug
cmake --build E:\STM32Projects\Shoot\cmake-build-shooter --config Debug
```

预期：生成 `.elf`、`.hex` 和 `.bin`，且链接错误中不再出现 Gimbal、Vision、INS 或 Shootc 的业务符号。

- [ ] **步骤 6：提交最小运行时**

```powershell
git -C E:\STM32Projects\Shoot add Core CMakeLists.txt User/Tasks/ShooterTask User/Shooter
git -C E:\STM32Projects\Shoot commit -m "feat: run minimal shooter task"
```

### 任务 6：硬件前安全验证和最终整理

**文件：**
- 修改：`docs/superpowers/specs/2026-09-21-minimal-shooter-test-design.md`（仅记录与实现一致的参数，如有必要）
- 创建：`docs/shooter-test-checklist.md`

**接口：**
- 消费：任务 5 的固件产物和状态机测试。
- 产出：可照着执行的无弹台架测试清单，不引入新业务功能。

- [ ] **步骤 1：运行全部主机测试**

```powershell
cmake --build E:\STM32Projects\Shoot\tests\build --config Debug
E:\STM32Projects\Shoot\tests\build\Debug\shooter_controller_test.exe
```

预期：退出码为 0。

- [ ] **步骤 2：检查固件产物和 CAN 映射**

确认：

```text
CAN1、标准帧 0x200
0x201 → 字节0~1
0x202 → 字节2~3
0x203 → 字节4~5
```

- [ ] **步骤 3：写无弹台架清单**

清单必须按以下顺序：

```text
断开电机动力电源 → 观察 SAFE 零电流 CAN 帧
单独启动左/右摩擦轮 → 核对方向
无弹启动摩擦轮 → 核对 ±6200 rpm 目标与到速门控
无弹单发 → 核对拨弹轮只移动45°
保持 fire_hold → 核对每发间隔和目标递增
断开DT7/按急停 → 核对三个电流立即归零
```

- [ ] **步骤 4：提交测试清单并完成最终审查**

```powershell
git -C E:\STM32Projects\Shoot add docs
git -C E:\STM32Projects\Shoot commit -m "docs: add shooter bench test checklist"
git -C E:\STM32Projects\Shoot status --short --branch
```

最终审查确认：没有把 `cmake-build-*`、`.idea` 或整个复制工程作为本次功能提交的一部分；没有启用热量、堵转、视觉或 UI 逻辑。
