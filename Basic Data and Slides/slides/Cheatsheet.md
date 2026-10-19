<link rel="stylesheet" href="./print.css">

# EE114 嵌入式系统 开卷考试 Cheatsheet

> 覆盖 Lect1–18：8051 → ARM Cortex‑M3 / STM32F103 → 外设（GPIO/USART/SPI/I2C/Timer/PWM/ADC/DAC/DMA） → CPS / 传感器 / 电路常识

---

## 1. 计算机体系结构基础

| 项 | 冯·诺依曼 (Von Neumann) | 哈佛 (Harvard) |
|---|---|---|
| 程序与数据 | **存储在同一存储器，统一编址** | 程序ROM、数据RAM **分开** |
| 总线 | 一条 | 指令总线 + 数据总线（可并行）|
| 代表 | 8086、早期PC | **8051、ARM Cortex‑M3** |

- **三级流水线顺序**：**取指 (Fetch) → 译码 (Decode) → 执行 (Execute)**
- **CPU基本组成**：ALU、寄存器组、控制器（CU）、PC、IR、内部总线
- **总线**：地址总线（单向）、数据总线（双向）、控制总线
- **RAM**：易失，存储**动态加载的程序代码、变量、堆栈**；**ROM/Flash**：存储启动代码与程序

### CPS / 嵌入式系统
- **CPS (Cyber‑Physical Systems)**：感知 → 计算 → 通信 → 执行 → 反馈，物理过程与计算紧耦合
- **嵌入式系统**：面向应用，软硬件可裁剪，对实时性、功耗、成本敏感
- **Transducer**：传感器（物理量→电信号）、执行器（电信号→物理量），需调理电路（放大/滤波/隔离）
- **采样定理**：fs ≥ 2 fmax 否则混叠

---

## 2. 8051 单片机要点

### 2.1 资源 & 体系
- 8 位 CISC，**哈佛结构**；**机器周期 = 12 个振荡周期**
- 内部 ROM 4KB（8052: 8KB），内部 RAM 128B（8052: 256B），SFR 区 80H–FFH
- 数据指针 DPTR=16 bit（DPH/DPL）；堆栈指针 SP 复位值 = 07H
- PSW 标志位：CY、AC、F0、RS1、RS0、OV、—、P

### 2.2 GPIO P0–P3
- **P0** 内部无上拉，作 I/O 需外接上拉；可复用为地址/数据低 8 位
- **P1** 准双向，内部上拉
- **P2** 准双向，作高 8 位地址
- **P3** 准双向，复用功能：

| Pin | 功能 |
|---|---|
| P3.0 | RXD |
| P3.1 | TXD |
| P3.2 | /INT0 |
| P3.3 | /INT1 |
| P3.4 | T0 |
| P3.5 | T1 |
| P3.6 | /WR |
| P3.7 | /RD |

### 2.3 中断（5 个中断源）

| 源 | 向量 | 优先顺位（默认）|
|---|---|---|
| /INT0 | 0003H | 1（最高）|
| T0 | 000BH | 2 |
| /INT1 | 0013H | 3 |
| T1 | 001BH | 4 |
| 串口 RI/TI | 0023H | 5 |

- IE = EA / — / ET2 / ES / ET1 / EX1 / ET0 / EX0
- 中断函数：`void isr() interrupt n [using m]`

### 2.4 定时器 / 计数器 (T0, T1)
- TMOD 高 4 位控 T1，低 4 位控 T0；GATE、C/T、M1、M0
- 模式：**0=13bit, 1=16bit, 2=8bit 自动重装, 3=T0 分裂为两个 8bit**
- TCON：TF1 TR1 TF0 TR0 IE1 IT1 IE0 IT0
- **软件定时缺点**：占 CPU、精度差、不能后台运行 ➜ 不可说"没有缺点"

### 2.5 串口
- SCON：SM0 SM1 SM2 REN TB8 RB8 TI RI
- 模式 1（8 位 UART）波特率 = (2^SMOD / 32) × T1 溢出率
- 9600bps @ 11.0592MHz，模式 2 → TH1 = TL1 = 0xFD

---

## 3. C 语言常考易错点

### 3.1 宏定义陷阱（**重点！**）
- 宏是**纯文本替换**，没有运算优先级 ➜ 必须给参数与整体加括号。
- `#define MIN(x,y) (x)<(y)?(x):(y)`
  - `10 * MIN(i,j)` 展开为 `10*(i)<(j)?(i):(j)` ≡ `(10*i<j)?i:j`
  - i=10,j=15 ⇒ `100<15? i:j` ⇒ 取 **j = 15** ➜ 输出 **15**
- `#define N 2`, `#define Y(n) ((N+1)*n)`, `z=2*(N+Y(5))`
  - = `2*(2 + (2+1)*5)` = `2*(2+15)` = **34**

### 3.2 其它
- `main()` 在源程序中**位置任意**（C 不强制）
- `&` 位与 vs `&&` 逻辑与；`|` vs `||`；`==` vs `=`
- `volatile` 修饰中断 / 寄存器 / 外部硬件变量，防止编译器优化
- C 运算符优先级（高→低）：`() [] -> .` → 单目 → `* / %` → `+ -` → `<< >>` → 关系 → `==/!=` → `& ^ |` → `&& ||` → `?:` → 赋值 → 逗号
- `sizeof` 是运算符，返回 `size_t`

### 3.3 STM32 寄存器三种访问方式
```c
GPIOB->ODR = 0xFFFF;                            // 标准库结构体（最推荐）
*(unsigned int*)(0x40010C0C) = 0xFFFF;          // 直接绝对地址
GPIO_SetBits(GPIOB, GPIO_Pin_All);              // 标准外设库函数
```
> `0xFFFF` 仅 16 位、是 GPIOB 的 ODR；直接指针方式不需要 `#define`、但要**类型转换**才能取写。

---

## 4. ARM Cortex‑M3 & STM32F103

### 4.1 内核
- 32‑bit ARMv7‑M，**哈佛结构**，**三级流水线**（取指‑译码‑执行）
- Thumb‑2 指令集（16/32 位混合），位带 (bit‑banding)，硬件除法
- **NVIC（嵌套向量中断控制器）**集成在内核
- 寄存器：R0–R12、R13=SP（MSP/PSP）、R14=LR、R15=PC，xPSR

### 4.2 总线 / 存储映射
- **AHB**：高速（≤72MHz）；**APB2**：高速外设（≤72MHz）；**APB1**：低速（≤36MHz）
- 0x0800_0000：Flash（启动后映射 0x0000_0000）
- 0x2000_0000：SRAM
- 0x4000_0000：外设；0xE000_0000：PPB（NVIC/SysTick）
- BOOT0/BOOT1 选择启动源：Flash / 系统存储器(Bootloader) / SRAM

| BOOT0 | BOOT1 | 启动介质 | 用途 |
|---|---|---|---|
| 0 | X | 主 Flash | 正常运行用户程序 |
| 1 | 0 | 系统存储器 | 内置 Bootloader，可经 USART 等下载 |
| 1 | 1 | SRAM | 调试/特殊场景 |

### 4.3 CMSIS / 库分层
- **CMSIS**：ARM 规定的统一硬件抽象**标准**（不是强制使用，但厂商遵守）。如 `SystemInit()`、`NVIC_SetPriority()`。
- **HAL / 标准外设库**：直接寄存器封装 → 函数接口
- **BSP（板级支持包）**：在 HAL 库基础上**再次封装，隐藏板级硬件细节，给应用层统一接口**（≠ 启动文件，≠ OS 内核）
- 关键头文件：
  - `stm32f10x.h` ≈ 8051 的 `reg52.h`：寄存器、结构体、位定义
  - `stm32f10x_conf.h`：外设库**配置开关**（注释/打开使用的模块）

### 4.4 启动流程
1. 上电 → 取栈顶地址（向量表 [0]）
2. 取 Reset_Handler 入口（向量表 [1]）
3. `startup_stm32f10x_xx.s` → 初始化堆栈 → 调用 **`SystemInit()`**（配 PLL/时钟） → 跳转 `__main` → `main()`

---

## 5. RCC 时钟树

- **HSI**：内部 8 MHz RC，**复位后默认 SYSCLK 源**
- **HSE**：外部晶振，常用 8 MHz；**LSE**：32.768 kHz（RTC）；**LSI**：约 40 kHz（IWDG / AWU）
- **PLL**：倍频；输入可选 HSI/2 或 HSE/HSE÷2，倍频 ×2~×16；最高输出 72 MHz
- **SYSCLK 来源**：HSI / HSE / **PLL**（三选一）✓
- 经 AHB 预分频 → **HCLK**（≤72M）
  - → APB1 分频 → **PCLK1**（≤36M）；默认 PCLK1=36M，TIM2~7 时钟 = 72M（APB 分频不为 1 时定时器时钟×2）
  - → APB2 分频 → **PCLK2**（≤72M）；默认 PCLK2=72M，TIM1/8 时钟 = 72M（APB 分频为 1 时不×2），ADC 由 PCLK2÷2/4/6/8
  - → SysTick：HCLK 或 **HCLK/8**
- 任何外设使用前都需先在 RCC_APBxENR/AHBENR 中**开时钟**

---

## 6. STM32 GPIO

### 6.1 8 种工作模式（CRL/CRH 中 4 位/引脚：CNF[1:0] + MODE[1:0]）
**输入 4 种**：模拟输入 / 浮空输入 / 上拉输入 / 下拉输入
**输出 4 种**：
1. 推挽 (PP) 输出
2. 开漏 (OD) 输出（需外部上拉）
3. 复用推挽 (AF_PP)
4. 复用开漏 (AF_OD)
> ⚠️ **没有 "浮空输出"**（浮空仅是输入）；只有"推挽/开漏/复用推挽/复用开漏"四种输出。

### 6.2 速度
- 2 MHz / 10 MHz / 50 MHz；高频信号（SPI、USART）选 50 MHz

### 6.3 电流
- 单个 I/O 最大灌/拉 **20 mA**；全部 I/O 总电流 **≤ 150 mA**

### 6.4 复位状态
- 复位后绝大多数 I/O **= 浮空输入**（JTAG/SWD 引脚例外，PA13/14/15、PB3/4 默认复用）
- **IDR 复位值不固定**（随外部电平），**ODR 复位值 = 0x0000**

### 6.5 关键寄存器
| 寄存器 | 作用 | 复位值 |
|---|---|---|
| GPIOx_CRL / CRH | 配置（每脚 4bit）| 0x44444444 = 全浮空输入 |
| GPIOx_IDR | 输入数据（只读，bit16‑31 保留）| 不定 |
| GPIOx_ODR | 输出数据 | 0x00000000 |
| GPIOx_BSRR | **写 1** 置位 / 复位（原子操作）|  — |
| GPIOx_BRR | 仅清零 | — |
| GPIOx_LCKR | 配置锁定 | — |

---

## 7. NVIC 中断 & EXTI & SysTick

### 7.1 NVIC
- 256 个向量；STM32F103 有 **60 个可屏蔽外部中断**
- **允许多个中断配置成相同优先级**；但抢占优先级相同时**不能互相嵌套打断**，响应/子优先级只决定 pending 后的响应顺序
- 固定异常优先级：**Reset = -3（最高）**，**NMI = -2**，**HardFault = -1**；普通可编程优先级数值越小越高
- 优先级寄存器 8bit，STM32 只用高 **4 位**（16 级）→ 5 种分组：

| Group | 抢占 (Preempt) | 子 (Sub) |
|---|---|---|
| 0 | 0 bit | 4 bit |
| 1 | 1 bit | 3 bit |
| 2 | 2 bit | 2 bit |
| 3 | 3 bit | 1 bit |
| 4 | 4 bit | 0 bit |

### 7.2 EXTI（外部中断 / 事件）
- 19 条：EXTI0–15（GPIO）、16=PVD、17=RTC Alarm、18=USB Wakeup
- **同号 Pin 互斥**：同一时刻 PAx、PBx、PCx 只能选一路接到 EXTIx
- 共享中断号：EXTI9_5_IRQn、EXTI15_10_IRQn
- 触发：上升 / 下降 / 双边沿
- 配置步骤：开 GPIO+AFIO 时钟 → GPIO 输入模式 → `GPIO_EXTILineConfig()` 映射端口到 EXTIx → `EXTI_Init()` 设置线/触发 → `NVIC_Init()` → 写 `EXTIx_IRQHandler()` 并清 pending

### 7.3 SysTick
- 24 位**向下计数**，自动重装
- CLKSOURCE：1=HCLK（72MHz）/ 0=HCLK/8（9MHz）
- `SysTick_Config(72000)` → 1 ms 中断
- 常用于 OS Tick 与延时

---

## 8. USART / UART

- 异步：1 起始 + 8/9 数据 + 可选奇偶 + 1/1.5/2 停止
- 波特率：`USARTDIV = fCK / (16 × BaudRate)`，`BRR` 存 USARTDIV 的 mantissa/fraction（例：USART1@72MHz, 115200bps → USARTDIV≈39.0625 → BRR≈0x0271）
- 关键寄存器：USART_SR (TXE/RXNE/TC/IDLE/ORE)、DR、CR1 (UE/TE/RE/PCE/PS/M)、BRR
- **TTL 0–3.3V** ；**RS‑232 ±3~15V，反相**；**RS‑485 差分平衡**
- 引脚：GPIO 复用推挽 50MHz（TX）+ 浮空/上拉输入（RX）

---

## 9. SPI

- 4 线全双工同步：**SCK / MOSI / MISO / NSS**
- 4 种模式 by **CPOL/CPHA**：
  - 0/0 空闲低，第一个上升沿采样
  - 0/1 空闲低，第一个下降沿采样
  - 1/0 空闲高，第一个下降沿采样
  - 1/1 空闲高，第一个上升沿采样
- STM32F1：`fSCK = fPCLK / Prescaler`，最小分频为 2。**SPI1 在 APB2，最高 36 Mbit/s = PCLK2/2**；**SPI2/3 在 APB1，最高 18 Mbit/s = PCLK1/2**。
- 寄存器 SPI_CR1（BR[2:0]分频、CPOL、CPHA、MSTR、SSI、SSM、LSBFIRST、SPE）

---

## 10. I²C

- 两线半双工：SCL + SDA（**开漏**，必须**外部上拉电阻**~4.7kΩ）
- 起始：SCL=H 时 SDA ↓；停止：SCL=H 时 SDA ↑
- 速率：标准 100k / 快速 400k / 高速 3.4M
- 第 9 bit 为 ACK（接收方拉低 SDA）
- 7 bit 或 10 bit 地址 + R/W
- 多主仲裁 + 时钟同步

---

## 11. 定时器 & PWM

### 11.1 TIM 分类（STM32F103）
| 类型 | 编号 | 特性 |
|---|---|---|
| 高级 | TIM1, TIM8 | 16 bit, 带死区 / 互补输出 / 刹车 |
| 通用 | TIM2–5 | 16 bit, 4 通道，可生成 PWM |
| 基本 | TIM6, TIM7 | 仅做计数 / 触发 DAC，无 I/O |

> ✅ 通用 **和** 高级定时器都可输出 PWM；基本定时器不行。

### 11.2 关键寄存器
- TIMx_PSC（预分频）：实际分频 = PSC+1
- TIMx_ARR（自动重装）：周期 = (PSC+1)(ARR+1)/fCK
- TIMx_CNT：当前计数
- TIMx_CCRx：捕获 / 比较值

### 11.3 PWM
- **频率由 ARR 决定**；**占空比由 CCR 决定**（占空比 = CCR/(ARR+1)）
- 模式：
  - **PWM1（mode 6）**：CNT < CCR 时输出**有效电平（高）**，≥ 则低
  - **PWM2（mode 7）**：CNT < CCR 时输出**低**，≥ 则高
  - 两者相位差 180°；亦可用 OCxREF 极性反相实现，但 PWM1/2 仍有意义（影响极性 + 触发逻辑）
- 对齐：边沿对齐 / 中央对齐 1/2/3

---

## 12. ADC / DAC

### 12.1 ADC
- 12 位 **逐次逼近 (SAR)**；分辨率 = Vref / 4096（典型 3.3 V → 0.806 mV）
- 转换流程：**采样 → 量化 → 编码**
- 通道：16 个外部 + 内部温度 + Vrefint；规则组（最多 16）+ 注入组（最多 4）
- 模式：单次 / 连续 / 扫描 / 不连续
- ADCCLK 最高 14 MHz（由 PCLK2 分频 2/4/6/8 得到）
- 转换时间 = 采样时间 + 12.5 ADCCLK

### 12.2 DAC
- 12 位电压输出，2 通道；可触发产生噪声 / 三角波；可与定时器、DMA 联动
- 输出缓冲使能时低输出阻抗

### 12.3 DMA
- F103：DMA1（7 通道）+ DMA2（5 通道）
- 方向：M2M / M2P / P2M
- 循环模式（Circular）：到达末尾自动从头开始
- 优先级：软件 4 级（低/中/高/极高）+ 硬件（通道号小优先）
- 配合 ADC/USART/SPI/I²C 减少 CPU 占用
- 过程：DMA 请求 → 仲裁/响应 → DMA 获得总线并按源/目的地址传输 → 计数到 0 后置标志/中断，CPU 不再逐字节搬运

---

## 13. 电路 & 硬件常识

- **三极管开关电路设计参数**：耐压 (Vce(max))、功率 (Pmax)、负载功率、**β 值**（决定 IB = IL/β · 饱和深度）— 这四项**都要考虑**
- **LDO（线性稳压器）特点**：
  - ✓ 输出**纹波小**
  - ✗ **效率低**（差压×电流 全部发热）
  - ✗ **降压幅度不能大**（差压大则发热大、效率更低）
  - ✗ 输出**功率受限**
- **共模噪声**：两条线对地同向噪声；**电池供电可立即排除电源/接地相关的共模噪声**（✓ 正确）
- 烙铁头易氧化 → **使用后给烙铁头上锡保护**（✓ 正确）
- **电路原理图（Schematic）**：用约定符号表示电路结构与工作原理，用于**设计与分析**（不是 PCB，不"承载"元器件）；PCB 才是支撑体。
- 共模 vs 差模：滤波电感的方向不同；EMI 设计中区别处理

---

## 14. 开发环境（MDK / Keil）

- 编译链 ARMCC + 链接 + .hex/.bin/.axf
- 调试器：ST‑Link / J‑Link，接口 SWD（2 线）/ JTAG（4 线）
- 调试常用操作：**单步 (Step)**、**断点 (Breakpoint)**、**暂停 (Halt) 查看当前 PC**、**变量加入 Watch 窗**、寄存器/外设窗、内存窗 — **以上四种都常用** ✓

---

## 15. 综合速查表（最常考数值/事实）

| 项目 | 值 |
|---|---|
| STM32F103 最高 SYSCLK | 72 MHz |
| AHB / APB2 最大 | 72 MHz |
| APB1 最大 | 36 MHz |
| GPIO 单引脚最大电流 | 20 mA |
| GPIO 全部最大总电流 | 150 mA |
| GPIO 输出速度选项 | 2 / 10 / 50 MHz |
| SPI 最高速率 | SPI1: **36 Mbit/s = PCLK2/2**；SPI2/3: **18 Mbit/s = PCLK1/2** |
| ADC 最高采样 | 1 MSPS (14 MHz ADCCLK) |
| ADC 位数 | 12 bit |
| DAC 位数 | 12 bit |
| 三级流水线 | 取指→译码→执行 |
| NVIC 固定最高优先级 | Reset = ‑3，NMI = ‑2，HardFault = ‑1 |
| EXTI 线 | 共 19 条 |
| LSE | 32.768 kHz |
| LSI | ≈ 40 kHz |
| HSI | 8 MHz |
| HSE | 4–16 MHz（典型 8 MHz） |
| SysTick 位数 | 24 bit 向下计数 |

---

## 16. 关键寄存器速查

### 16.1 8051 SFR
| 寄存器 | 地址/位 | 作用/易错点 |
|---|---|---|
| `P0/P1/P2/P3` | 80H/90H/A0H/B0H | GPIO 端口；P0 无内部上拉；P3 复用 RXD/TXD/INT/T0/T1/WR/RD |
| `SP` | 81H | 堆栈指针，复位值 07H；压栈先加 1 再写入 |
| `DPL/DPH` | 82H/83H | 16 位数据指针 `DPTR`，常用于查表/外部存储器访问 |
| `PCON` | 87H | 电源控制；`SMOD` 可使串口波特率加倍 |
| `TCON` | 88H | `TF1 TR1 TF0 TR0 IE1 IT1 IE0 IT0`；定时器启停/溢出标志/外部中断触发方式 |
| `TMOD` | 89H | 高 4 位控 T1，低 4 位控 T0：`GATE C/T M1 M0` |
| `TL0/TH0/TL1/TH1` | 8A/8C/8B/8D | 定时器初值/计数值；模式 1 为 16 bit，模式 2 为 8 bit 自动重装 |
| `SCON` | 98H | `SM0 SM1 SM2 REN TB8 RB8 TI RI`；串口模式/收发使能/收发中断标志 |
| `SBUF` | 99H | 串口收发缓冲；写发送，读接收 |
| `IE` | A8H | `EA - ET2 ES ET1 EX1 ET0 EX0`；`EA=1` 后各源使能才有效 |
| `IP` | B8H | 中断优先级；1=高优先级，0=低优先级 |
| `PSW` | D0H | `CY AC F0 RS1 RS0 OV - P`；`RS1/RS0` 选寄存器组 |
| `ACC/B` | E0H/F0H | 累加器/乘除辅助寄存器 |

### 16.2 Cortex-M3 / NVIC / SysTick
| 寄存器 | 作用/关键位 |
|---|---|
| 向量表 `[0] / [1]` | `[0]` 初始 MSP，`[1]` Reset_Handler 入口 |
| `SCB->VTOR` | 向量表基地址重定位 |
| `SCB->AIRCR` | 优先级分组 `PRIGROUP`；写入需带 `VECTKEY=0x5FA` |
| `NVIC_ISERx / ICERx` | 使能/失能外部 IRQ |
| `NVIC_ISPRx / ICPRx` | 置位/清除 pending |
| `NVIC_IPRx` | 中断优先级；STM32F1 只实现高 4 位，数值越小优先级越高 |
| `SysTick->LOAD` | 24 位重装值；`ticks-1` 写入 |
| `SysTick->VAL` | 当前计数值；写任意值清零 |
| `SysTick->CTRL` | `ENABLE` 使能，`TICKINT` 中断，`CLKSOURCE` 选 HCLK 或 HCLK/8，`COUNTFLAG` 到 0 标志 |

### 16.3 RCC / AFIO / EXTI
| 模块 | 寄存器 | 作用/关键位 |
|---|---|---|
| RCC | `RCC_CR` | HSI/HSE/PLL 开关与就绪：`HSION/HSIRDY/HSEON/HSERDY/PLLON/PLLRDY` |
| RCC | `RCC_CFGR` | 系统时钟与分频：`SW/SWS` 选 SYSCLK，`HPRE/PPRE1/PPRE2` 分频，`PLLSRC/PLLMUL` 配 PLL |
| RCC | `RCC_APB2ENR` | 使能 GPIOA-G、AFIO、USART1、SPI1、ADC1/2、TIM1 等 |
| RCC | `RCC_APB1ENR` | 使能 TIM2-7、USART2/3、SPI2/3、I2C1/2、DAC、CAN 等 |
| RCC | `RCC_AHBENR` | 使能 DMA1/DMA2、SRAM/FLITF、FSMC、SDIO 等 |
| RCC | `RCC_BDCR` | LSE/RTC/备份域控制 |
| RCC | `RCC_CSR` | LSI 和复位标志 |
| AFIO | `AFIO_MAPR` | 引脚重映射；也用于关闭/保留 JTAG/SWD |
| AFIO | `AFIO_EXTICR[1..4]` | 选择 EXTIx 连接到 PA/PB/PC...；同号 GPIO 互斥 |
| EXTI | `IMR/EMR` | 中断屏蔽 / 事件屏蔽 |
| EXTI | `RTSR/FTSR` | 上升沿 / 下降沿触发 |
| EXTI | `SWIER` | 软件触发 |
| EXTI | `PR` | pending 标志；**写 1 清除** |

### 16.4 GPIO
| 寄存器 | 作用/关键点 |
|---|---|
| `GPIOx_CRL` | 配置 pin0-7；每脚 4 bit = `CNF[1:0] + MODE[1:0]` |
| `GPIOx_CRH` | 配置 pin8-15；复位后通常为 `0x44444444`（浮空输入） |
| `GPIOx_IDR` | 输入数据，只读，读实际引脚电平 |
| `GPIOx_ODR` | 输出数据；可读写，但读-改-写不如 BSRR 原子 |
| `GPIOx_BSRR` | bit0-15 写 1 置位；bit16-31 写 1 复位；原子操作 |
| `GPIOx_BRR` | bit0-15 写 1 复位 |
| `GPIOx_LCKR` | 锁定 GPIO 配置，需特定写序列 |

常用模式编码思路：`MODE=00` 输入；`MODE=01/10/11` 分别为 10/2/50MHz 输出；输入时 `CNF=00` 模拟、`01` 浮空、`10` 上拉/下拉；输出时 `CNF=00` 推挽、`01` 开漏、`10` 复用推挽、`11` 复用开漏。

### 16.5 USART / SPI / I2C
| 外设 | 寄存器 | 作用/关键位 |
|---|---|---|
| USART | `USART_SR` | 状态：`TXE` 发送数据寄存器空，`TC` 发送完成，`RXNE` 接收非空，`ORE/NE/FE/PE` 错误 |
| USART | `USART_DR` | 数据收发寄存器；读清 RXNE，写启动发送 |
| USART | `USART_BRR` | 波特率分频值 |
| USART | `USART_CR1` | `UE` USART 使能，`TE/RE` 收发使能，`RXNEIE/TXEIE/TCIE` 中断，`M/PCE/PS` 数据位/校验 |
| USART | `USART_CR2` | 停止位 `STOP`，同步时钟相关 |
| USART | `USART_CR3` | `DMAT/DMAR` DMA 收发，`RTSE/CTSE` 硬件流控 |
| SPI | `SPI_CR1` | `MSTR` 主机，`BR[2:0]` 分频，`CPOL/CPHA` 模式，`SPE` 使能，`LSBFIRST`，`SSM/SSI` 软件片选 |
| SPI | `SPI_CR2` | `TXEIE/RXNEIE/ERRIE` 中断，`TXDMAEN/RXDMAEN` DMA |
| SPI | `SPI_SR` | `TXE` 可写，`RXNE` 可读，`BSY` 忙，`OVR/MODF` 错误 |
| SPI | `SPI_DR` | 收发数据；全双工时写入同时移出/移入 |
| I2C | `I2C_CR1` | `PE` 使能，`START/STOP/ACK`，`SWRST` 软件复位 |
| I2C | `I2C_CR2` | 外设输入时钟频率 `FREQ`，中断/DMA 使能 |
| I2C | `I2C_OAR1/OAR2` | 自身地址，7/10 位寻址 |
| I2C | `I2C_DR` | 数据寄存器 |
| I2C | `I2C_SR1/SR2` | 状态：`SB/ADDR/BTF/RXNE/TXE/AF/BERR/ARLO`；读 SR1 后读 SR2 可清 ADDR |
| I2C | `I2C_CCR/TRISE` | SCL 时钟控制 / 最大上升时间 |

### 16.6 TIM / PWM
| 寄存器 | 作用/关键位 |
|---|---|
| `TIMx_CR1` | `CEN` 使能，`DIR` 方向，`CMS` 中央对齐，`ARPE` ARR 预装载 |
| `TIMx_PSC` | 预分频值；实际分频 = `PSC+1` |
| `TIMx_ARR` | 自动重装值；决定周期/频率 |
| `TIMx_CNT` | 当前计数值 |
| `TIMx_DIER` | `UIE/CCxIE` 中断使能，`UDE/CCxDE` DMA 请求 |
| `TIMx_SR` | `UIF/CCxIF` 中断标志；处理后清除 |
| `TIMx_EGR` | `UG` 产生更新事件，让 PSC/ARR 立即装载 |
| `TIMx_CCMR1/2` | 通道模式；`OCxM=110` PWM1，`111` PWM2；`OCxPE` CCR 预装载 |
| `TIMx_CCER` | `CCxE` 输出使能，`CCxP` 输出极性 |
| `TIMx_CCRx` | 捕获/比较值；PWM 中决定占空比 |
| `TIM1/8_BDTR` | 高级定时器：`MOE` 主输出使能，死区 `DTG`，刹车 `BKE/BKP` |

公式：`T = (PSC+1)(ARR+1)/TIMxCLK`；边沿对齐 PWM 占空比约 `CCR/(ARR+1)`。改占空比常用 `TIM_SetCompare1/2/3/4()` 或直接写 `TIMx->CCRx`。

### 16.7 ADC / DAC / DMA
| 外设 | 寄存器 | 作用/关键位 |
|---|---|---|
| ADC | `ADC_SR` | `EOC` 规则转换完成，`JEOC` 注入转换完成，`AWD` 模拟看门狗 |
| ADC | `ADC_CR1` | 扫描 `SCAN`，中断 `EOCIE/JEOCIE`，模拟看门狗配置 |
| ADC | `ADC_CR2` | `ADON` 使能，`CONT` 连续，`DMA`，`ALIGN`，`EXTSEL/EXTTRIG`，`SWSTART`，`CAL/RSTCAL` |
| ADC | `ADC_SMPR1/2` | 各通道采样时间 |
| ADC | `ADC_SQR1/2/3` | 规则组转换序列与长度 `L` |
| ADC | `ADC_DR` | 规则组转换结果 |
| ADC | `ADC_JSQR/JDRx` | 注入组序列/结果 |
| DAC | `DAC_CR` | `ENx` 使能，`BOFFx` 输出缓冲，`TENx/TSELx` 触发，`DMAENx` |
| DAC | `DAC_DHRx` | 数据保持寄存器，支持 8/12 位、左/右对齐 |
| DAC | `DAC_DORx` | 输出数据寄存器 |
| DMA | `DMA_ISR` | 全局/完成/半完成/错误标志：`GIF/TCIF/HTIF/TEIF` |
| DMA | `DMA_IFCR` | 写 1 清 DMA 标志 |
| DMA | `DMA_CCRx` | `EN` 使能，`TCIE/HTIE/TEIE` 中断，`DIR` 方向，`CIRC` 循环，`PINC/MINC` 地址递增，`PSIZE/MSIZE` 数据宽度，`PL` 优先级，`MEM2MEM` |
| DMA | `DMA_CNDTRx` | 传输数量 |
| DMA | `DMA_CPARx` | 外设地址 |
| DMA | `DMA_CMARx` | 存储器地址 |

配置顺序口诀：先开 RCC → 配 GPIO/复用 → 配外设参数 → 配 NVIC/DMA（若用）→ 清标志 → 使能外设/通道 → 写 ISR 或轮询状态位。

---

## 17. 例题（完整选项 + 答案）

### 17.1 Example.txt 原题整理

**1. 单选** 冯·诺依曼结构中，程序指令和数据存储在何处？  
A. 分别存储在独立的指令存储器和数据存储器中  
B. 存储在高速缓存 Cache 中，独立访问  
C. 存储在同一个存储器中，统一编址  
D. 存储在外部存储设备中，通过总线动态加载  
**Ans: C**

**2. 单选** 关于 BSP（板级支持包）的作用，哪项最准确？  
A. BSP 是硬件抽象层，直接操作寄存器以控制 GPIO、定时器等外设  
B. BSP 是在 HAL 库基础上的再次封装，隐藏硬件细节并为应用层提供统一接口  
C. BSP 是 STM32 的启动文件，负责初始化堆栈和调用 main 函数  
D. BSP 是操作系统内核，负责任务调度和内存管理  
**Ans: B**

**3. 单选** `#define MIN(x,y) (x)<(y)?(x):(y)`，`i=10,j=15; k=10*MIN(i,j);` 的输出是？  
A. 150.0  
B. 100.0  
C. 15.0  
D. 10.0  
**Ans: C**。宏展开为 `(10*i<j)?i:j`，得到 15。

**4. 单选** RAM 主要用于：  
A. 保存部分程序代码  
B. 动态加载的程序代码  
C. 保持初始化数值  
D. 启动程序代码  
**Ans: B**

**5. 判断** 烙铁头容易被氧化，所以使用后应该给烙铁头上锡。  
A. 正确  
B. 错误  
**Ans: A**

**6. 多选** 设计三极管开关电路时，需要考虑的参数有：  
A. 三极管耐压  
B. 三极管功率  
C. 负载功率  
D. beta 值  
**Ans: A B C D**

**7. 多选** MDK 进入 debug 模式后常用操作有：  
A. 单步调试  
B. 添加变量到观察窗，查看变量  
C. 打断点  
D. 暂停查看程序停在哪  
**Ans: A B C D**

**8. 多选** LDO 的工作特点有：  
A. 效率低  
B. 输出功率大  
C. 输出纹波特性好  
D. 降压幅度大  
**Ans: A C**

**9. 单选** `#define N 2`，`#define Y(n) ((N+1)*n)`，执行 `z=2*(N+Y(5));` 后：  
A. z 无定值  
B. z=70  
C. 语句有错误  
D. z=34  
**Ans: D**

**10. 判断** 软件定时准确、不需外加硬件电路、没有什么缺点。  
A. 正确  
B. 错误  
**Ans: B**。软件定时占 CPU，精度受中断/分支/优化影响。

**11. 单选** GPIO 配置成输出时，单个引脚最大输出电流为（）mA，所有引脚总体输出电流不能超过（）mA。  
A. 10 / 100  
B. 50 / 500  
C. 20 / 150  
D. 10 / 500  
**Ans: C**

**12. 单选** C 语言规定，在一个源程序中，`main` 主函数的位置是：  
A. 必须在最前面  
B. 可以在任意位置  
C. 必须在系统调用的库函数后面  
D. 必须在最后面  
**Ans: B**

**13. 判断** 电路存在共模噪声时，可使用电池供电快速排除电源问题。  
A. 正确  
B. 错误  
**Ans: A**

**14. 多选** 有关电路原理图，说法正确的有：  
A. 原理图是电子元器件的支撑体，是电气连接的提供者  
B. 原理图直接体现电路结构和工作原理，一般用于设计、分析电路  
C. 识别原理图中的元件符号及连接方式，可了解电路实际工作的潜在情况  
D. 原理图是用约定符号绘制的一种表示电路结构的图形  
**Ans: B C D**。A 描述的是 PCB。

**15. 判断** SYSCLK 时钟源有三个来源：HSI RC、HSE OSC、PLL。  
A. 正确  
B. 错误  
**Ans: A**

**16. 判断** 端口输入数据寄存器 IDR 的复位值为 `00000000H`。  
A. 错误  
B. 正确  
**Ans: A**。IDR 读实际引脚电平，复位后不应机械记作固定 0。

**17. 单选** 下面哪一种不是 STM32 GPIO 的输出模式？  
A. 浮空输出  
B. 开漏输出  
C. 推挽输出  
D. 复用推挽输出  
**Ans: A**

**18. 判断** 向量中断控制器允许有相同的优先级。  
A. 错误  
B. 正确  
**Ans: B**。允许配置相同优先级；但相同抢占优先级不能互相嵌套打断。

**19. 单选** 三级流水线技术的顺序是：  
A. 译码 -> 取指 -> 执行  
B. 取指 -> 译码 -> 执行  
C. 取指 -> 编译 -> 执行  
D. 编译 -> 取指 -> 执行  
**Ans: B**

**20. 判断** STM32F1 SPI1 的最高频率为 36MHz，即 `fpclk/2`。  
A. 正确  
B. 错误  
**Ans: A**。SPI1 挂 APB2；SPI2/3 挂 APB1，最高 18Mbit/s。

**21. 判断** LSE 一般为 32.768kHz，LSI 约 40kHz，可提供看门狗和自动唤醒单元时钟源。  
A. 错误  
B. 正确  
**Ans: B**

**22. 判断** Cortex-M3 内核采用冯·诺依曼结构的三级流水线。  
A. 错误  
B. 正确  
**Ans: A**。Cortex-M3 是哈佛结构 + 三级流水线。

**23. 单选** ADC 转换过程正确的是：  
A. 编码 -> 采样 -> 量化  
B. 量化 -> 采样 -> 编码  
C. 采样 -> 量化 -> 编码  
D. 采样 -> 编码 -> 量化  
**Ans: C**

**24. 判断** PWM1 和 PWM2 模式的区别仅在于相位 180 度，因此没有意义。  
A. 正确  
B. 错误  
**Ans: B**

**25. 多选** 关于 STM32 PWM，说法正确的有：  
A. PWM 输出占空比由 `TIMx_CCR` 决定  
B. PWM 又称脉冲宽度调制  
C. PWM 输出频率由 `TIMx_ARR` 决定  
D. STM32 的通用和高级定时器都可以产生 PWM  
**Ans: A B C D**

**26. 判断** CMSIS 是强制使用的芯片设计标准，且 `GPIO_ResetBits()` 属于 CMSIS。  
A. 错误  
B. 正确  
**Ans: A**。CMSIS 是接口标准；`GPIO_ResetBits()` 属于 ST 标准外设库。

**27. 判断** `stm32f10x.h` 相当于 `reg52.h`；`stm32f10x_conf.h` 负责控制外设库头文件开关。  
A. 错误  
B. 正确  
**Ans: B**

**28. 判断** STM32 NVIC 中固定优先级最高的是 -3。  
A. 正确  
B. 错误  
**Ans: A**。Reset=-3，NMI=-2，HardFault=-1。

**29. 判断** STM32F1 复位后所有 I/O 口均为浮空输入状态。  
A. 错误  
B. 正确  
**Ans: B**。考试题通常按普通 GPIO 记忆；JTAG/SWD 引脚有默认复用例外。

**30. 单选** 在 STM32 中，如何通过 C 语言访问 GPIOB 的 ODR 以设置引脚输出高电平？  
A. 通过串口发送指令控制 GPIOB  
B. 调用 `GPIO_SetBits(GPIOB, 0xFFFF);`  
C. 使用指针访问：`*(unsigned int*)(0x40010C0C) = 0xFFFF;`  
D. 直接调用 `GPIOB->ODR = 0xFFFF;` 无需定义  
**Ans: C**

### 17.2 新增覆盖题

**31. 单选** STM32F103 复位后默认系统时钟源通常是：  
A. HSI 8MHz  
B. HSE 8MHz  
C. PLL 72MHz  
D. LSE 32.768kHz  
**Ans: A**

**32. 多选** STM32F1 中挂在 APB2 总线上的外设通常包括：  
A. GPIOA-G  
B. USART1  
C. SPI1  
D. TIM2  
**Ans: A B C**。TIM2 在 APB1。

**33. 单选** `BOOT0=1, BOOT1=0` 时，STM32F1 从哪里启动？  
A. 主 Flash  
B. 系统存储器 Bootloader  
C. SRAM  
D. 外部 EEPROM  
**Ans: B**

**34. 多选** 配置 EXTI0 外部中断通常需要：  
A. 使能 GPIO 和 AFIO 时钟  
B. 用 `GPIO_EXTILineConfig()` 选择 PA0/PB0/... 映射到 EXTI0  
C. 配置 `EXTI_Init()` 的触发边沿  
D. 配置 NVIC 并编写 `EXTI0_IRQHandler()`  
**Ans: A B C D**

**35. 单选** EXTI pending 标志通常如何清除？  
A. 读 `EXTI_PR` 自动清除  
B. 对 `EXTI_PR` 对应位写 0 清除  
C. 对 `EXTI_PR` 对应位写 1 清除  
D. 关闭 RCC 时钟清除  
**Ans: C**

**36. 单选** `GPIOx_BSRR` 的高 16 位写 1 的作用是：  
A. 设置对应 ODR 位为 1  
B. 复位对应 ODR 位为 0  
C. 读取对应 IDR 位  
D. 锁定 GPIO 配置  
**Ans: B**

**37. 多选** 关于 GPIO 开漏输出，正确的是：  
A. 只能主动拉低，输出高电平依赖上拉  
B. 适合 I2C 等线与总线  
C. 与推挽输出一样内部上下管都主动驱动  
D. 复用开漏输出也常需要上拉电阻  
**Ans: A B D**

**38. 单选** SysTick 是几位定时器？计数方向如何？  
A. 16 位向上计数  
B. 16 位向下计数  
C. 24 位向下计数  
D. 32 位向上计数  
**Ans: C**

**39. 单选** `SysTick_Config(SystemCoreClock/1000)` 在 `SystemCoreClock=72MHz` 时通常产生：  
A. 1us 中断  
B. 1ms 中断  
C. 10ms 中断  
D. 1s 中断  
**Ans: B**

**40. 多选** USART 发送/接收相关状态位正确的是：  
A. `TXE` 表示发送数据寄存器空  
B. `TC` 表示一帧发送完成  
C. `RXNE` 表示接收数据寄存器非空  
D. `ORE` 表示溢出错误  
**Ans: A B C D**

**41. 单选** USART1 的 TX/RX 引脚在常用默认映射下分别是：  
A. PA9 / PA10  
B. PA2 / PA3  
C. PB6 / PB7  
D. PA5 / PA6  
**Ans: A**

**42. 多选** SPI 与 I2C 的典型区别正确的是：  
A. SPI 通常全双工，I2C 通常半双工  
B. SPI 有 SCK/MOSI/MISO/NSS，I2C 有 SCL/SDA  
C. I2C 需要开漏和上拉电阻  
D. SPI 通过地址选择从设备，I2C 只靠片选线选择从设备  
**Ans: A B C**

**43. 单选** I2C 总线的起始条件是：  
A. SCL 高电平时 SDA 由高变低  
B. SCL 高电平时 SDA 由低变高  
C. SCL 低电平时 SDA 由高变低  
D. SCL 低电平时 SDA 由低变高  
**Ans: A**

**44. 单选** 定时器定时时间公式正确的是：  
A. `T=(PSC+1)(ARR+1)/TIMxCLK`  
B. `T=TIMxCLK/((PSC+1)(ARR+1))`  
C. `T=PSC*ARR*TIMxCLK`  
D. `T=ARR/(PSC+TIMxCLK)`  
**Ans: A**

**45. 单选** `TIMx_PSC=71, TIMx_ARR=999, TIMxCLK=72MHz` 时更新频率约为：  
A. 1Hz  
B. 10Hz  
C. 1kHz  
D. 72kHz  
**Ans: C**。周期 `72*1000/72MHz=1ms`。

**46. 多选** 关于 STM32F1 定时器，正确的是：  
A. TIM6/TIM7 是基本定时器，无 GPIO 输出通道  
B. TIM2-TIM5 是通用定时器  
C. TIM1/TIM8 是高级定时器，可支持互补输出、死区、刹车  
D. 所有定时器都能直接产生 PWM 输出  
**Ans: A B C**

**47. 单选** ADC 输入引脚应配置成什么 GPIO 模式？  
A. 推挽输出  
B. 浮空输入  
C. 模拟输入  
D. 复用推挽输出  
**Ans: C**

**48. 多选** STM32 ADC 相关说法正确的是：  
A. F1 ADC 通常为 12 位 SAR ADC  
B. 转换时间 = 采样时间 + 12.5 个 ADC 时钟周期  
C. ADCCLK 不能超过规定上限，F1 常记最大 14MHz  
D. ADC 可配合 DMA 将转换结果搬运到内存  
**Ans: A B C D**

**49. 单选** DMA 循环模式 Circular 的特点是：  
A. 传输一次后自动关闭通道且不能再启用  
B. 到达缓冲区末尾后自动回到开头继续传输  
C. 只能用于内存到内存传输  
D. 只能用于 DAC，不能用于 ADC/USART  
**Ans: B**

**50. 多选** DMA 配置中常见字段及含义正确的是：  
A. `DMA_CPARx` 为外设地址  
B. `DMA_CMARx` 为存储器地址  
C. `DMA_CNDTRx` 为传输数量  
D. `DMA_CCRx` 中可设置方向、循环、地址递增、数据宽度、优先级  
**Ans: A B C D**

**51. 单选** Cortex-M3 向量表第 0 项和第 1 项分别存放：  
A. Reset_Handler 和 HardFault_Handler  
B. 初始 MSP 和 Reset_Handler  
C. 初始 PC 和初始 LR  
D. SysTick_Handler 和 PendSV_Handler  
**Ans: B**

**52. 多选** 下列哪些通常属于“写 1 清除”类标志？  
A. `EXTI_PR` 对应 pending 位  
B. `DMA_IFCR` 中对应清除位  
C. `GPIOx_IDR` 输入数据位  
D. `GPIOx_BSRR` 的置位/复位控制位  
**Ans: A B**。D 是写 1 执行动作，不是清状态标志。

**53. 单选** 在 STM32F1 标准外设库中，修改 TIM3_CH2 的 PWM 占空比通常使用：  
A. `TIM_SetCompare2(TIM3, value)`  
B. `GPIO_SetBits(TIM3, value)`  
C. `RCC_APB2PeriphClockCmd(TIM3, value)`  
D. `USART_SendData(TIM3, value)`  
**Ans: A**

**54. 多选** C 语言中 `volatile` 常用于修饰：  
A. 中断服务程序与主程序共享的变量  
B. 硬件寄存器映射地址对应的变量  
C. 会被外部硬件改变的变量  
D. 普通局部循环变量，且永远都必须加  
**Ans: A B C**

**55. 单选** 若宏定义为 `#define SQR(x) x*x`，则 `SQR(1+2)` 的结果是：  
A. 9  
B. 5  
C. 6  
D. 编译错误  
**Ans: B**。展开为 `1+2*1+2=5`，正确写法应为 `#define SQR(x) ((x)*(x))`。

---

## 18. 公式与代码片段速备

### 18.1 STM32 GPIO 标准库初始化模板
```c
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
GPIO_InitTypeDef gi;
gi.GPIO_Pin   = GPIO_Pin_5;
gi.GPIO_Mode  = GPIO_Mode_Out_PP;        // 推挽输出
gi.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &gi);
GPIO_SetBits(GPIOA, GPIO_Pin_5);          // 置高
```

### 18.2 SysTick 1 ms
```c
if (SysTick_Config(SystemCoreClock / 1000)) while (1);   // 失败死循环
void SysTick_Handler(void) { tick_ms++; }
```

### 18.3 EXTI 外部中断
```c
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
EXTI_InitTypeDef ei = {EXTI_Line0, EXTI_Mode_Interrupt, EXTI_Trigger_Rising, ENABLE};
EXTI_Init(&ei);
NVIC_InitTypeDef ni = {EXTI0_IRQn, 1, 1, ENABLE};
NVIC_Init(&ni);
```

### 18.4 PWM (TIM3 CH1)
```c
TIM_TimeBaseInitTypeDef tb = { .TIM_Period=999, .TIM_Prescaler=71,        // 1 kHz @ 72MHz
                               .TIM_CounterMode=TIM_CounterMode_Up };
TIM_TimeBaseInit(TIM3, &tb);
TIM_OCInitTypeDef oc = { .TIM_OCMode=TIM_OCMode_PWM1, .TIM_OutputState=TIM_OutputState_Enable,
                         .TIM_Pulse=500, .TIM_OCPolarity=TIM_OCPolarity_High };
TIM_OC1Init(TIM3, &oc);                                                   // 50% 占空比
TIM_Cmd(TIM3, ENABLE);
```

### 18.5 ADC 单次 + DMA
```c
RCC_ADCCLKConfig(RCC_PCLK2_Div6);                                          // 12 MHz
ADC_InitTypeDef ai = {ADC_Mode_Independent, DISABLE, DISABLE,
                      ADC_ExternalTrigConv_None, ADC_DataAlign_Right, 1};
ADC_Init(ADC1, &ai);
ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
ADC_DMACmd(ADC1, ENABLE);
ADC_Cmd(ADC1, ENABLE);
ADC_ResetCalibration(ADC1); while(ADC_GetResetCalibrationStatus(ADC1));
ADC_StartCalibration(ADC1);  while(ADC_GetCalibrationStatus(ADC1));
ADC_SoftwareStartConvCmd(ADC1, ENABLE);
```

---

> 临场提示：① 看到题目说"哈佛 / 冯·诺依曼"先想 Cortex‑M3 = 哈佛、8051 = 哈佛、Cortex‑A 系列 = 冯诺依曼简化扩展。
> ② 见到 PWM 立刻条件反射：**ARR↔频率，CCR↔占空比**。
> ③ 见到 SPI 频率立刻反应：**fPCLK/2**；F103 中 SPI1 上限 **36 Mbit/s**，SPI2/3 上限 **18 Mbit/s**。
> ④ 见到 ADC 流程：**采—量—编**。
> ⑤ 看到 `#define` 立刻数括号：宏展开 = 文本替换。
