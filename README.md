# README

> 这个严格意义上是打别的比赛用的，但是工程本身是一个配置好的，触类旁通的，故整理发出

## 具体文件树

```txt
root              
│                   │.clang-format 代码格式化设置 可以按照自己风格改
│                   │README.md     本文件
│                   │其余为生成的文件，包括锁等等官方生成的文件
│                   |---
├─.vscode           |里面`launch`要修改`.elf`（调试）
│                   |`settings`根据自己电脑修改工具链位置（clangd）
│                   |`tasks`是终端执行的任务 看自己改不改了
├─build             |---
│  └─Debug          |这里面有一个`compile_commands.json`（clangd）
├─cmake             |---
│  └─stm32cubemx    |没病不用动
├─Core              |---
│  ├─Inc            |略
│  └─Src            |略
├─Drivers           |---
├─Flash             |个人写的烧录相关 修改flash的.elf内容（openocd） 有条件可以加除了dap之外的烧录器
├─Middlewares       |官方生成库，DSP 和 FreeRTOS
└─User              |---
    ├─Bsp           |板载支持驱动
    ├─Device        |驱动层 / 设备层
    ├─Module        |模块层（ 多个设备组合 ）
    ├─Middleware    |中间层
    │  ├─Algorithm  |  算法层
    │  └─Service    |  服务层 
    └─App           |应用层 / c cpp混编接口层

```
> 对这几个层的理解是 板载支持包完全是按照芯片引脚等等信息来写一层封装，在BSP的基础上，一点点的封装出对应的驱动/设备Drv，然后多个驱动/设备Drv连接起来，这就是中间层Mid。应用层就是把Mid和Drv的东西都拎出来，变成具体的任务。算法Alg就是穿插在这里面的。
>
> 也就是说，移植工程只需重写BSP的接口，就可以完整的用上之前的工程。那就得做到尽可能的解耦，以及树状结构，还有良好的封装。
>
> 会存在freertos的组成部分的，相信后人的智慧（App层用的多 可能其他的也要用）

## 使用说明

配置和编译应该是使用CMake官方的按钮，当然大部分时候是CMake会自动配置

配置和编译：CMake处有固定的命令：（自行添加，官方提供了）
- 使用cmake栏中的上半部分的内容也可以
- 配置
- 生成
- 清除重新生成
- 清楚缓存并重新配置 

烧录可以使用VSCode官方的Task，使用的openocd实现。烧录时只需要执行这个任务就可以

调试使用vscode的运行与调试栏

## 如何开发

1. 修改成适合自己电脑的一些配置文件，默认配置就是相应的文件和主文件夹名称一样（使用的主文件夹名称作为的索引）
2. 编译烧录调试都使用vscode中的内容

### 因为底层驱动涉及cpp的class等东西，有两种解决方案

1. 让main.c以cpp格式编译，所有问题不需要管，但是你只能在官方生成的代码里面的，main.cpp中进行操作，如果使用freertos同理
2. 严格在每一个.c .cpp中写接口转接文档，每个不能调用的东西都进行接口转接。

我这边选择的第二个（第一个的问题是每一次都得改，以及可能会有兼容性问题）

写接口转接文档，发现在嵌入式中，只需要给`main.c`进行转接，使用`api_main`来当作提取出来的main即可。这样写的中断回调 初始化 while循环都没啥问题

HAL库也早就写好了cpp调用c的`extern "C"`内容

作为c调用cpp的转接cpp文件的就写`api_xxxx.cpp`和`api_xxx.h` 作为区分 用**`h`**

正常cpp文件为：`xxx_yyyy.cpp` 和 `xxx_yyyy.hpp` 作为区分 用**`hpp`**

`xxx为 app mid bsp drv alg`

**最终迭代成使用CMAKE，工程使用C++书写，开盒即用**

# 此处记载为了实现某些功能，修改了ST生成的文件 & NOTE

USART1 为经过了CH340的type-c接口 可以直连上位机

USART5 为接收机 只有RX端

两个继电器BREAKER控制XT30的座子是否导通 靠近电源的为1

三路FDCAN

四个舵机口

有SPI Flash

## CMakeList

为了更方便的添加.c .cpp文件 在CMakeList中写了：

```bash
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# 当源文件变化时强制重新生成，包含所有的.c .cpp
file(GLOB_RECURSE USER_SOURCES 
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    CONFIGURE_DEPENDS  # 关键：启用依赖检查
    User/App/*.c
    User/App/*.cpp
    User/Bsp/*.c
    User/Bsp/*.cpp
    User/Drv/*.c
    User/Drv/*.cpp
    User/Mid/*.c
    User/Mid/*.cpp
    User/Alg/*.c
    User/Alg/*.cpp
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE 
    # Add user source
    ${USER_SOURCES}
)


# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ./User/App
    ./User/Bsp
    ./User/Drv
    ./User/Mid
    ./User/Alg
    ./Middlewares/ARM/DSP/Include
)
```

## .ld文件

为了DMA传输，把需要用到DMA的东西的存储，换到了DTCM之外

```c
__attribute__((section(".dma_buffer"))) 使用这一个缀修饰
```

修改ld文件的内容如下，中文注释之间为添加内容，下方的.data是原来带的，用于确定方位

```c

 /* 用户为dma传输配置的内存地址 */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    _sdma_buffer = .;
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(32);
    _edma_buffer = .;
  } >RAM_D1

  /* 用户dma相关配置结束 */

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
  } >DTCMRAM AT> FLASH

```

## stm32h7xx_it.c

实现idle处理

```c
/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */
  HAL_BSP_UART_IRQHandler(&huart6); // 添加了这一行，从而实现处理

  /* USER CODE END USART6_IRQn 1 */
}

```

## 让clangd 不报头文件未使用的错误（间接使用 clangd识别不出来）

使用： 在include头文件后面添加

`// IWYU pragma: keep`

可以规避 `Included header FreeRTOS.h is not used directly (fixes available)` 这个错误