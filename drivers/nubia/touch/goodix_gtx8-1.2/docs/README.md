# Goodix Touch Driver - Sunrise Project #

Sunrise项目是Goodix的新的Touch driver设计。融合了GT9/GT9P两个系列touch driver的
设计优点，并提出了全新的分层设计架构。其代码复杂程度略高于GT9/GT9P的driver，但
松耦合的设计会减轻维护成本以及缩短新项目的驱动开发周期。

> Authors：
>
>	Yulong Cai<caiyulong@goodix.com>
>   Yafei Wang<wangyafei@goodix.com>


驱动分成设计
------------

Sunrise touch driver的架构主要分为三层：
 - Core Layer, 核心层
 - Hardware Layer，硬件层
 - External Module，外部功能模块

如下图所示  
`
|-------------------------------------------------   
|                                                |   
|  input device, sysfs interface, chrdev/miscdev |       external modules   
|                                                |       |   
|-------------------------------------------------       --------------------------   
|                                                |<----->| goodix external module |   
|         Goodix Touch Core Layer                |       --------------------------   
|            * struct goodix_ts_core;            |   
|            * struct goodix_ts_device;          |   
|            * struct goodix_ext_module;         |       --------------------------   
|                                                |<----->| goodix external module |   
|------------------------------------------------|   ^   --------------------------   
|                                                |   |   
|         Goodix Touch Hardware Layer            |   |---- goodix_ext_module_funcs   
|                                                |   
|------------------------------------------------   
                     |   
         i2c/spi bus |   
                     |   
|------------------------------------------------|   
|                   Touch IC                     |   
|------------------------------------------------|   
`

这样的设计模式是借鉴了内核其他子系统的设计思路，例如GPIO子系统、I2C子系统等，这
写子系统都将与硬件设备无关的代码抽象到核心层中，结合其硬件的特点合理设计接口方法
和数据结构，一定层度上屏蔽了硬件的细节和差异，同时利于driver的维护升级。

Sunrise项目旨在统一新Touch IC的driver接口，保证新IC的driver只用稍微修改Hardware
Layer的部分就能很快在Android/Linux上驱动我们的Touch IC。

例如Nanjing项目与Boston项目的Firmware不同，Firmware与driver交互的协议也有很大区别，
我们分别为两个项目编写Hardware Layer部分的代码（区别可能是发送配置部分、读取坐标
分布，休眠唤醒部分等），而对于这两个项目，其CoreLayer部分的代码以及外部功能模块的
代码是统一的。

> 各层之间衔接的接口设计以及所有公用的数据结构都在goodix_ts_core.h中，并附有详细
> 的注释。


Core Layer核心层
----------------

核心层的功能：

 - Touch driver的资源管理，GPIO、Pinctrl、Regulator、Interrupt、Input、PowerManage。
   这些资源有核心层负责申请和释放以及使用。
 - 事件管理，例如当中断发生时通过回调函数通知外部功能模块硬件有中断事件产生。
   当升级功能开始时，通知监听者，固件正在升级，你们不要再做其他事情。
 - 管理外部功能模，管理模块的注册、优先级、各种场景下回调模块的方法。
 - 管理input device，report touch event以及向用户空间提供一些sysfs属性.


重要的数据结构：

struct goodix_ts_core;
 - 众多数据结构都包含在此数据结构中
 - ts_dev代表硬件层的Touch deivce
 - pdev代表Core Layer本身（CoreLayer是一个Platform driver）


HardwareLayer硬件层
-------------------

硬件层的功能：

 - 为其他层提供基本的数据读写接口（i2c/spi/其他）：hw_ops->read(),hw_ops->write().
 - Touch IC的初始化（复位时序、配置发送流程等）：hw_ops->init()，hw_ops->reset(),hw_ops->send_config().
 - Touch事件的处理（读取坐标、响应Firmware的请求）：hw_ops->event_handler().
 - Suspend/Resume流程，让IC进入低功耗模式或者唤醒IC：hw_ops->suspend(), hw_ops->resume().
 - 提供发送命令（hw_ops->send_cmd）和读取版本号（hw_ops->send_config）接口。
 - Devicetee、ACPI中的属性解析，（板级硬件信息、配置数据等于硬件相关的数据都放在DT或者ACPI中）

   
重要的数据结构:

`struct goodix_ts_device;`
 - 该数据结构代表一个整个Touch IC的硬件层，是与CoreLayer衔接的主要数据结构；
 - board_data成员包含板级硬件信息、配置数据、屏的分辨率信息等；
 - hw_ops成员中包含所有操作硬件的方法，如read/write/init/reset等；
 - dev成员可以是i2c_client设备也可以是spi_device，可根据bus_type成员的值区分具体是
  什么类型的设备，然后调用类似to_i2c_client(dev)这样的方法或者设备的实际数据结构。

   
`struct goodix_ts_hw_ops;`
 - init：由Core Layer在probe阶段调用，用于初始化Touch IC，使其处于正常工作状态，
   Power On由Core Layer负责完成，init阶段Hardware Layer只用完成复位Touch IC然后
   下发配置信息等必要的初始化操作， 注意这个方法可能被多次调用，请注意重入的问题。
 - reset：按照芯片的复位时序要求，复位IC。
 - read/write：通过总线（i2c/spi）读写芯片的数据。
   注意：如果发生通讯错误请返回-EBUS，外部功能模块中的代码在调用read/write后会
   依据此返回值判断是否出现通讯异常，返回其他负值可能不会被识别。另外，retry动作
   请在read/write函数内部实现，一般情况下调用者无需再做retry处理。
 - send_cmd：发送command到Firmware，实现此方法时应考虑并发访问问题。
 - read_version：读取IC的版本信息。
 - event_handler：Touch事件的处理，包括处理坐标数据或者处理Firmware请求。
 - check_hw：ESD保护线程使用该方法来check hw有没有正常工作，返回负值代表硬件异常，
   返回0代表硬件状态OK。
 - suspend/resume：让IC进入sleep mode或者唤醒IC。

   
`struct goodix_ts_board_data;`



External Module外部功能模块
---------------------------

外部功能模块：除基本touch功能之外的其他附加功能都作为外部功能模块处理。

> 例如：固件升级功能模块、Gesture功能模块、Tools Support功能模块等。
>
> 注意：如果没有充分的理由，不要把新功能直接增加到Core Layer源文件中，推荐新建一个
> 外部功能模块来实现你的需求。

外部功能模块由Core Layer来管理其注册、卸载、以及各种场景下的回调。Core Layer维护
着一个链表，用于链接所有注册到Core Layer的外部功能模块。

你可以使用goodix_register_ext_module()方法来将一个外部模块注册到Core Layer中，
以及使用goodix_unregister_ext_module()方法将一个外部模块从Core Layer中卸载。推荐
的做法是在必要的情况下才将外部功能模块注册到Core Layer中，例如当用户调用Tools
模块的Open方法时，将Tools这个外部功能模块注册到Core Layer中，当用户操作完毕并
调用release方法时，将Tools这个外部功能模块从Core Layer中卸载。这样做的好处是减少
事件回调时的开销（例如当中断发生时，每个外部功能模块的irq_event方法都会被调用）。


目前有的外部功能模块有：
- 固件升级模块
- Gesture功能模块
- Tools Support功能模块

固件升级模块在driver init阶段就register到Core Layer中，在其init方法中创建了一个
升级线程，并启动升级任务，当升级完毕后其调用unregister方法从Core Layer中卸载。
另外，固件升级功能模块通过sysfs导出了一些属性，当用户通过向update_en属性写1时，
将调用register函数注册固件升级功能模块到Core Layer中，之后的流程就与开机固件升
级时的相似了。

Tools Support功能模块的处理逻辑相似，该模块向用户空间导出了一个名为`gtp_tools`
的misc device，当用户调用open系统调用时，该模块将注册到CoreLayer中，当用户使用
完毕并调用close系统调用时，该模块将从CoreLayer中卸载。

   
重要的数据结构：

`struct goodix_ext_module`
 - priority：外部功能模块的调用优先级，详见enum goodix_ext_priority;
 - funcs：外部功能模块的回调方法；
 - priv_data：私有数据域；

