This is an internal working file generated by the Source Browser.
21:21 28s
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\Source\SimpleTempSensor.c
-f
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\Tools\CC2530DB\f8wEndev.cfg
-f
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\Tools\CC2530DB\f8wConfig.cfg
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\Source\SimpleTempSensor.c
-D
NWK_AUTO_POLL
-D
HOLD_AUTO_START
-D
REFLECTOR
-D
xPOWER_SAVING
-D
NV_INIT
-D
xNV_RESTORE
-D
xZTOOL_P1
-D
xMT_TASK
-D
xMT_SYS_FUNC
-D
xMT_SAPI_FUNC
-D
xMT_SAPI_CB_FUNC
-lC
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\SimpleHumiditySensor\List\
-lA
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\SimpleHumiditySensor\List\
--diag_suppress
Pe001,Pa010
-o
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\SimpleHumiditySensor\Obj\
-e
--require_prototypes
--no_code_motion
--debug
--core=plain
--dptr=16,1
--data_model=large
--code_model=banked
--calling_convention=xdata_reentrant
--place_constants=data_rom
--nr_virtual_regs
16
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\Source\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\ZMain\TI2530DB\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\hal\include\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\hal\target\CC2530EB\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\mac\include\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\mac\high_level\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\mac\low_level\srf04\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\mac\low_level\srf04\single_chip\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\mt\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\osal\include\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\osal\mcu\ccsoc\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\services\saddr\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\services\sdata\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\stack\af\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\stack\nwk\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\stack\sapi\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\stack\sec\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\stack\sys\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\stack\zdo\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\zmac\
-I
H:\3gbox\zigbee\zigbee综合应用演示代码\cc2530 综合实验 z-stack 2.4.0-1.4.0\Projects\zstack\Samples\SimpleApp\CC2530DB\..\..\..\..\..\Components\zmac\f8w\
-I
D:\Program Files\IAR Systems\Embedded Workbench 5.4\8051\INC\
-I
D:\Program Files\IAR Systems\Embedded Workbench 5.4\8051\INC\CLIB\
-Ohz
