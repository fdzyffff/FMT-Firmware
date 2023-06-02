
#ifndef _SH366006_H_
#define _SH366006_H_

#include <rtthread.h>
//////////////////////SH366002标准指令集//////////////

///////////////nomal模式下(用户config = 0x0000)///////
//                                  加密  解密
#define  CNTL    0x00  // 0x00/0x01 R/W       (control) 组合指令，写入不同指令返回不同数据
#define  AR      0x02  // 0x02/0x03 R/W  			(at rate) 预估放电电流(负值为放电)
#define  ARTTE   0x04  // 0x04/0x05 R    			(at rate time to empty)以(预估放电电流)放电到0所需时间
#define  TEMP    0x06  // 0x06/0x07 R    			(temperature)
#define  VOLT    0x08  // 0x08/0x08 R    			(voltage)
#define  FLAGS   0x0A  // 0x0A/0x0B R    			(flags) 返回测量状态寄存器
#define  NAC     0x0C  // 0x0C/0x0D R    			(nominal available capacity)返回电芯剩余绝对电量(mAh);
#define  FAC     0x0E  // 0x0E/0x0F R    			(full available capacity) 返回电芯充满绝对点亮(mAh);

#define  FRC     0x10  // 0x10/0x11 R    			(filtered RC) 返回电芯剩余电量(经过补偿)mAh
#define  FFCC    0x12  // 0x12/0x13 R    			(filtered FCC)返回电芯充满电量(经过补偿)mAh
#define  AC      0x14  // 0x14/0x15 R    			(average current)经过取样电阻电流(mA)
#define  TTE     0x16  // 0x16/0x17 R    			(average time to empty)以AC放电，放空时间
#define  TTF     0x18  // 0x18/0x19 R   		 	(average time to full)以AC充电，充满时间
#define  SI      0x1A  // 0x1A/0x1B R   		  (standby current)  返回待机状态电流
#define  STTE    0x1C  // 0x1C/0x1D R    			(standby time to empty)以待机电流放电所需时间
#define  MLI     0x1E  // 0x1E/0x1F R    			(max load current)返回最大负载电流

#define  MLTTE   0x20  // 0x20/0x21 R   		  (max load time to empty)  以max load current放电放空时间
#define  AE      0x22  // 0x22/0x23 R    			(available engergy)       预估有效电量 mWh
#define  AP      0x24  // 0x24/0x25 R    			(average power)           充放电过程中的功率(负数：放电，正数：充电)
#define  TTECP   0x26  // 0x26/0x27 R    			(TTR at constant power)   放空当前剩余电流所需时间
#define  INTTEMP 0x28  // 0x28/0x29 R    			(internal temp)           返回内部温度值
#define  CC      0x2A  // 0x2A/0x2B R    			(cycle count)             累计放电计数
#define  RSOC    0x2C  // 0x2C/0x2D R    			(relative state of charge)剩余/总量百分比
#define  SOH     0x2E  // 0x2E/0x2F R    			(state of health)         full charge capacity/design capacity

#define  JCV     0x30  // 0x30/0x31 R    			(jeita chg vol)
#define  JCC     0x32  // 0x32/0x33 R    			(jeita chg cur)
#define  PCHG    0x34  // 0x34/0x35 R    			(passed charge)          流过采样电阻电荷量
#define  DOD0    0x36  // 0x36/0x37 R    			(DOD0)
#define  SefDSGCUR 0x38// 0x38/0x39 R    			(self discharge current) 估算电池的自放电电流大小
#define  PACKCFG 0x3A  // 0x3A/0x3B R    	 R	()返回pack_config寄存器的值
/*pack_configuration
15_RESCAP: 0_空载补偿率，  1_将补偿负荷率(默认)   14_NormTWI: 0_TWI通信频率100k(默认)，1_400K
13_10mWh : 0_电量单位1mWh, 1_10mWh                12 保留
11 保留  
10:8_IWAKE_RSNS1_RSNS0: 000/100_禁止 001_ASRP~ASRN之间电压-1.4mV-+1.4mV，010/101 -3.0mV-+3.0mV, 110/011 -6.0-+6.0mV, 111 -12.0-+12.0mV
7:6 保留
5_sleep
4:1_保留
0_temps: 0_选择内部热敏  1_选择外部热敏
*/

////////////////////////////////////User_CFG1模式//////////////////////////////////
#define  CUR             0x04 //0x04/0x05   R  
#define  Time2FullinSec  0x1C //0x1C/0x1D   R  根据average current返回预测当前电池电量充满所需时间,655 35表示没有充电
#define  TermVolt        0x20 //0x20/0x21   R  放电绝对最小电压值
#define  DesignCapacity  0x22 //0x22/0x23   R
#define  No_action       0x30 //0x30/0x31
#define  FilteredRC      0x32 //0x32/0x33   R  基于当前电流和温度计算的剩余电量
#define  FilteredFCC     0x72 //0x72/0x73   R  基于当前电流和温度计算的充满电量
//////0x00-0x63和nomal模式一样//////////////////////////////////


//////////////////////////////////User_CFG2模式//////////////////////////
#define  MaxLoadCurrent  0x1E //0x1E/0x1F   R  电池可支持的最大放电电流
#define  BTPSOC1Set      0x1C //0x1C/0x1D   R
#define  BTPSOC1Clr      0x20 //0x20/0x21   R
#define  FullChargedEnergy 0x76// 0x76/0x77 R  以10mWh为单位返回充满电量
//////0x00-0x63和nomal模式一样//////////////////////////////////


//////////////////////////////////User_CFG3模式///////////////////////////
#define  Control         0x00 //0x00/0x01


////////////////////////////////////扩展指令///////////////////////////////////

#define DCAP     0x3C  // 0x3C/0x3D R      R   design capacity  电芯设计电量
#define DFCLS    0x3E  // 0x3E      N/A    R/W dataflashclass   设置0x40命令访问的subclass(子类) 加密状态下 指令无效
#define DFBLK    0x3F  // 0x3F      R/W    R/W dataflashblock   设置block,0x00-0x07
#define A_DF     0x40  // 0x40..0x53R/W    R/W blockData/authenticate    
#define ACLS_DFD 0x54  // 0x54      R/W    R/W blockdata/authenticate checksum
#define DFD      0x55  // 0x55..0x5f  R    R/W blockdata
#define DFDCKS   0x60  // 0x60      R/W    R/W blockdata checksum
#define DFDCNTL  0x61  // 0x61      N/A    R/W blockdatacontrol
#define DNAMELEN 0x62  // 0x62      R      R   device name length
#define DNAME    0x63  // 0x63..0x6A  R    R   device name
#define RSVD     0x6B  // 0x6B..0x7F  R    R   reserved





///////////////////////Control()子指令////////////////////////

//////////////////加密可操作/////////////////
#define  CONTROL_STATUS    ((uint16_t)0x0000)  //report the status word  获取状态字(control_status)
#define  DEVICE_TYPE       ((uint16_t)0x0001)  //report the information in subclass 122 offset 1
#define  FW_VERSION        ((uint16_t)0x0002)  //report the information in subclass 122 offset 3
#define  HW_VERSION        ((uint16_t)0x0003)  //report the information in subclass 122 offset 5

//////////////////加密不可操作//////////////
#define  DF_CHECKSUM       ((uint16_t)0x0004)  //enables a data flash checksum to be generated and reports on a read
#define  RESET_DATA        ((uint16_t)0x0005)  //returns resetdata
#define  PREV_MACWRITE     ((uint16_t)0x0007)  //returns previous MAC command code

//////////////////加密可操作(除0x0010)
#define  CHEM_ID           ((uint16_t)0x0008)  //reports the chemical identifier
#define  DF_VERSION        ((uint16_t)0x000C)  //reports the data flash version on the device
#define  SET_FULLSLEEP     ((uint16_t)0x0010)  //set the [fullsleep]bit in control_status register to 1
#define  SET_HIBERNATE     ((uint16_t)0x0011)  //set the [hibernate]bit in control_status register to 1
#define  CLEAR_HIBERNATE   ((uint16_t)0x0012)  //clear the [hibernate]bit in control_status register to 0
#define  SET_SHUTDOWN      ((uint16_t)0x0013)  //enable the STE pin to change state
#define  CLEAR_SHUTDOWN    ((uint16_t)0x0014)  //disable the STE pin from changing state
#define  SET_SWI_INTEN     ((uint16_t)0x0015)  //set [SWintEN] in control_status to 1
#define  CLEAR_SWI_INTEN   ((uint16_t)0x0016)  //clear[SWintEN] in control status to 0

/////////////////加密不可操作////////////
/*

*/
#define  SEALED            ((uint16_t)0x0020)  //force the SH366002 into SEALED mode
#define  RESET             ((uint16_t)0x0041)  //force a full reset of the SH366002
#define  CAL_OFFSET        ((uint16_t)0x0061)  //force the SH366002 to perform a zero-current calibration
#define  CAL_CUR           ((uint16_t)0x0062)  //force the SH366002 to perform a load-current calibration
#define  CAL_VOL           ((uint16_t)0x0063)  //force the SH366002 to perform a voltage calibration
#define  CAL_EXTT          ((uint16_t)0x0064)  //force the SH366002 to perform an external-temperature calibration
#define  CAL_INTT          ((uint16_t)0x0065)  //force the SH366002 to perform an internal-temperatire calibration
#define  EXIT_CAL          ((uint16_t)0x0080)  //force the SH366002 to quit calibration mode
#define  ENTER_CAL         ((uint16_t)0x0081)  //force the SH366002 to enter calibration mode

/////////////////加密可操作///////////
#define  SEfuseflag        ((uint16_t)0x0090)  //indicate the STE Fuse flag for the fuse state
#define  STE_Fuse_Enable   ((uint16_t)0x0091)  //enable STE fuse fuction
#define  STE_Fuse_Disable  ((uint16_t)0x0092)  //disable STE fuse fuction


#define  Burn_IRQ_Enable   (EXTI->IMR |= EXTI_Line0)//开启指定外设中断
#define  Burn_IRQ_Disable  (EXTI->IMR &= ~EXTI_Line0)//关闭指定外设中断




extern uint8_t Burn_Flag;//是否烧录参数标志 
typedef enum
{
	Board_Offset_Calibration    = CAL_OFFSET ,
	Voltage_Calibration         = CAL_VOL,
	Ext_Temperature_Calibration = CAL_EXTT,
	Int_Temperature_Calibration = CAL_INTT,
	Current_Calibration         = CAL_CUR
	
}Calibration_Type;


typedef enum
{
	Calibration_Success,
	Calibration_Error
}Calibration_Status;

typedef enum
{
	DataFlash_OP_Success,
	DataFlash_OP_Error
}DataFlash_OP_Status;

typedef enum
{
	EEPROM_Write_Success,
	EEPROM_Write_Error
}EEPROM_Write_Status;


typedef enum
{
	STA_Standby = 0,
	STA_Burn_On,
	STA_Burn_Succeed,
	STA_Burn_Fail,
	STA_Read_AFI,
	STA_Read_AFI_Fail,
	STA_Burn_AFI,
	STA_Burn_AFI_Fail,
	
	STA_Calibration,
	STA_Current_Calibration,
	STA_SetPower,
	
}System_Status_t;


#define Device_Address             0x16
////子命令////// 
#define Device_Type                0x0001     //读取芯片部分编号，返回两个字节
#define Firmware_Version           0x0002     //读取芯片版本号码，返回两个字节
#define Hardware_Version           0x0003     //读取芯片硬件版本，返回两个字节
#define IF_Checksum                0x0004     //返回flash校验和
#define Static_DF_Checksum         0x0005     //
#define Chemistry_ID               0x0006     //

#define Cell_Para_Checksum         0x0008     //
#define All_DF_Checksum            0x0009     //

#define Shutdown                   0x0010     //写命令  
#define Sleep_Command              0x0011     //发送命令使芯片
#define Fuse_Toggle                0x001D     //发送命令停止或启动熔断器输出
#define PCHG_FET_Toggle            0x001E     //此命令手动打开或关闭预充MOSFET
#define CHG_FET_Toggle             0x001F     //此命令手动打开或关闭充电MOSFET
#define DSG_FET_Toggle             0x0020     //此命令手动打开或关闭放电MOSFET

#define FET_Control                0x0022     //此命令禁用或启用固件控制的预充电，充电，放电MOSFET
#define Lifetime_Data_Collection   0x0023     //禁用或启用生命周期数据收集功能

#define Permanent_Failure_Detection 0x0024    //禁用或启用永久性故障检测
#define Fuse_Control               0x0026     //禁用或启用基于固件的fuse激活
#define Life_Data_Reset            0x0028     //发送此命令重置数据flash 中的生存期数据
#define Permanet_Fail_Data_Reset   0x0029     //发送此命令将重置数据flash中的永久失败数据

#define Seal_Device                0x0030     //发送此命令将使006进入加密模式
#define Security_Keys              0x0035     //这是一个读或写命令的两个字解开和完全访问密钥

#define Authentication_Key         0x0037     //更新身份证密钥到366006

#define Calibration_Mode           0x0040     //指示366006进入校准模式
#define Device_Reset               0x0041     //使366006复位

#define SafetyStatus               0x0051     //改命令返回safetystatus上的标志
#define PF_Status                  0x0053     //该命令返回PFStatus标志
#define OperationStatus            0x0054     //该命令返回OperationStatus中的标志
#define ChargingStatus             0x0055     //该命令返回ChargingStatus标志
#define ManufacturingStatus        0x0057     //该命令返回ManufacturingStatus标志
#define AFE_Data                   0x0058     //该命令返回AFE数据
#define Lifetime_Data_Block1       0x0060     //该命令返回block1的lifetime数据
#define Lifetime_Data_Block2       0x0061     //该命令返回block2的lifetime数据
#define Lifetime_Data_Block3       0x0062     //该命令返回block3的lifetime数据
#define ManufacturerInfo           0x0070     //该命令返回制造商信息

#define DAStatus1                  0x0071     //该命令返回单元电压，packvoltage power,averagepower等
#define DAStatus2                  0x0072     //该命令返回TS1,TS2,TS3,TS4,Cell,FET Temperature


////////////////SBS Command////////////////////////
#define Manufacturer_Access        0x00       //HEX   0-0xFFFF
#define Remaining_Capacity_Alarm  0x01       //UI    0-65535 mah
#define Remaining_Time_Alarm       0x02       //UI    0-65535 min
#define Battery_Mode               0x03       //HEX   0-0xFFFF
#define AtRate                     0x04       //SI    -32768-32767 ma
#define AtRate_Time_ToFull         0x05       //UI    0-65535 min
#define AtRate_Time_ToEmpty        0x06       //UI    0-65535 min
#define AtRate_OK                  0x07       //UI    0-65535 --
#define Temperature                0x08       //UI    0-65535 0.1°k
#define Voltage                    0x09       //UI    0-20000 mv
#define Current                    0x0A       //SI    -32768-32767 ma
#define Average_Current            0x0B       //SI    -32768-32767 ma
#define MaxError                   0x0C       //UI    0-100 %
#define Relative_State_OfCharge    0x0D       //UI    0-100相对电量百分比
#define Absolute_State_OfCharge    0x0E       //UI    0-100
#define Remaining_Capacity         0x0F       //UI    0-65535剩余电量
#define Full_Charge_Capacity       0x10       //UI    0-65535最大可用电量

#define Run_Time_ToEmpty           0x11       //UI    0-65535
#define Average_Time_ToEmpty       0x12       //UI    0-65535
#define Average_Time_ToFull        0x13       //UI    0-65535
#define Charging_Current           0x14       //UI    0-65535
#define Charging_Voltage           0x15       //UI    0-65535
#define Battery_Status             0x16       //hex   0-0xFFFF
#define CycleCount                 0x17       //UI    0-65535
#define Design_Capacity            0x18       //UI    0-65535
#define Design_Voltage             0x19       //UI    700-16000
#define Specification_Info         0x1A       //hex   0-0xFFFF
#define Manufacture_Data           0x1B       //UI    0-65535
#define Seria_Number               0x1C       //hex   0-0xFFFF
#define Real_Remaining_Capacity    0x1D       //UI    0-65535
#define Real_FullCharge_Capacity   0x1E       //UI    0-65535
#define Manufacturer_Name          0x20       //String Sinowealth
#define Device_Name                0x21       //String SH366006
#define Device_Chemistry           0x22       //String LION
#define Manufacture_Data_B         0x23       //Block
#define Authenticate               0x2F       //Block  20+1

#define CellVoltage4               0x3C       //UI    0-65535
#define CellVoltage3               0x3D       //UI    0-65535
#define CellVoltage2               0x3E       //UI    0-65535
#define CellVoltage1               0x3F       //UI    0-65535

#define StateOfHealth              0x4F       //0-100%

#define PackResistance             0x5B       //  mΩ
#define StsResistance              0x5C       //



/////////子集名称                      子集号
#define Voltage_1st_ID                (uint8_t)0  //长度26
#define Current_1st_ID                (uint8_t)1  //长度25
#define Temperature_1st_ID            (uint8_t)2  //长度25
#define Pre_CHG_Current_1st_ID        (uint8_t)3  //长度6
#define Time_Out_1st_ID               (uint8_t)4  //长度16
#define OverCharge_1st_ID             (uint8_t)5  //长度5
#define Voltage_2st_ID                (uint8_t)16 //长度15
#define Current_2st_ID                (uint8_t)17 //长度6
#define Temperature_2st_ID            (uint8_t)18 //长度7
#define FET_Verification_ID           (uint8_t)19 //长度6
#define AFE_Verification_ID           (uint8_t)20 //长度1

#define Internal_Short_Detection_ID   (uint8_t)21 //长度8
#define TabDisconnection_Detection_ID (uint8_t)22 //长度4
#define Old_WeakBattery_ID            (uint8_t)23 //长度3
#define Fuse_Permanent_Fail_ID        (uint8_t)24 //长度3
 
#define Charge_Cfg_ID                 (uint8_t)34 //长度152
#define Termination_Cfg_ID            (uint8_t)36 //长度6
#define Cell_Balancing_Cfg_ID         (uint8_t)37 //长度2
#define Data_ID                       (uint8_t)48 //长度63
#define Configuration_ID              (uint8_t)49 //长度8
#define DBPT_Cfg_ID                   (uint8_t)51 //长度6

#define LifeTime_Data_ID              (uint8_t)52 //长度86
#define Manufacturer_Info_ID          (uint8_t)58 //长度33

#define Registers_ID                  (uint8_t)64 //长度21
#define Power_ID                      (uint8_t)68 //长度13
#define Configuration_Fusion_ID       (uint8_t)79 //长度10
#define Current_Thresholds_ID         (uint8_t)81 //长度8
#define State_ID                      (uint8_t)82 //长度9
#define PF_Event_Data_ID              (uint8_t)96 //长度28
#define Data_Calibration_ID           (uint8_t)104//长度20

#define Current_ID                    (uint8_t)107 //长度2
#define IATA_Store_ID                 (uint8_t)108 //长度9




rt_err_t drv_sh366006_init(const char* i2c_device_name, const char* bms_device_name);

uint8_t crc8_check(uint8_t *data , uint16_t len);
rt_err_t Read_SBS_Conmmand_2byte(uint8_t ID , uint16_t * value );
rt_err_t Read_SBS_Conmmand_1byte(uint8_t ID , uint16_t * value );
#endif
