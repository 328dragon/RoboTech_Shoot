/*
 * @Author: Elaina
 * @Date: 2024-08-17 21:13:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 23:09:14
 * @FilePath: \MDK-ARMg:\project\stm32\f405rgb6\08_guosai\Core\Src\maincpp.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
__asm(".global __use_no_semihosting");
//#define debug_chassis 1 

#include "maincpp.h"
#include "Eigen"
#include "controller.h"
#include "Lib_List.h"
#include "connect.h"
#include "planner.h"
#include "map.h"
#include "grasycalse_planner.h"
//#include "imu.h"
#include "ch040.h"
#include "WS2812_yx.h"
#include "gw_grasycalse.h"

#define PI 3.1415926535

int flaga = 0;
int flagb = 0;
extern "C"
{
  extern UART_HandleTypeDef huart2;
#include "soft_pwm.h"
}
// 实例化Map并将初始点设置成startInfo
Controller::Controller_t ChassisControl;
Kinematic::Kinematic_t kinematic;
Connect::Host_t host;
Planner::Planner_t planner;
// Grayscale_t Grayscale;
// IMU::IMU_t imu;
uint8_t ch040_imu_buffer[100];
TaskHandle_t Motor_control_handle;    // 电机转速控制
TaskHandle_t Kinematic_update_handle; // 运动学更新
TaskHandle_t main_cpp_handle;         // 主函数
TaskHandle_t Planner_update_handle;   // 轨迹规划
TaskHandle_t gray_read_handle;        // 灰度传感器
List::List_t<Motor::MotorCommon_t *> MotorList;
List::List_t<Map::MapInfo_t *> MapList;
PwmOut_t pwm[3];
SoftPwmChannel soft_pwm_fireball;
SoftPwmChannel *soft_pwm_fire = &soft_pwm_fireball;
GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor;
void OnMotorControl(void *pvParameters);
void OnKinematicUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void gray_read_task(void *pvParameters);
void ball_down();
void choice_task(uint8_t id);
void ball_up();
void shoot_ready();
void shootdown();
void once_loop(Map::MapInfo_t *map);
int debug_fire_ball=0;
int loop_time=0;
// 现在有三种控制方法
/*一是基于自身坐标系下的速度闭环*/
/*二是基于大地坐标系下的速度闭环*/
/*三是基于自身坐标系下的位置闭环*/
/*一只要一开始给一个控制量*/
/*二与三需要实时更新*/
void main_cpp(void)
{
  // MotorList.Add(new Motor::MotorCommon_t(&htim8, TIM_CHANNEL_2, Motor2_PH_GPIO_Port, Motor2_PH_Pin, &htim2, -1, 0));
  // MotorList.Add(new Motor::MotorCommon_t(&htim8, TIM_CHANNEL_4, Motor4_PH_GPIO_Port, Motor4_PH_Pin, &htim4, 1, 1));
  // MotorList.Add(new Motor::MotorCommon_t(&htim8, TIM_CHANNEL_1, Motor1_PH_GPIO_Port, Motor1_PH_Pin, &htim1, -1, 2));
  // MotorList.Add(new Motor::MotorCommon_t(&htim8, TIM_CHANNEL_3, Motor3_PH_GPIO_Port, Motor3_PH_Pin, &htim3, 1, 3));
   MotorList.Add(new Motor::MotorCommon_t(&htim8, TIM_CHANNEL_2, Motor2_PH_GPIO_Port, Motor2_PH_Pin, &htim2, 1, 1));
  MotorList.Add(new Motor::MotorCommon_t(&htim8, TIM_CHANNEL_4, Motor4_PH_GPIO_Port, Motor4_PH_Pin, &htim4, -1, 0));
  MotorList.Add(new Motor::MotorCommon_t(&htim8, TIM_CHANNEL_1, Motor1_PH_GPIO_Port, Motor1_PH_Pin, &htim1, -1, 2));
  MotorList.Add(new Motor::MotorCommon_t(&htim8, TIM_CHANNEL_3, Motor3_PH_GPIO_Port, Motor3_PH_Pin, &htim3, 1, 3));
  //车长30，车宽20
  ChassisControl = Controller::Controller_t(reinterpret_cast<List::List_t<Motor::IMotorSpeed_t *> *>(&MotorList), &kinematic);
  planner = Planner::Planner_t(&ChassisControl);
  host = Connect::Host_t(&huart4, &ChassisControl, &planner);
  // 灰度
  //  Grayscale = Grayscale_t({{Grayscale1_GPIO_Port, Grayscale2_GPIO_Port, Grayscale3_GPIO_Port, Grayscale4_GPIO_Port, Grayscale5_GPIO_Port, Grayscale6_GPIO_Port, Grayscale7_GPIO_Port},
  //                           {Grayscale1_Pin, Grayscale2_Pin, Grayscale3_Pin, Grayscale4_Pin, Grayscale5_Pin, Grayscale6_Pin, Grayscale7_Pin}});

  Gw_GrayscaleSensor = GW_grasycalse::Gw_Grayscale_t(&hi2c1, GW_GRAY_ADDR_DEF);

  // pwm设置
  // pwm[0] = {&htim5, TIM_CHANNEL_1};
  pwm[1] = {&htim5, TIM_CHANNEL_3};
  pwm[2] = {&htim5, TIM_CHANNEL_4};
  pwm[1].set_duty_cycle(5.3);
  pwm[1].debug = 5.3;
  HAL_TIM_Base_Start_IT(&htim13);
  SoftPwmRegister(soft_pwm_fire, GPIOB, GPIO_PIN_2, 20, 180); // 周期20ms,最大角度180度
  // 灯带
  WS2812_InitBuffer();
  WS2812_StateDescription statea = {
      .state = WS2812_flow,
      .priority = 1,

      .R = 255,
      .G = 0,
      .B = 225,
  };
  WS2812_StateDescription WS2812_Empty_State =
      {
          .state = WS2812_clear,
          .priority = 99,

          .R = 0,
          .G = 0,
          .B = 0,
      };
  WS2812_AddStateLink(&flaga, statea);
  WS2812_AddStateLink(&flagb, WS2812_Empty_State);
  // imu串口接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,ch040_imu_buffer, 100);
  // 任务创建
  BaseType_t ok = xTaskCreate(OnMotorControl, "Motor_control", 600, NULL, 3, &Motor_control_handle);
  BaseType_t ok2 = xTaskCreate(OnKinematicUpdate, "Kinematic_update", 600, NULL, 2, &Kinematic_update_handle);
  BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
  BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 1000, NULL, 4, &Planner_update_handle);
  BaseType_t ok5 = xTaskCreate(gray_read_task, "gray_read_task", 300, NULL, 2, &gray_read_handle);
  if (ok != pdPASS || ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS || ok5 != pdPASS)
  {
    while (1)
    {
      // uart_printf("create task failed\n");
    }
  }

  // RTOS不用while1
}

void Onmaincpp(void *pvParameters)
{
  #ifndef debug_chassis
    
 while (host.task_id == -1)
 {
  vTaskDelay(100);
 }
  choice_task(host.task_id);

  vTaskDelay(100);
	ch040.setYawZero();
  vTaskDelay(100);
//动作序列
  for (int i = 1; i < 3; i++)
  {
    pwm[i].set_duty_cycle(5.0);
  }

  for (int i = 0; i < 3; i++)
  {
    MapList.Foreach(once_loop);
  }

//回家
  auto &state = planner.LoactaionCloseControl({-0.15, 0.1, 0}, 1.6, {0.01, 0.01, 0.02});
  while (state.isResolved() == false)
  {
    vTaskDelay(50);
  }

  ChassisControl.set_vel_target({0, 0, 0});
   flaga = 1;
  #else //debug模式
//	ChassisControl.set_vel_target({0, 0,0.5});
//auto &state = planner.LoactaionCloseControl({0, 0,0},0.8, {0.01, 0.01, 0.02});
//  while (state.isResolved() == false)
//  {
//    vTaskDelay(50);
//  }

  #endif // !debug_chassis

  while (1)
  {
		  // SoftSetAngle(soft_pwm_fire, debug_fire_ball);
    vTaskDelay(100);
  }
}

void gray_read_task(void *pvParameters)
{
  while (Gw_GrayscaleSensor.gw_ping())
  {
    vTaskDelay(100);
  }
  while (1)
  {
    Gw_GrayscaleSensor.read_data();
    vTaskDelay(10);
  }
}


void once_loop(Map::MapInfo_t *map)
{

  //目标值先减少一段距离(30框的一半+15车身一半+15预留校准跑道)
	 Kinematic::odom_t odom = map->odom;
   odom.x -= 0.75;	 
	odom.yaw+=((loop_time==0)?0:1)*0.016;
  //正常情况
  auto &state = planner.LoactaionCloseControl(odom, 6, {0.01, 0.005, 0.01});

  while (state.isResolved() == false)
  {
    vTaskDelay(50);
  }
while(!(Gw_GrayscaleSensor.data[3]==1&&Gw_GrayscaleSensor.data[4]==1&&Gw_GrayscaleSensor.data[5]==1))
{
ChassisControl.set_vel_target({0.15, Gw_GrayscaleSensor.ReturnXControl(), 0});
if(!Gw_GrayscaleSensor.IsCurrentMode(GW_grasycalse::OutLine))
{
break;
}
 vTaskDelay(20); 


}
vTaskDelay(20); 
	
//  //对准y,当全白时候停止
  while (Gw_GrayscaleSensor.IsCurrentMode(GW_grasycalse::OutLine))
 {
    ChassisControl.set_vel_target({0.15,0, 0});
    vTaskDelay(20); 
  }
//	 vTaskDelay(50); 
  ChassisControl.set_vel_target({0, 0, 0});
  //更新当前里程计
  kinematic.current_odom.x = map->odom.x - 0.45;
	 kinematic.current_odom.y=map->odom.y;
	kinematic.current_odom.yaw=map->odom.yaw;
 vTaskDelay(50);
//	ch040.setYawZero();
//继续跑向实际目标
state = planner.LoactaionCloseControl( map->odom, 3, {0.01, 0.005, 0.01});

  while (state.isResolved() == false)
  {
    vTaskDelay(50);
  }
  //发射,不用改
  shoot_ready();
  ball_down();
  vTaskDelay(700);
  ball_up();
  vTaskDelay(1600);
  shootdown();
	loop_time++;
}


void ball_down()
{
  SoftSetAngle(soft_pwm_fire, 90);
}
void ball_up()
{
  SoftSetAngle(soft_pwm_fire, 180);
}
void shoot_ready()
{
  for (int i = 1; i < 3; i++)
  {
//    pwm[i].set_duty_cycle(6.85);
		 pwm[i].set_duty_cycle(6.95);//5.85到6.85
  }
}
void shootdown()
{
  for (int i = 1; i < 3; i++)
  {
//    pwm[i].set_duty_cycle(5.8);
		
		  pwm[i].set_duty_cycle(5.8);
  }
}

// 任务选择
void choice_task(uint8_t id)
{
  switch (id)
  {
  case 0:
    MapList.Add(&Map::MapLeft);
    MapList.Add(&Map::MapRight);
    MapList.Add(&Map::MapMide);

    break;
  case 1:
    MapList.Add(&Map::MapLeft);
    MapList.Add(&Map::MapMide);
    MapList.Add(&Map::MapRight);
    break;
  case 2:
    MapList.Add(&Map::MapMide);
    MapList.Add(&Map::MapLeft);
    MapList.Add(&Map::MapRight);
    break;
  case 3:
    MapList.Add(&Map::MapRight);
    MapList.Add(&Map::MapLeft);
    MapList.Add(&Map::MapMide);
    break;
  case 4:
    MapList.Add(&Map::MapMide);
    MapList.Add(&Map::MapRight);
    MapList.Add(&Map::MapLeft);
    break;
  case 5:
    MapList.Add(&Map::MapRight);
    MapList.Add(&Map::MapMide);
    MapList.Add(&Map::MapLeft);
    break;
  default:
    break;
  }
}

// 底层不用动
void OnMotorControl(void *pvParameters)
{
  uint16_t last_tick = 0;
  while (1)
  {
    // 通过任务通知机制获取电机控制速度
    uint16_t dt = (uint16_t)((xTaskGetTickCount() - last_tick) % portMAX_DELAY);
    last_tick = xTaskGetTickCount();
    // 为了防止第一次出错
    if (dt == 0)
    {
      continue;
    }
    ChassisControl.MotorUpdate(dt);
    vTaskDelay(3);
  }
}

void OnPlannerUpdate(void *pvParameters)
{
  uint16_t last_tick = xTaskGetTickCount();
  // Kinematic.init(0.6, 2, 0.2); // 初始化运动学模型
  while (1)
  {
    uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
    last_tick = xTaskGetTickCount();
    planner.update(dt);
    WS2812_State_Handler();
    vTaskDelay(50);
  }
}
void OnKinematicUpdate(void *pvParameters)
{
  uint16_t last_tick = xTaskGetTickCount();

  while (1)
  {
    uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
    last_tick = xTaskGetTickCount();
    ChassisControl.KinematicAndControlUpdate(dt, ch040.getYaw());
    vTaskDelay(13);
  }
}

void MyHAL_UARTECallback()
{
  HAL_UARTEx_ReceiveToIdle_DMA(host._huart, host._rx_buffer, 20);
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, ch040_imu_buffer, 100);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == UART4)
  {
    host.Data_Analyse(host._rx_buffer);	
		 HAL_UARTEx_ReceiveToIdle_DMA(host._huart, host._rx_buffer, 20);
		__HAL_DMA_DISABLE_IT(host._huart->hdmarx,DMA_IT_HT);
  }
  else if (huart->Instance == USART3)
  {
    // imu.update();
    ch040.analyze_data(ch040_imu_buffer);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, ch040_imu_buffer, 100);
  }
	
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    // imu.update();
     ch040.analyze_data(ch040_imu_buffer);
    HAL_UART_Receive_DMA(&huart3, ch040_imu_buffer, 100);
  }
}
void WS2812_Refresh()
{
	if(flaga==1)
  HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)WS2812buf2send, 24 * (LED_Nums + 1));

		
}
extern "C"
{

#ifdef __MICROLIB
#include <stdio.h>

  int fputc(int ch, FILE *f)
  {
    (void)f;
    (void)ch;

    return ch;
  }
#else
#include <rt_sys.h>

  FILEHANDLE $Sub$$_sys_open(const char *name, int openmode)
  {
    (void)name;
    (void)openmode;
    return 0;
  }
#endif

  void _sys_exit(int ret)
  {
    (void)ret;
    while (1)
    {
    }
  }
  void _ttywrch(int ch)
  {
    (void)ch;
  }
}
// .............................................'RW#####EEEEEEEEEEEEEEEEEEEEEEEEWW%%%%%%N%%%%%%NW"...........
// ............................................/W%E$$$$EEEE######EEEEEEEEEEEEEEEE%%@NN@@$@@N%%%%N%]~`........
// ........................................i}}I&XIIYYXF&R#E$$$$$EEE##EEEEEEEEEEEE$N$#$K1:!YW@N%%%%@N$KY]+";..
// .....................................!>>li!"~~~'~~~~~!"i/1lIFK#E$$$EEEEEEE$$EEE%I::.....,]E@@@NNN@M$E$R>..
// ....................................+1"""i>"""""!~''''~~~!!~~!>/]Y&#$$$EEEEWWEEE$F,.......:>IRE$#&I/>'....
// ...................................;*lX&NM@@NW$#RFIl1i"!~~"">>!~~~!i}Y&#$W$EW%$EEMi...........::...'l1....
// ]}/+>~,............................,*YRNNNN@@MMMMMMMM@WRF*1>!~"!~!!~~!>+1IK$W%%W%1.................!*+....
// FFF&K&FYYYI]/"'`....................!K%W$$$$$$$EEEEEEE$W%%%WE&I]+!~~~!">"~~i*#%@#...................';....
// }}}}}}]l*XR#$WWERXl/!,:........,>>i/YK&&&&KKKKRR##EE$$$$$EEEE$$$EKYl/>!'~!"!+]IRNI..................'':...
// lllll]]]]}IYYXFK#W%N%$RFl+~`..`X/>>>!~~~~~!!"""">>ii+/}*YXK#EE$$WWWW$#Fl+"'~+**]*FI"................>i....
// ]]]]]]]]]]YXXXXYYXFRE$WW%%W#FlXl;!">+//i">"~'''''~!""!~~~!""i/1]*YFR#$%%WE&l/1]**lI&!.............>]]ll~..
// ]]]]]]]]]*XXXXXXXXYYX&R$$EE$$WWRR#WWWWW$E##KXI*1>!~!!""""!!!~~~'~~!!"+}I&R$NNWKYll*E"............"}/,~I&'.
// ]]]]]]]]lYXXXXXXXXXXXYYXKE$$E#E$$$$$$$$$$$$$$WWW$#X}1+>>""!''''~!!">""""""/]Y#W%$FRY............./+,.~lF>.
// ]]]]]]]]YXXXXXXXXXXXXXXXYYFKEW$E#EEEEEEEEEEEEEEEE$$WWW$$E#RFYl/+i!''!>"!!~!i]]]*XR#1'............!I/!]XI`.
// ]]]]]]]IXXXXXXXXXXXXXXXXXXYYYFE%WEEEEEEEEEEEEEEEEEEEEEEE$$$%%NN%$EKY]+i"!!"ilII*l]lXK/.:..........;+1/>:..
// ]]]]]]IXYXXXXXXXYYYYXXXXXXXYYR$RK$%$EEEEEEEEEEEEEEEEEEEEEEEE##EE$%NNNWE#R&&XI**llll]Y*.......::`,,`:::....
// ]]]]]*XXXXXXXXXXYYYYYYYXXXY&$#I/>/YE%$EEEEEEEEEEEEEEEEEEEEEEEEEEEE#EE$$WW$$W$ER&Y**]&}~+]IFRE$WW%%%%W$$#KX
// ]]]]lYXXXXXXXYYYYYYYYYYXXYK#I/ii+i>lYKWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$$WW$$#@%NMMM@@NNNN%%NNNNNN@@
// ]]]]YXXXXXXYYYYYYYYYYYXYYKX1iiiii+l1>i}KWWE#EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$W%@N%%%%%%%%%%%%%%%%%%%
// ]]]*XXXXXXXXXXXXYYYYYXYY&*++iiii+]+>++>11X$%$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$%N@@NNNN%%%%%%NNN%%
// ]]]YXXXXXXXYYYYYYXXYXYX&}i+iiiii1+iiii+*>>+*RWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$%N%W$$$$$$$$$WW%%
// ]]*XXYYYYYXX&K#$&YXXXXK}>+iiiii++iiii+FI>+i>>}F$W$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// }}IYYXFK#E%N%%NEYXXXYK}>iiiiiiiiii+>1I}]>iiii>"+*R$W$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// &XK#EWN@@%#YWN$YYXXY&l>iiiiii++iii>}I"il>iiiiiii"+1IR$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// $NN@N$&}!`.*NWXYXXYFI1/ii++i"!i>+>}l"!i]"iiiiiiii/i>/1IEW$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// "Y&l>,.:~1F@%KYXXXXF}Yi+i"',.';:,1]"+!'/;i">iiii>/i/1"1]IIX#$$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// ......*@M@%RFIYYXYF1F}!'`::::!`."]""~'!]~"":~"iii/i}/+F***>i*YF#E$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// ......`+FWNWERFXYXl+Y`::::::,".!}~,:.:.~',*:::,'!+/}!/]!>l/"}X>i1lXRE$$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// .........;/XEN@N%Wi&].`:::`:!',]/1i~,::`>`}>.`::.~1""1;::~1"+]li">>/llXRE$$$$$EE##EEEEEEEEEEEEEEE####EEEE$
// ............:'+lFK}N+:`:::`:!!&%NW$W$&]~,'!/~.``.~!.!''>+/IY>]lX/""11>ii+1lY&#$WWWW$$$$$E$$$$$$$WWW%%%W$#&
// .................>i~~,`:::`,}#M#}"'F%$W$}`;'!;.:`,','1XFK@@@RF@@@&~~~,!>ii>>+i/}lIXF&KR#$R#RRRKK&XIl1i!'`.
// ................`+'.~'`::`:`}$X`.::"&KFK&,:`'~!:`,'`~!;.;X&FKK$&l##l'::`;!>+/+i>>>>>>ii>I!.:..............
// ...:;..`!/:.....;i;."!`::`:;I+~.:.!E&FW#K':'',!"''';...;lYR#K&#K;"1#]~`::,:>i~i++++iii/}++................
// ...;Y.;/Y`......'>;.>+,::`.~E]:::.'K/"l}i`:`:::`;;'!;`.iK/}%&lRI.`1*'``:,,:+"~,,~"i++i1#+},...............
// ...:,>/.......~"'.;l;:``:'Y]'```./+~'';::``::::::`,`:~/~~i"!/'.:i;:`:,,:/+;''.::,'!"**>li...............
// ....:,.;;.......~"':.>i:`:',+l>;'';'";;,``:`:::::::::`,`'~~~~+":`~~```;;'>li!;`::::,.`Xi.1l`..............
// .......'];......`i;;.:I'::'I>1>'~~'';;,,```::::::::```,;;''~~''~~'`,'>>>>'/"~`,::::,`,X".`l+..............
// ........~~......."'~`/>+:::lll";'';;;,,,``:``;~::::```,;;'''~!>>"!!>ii!;:,~,'+!::::`;,Y~..,Y'.....::......
// .......:,.........!~/!.~+`.>]*>:,,,,,,````:`;;;`::````,,,,;;;~!'1/"!;`:::!~+]+"`::`:;;*~...~l.....`>,.....
// ........>..........>i!!++!'`/1Ii`::```````:::::`:::::`````,,,,`'i``:::::;]}/iii,:`::`!*~....>".....`/`....
// ........,`........;>.'>"~.i++]/ll+'`::::::``````::::::````::..,+```:`:`:"+ii+++':`:`:;Fi.....i`.....;+....
// ........`,.......`+:~!.;':i++/+i/}}1+>~;`::.....::::::...:,~i]X~`,:`:`:;/+++++1!:`::`.]I.....`"......+;...
// ........:~......:i,~'..";,ii+/+++++/1}]]}1/i>"!!;,,,,,>}lII**Y>`,:`:;,,/++++/+1+`:::`:,F,.....~,.....'>...
// ................"~'`..:1,,+i1+++//////}111}}}IY$K">>>!*NFl&X]>,,:`.'~`*]++++//+}':`:``.>1.....:".....'i...
// ...............~i':...!1.'+//+++//+/+]l+1]lIF]/Kl"">>+11>"1&i``::`!~;]I]++++//i1i:`::~;./;.....",....!;...
// ..............,1;.....]~.~+}++++/+///*]Y&F$Kl+}1!i++}1+i"11'`:::;!~i*]l]+//+/1++/,:`:~i``/.....'".........
// .............:]~....."}.:"}/+++////i1XRRYF*]lFKY/lI/`;,:"+~!:::,!"l&XYFY++/+/]++/~:`:~+".~i....;/.........
// .............+>.....`*;::/}++++//+/*#El}FIl*F&X]I*IX+`;1}'i::,~>+FYIX&#%#/++1]i++>:`:~++;.+'...~l.........
// ............;}......+!.:'l+++++1+/#N@/'i#F1!1]*"l*I*]+"+l}1+i+i>iRRE$$$ENIi+1]++//;::!++":`i.:.1].........
// ............/~....."i.`.+1i+++/1i*WW&~!1Wi`,+i/]Il>'`:.'I*Y>!>}FE$$EEEE#%*i+}}++1}!:`>+i+,.!~."Ii.........
// ............]'....'/.`;`1+++++//i&EW*~~YF'>+}]1//i"`.:''.Y+iY#$$EEEEEEEEW]i+}/++/*i`,++i/]::>+*l,.........
// ...........`l"...`/`:"~'/}/+++//YWEW*!+*+>iiI]]/">>i;,!`.lX$$EEEEEEEEEEEWN*+1++++Y1;'+i+i*+.~l}~..........
// ...........`l/,..+;.,+'~]*+//+1EW$E$X+1"""iY&i1l>"""ii+`,EWEEEEEEEEEEEEEE$W1++++i*]'>+i++//~.+;...........
// ............]}/`"".:!+~!I}+//iY%#$$EX1~""iFI*~i1]"">!/]`"%EEEEEEEEEEEEEEE$$1i++++/1"+++++};".!;...........
// ............'Y/1+.`:>+>/]1+//iXW#E$$**X>+F*/";">]l!"+&%I+WE$$$$$$$E$$WW%N%}i++++/>i+i++++]:;;,>...........
// .............~l}`::;i+i]i}+/+l@%$$E$&XYYY}!1,.!i>XIYX#N$REWEEEEEEEEEEEEE$N1+/+++/'iiiiii/1.`>:+...........
// ..............,i::;}i+i],*+++$@$EE$E%&']!~"+;~;>!"*}>$$$$EEE$EEEEEEEEEEE#%Fi//+1~'/i+++il>..>`i`..........
// ..............';:./]i++}.}li]NEEEE$W$}:i]+i!;~;i>i>,>%F/*$EEWWEEEEEEEEEEEE%Xi+//./+i++++*`..";i`..........
