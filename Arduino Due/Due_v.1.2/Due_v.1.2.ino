// Arduino IDE v.1.6.7
// Arduino SAM Boards (32-bits ARM Cortex-M3) by Arduino v.1.6.7
            
            /*******************************************************/   
            /*          HACKERMAN & LA-Z Rider (22.07.2020) v.1.2  */
            /*******************************************************/
//http://fpv-community.ru/forums/topic/1581-apparatura-ru-iz-dzhoystika-defender-cobra-m5/

//===============================================================================//
//****************************Подключаемые библиотеки****************************//

#include <MenuBackend.h>    //MenuBackend library
#include <SPI.h>
#include <Wire.h> 
#include <DueFlashStorage.h>
DueFlashStorage dueFlashStorage;
#include <ArduinoProMini.h>
#include <ArduinoDue.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//*******************************************************************************//
//===============================================================================//



//===============================================================================//
//******************************Глобальные переменные****************************//

ArduinoDue Due;
// Инициализация пинов на плате Arduino Due
int   analogValue[10]; 
int   Aux1_I = 14;      // Aux1_I
int   Aux1_II = 15;     // Aux1_II
int   Aux2_I = 16;      // Aux1_I
int   Aux2_II = 17;     // Aux1_II
unsigned char PPM_Out     = 2;

ArduinoProMini ProMini;
// Инициализация пинов на плате Arduino Pro Mini
enum  IButton {up,down,left,right,enter,cancel};
enum  {First, Second, Third} IPlane;
int   Up; bool Up_flag = false;         // pin for the Up button     (Menu Up)
int   Down; bool Down_flag = false;       // pin for the Down button   (Menu Down)
int   Left; bool Left_flag = false;       // pin for the Left button   (Menu Left)
int   Right; bool Right_flag = false;      // pin for the Right button  (Menu Right)
int   Enter; bool Enter_flag = false;      // pin for the Enter button  (Menu Select - unlock)
int   Cancel; bool Cancel_flag = false;     // pin for the Esc button    (Menu Cancel - lock)
int   CameraUp;    // Camera Up
int   CameraDown;  // Camera Down
int   CameraLeft;  // Camera Left
int   CameraRight; // Camera Right
int   Trigger; // Курок
int   RatesHigh; bool RatesHigh_flag = false;  // Rates High
int   RatesLow; bool RatesLow_flag = false;   // Rates High
//-------------------------------------------------------------------------------
// начальный адрес памяти EEPROM
bool ActiveCange = false;
int minValue = 1024;
int maxValue = 0;
int cenValue = 0, iterations = 0;
int delta = 0;
int power = 0;
int PPM_power = 0;
float prokrutka = 1;
int OnlineTrimAileron = 0;
int OnlineTrimElevator = 0;
bool OnlineTrimingShow = false;

byte minAileron         = dueFlashStorage.read(0); //минимальное значение для Элерона (джойстик влево)
byte maxAileron         = dueFlashStorage.read(1); //максимальное значение для Элерона (джойстик вправо) 
byte centerAileron      = dueFlashStorage.read(2); //центральное значение для Элерона (джойстик отпустить)
byte deltaAileron[]     ={dueFlashStorage.read(3), dueFlashStorage.read(4), dueFlashStorage.read(5)}; //подстроечное значение Элерона для 1,2 и 3 экземпляр самолета 

byte minElevator        = dueFlashStorage.read(6); //минимальное значение для Элеватора (джойстик вперёд)
byte maxElevator        = dueFlashStorage.read(7); //максимальное значение для Элерона (джойстик назад) 
byte centerElevator     = dueFlashStorage.read(8); //центральное значение для Элеватора (джойстик отпустить)
byte deltaElevator[]    ={dueFlashStorage.read(9), dueFlashStorage.read(10), dueFlashStorage.read(11)}; //подстроечное значение Элеватора для 1,2 и 3 экземпляр самолета 

byte minRudder          = dueFlashStorage.read(12); //минимальное значение для Рудера (ось-джойстика влево)
byte maxRudder          = dueFlashStorage.read(13); //максимальное значение для Рудера (ось-джойстика вправо) 
byte centerRudder       = dueFlashStorage.read(14); //центральное значение для Рудера (ось-джойстика отпустить)
byte deltaRudder[]      ={dueFlashStorage.read(15), dueFlashStorage.read(16), dueFlashStorage.read(17)}; //подстроечное значение Рудера для 1,2 и 3 экземпляр самолета

byte minThrottle        = dueFlashStorage.read(18); //минимальное значение для Тротела (газ в минимум)
byte deltaMinThrottle[] ={dueFlashStorage.read(19), dueFlashStorage.read(20), dueFlashStorage.read(21)}; //подстроечное значение минимально Тротела для 1,2 и 3 экземпляр самолета
byte maxThrottle        = dueFlashStorage.read(22); //максимальное значение для Тротела (газ в максимум) 
byte deltaMaxThrottle[] ={dueFlashStorage.read(23), dueFlashStorage.read(24), dueFlashStorage.read(25)}; //подстроечное значение максимального Тротела для 1,2 и 3 экземпляр самолета

byte delta_minCam_Digital_V[] ={dueFlashStorage.read(26),dueFlashStorage.read(27),dueFlashStorage.read(28)}; //подстроечное минимальное значение для цифрового вертикального управления Камерой
byte delta_centerCam_Digital_V[] ={dueFlashStorage.read(93),dueFlashStorage.read(94),dueFlashStorage.read(95)}; //подстроечное максимальное значение для цифрового вертикального управления Камерой
byte delta_maxCam_Digital_V[] ={dueFlashStorage.read(30),dueFlashStorage.read(31),dueFlashStorage.read(32)}; //подстроечное ценртальноне значение для цифрового вертикального управления Камерой 

byte delta_minCam_Digital_H[] ={dueFlashStorage.read(33),dueFlashStorage.read(34),dueFlashStorage.read(35)}; //подстроечное минимальное значение для цифрового горизонтального управления Камерой
byte delta_maxCam_Digital_H[] ={dueFlashStorage.read(36),dueFlashStorage.read(37),dueFlashStorage.read(38)}; //подстроечное максимальное значение для цифрового горизонтального управления Камерой
byte delta_centerCam_Digital_H[] ={dueFlashStorage.read(39),dueFlashStorage.read(40),dueFlashStorage.read(41)}; //подстроечное ценртальноне значение для цифрового горизонтального управления Камерой

byte minCam_Analog_V    = dueFlashStorage.read(42); //минимальное значение для аналогово вертикального управления Камерой
byte maxCam_Analog_V    = dueFlashStorage.read(43); //максимальное значение для аналогово вертикального управления Камерой
byte centerCam_Analog_V = dueFlashStorage.read(44); //центральное значение для аналогово вертикального управления Камерой
byte deltaCam_Analog_V[]={dueFlashStorage.read(45), dueFlashStorage.read(46), dueFlashStorage.read(47)}; //подстроечное значение значение для аналогово вертикального управления Камерой 1,2 и 3 экземпляр самолета

byte minCam_Analog_H    = dueFlashStorage.read(48); //минимальное значение для аналогово горизонтального управления Камерой
byte maxCam_Analog_H    = dueFlashStorage.read(49); //максимальное значение для аналогово горизонтального управления Камерой
byte centerCam_Analog_H = dueFlashStorage.read(50); //центральное значение для аналогово горизонтального управления Камерой
byte deltaCam_Analog_H[]={dueFlashStorage.read(51), dueFlashStorage.read(52), dueFlashStorage.read(53)}; //подстроечное значение значение для аналогово горизонтального управления Камерой 1,2 и 3 экземпляр самолета

byte Aux1_Digital_I[]     ={dueFlashStorage.read(54),dueFlashStorage.read(55),dueFlashStorage.read(56)}; //
byte Aux1_Digital_O[]     ={dueFlashStorage.read(57),dueFlashStorage.read(58),dueFlashStorage.read(59)}; //значение выставляется в с шагом 4 PPM в диапазоне 0-1000
byte Aux1_Digital_II[]    ={dueFlashStorage.read(60),dueFlashStorage.read(61),dueFlashStorage.read(62)}; //

byte Aux2_Digital_I[]     ={dueFlashStorage.read(63),dueFlashStorage.read(64),dueFlashStorage.read(65)}; //
byte Aux2_Digital_O[]     ={dueFlashStorage.read(66),dueFlashStorage.read(67),dueFlashStorage.read(68)}; //значение выставляется в с шагом 4 PPM в диапазоне 0-1000
byte Aux2_Digital_II[]    ={dueFlashStorage.read(69),dueFlashStorage.read(70),dueFlashStorage.read(71)}; //

byte minAux1Analog        = dueFlashStorage.read(72); //минимальное значение для Aux1Analog (верхний скролл)
byte maxAux1Analog        = dueFlashStorage.read(73); //максимальное значение для Aux1Analog (верхний скролл) 
byte centerAux1Analog     = dueFlashStorage.read(74); //центральное значение для Aux1Analog (верхний скролл)
byte deltaAux1Analog[]    ={dueFlashStorage.read(75), dueFlashStorage.read(76), dueFlashStorage.read(77)}; //подстроечное значение Aux1Analog для 1,2 и 3 экземпляр самолета

byte minAux1Analog_        = dueFlashStorage.read(99); //минимальное значение для Aux1Analog (верхний скролл)
byte maxAux1Analog_        = dueFlashStorage.read(100); //максимальное значение для Aux1Analog (верхний скролл) 
byte centerAux1Analog_     = dueFlashStorage.read(101); //центральное значение для Aux1Analog (верхний скролл)
byte deltaAux1Analog_[]    ={dueFlashStorage.read(102), dueFlashStorage.read(103), dueFlashStorage.read(104)}; //подстроечное значение Aux1Analog для 1,2 и 3 экземпляр самолета

byte minAux2Analog        = dueFlashStorage.read(78); //минимальное значение для Aux1Analog (верхний скролл)
byte maxAux2Analog        = dueFlashStorage.read(79); //максимальное значение для Aux1Analog (верхний скролл) 
byte centerAux2Analog     = dueFlashStorage.read(80); //центральное значение для Aux1Analog (верхний скролл)
byte deltaAux2Analog[]    ={dueFlashStorage.read(81), dueFlashStorage.read(82), dueFlashStorage.read(83)}; //подстроечное значение Aux1Analog для 1,2 и 3 экземпляр самолета

byte PlaneActive          = dueFlashStorage.read(84); //какой используется экземпляр самолета

byte InvertChanelsPart1   = dueFlashStorage.read(85); //инвертируемые каналы (1111 1111) - Aileron,Elevator,Rudder,Throttle,Cam_Digital_V,Cam_Digital_H,Cam_Analog_V,Cam_Analog_H
byte InvertChanelsPart2   = dueFlashStorage.read(86); //инвертируемые каналы (1100 0000) - Aux1_Analog,Aux2_Analog,-,-,-,-,-,-.

byte ExponentialChanels   = dueFlashStorage.read(87); //каналы с экспанентой (1111 1100) - Aileron,Elevator,Rudder,Throttle,Aux1_Analog,Aux2_Analog,-,-.

byte PlaneModeActive[]    = {dueFlashStorage.read(96), dueFlashStorage.read(97), dueFlashStorage.read(98)}; //тип управляющего крыла (1110 0000) - Classic,Flying_Wing,V-tail,-,-,-,-,-.

//память для меню "Other settings" - другие настройки
byte InvertPPMActive      = dueFlashStorage.read(89); //инвертировать PPM: 0-Нет, 1-Да.
byte CamModeActive        = dueFlashStorage.read(90); //Управление камерой: 0-Analog (управление потенциометрами подключенными к входам CamH А4, CamV А5) 
                                                      //                    1-Digital (default, управление с кнопок джойстика)
byte AUX1ModeActive       = dueFlashStorage.read(91); //(управление потенциометром: 0-Analog (управление потенциометром подключенным к входу А3) 
                                                      //                            1-Analog'(управление потенциометром подключенным к входу А1)
                                                      //                            2-Digital(default, управление кнопкой on-off-on (I-O-II) подключенной к входам D14, D15)
                                                      //                            3-Курок
byte AUX2ModeActive       = dueFlashStorage.read(92); //(управление потенциометром: 0-Analog (управление потенциометром подключенным к входу А2) 
                                                      //                            1-Digital(default, управление кнопкой on-off-on (I-O-II) подключенной к входам D16, D17)
                                                      //                            2-Курок

byte Trigger_Digital_On[] ={dueFlashStorage.read(105),dueFlashStorage.read(106),dueFlashStorage.read(107)};//значение для кнопки «Курок» при НАжатии   
byte Trigger_Digital_Off[]={dueFlashStorage.read(108),dueFlashStorage.read(109),dueFlashStorage.read(110)};//значение для кнопки «Курок» при ОТжатии    

int CameraV_old = 0;
int CameraH_old = 0;
byte CamDigitalDelay      = dueFlashStorage.read(111);//фиксированное дискретное приращением каждого нового значения канала управления камерой в PPM пакете.
byte CamAnalogDelay       = dueFlashStorage.read(112);//Приращение производится до тех пор пока не достигнет текущего значения задаваемого оператором ЛА.
//-------------------------------------------------------------------------------

// Переменные и настройки для PPM
// timer is clocked at 42MHz, so 42 ticks per us
uint32_t periods[]={42000,42000,42000,42000,42000,42000,42000,42000,42000}; 
uint32_t num_periods=8+1;// number of channels +1

int   ppm_channels[8];
int   ppm_channels_1000_2000[8];
float PPMFrame;
float PPMFrame_ms = 22.5;
float PPMPulse_ms = 0.3;
long  PPMSum;
//bool  InvertPPM = true; 

void TC0_Handler()
{
   long dummy=REG_TC0_SR0;    // vital - reading this clears some flag
                              // otherwise you get infinite interrupts
   static int i=0;
   REG_TC0_RC0=periods[i++];
   if (i>=num_periods)i=0;
}
//-------------------------------------------------------------------------------
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

//unsigned char* MyLogo_128x64;
//unsigned char* CheckItem_16x16;

static void menuUsed(MenuUseEvent used);
static void menuChanged(MenuChangeEvent changed);
//Menu variables
MenuBackend menu = MenuBackend(menuUsed,menuChanged);
String    MenuItem_is_Open; //меню, которое в данный момент открыто
String    MenuItem_is_Change = " "; //меню, которое в данный изменяется
bool      isMainScreen_Show = false;
bool      isTest_Show = false;
//initialize menuitems

    MenuItem Plane/*........................................................*/=MenuItem("Plane");
        MenuItem First_Plane/*..................................................*/=MenuItem("First Plane");
        MenuItem Second_Plane/*.................................................*/=MenuItem("Second Plane");
        MenuItem Third_Plane/*..................................................*/=MenuItem("Third Plane");
        
    MenuItem Trimmer_Mode/*.................................................*/=MenuItem("Trimmer Mode");
        MenuItem Trimmer_Aileron/*..............................................*/=MenuItem("Trimmer Aileron");
            MenuItem Trimmer_Aileron_Center/*.......................................*/=MenuItem("Trimmer Aileron Center");
            MenuItem Trimmer_Aileron_DefaultSettings/*..............................*/=MenuItem("Trimmer Aileron DefaultSettings");
                MenuItem Trimmer_Aileron_DefaultSettings_YES/*..........................*/=MenuItem("Trimmer Aileron DefaultSettings YES");
                MenuItem Trimmer_Aileron_DefaultSettings_NO/*...........................*/=MenuItem("Trimmer Aileron DefaultSettings NO");
        MenuItem Trimmer_Elevator/*.............................................*/=MenuItem("Trimmer Elevator");
            MenuItem Trimmer_Elevator_Center/*......................................*/=MenuItem("Trimmer Elevator Center");
            MenuItem Trimmer_Elevator_DefaultSettings/*.............................*/=MenuItem("Trimmer Elevator DefaultSettings");
                MenuItem Trimmer_Elevator_DefaultSettings_YES/*.........................*/=MenuItem("Trimmer Elevator DefaultSettings YES");
                MenuItem Trimmer_Elevator_DefaultSettings_NO/*..........................*/=MenuItem("Trimmer Elevator DefaultSettings NO");
        MenuItem Trimmer_Rudder/*...............................................*/=MenuItem("Trimmer Rudder");
            MenuItem Trimmer_Rudder_Center/*........................................*/=MenuItem("Trimmer Rudder Center");
            MenuItem Trimmer_Rudder_DefaultSettings/*...............................*/=MenuItem("Trimmer Rudder DefaultSettings");
                MenuItem Trimmer_Rudder_DefaultSettings_YES/*...........................*/=MenuItem("Trimmer Rudder DefaultSettings YES");
                MenuItem Trimmer_Rudder_DefaultSettings_NO/*............................*/=MenuItem("Trimmer Rudder DefaultSettings NO");
        MenuItem Trimmer_Throttle/*.............................................*/=MenuItem("Trimmer Throttle");
            MenuItem Trimmer_Throttle_Min/*.........................................*/=MenuItem("Trimmer Throttle Min");
            MenuItem Trimmer_Throttle_Max/*.........................................*/=MenuItem("Trimmer Throttle Max");
            MenuItem Trimmer_Throttle_DefaultSettings/*.............................*/=MenuItem("Trimmer Throttle DefaultSettings");
                MenuItem Trimmer_Throttle_DefaultSettings_YES/*.........................*/=MenuItem("Trimmer Throttle DefaultSettings YES");
                MenuItem Trimmer_Throttle_DefaultSettings_NO/*..........................*/=MenuItem("Trimmer Throttle DefaultSettings NO");
        MenuItem Trimmer_Cam_Digital_V/*........................................*/=MenuItem("Trimmer Cam Digital V");
            MenuItem Trimmer_Cam_Digital_V_Down/*...................................*/=MenuItem("Trimmer Cam Digital V Down");
            MenuItem Trimmer_Cam_Digital_V_Center/*.................................*/=MenuItem("Trimmer Cam Digital V Center");
            MenuItem Trimmer_Cam_Digital_V_Up/*.....................................*/=MenuItem("Trimmer Cam Digital V Up");  
            MenuItem Trimmer_Cam_Digital_V_DefaultSettings/*........................*/=MenuItem("Trimmer Cam Digital V DefaultSettings");
                MenuItem Trimmer_Cam_Digital_V_DefaultSettings_YES/*....................*/=MenuItem("Trimmer Cam Digital V DefaultSettings YES");
                MenuItem Trimmer_Cam_Digital_V_DefaultSettings_NO/*.....................*/=MenuItem("Trimmer Cam Digital V DefaultSettings NO");   
        MenuItem Trimmer_Cam_Digital_H/*........................................*/=MenuItem("Trimmer Cam Digital H");
            MenuItem Trimmer_Cam_Digital_H_Down/*...................................*/=MenuItem("Trimmer Cam Digital H Down");
            MenuItem Trimmer_Cam_Digital_H_Center/*.................................*/=MenuItem("Trimmer Cam Digital H Center");
            MenuItem Trimmer_Cam_Digital_H_Up/*.....................................*/=MenuItem("Trimmer Cam Digital H Up");   
            MenuItem Trimmer_Cam_Digital_H_DefaultSettings/*........................*/=MenuItem("Trimmer Cam Digital H DefaultSettings");
                MenuItem Trimmer_Cam_Digital_H_DefaultSettings_YES/*....................*/=MenuItem("Trimmer Cam Digital H DefaultSettings YES");
                MenuItem Trimmer_Cam_Digital_H_DefaultSettings_NO/*.....................*/=MenuItem("Trimmer Cam Digital H DefaultSettings NO"); 
        MenuItem Trimmer_Cam_Analog_V/*.........................................*/=MenuItem("Trimmer Cam Analog V");
            MenuItem Trimmer_Cam_Analog_V_Center/*..................................*/=MenuItem("Trimmer Cam Analog V Center");
            MenuItem Trimmer_Cam_Analog_V_DefaultSettings/*.........................*/=MenuItem("Trimmer Cam Analog V DefaultSettings");
                MenuItem Trimmer_Cam_Analog_V_DefaultSettings_YES/*.....................*/=MenuItem("Trimmer Cam Analog V DefaultSettings YES");
                MenuItem Trimmer_Cam_Analog_V_DefaultSettings_NO/*......................*/=MenuItem("Trimmer Cam Analog V DefaultSettings NO");  
        MenuItem Trimmer_Cam_Analog_H/*.........................................*/=MenuItem("Trimmer Cam Analog H");
            MenuItem Trimmer_Cam_Analog_H_Center/*..................................*/=MenuItem("Trimmer Cam Analog H Center");
            MenuItem Trimmer_Cam_Analog_H_DefaultSettings/*.........................*/=MenuItem("Trimmer Cam Analog H DefaultSettings");
                MenuItem Trimmer_Cam_Analog_H_DefaultSettings_YES/*.....................*/=MenuItem("Trimmer Cam Analog H DefaultSettings YES");
                MenuItem Trimmer_Cam_Analog_H_DefaultSettings_NO/*......................*/=MenuItem("Trimmer Cam Analog H DefaultSettings NO");  
        MenuItem Trimmer_Aux1_Digital/*.........................................*/=MenuItem("Trimmer Aux1 Digital");
            MenuItem Trimmer_Aux1_Digital_I/*.......................................*/=MenuItem("Trimmer Aux1 Digital I");
            MenuItem Trimmer_Aux1_Digital_O/*.......................................*/=MenuItem("Trimmer Aux1 Digital O");
            MenuItem Trimmer_Aux1_Digital_II/*......................................*/=MenuItem("Trimmer Aux1 Digital II");
            MenuItem Trimmer_Aux1_Digital_DefaultSettings/*.........................*/=MenuItem("Trimmer Aux1 Digital DefaultSettings");
                MenuItem Trimmer_Aux1_Digital_DefaultSettings_YES/*.....................*/=MenuItem("Trimmer Aux1 Digital DefaultSettings YES");
                MenuItem Trimmer_Aux1_Digital_DefaultSettings_NO/*......................*/=MenuItem("Trimmer Aux1 Digital DefaultSettings NO");
        MenuItem Trimmer_Aux2_Digital/*.........................................*/=MenuItem("Trimmer Aux2 Digital");
            MenuItem Trimmer_Aux2_Digital_I/*.......................................*/=MenuItem("Trimmer Aux2 Digital I");
            MenuItem Trimmer_Aux2_Digital_O/*.......................................*/=MenuItem("Trimmer Aux2 Digital O");
            MenuItem Trimmer_Aux2_Digital_II/*......................................*/=MenuItem("Trimmer Aux2 Digital II");
            MenuItem Trimmer_Aux2_Digital_DefaultSettings/*.........................*/=MenuItem("Trimmer Aux2 Digital DefaultSettings");
                MenuItem Trimmer_Aux2_Digital_DefaultSettings_YES/*.....................*/=MenuItem("Trimmer Aux2 Digital DefaultSettings YES");
                MenuItem Trimmer_Aux2_Digital_DefaultSettings_NO/*......................*/=MenuItem("Trimmer Aux2 Digital DefaultSettings NO");
        MenuItem Trimmer_Aux1_Analog /*.........................................*/=MenuItem("Trimmer Aux1 Analog");
            MenuItem Trimmer_Aux1_Analog_Center/*...................................*/=MenuItem("Trimmer Aux1 Analog Center");
            MenuItem Trimmer_Aux1_Analog_DefaultSettings/*..........................*/=MenuItem("Trimmer Aux1 Analog DefaultSettings");
                MenuItem Trimmer_Aux1_Analog_DefaultSettings_YES/*......................*/=MenuItem("Trimmer Aux1 Analog DefaultSettings YES");
                MenuItem Trimmer_Aux1_Analog_DefaultSettings_NO/*.......................*/=MenuItem("Trimmer Aux1 Analog DefaultSettings NO"); 
        MenuItem Trimmer_Aux2_Analog /*.........................................*/=MenuItem("Trimmer Aux2 Analog");
            MenuItem Trimmer_Aux2_Analog_Center/*...................................*/=MenuItem("Trimmer Aux2 Analog Center");
            MenuItem Trimmer_Aux2_Analog_DefaultSettings/*..........................*/=MenuItem("Trimmer Aux2 Analog DefaultSettings");
                MenuItem Trimmer_Aux2_Analog_DefaultSettings_YES/*......................*/=MenuItem("Trimmer Aux2 Analog DefaultSettings YES");
                MenuItem Trimmer_Aux2_Analog_DefaultSettings_NO/*.......................*/=MenuItem("Trimmer Aux2 Analog DefaultSettings NO");   
        MenuItem Trimmer_Trigger_Digital/*......................................*/=MenuItem("Trimmer Trigger Digital");
            MenuItem Trimmer_Trigger_Digital_On/*...................................*/=MenuItem("Trimmer Trigger Digital On");
            MenuItem Trimmer_Trigger_Digital_Off/*..................................*/=MenuItem("Trimmer Trigger Digital Off");
            MenuItem Trimmer_Trigger_Digital_DefaultSettings/*......................*/=MenuItem("Trimmer Trigger Digital DefaultSettings");     
                MenuItem Trimmer_Trigger_Digital_DefaultSettings_YES/*......................*/=MenuItem("Trimmer Trigger Digital DefaultSettings YES");
                MenuItem Trimmer_Trigger_Digital_DefaultSettings_NO/*.......................*/=MenuItem("Trimmer Trigger Digital DefaultSettings NO");          
          
    MenuItem Invert_Mode/*..................................................*/=MenuItem("Invert Mode");
        MenuItem Invert_Aileron/*...............................................*/=MenuItem("Invert Aileron");
        MenuItem Invert_Elevator/*..............................................*/=MenuItem("Invert Elevator");
        MenuItem Invert_Rudder/*................................................*/=MenuItem("Invert Rudder");
        MenuItem Invert_Throttle/*..............................................*/=MenuItem("Invert Throttle");
        MenuItem Invert_Cam_Digital_V/*.........................................*/=MenuItem("Invert Cam Digital V");
        MenuItem Invert_Cam_Digital_H/*.........................................*/=MenuItem("Invert Cam Digital H");
        MenuItem Invert_Cam_Analog_V/*..........................................*/=MenuItem("Invert Cam Analog V");
        MenuItem Invert_Cam_Analog_H/*..........................................*/=MenuItem("Invert Cam Analog H");
        MenuItem Invert_Aux1/*..................................................*/=MenuItem("Invert Aux1");
        MenuItem Invert_Aux2/*..................................................*/=MenuItem("Invert Aux2");
        
    MenuItem Exponential_Mode/*.............................................*/=MenuItem("Exponential Mode");
        MenuItem Exponential_Aileron/*..........................................*/=MenuItem("Exponential Aileron");
        MenuItem Exponential_Elevator/*.........................................*/=MenuItem("Exponential Elevator");
        MenuItem Exponential_Rudder/*...........................................*/=MenuItem("Exponential Rudder");
        MenuItem Exponential_Throttle/*.........................................*/=MenuItem("Exponential Throttle");
        MenuItem Exponential_Aux1/*.............................................*/=MenuItem("Exponential Aux1");
        MenuItem Exponential_Aux2/*.............................................*/=MenuItem("Exponential Aux2");

    MenuItem Plane_Mode/*...................................................*/=MenuItem("Plane Mode");
        MenuItem Plane_Classic/*................................................*/=MenuItem("Plane Classic");
        MenuItem Plane_Flying_Wing/*............................................*/=MenuItem("Plane Flying Wing");
        MenuItem Plane_V_Tail_Wing/*............................................*/=MenuItem("Plane V-tail");

    MenuItem Joystick_Auto_Trimming/*.......................................*/=MenuItem("Joystick Auto Trimming");
        MenuItem Auto_Aileron/*.................................................*/=MenuItem("Auto Aileron");
        MenuItem Auto_Elevator/*................................................*/=MenuItem("Auto Elevator");
        MenuItem Auto_Rudder/*..................................................*/=MenuItem("Auto Rudder");
        MenuItem Auto_Throttle/*................................................*/=MenuItem("Auto Throttle");
        MenuItem Auto_Cam_Analog_V/*............................................*/=MenuItem("Auto Cam Analog V");
        MenuItem Auto_Cam_Analog_H/*............................................*/=MenuItem("Auto Cam Analog H");
        MenuItem Auto_Aux1/*....................................................*/=MenuItem("Auto Aux1");
        MenuItem Auto_Aux2/*....................................................*/=MenuItem("Auto Aux2");       

    MenuItem DefaultSettings_for_this_Plane/*...............................*/=MenuItem("DefaultSettings for this Plane");
        MenuItem DefaultSettings_for_this_Plane_YES/*...........................*/=MenuItem("DefaultSettings for this Plane YES");
        MenuItem DefaultSettings_for_this_Plane_NO/*............................*/=MenuItem("DefaultSettings for this Plane NO");

    MenuItem Other_Settings/*...............................................*/=MenuItem("Other Settings");
        MenuItem Invert_PPM /*..................................................*/=MenuItem("Invert PPM");
        MenuItem Cam_Mode /*....................................................*/=MenuItem("Cam Mode");
        MenuItem AUX1_Mode /*...................................................*/=MenuItem("AUX1 Mode");
        MenuItem AUX2_Mode /*...................................................*/=MenuItem("AUX2 Mode");
        MenuItem CamDigitalDelay_Mode /*.............................................*/=MenuItem("CamDigitalDelay Mode");
        MenuItem CamAnalogDelay_Mode /*..............................................*/=MenuItem("CamAnalogDelay Mode");
       
    MenuItem Test_Channel/*.................................................*/=MenuItem("Test Channel");
        MenuItem Test_Aileron/*.................................................*/=MenuItem("Test Aileron");
        MenuItem Test_Elevator/*................................................*/=MenuItem("Test Elevator");
        MenuItem Test_Rudder/*..................................................*/=MenuItem("Test Rudder");
        MenuItem Test_Throttle/*................................................*/=MenuItem("Test Throttle");
        MenuItem Test_Cam_Analog_V/*............................................*/=MenuItem("Test Cam Analog V");
        MenuItem Test_Cam_Analog_H/*............................................*/=MenuItem("Test Cam Analog H");
        MenuItem Test_Aux1/*....................................................*/=MenuItem("Test Aux1");
        MenuItem Test_Aux2/*....................................................*/=MenuItem("Test Aux2");

//*******************************************************************************//
//===============================================================================//



//===============================================================================//
//******************************Глобальные функции*******************************//
void MenuInit()
{
  menu.getRoot().add(Test_Channel);
  menu.getRoot().add(Other_Settings);
  menu.getRoot().add(DefaultSettings_for_this_Plane);
  menu.getRoot().add(Joystick_Auto_Trimming);
  menu.getRoot().add(Plane_Mode);
  menu.getRoot().add(Exponential_Mode);
  menu.getRoot().add(Invert_Mode);
  menu.getRoot().add(Trimmer_Mode);
  menu.getRoot().add(Plane).addRight(Trimmer_Mode).addRight(Invert_Mode).addRight(Exponential_Mode).addRight(Plane_Mode).addRight(Joystick_Auto_Trimming).addRight(DefaultSettings_for_this_Plane).addRight(Other_Settings).addRight(Test_Channel).addRight(Plane);

    Plane.add(Third_Plane);
    Plane.add(Second_Plane);
    Plane.add(First_Plane).addRight(Second_Plane).addRight(Third_Plane).addRight(First_Plane);

    Trimmer_Mode.add(Trimmer_Trigger_Digital);
    Trimmer_Mode.add(Trimmer_Aux2_Analog);
    Trimmer_Mode.add(Trimmer_Aux1_Analog);
    Trimmer_Mode.add(Trimmer_Aux2_Digital);
    Trimmer_Mode.add(Trimmer_Aux1_Digital);
    Trimmer_Mode.add(Trimmer_Cam_Analog_H);
    Trimmer_Mode.add(Trimmer_Cam_Analog_V);
    Trimmer_Mode.add(Trimmer_Cam_Digital_H);
    Trimmer_Mode.add(Trimmer_Cam_Digital_V);
    Trimmer_Mode.add(Trimmer_Throttle);
    Trimmer_Mode.add(Trimmer_Rudder);
    Trimmer_Mode.add(Trimmer_Elevator);
    Trimmer_Mode.add(Trimmer_Aileron).addRight(Trimmer_Elevator).addRight(Trimmer_Rudder).addRight(Trimmer_Throttle).addRight(Trimmer_Cam_Digital_V).addRight(Trimmer_Cam_Digital_H).addRight(Trimmer_Cam_Analog_V).addRight(Trimmer_Cam_Analog_H).addRight(Trimmer_Aux1_Digital).addRight(Trimmer_Aux2_Digital).addRight(Trimmer_Aux1_Analog).addRight(Trimmer_Aux2_Analog).addRight(Trimmer_Trigger_Digital).addRight(Trimmer_Aileron);
        Trimmer_Aileron.add(Trimmer_Aileron_DefaultSettings);
        Trimmer_Aileron.add(Trimmer_Aileron_Center).addRight(Trimmer_Aileron_DefaultSettings).addRight(Trimmer_Aileron_Center);
            Trimmer_Aileron_DefaultSettings.add(Trimmer_Aileron_DefaultSettings_NO);
            Trimmer_Aileron_DefaultSettings.add(Trimmer_Aileron_DefaultSettings_YES).addRight(Trimmer_Aileron_DefaultSettings_NO).addRight(Trimmer_Aileron_DefaultSettings_YES);
        Trimmer_Elevator.add(Trimmer_Elevator_DefaultSettings);
        Trimmer_Elevator.add(Trimmer_Elevator_Center).addRight(Trimmer_Elevator_DefaultSettings).addRight(Trimmer_Elevator_Center);
            Trimmer_Elevator_DefaultSettings.add(Trimmer_Elevator_DefaultSettings_NO);
            Trimmer_Elevator_DefaultSettings.add(Trimmer_Elevator_DefaultSettings_YES).addRight(Trimmer_Elevator_DefaultSettings_NO).addRight(Trimmer_Elevator_DefaultSettings_YES);
        Trimmer_Rudder.add(Trimmer_Rudder_DefaultSettings);
        Trimmer_Rudder.add(Trimmer_Rudder_Center).addRight(Trimmer_Rudder_DefaultSettings).addRight(Trimmer_Rudder_Center);
            Trimmer_Rudder_DefaultSettings.add(Trimmer_Rudder_DefaultSettings_NO);
            Trimmer_Rudder_DefaultSettings.add(Trimmer_Rudder_DefaultSettings_YES).addRight(Trimmer_Rudder_DefaultSettings_NO).addRight(Trimmer_Rudder_DefaultSettings_YES);
        Trimmer_Throttle.add(Trimmer_Throttle_DefaultSettings);
        Trimmer_Throttle.add(Trimmer_Throttle_Max);
        Trimmer_Throttle.add(Trimmer_Throttle_Min).addRight(Trimmer_Throttle_Max).addRight(Trimmer_Throttle_DefaultSettings).addRight(Trimmer_Throttle_Min);
            Trimmer_Throttle_DefaultSettings.add(Trimmer_Throttle_DefaultSettings_NO);
            Trimmer_Throttle_DefaultSettings.add(Trimmer_Throttle_DefaultSettings_YES).addRight(Trimmer_Throttle_DefaultSettings_NO).addRight(Trimmer_Throttle_DefaultSettings_YES);
        Trimmer_Cam_Digital_V.add(Trimmer_Cam_Digital_V_DefaultSettings);
        Trimmer_Cam_Digital_V.add(Trimmer_Cam_Digital_V_Up);
        Trimmer_Cam_Digital_V.add(Trimmer_Cam_Digital_V_Center);
        Trimmer_Cam_Digital_V.add(Trimmer_Cam_Digital_V_Down).addRight(Trimmer_Cam_Digital_V_Center).addRight(Trimmer_Cam_Digital_V_Up).addRight(Trimmer_Cam_Digital_V_DefaultSettings).addRight(Trimmer_Cam_Digital_V_Down);
             Trimmer_Cam_Digital_V_DefaultSettings.add(Trimmer_Cam_Digital_V_DefaultSettings_NO);
             Trimmer_Cam_Digital_V_DefaultSettings.add(Trimmer_Cam_Digital_V_DefaultSettings_YES).addRight(Trimmer_Cam_Digital_V_DefaultSettings_NO);
        Trimmer_Cam_Digital_H.add(Trimmer_Cam_Digital_H_DefaultSettings);
        Trimmer_Cam_Digital_H.add(Trimmer_Cam_Digital_H_Up);
        Trimmer_Cam_Digital_H.add(Trimmer_Cam_Digital_H_Center);
        Trimmer_Cam_Digital_H.add(Trimmer_Cam_Digital_H_Down).addRight(Trimmer_Cam_Digital_H_Center).addRight(Trimmer_Cam_Digital_H_Up).addRight(Trimmer_Cam_Digital_H_DefaultSettings).addRight(Trimmer_Cam_Digital_H_Down);
             Trimmer_Cam_Digital_H_DefaultSettings.add(Trimmer_Cam_Digital_H_DefaultSettings_NO);
             Trimmer_Cam_Digital_H_DefaultSettings.add(Trimmer_Cam_Digital_H_DefaultSettings_YES).addRight(Trimmer_Cam_Digital_H_DefaultSettings_NO);
        Trimmer_Cam_Analog_V.add(Trimmer_Cam_Analog_V_DefaultSettings);
        Trimmer_Cam_Analog_V.add(Trimmer_Cam_Analog_V_Center).addRight(Trimmer_Cam_Analog_V_DefaultSettings).addRight(Trimmer_Cam_Analog_V_Center);
            Trimmer_Cam_Analog_V_DefaultSettings.add(Trimmer_Cam_Analog_V_DefaultSettings_NO);
            Trimmer_Cam_Analog_V_DefaultSettings.add(Trimmer_Cam_Analog_V_DefaultSettings_YES).addRight(Trimmer_Cam_Analog_V_DefaultSettings_NO).addRight(Trimmer_Cam_Analog_V_DefaultSettings_YES);
        Trimmer_Cam_Analog_H.add(Trimmer_Cam_Analog_H_DefaultSettings);
        Trimmer_Cam_Analog_H.add(Trimmer_Cam_Analog_H_Center).addRight(Trimmer_Cam_Analog_H_DefaultSettings).addRight(Trimmer_Cam_Analog_H_Center);
            Trimmer_Cam_Analog_H_DefaultSettings.add(Trimmer_Cam_Analog_H_DefaultSettings_NO);
            Trimmer_Cam_Analog_H_DefaultSettings.add(Trimmer_Cam_Analog_H_DefaultSettings_YES).addRight(Trimmer_Cam_Analog_H_DefaultSettings_NO).addRight(Trimmer_Cam_Analog_H_DefaultSettings_YES);
        Trimmer_Aux1_Digital.add(Trimmer_Aux1_Digital_DefaultSettings);
        Trimmer_Aux1_Digital.add(Trimmer_Aux1_Digital_II);
        Trimmer_Aux1_Digital.add(Trimmer_Aux1_Digital_O);
        Trimmer_Aux1_Digital.add(Trimmer_Aux1_Digital_I).addRight(Trimmer_Aux1_Digital_O).addRight(Trimmer_Aux1_Digital_II).addRight(Trimmer_Aux1_Digital_DefaultSettings).addRight(Trimmer_Aux1_Digital_I);
            Trimmer_Aux1_Digital_DefaultSettings.add(Trimmer_Aux1_Digital_DefaultSettings_NO);
            Trimmer_Aux1_Digital_DefaultSettings.add(Trimmer_Aux1_Digital_DefaultSettings_YES).addRight(Trimmer_Aux1_Digital_DefaultSettings_NO).addRight(Trimmer_Aux1_Digital_DefaultSettings_YES);
        Trimmer_Aux2_Digital.add(Trimmer_Aux2_Digital_DefaultSettings);
        Trimmer_Aux2_Digital.add(Trimmer_Aux2_Digital_II);
        Trimmer_Aux2_Digital.add(Trimmer_Aux2_Digital_O);
        Trimmer_Aux2_Digital.add(Trimmer_Aux2_Digital_I).addRight(Trimmer_Aux2_Digital_O).addRight(Trimmer_Aux2_Digital_II).addRight(Trimmer_Aux2_Digital_DefaultSettings).addRight(Trimmer_Aux2_Digital_I);
            Trimmer_Aux2_Digital_DefaultSettings.add(Trimmer_Aux2_Digital_DefaultSettings_NO);
            Trimmer_Aux2_Digital_DefaultSettings.add(Trimmer_Aux2_Digital_DefaultSettings_YES).addRight(Trimmer_Aux2_Digital_DefaultSettings_NO).addRight(Trimmer_Aux2_Digital_DefaultSettings_YES);
        Trimmer_Aux1_Analog.add(Trimmer_Aux1_Analog_DefaultSettings);
        Trimmer_Aux1_Analog.add(Trimmer_Aux1_Analog_Center).addRight(Trimmer_Aux1_Analog_DefaultSettings).addRight(Trimmer_Aux1_Analog_Center);
            Trimmer_Aux1_Analog_DefaultSettings.add(Trimmer_Aux1_Analog_DefaultSettings_NO);
            Trimmer_Aux1_Analog_DefaultSettings.add(Trimmer_Aux1_Analog_DefaultSettings_YES).addRight(Trimmer_Aux1_Analog_DefaultSettings_NO).addRight(Trimmer_Aux1_Analog_DefaultSettings_YES);
        Trimmer_Aux2_Analog.add(Trimmer_Aux2_Analog_DefaultSettings);
        Trimmer_Aux2_Analog.add(Trimmer_Aux2_Analog_Center).addRight(Trimmer_Aux2_Analog_DefaultSettings).addRight(Trimmer_Aux2_Analog_Center);
            Trimmer_Aux2_Analog_DefaultSettings.add(Trimmer_Aux2_Analog_DefaultSettings_NO);
            Trimmer_Aux2_Analog_DefaultSettings.add(Trimmer_Aux2_Analog_DefaultSettings_YES).addRight(Trimmer_Aux2_Analog_DefaultSettings_NO).addRight(Trimmer_Aux2_Analog_DefaultSettings_YES);       

        Trimmer_Trigger_Digital.add(Trimmer_Trigger_Digital_DefaultSettings);
        Trimmer_Trigger_Digital.add(Trimmer_Trigger_Digital_Off);
        Trimmer_Trigger_Digital.add(Trimmer_Trigger_Digital_On).addRight(Trimmer_Trigger_Digital_Off).addRight(Trimmer_Trigger_Digital_DefaultSettings).addRight(Trimmer_Trigger_Digital_On);
            Trimmer_Trigger_Digital_DefaultSettings.add(Trimmer_Trigger_Digital_DefaultSettings_NO);
            Trimmer_Trigger_Digital_DefaultSettings.add(Trimmer_Trigger_Digital_DefaultSettings_YES).addRight(Trimmer_Trigger_Digital_DefaultSettings_NO).addRight(Trimmer_Trigger_Digital_DefaultSettings_YES);

    DefaultSettings_for_this_Plane.add(DefaultSettings_for_this_Plane_NO);
    DefaultSettings_for_this_Plane.add(DefaultSettings_for_this_Plane_YES).addRight(DefaultSettings_for_this_Plane_NO).addRight(DefaultSettings_for_this_Plane_YES);

    Invert_Mode.add(Invert_Aux2);
    Invert_Mode.add(Invert_Aux1);
    Invert_Mode.add(Invert_Cam_Analog_H);
    Invert_Mode.add(Invert_Cam_Analog_V);
    Invert_Mode.add(Invert_Cam_Digital_H);
    Invert_Mode.add(Invert_Cam_Digital_V);
    Invert_Mode.add(Invert_Throttle);
    Invert_Mode.add(Invert_Rudder);
    Invert_Mode.add(Invert_Elevator);
    Invert_Mode.add(Invert_Aileron).addRight(Invert_Elevator).addRight(Invert_Rudder).addRight(Invert_Throttle).addRight(Invert_Cam_Digital_V).addRight(Invert_Cam_Digital_H).addRight(Invert_Cam_Analog_V).addRight(Invert_Cam_Analog_H).addRight(Invert_Aux1).addRight(Invert_Aux2).addRight(Invert_Aileron);

    Exponential_Mode.add(Exponential_Aux2);
    Exponential_Mode.add(Exponential_Aux1);
    Exponential_Mode.add(Exponential_Throttle);
    Exponential_Mode.add(Exponential_Rudder);
    Exponential_Mode.add(Exponential_Elevator);
    Exponential_Mode.add(Exponential_Aileron).addRight(Exponential_Elevator).addRight(Exponential_Rudder).addRight(Exponential_Throttle).addRight(Exponential_Aux1).addRight(Exponential_Aux2).addRight(Exponential_Aileron);

    Plane_Mode.add(Plane_V_Tail_Wing);
    Plane_Mode.add(Plane_Flying_Wing);
    Plane_Mode.add(Plane_Classic).addRight(Plane_Flying_Wing).addRight(Plane_V_Tail_Wing).addRight(Plane_Classic);

    Joystick_Auto_Trimming.add(Auto_Aux2);
    Joystick_Auto_Trimming.add(Auto_Aux1);
    Joystick_Auto_Trimming.add(Auto_Cam_Analog_H);
    Joystick_Auto_Trimming.add(Auto_Cam_Analog_V);
    Joystick_Auto_Trimming.add(Auto_Throttle);
    Joystick_Auto_Trimming.add(Auto_Rudder);
    Joystick_Auto_Trimming.add(Auto_Elevator);
    Joystick_Auto_Trimming.add(Auto_Aileron).addRight(Auto_Elevator).addRight(Auto_Rudder).addRight(Auto_Throttle).addRight(Auto_Cam_Analog_V).addRight(Auto_Cam_Analog_H).addRight(Auto_Aux1).addRight(Auto_Aux2).addRight(Auto_Aileron); 

    Other_Settings.add(CamAnalogDelay_Mode);
    Other_Settings.add(CamDigitalDelay_Mode);
    Other_Settings.add(AUX2_Mode);
    Other_Settings.add(AUX1_Mode);
    Other_Settings.add(Cam_Mode);
    Other_Settings.add(Invert_PPM).addRight(Cam_Mode).addRight(AUX1_Mode).addRight(AUX2_Mode).addRight(CamDigitalDelay_Mode).addRight(CamAnalogDelay_Mode).addRight(Invert_PPM);

    Test_Channel.add(Test_Aux2);
    Test_Channel.add(Test_Aux1);
    Test_Channel.add(Test_Cam_Analog_H);
    Test_Channel.add(Test_Cam_Analog_V);
    Test_Channel.add(Test_Throttle);
    Test_Channel.add(Test_Rudder);
    Test_Channel.add(Test_Elevator);
    Test_Channel.add(Test_Aileron).addRight(Test_Elevator).addRight(Test_Rudder).addRight(Test_Throttle).addRight(Test_Cam_Analog_V).addRight(Test_Cam_Analog_H).addRight(Test_Aux1).addRight(Test_Aux2).addRight(Test_Aileron);
         
    menu.toRoot();
}
//-------------------------------------------------------------------------------------------------------
void DisplayInit()
{
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // Clear the buffer.
  display.clearDisplay();
  display.drawBitmap(30, 0,  Due.MyLogo(0), 128, 64, 1);
  display.display();
  delay(500);
  for(int i = 1; i < 41; i++)
  {
    display.clearDisplay();
    display.drawBitmap(30, 0,  Due.MyLogo(i), 128, 64, 1);
    if (i == 15) delay(300);
    if (i < 16)
    { 
      delay(6*i);
      if (i == 20) delay(500);
    }
    if (i == 25) delay(300);
    display.display();
  }
}
//-------------------------------------------------------------------------------------------------------
void readButtons()
{
  Due.AnalogRead (analogValue);
  Due.DigitalRead(Aux1_I, Aux1_II, Aux2_I, Aux2_II);
  
  ProMini.setControls(Enter,Cancel,RatesLow,RatesHigh,Right,Up,Left,Down,CameraRight,CameraDown,CameraLeft,CameraUp,Trigger);
}
//-------------------------------------------------------------------------------------------------------
void SerialPrint_all_controls()
{
  Serial.println();
  Serial.print("Battery=");
  Serial.print(analogValue[0]);
  Serial.print(" Aux1*=");
  Serial.print(analogValue[1]);
  Serial.print(" Aux1=");
  Serial.print(analogValue[3]);
  Serial.print(" Aux2=");
  Serial.print(analogValue[2]);
  Serial.print(" CamH=");
  Serial.print(analogValue[4]);
  Serial.print(" CamV=");
  Serial.print(analogValue[5]);
  Serial.print(" Rudder=");
  Serial.print(analogValue[6]);
  Serial.print(" Throttle=");
  Serial.print(analogValue[7]);
  Serial.print(" Aileron=");
  Serial.print(analogValue[8]);
  Serial.print(" Elevator=");
  Serial.print(analogValue[9]);
  Serial.print(" frame1=");
  Serial.print(Enter);Serial.print(Cancel);Serial.print(RatesLow);Serial.print(RatesHigh);Serial.print(CameraRight);Serial.print(CameraUp);Serial.print(CameraLeft);Serial.print(CameraDown);Serial.print(Right);Serial.print(Down);Serial.print(Left);Serial.print(Up);
  Serial.print(" Aux1_I=");
  Serial.print(Aux1_I);
  Serial.print(" Aux1_II=");
  Serial.print(Aux1_II);
  Serial.print(" Aux2_I=");
  Serial.print(Aux2_I);
  Serial.print(" Aux2_II=");
  Serial.print(Aux2_II);
}
//-------------------------------------------------------------------------------------------------------
void MessageBox_AutoTrimmingSaved_Show()
{
  display.setCursor(0,0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.println("   SAVED");
  display.println();
  display.setTextSize(1);
  display.print  ("   min="); display.print(minValue); display.print  ("   max="); display.println(maxValue); 
  display.setCursor(0,52) ;
  if (iterations != 0){ display.print  ("      center="); display.print(cenValue/iterations);} else { display.print  ("      center="); display.print(cenValue);}
  display.display();
}
void MessageBox_AutoTrimmingTimerGo_Show()
{
  display.clearDisplay();display.setTextSize(1);display.setTextColor(WHITE);display.setCursor(0,8);display.println(" Please, don't touch!");display.setCursor(42,28);display.setTextSize(2);display.println(Due.Haw_much_time_elapsed());display.setTextSize(1);display.println();display.print  ("      center="); display.print(cenValue/iterations);  display.display();
}
void AutoTrimmingTable1_Show()
{
  display.clearDisplay();
  display.setCursor(0,0); display.setTextSize(1);display.setTextColor(WHITE);
  display.drawRect(0,0,display.width(),12, WHITE);
  display.drawRect(0,11,display.width(),14, WHITE);
  display.drawRect(0,24,display.width(),14, WHITE);
  display.drawRect(0,37,display.width(),14, WHITE);
  display.drawRect(0,50,display.width(),14, WHITE);
  display.drawLine(64, 0, 64, display.height(), WHITE);
  display.drawLine(96, 0, 96, display.height(), WHITE);
  display.setCursor(0,3);
  display.println("  AUTO      MIN  MAX");
  display.setCursor(3,15);  
}
void TrimmingTable_Show(int Value)
{
  display.clearDisplay();   display.setTextSize(4);display.setTextColor(WHITE);
  if (Value >= 1000) display.setCursor(18,20); else display.setCursor(33,20);
  if (Value<10) display.print(" "); else if (Value<100)  display.setCursor(44,20);
  display.println(Value); 
  
  display.display();
}
void MessageBox_TrimmingSaved_Show(int Value)
{
  display.setCursor(0,0);  display.setTextSize(2);  display.setTextColor(WHITE);  display.clearDisplay();
  display.println("   SAVED");
  display.setCursor(0,37);display.setTextSize(1);display.print  ("      center="); display.print(Value);
  //display.setCursor(33,20); display.setTextSize(2);
  //display.println(Value); 
  display.display();
}
void SerialPrint_PPM()
{
  for (int i = 0; i < 9; i++)
  {
    Serial.print(" PPM[");
    Serial.print(i);
    Serial.print("]=");
    Serial.print(ppm_channels[i]-1000);
  }
  Serial.println("");
}
void CangeAndSave(String MenuName, bool Flag)
{
  if(MenuName != " ")
  {
    if ((MenuName == "First Plane") && (Flag == true))
    {
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED");
      display.println();
      display.setTextSize(1);
      display.println(" Selected first Plane");
      display.display();
      dueFlashStorage.write(84, (byte)0);
      PlaneActive = 0;
      delay(1500);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Second Plane") && (Flag == true))
    {
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED");
      display.println();
      display.setTextSize(1);
      display.setCursor(1,32);
      display.println("Selected second Plane");
      display.display();
      dueFlashStorage.write(84, (byte)1);
      PlaneActive = 1;
      delay(1500);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Third Plane") && (Flag == true))
    {
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED");
      display.println();
      display.setTextSize(1);
      display.println(" Selected Third Plane");
      display.display();
      dueFlashStorage.write(84, (byte)2);
      PlaneActive = 2;
      delay(1500);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Auto Aileron") && (Flag == true))
    {
      AutoTrimmingTable1_Show();
      display.fillRect(64, 11, 128, 14, WHITE); display.drawLine(96, 11, 96, 24, BLACK);
      if (analogValue[8] <= minValue)
      minValue = analogValue[8];
      if (analogValue[8] >= maxValue)
      maxValue = analogValue[8];
      display.print("Aileron    ");display.setTextColor(BLACK); if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,15); if(maxValue<1000)display.print(" "); display.println(maxValue);
      display.setTextColor(WHITE);
      display.setCursor(3,28);
      display.print("Elevator   ");if(minElevator<250)display.print(" ");if(minElevator<25)display.print(" ");if(minElevator<=2)display.print(" ");display.print(minElevator*4); display.setCursor(99,28); if(maxElevator<250 )display.print(" "); display.println(maxElevator*4);
      display.setCursor(3,41);
      display.print("Rudder     ");if(minRudder<250)display.print(" ");if(minRudder<25)display.print(" ");if(minRudder<=2)display.print(" ");display.print(minRudder*4); display.setCursor(99,41); if(maxRudder<250 )display.print(" "); display.println(maxRudder*4);
      display.setCursor(3,54);
      display.print("Throttle   ");if(minThrottle<250)display.print(" ");if(minThrottle<25)display.print(" ");if(minThrottle<=2)display.print(" ");display.print(minThrottle*4); display.setCursor(99,54); if(maxThrottle<250 )display.print(" "); display.println(maxThrottle*4);   
      display.display();
    } else  if ((MenuName == "Auto Aileron") && (Flag == false))
    {
      Due.startTimer();
      if (Due.isTimerEnd())
      {
        MessageBox_AutoTrimmingSaved_Show();
        minAileron = minValue/4+1;                                                    minValue = 1024;                dueFlashStorage.write(0, (byte)(minAileron));
        if (maxValue%4!=0)maxAileron = maxValue/4;else maxAileron = maxValue/4-1;     maxValue = 0;                   dueFlashStorage.write(1, (byte)(maxAileron));
        centerAileron = cenValue/iterations/4;                                        cenValue = 0; iterations = 0;   dueFlashStorage.write(2, (byte)(centerAileron)); 
        delay(1500);
        MenuItem_is_Change = " ";
        menu.moveRight();
      }else if (Due.Haw_much_time_elapsed()<5){cenValue += analogValue[8]; iterations++; MessageBox_AutoTrimmingTimerGo_Show();} 
    }
    if ((MenuName == "Auto Elevator") && (Flag == true))
    {
      AutoTrimmingTable1_Show();
      display.print("Aileron    ");if(minAileron<250)display.print(" ");if(minAileron<25)display.print(" ");if(minAileron<=2)display.print(" ");display.print(minAileron*4); display.setCursor(99,15); if(maxAileron<250 )display.print(" "); display.println(maxAileron*4);
      display.setCursor(3,28);
      display.fillRect(64, 24, 128, 14, WHITE); display.drawLine(96, 24, 96, 37, BLACK);
      if (analogValue[9] <= minValue)
      minValue = analogValue[9];
      if (analogValue[9] >= maxValue)
      maxValue = analogValue[9];
      display.print("Elevator   ");display.setTextColor(BLACK); if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,28); if(maxValue<1000)display.print(" "); display.println(maxValue);
      display.setTextColor(WHITE);
      display.setCursor(3,41);
      display.print("Rudder     ");if(minRudder<250)display.print(" ");if(minRudder<25)display.print(" ");if(minRudder<=2)display.print(" ");display.print(minRudder*4); display.setCursor(99,41); if(maxRudder<250 )display.print(" "); display.println(maxRudder*4);
      display.setCursor(3,54);
      display.print("Throttle   ");if(minThrottle<250)display.print(" ");if(minThrottle<25)display.print(" ");if(minThrottle<=2)display.print(" ");display.print(minThrottle*4); display.setCursor(99,54); if(maxThrottle<250 )display.print(" "); display.println(maxThrottle*4);   
      display.display();
    } else  if ((MenuName == "Auto Elevator") && (Flag == false))
    {
      Due.startTimer();
      if (Due.isTimerEnd())
      {
      MessageBox_AutoTrimmingSaved_Show();
      minElevator = minValue/4+1;                                                     minValue = 1024;                dueFlashStorage.write(6, (byte)(minElevator));
      if (maxValue%4!=0)maxElevator = maxValue/4;else maxElevator = maxValue/4-1;     maxValue = 0;                   dueFlashStorage.write(7, (byte)(maxElevator));
      centerElevator = cenValue/iterations/4;                                         cenValue = 0; iterations = 0;   dueFlashStorage.write(8, (byte)(centerElevator));
      delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
      }else if (Due.Haw_much_time_elapsed()<5){cenValue += analogValue[9]; iterations++; MessageBox_AutoTrimmingTimerGo_Show();}
    }
    if ((MenuName == "Auto Rudder") && (Flag == true))
    {
      AutoTrimmingTable1_Show();
      display.print("Aileron    ");if(minAileron<250)display.print(" ");if(minAileron<25)display.print(" ");if(minAileron<=2)display.print(" ");display.print(minAileron*4); display.setCursor(99,15); if(maxAileron<250 )display.print(" "); display.println(maxAileron*4);
      display.setCursor(3,28);
      display.print("Elevator   ");if(minElevator<250)display.print(" ");if(minElevator<25)display.print(" ");if(minElevator<=2)display.print(" ");display.print(minElevator*4); display.setCursor(99,28); if(maxElevator<250 )display.print(" "); display.println(maxElevator*4);
      display.setCursor(3,41);
      display.fillRect(64, 37, 128, 14, WHITE); display.drawLine(96, 37, 96, 50, BLACK);
      if (analogValue[6] <= minValue)
      minValue = analogValue[6];
      if (analogValue[6] >= maxValue)
      maxValue = analogValue[6];
      display.print("Rudder     ");display.setTextColor(BLACK); if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,41); if(maxValue<1000)display.print(" "); display.println(maxValue);
      display.setTextColor(WHITE);
      display.setCursor(3,54);
      display.print("Throttle   ");if(minThrottle<250)display.print(" ");if(minThrottle<25)display.print(" ");if(minThrottle<=2)display.print(" ");display.print(minThrottle*4); display.setCursor(99,54); if(maxThrottle<250 )display.print(" "); display.println(maxThrottle*4);   
      display.display();
    } else  if ((MenuName == "Auto Rudder") && (Flag == false))
    {
      Due.startTimer();
      if (Due.isTimerEnd())
      {
      MessageBox_AutoTrimmingSaved_Show();
      minRudder = minValue/4+1;                                                   minValue = 1024;                dueFlashStorage.write(12, (byte)(minRudder));
      if (maxValue%4!=0)maxRudder = maxValue/4;else maxRudder = maxValue/4-1;     maxValue = 0;                   dueFlashStorage.write(13, (byte)(maxRudder));
      centerRudder = cenValue/iterations/4;                                       cenValue = 0; iterations = 0;   dueFlashStorage.write(14, (byte)(centerRudder));
      delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
      }else if (Due.Haw_much_time_elapsed()<5){cenValue += analogValue[6]; iterations++;MessageBox_AutoTrimmingTimerGo_Show();}
    }
    if ((MenuName == "Auto Throttle") && (Flag == true))
    {
      AutoTrimmingTable1_Show();
      display.print("Aileron    ");if(minAileron<250)display.print(" ");if(minAileron<25)display.print(" ");if(minAileron<=2)display.print(" ");display.print(minAileron*4); display.setCursor(99,15); if(maxAileron<250 )display.print(" "); display.println(maxAileron*4);
      display.setCursor(3,28);
      display.print("Elevator   ");if(minElevator<250)display.print(" ");if(minElevator<25)display.print(" ");if(minElevator<=2)display.print(" ");display.print(minElevator*4); display.setCursor(99,28); if(maxElevator<250 )display.print(" "); display.println(maxElevator*4);
      display.setCursor(3,41);
      display.print("Rudder     ");if(minRudder<250)display.print(" ");if(minRudder<25)display.print(" ");if(minRudder<=2)display.print(" ");display.print(minRudder*4); display.setCursor(99,41); if(maxRudder<250 )display.print(" "); display.println(maxRudder*4);
      display.setCursor(3,54);
      display.fillRect(64, 50, 128, 14, WHITE); display.drawLine(96, 50, 96, 50, BLACK);
      if (analogValue[7] <= minValue)
      minValue = analogValue[7];
      if (analogValue[7] >= maxValue)
      maxValue = analogValue[7];
      display.print("Throttle   ");display.setTextColor(BLACK); if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,54); if(maxValue<1000)display.print(" "); display.println(maxValue); 
      display.setTextColor(WHITE);
      display.display();
    } else  if ((MenuName == "Auto Throttle") && (Flag == false))
    {
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("   min="); display.print(minValue); display.print  ("   max="); display.println(maxValue); 
      display.display();
      minThrottle = minValue/4+1;                                                     minValue = 1024;                dueFlashStorage.write(18, (byte)(minThrottle));
      if (maxValue%4!=0)maxThrottle = maxValue/4;else maxThrottle = maxValue/4-1;     maxValue = 0;                   dueFlashStorage.write(22, (byte)(maxThrottle));
      delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Auto Cam Analog V") && (Flag == true))
    {
      AutoTrimmingTable1_Show();
      display.fillRect(64, 11, 128, 14, WHITE); display.drawLine(96, 11, 96, 24, BLACK);
      if (analogValue[5] <= minValue)
      minValue = analogValue[5];
      if (analogValue[5] >= maxValue)
      maxValue = analogValue[5];
      display.print("CamV An    ");display.setTextColor(BLACK); if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,15); if(maxValue<1000)display.print(" "); display.println(maxValue);
      display.setTextColor(WHITE);
      display.setCursor(3,28);
      display.print("CamH An    ");if(minCam_Analog_H<250)display.print(" ");if(minCam_Analog_H<25)display.print(" ");if(minCam_Analog_H<=2)display.print(" ");display.print(minCam_Analog_H*4); display.setCursor(99,28); if(maxCam_Analog_H<250 )display.print(" "); display.println(maxCam_Analog_H*4);
      display.setCursor(3,41);
      if (AUX1ModeActive == (B00000010))
      {
        display.print("Aux1' An   ");if(minAux1Analog_<250)display.print(" ");if(minAux1Analog_<25)display.print(" ");if(minAux1Analog_<=2)display.print(" ");display.print(minAux1Analog_*4); display.setCursor(99,41); if(maxAux1Analog_<250 )display.print(" "); display.println(maxAux1Analog_*4);
      }else if (AUX1ModeActive == (B00000100))
      {
        display.print("Aux1 An    ");if(minAux1Analog<250)display.print(" ");if(minAux1Analog<25)display.print(" ");if(minAux1Analog<=2)display.print(" ");display.print(minAux1Analog*4); display.setCursor(99,41); if(maxAux1Analog<250 )display.print(" "); display.println(maxAux1Analog*4);
      }else display.println("Aux1/Aux1'");
      display.setCursor(3,54);
      display.print("Aux2 An    ");if(minAux2Analog<250)display.print(" ");if(minAux2Analog<25)display.print(" ");if(minAux2Analog<=2)display.print(" ");display.print(minAux2Analog*4); display.setCursor(99,54); if(maxAux2Analog<250 )display.print(" "); display.println(maxAux2Analog*4);   
      display.display();
    } else  if ((MenuName == "Auto Cam Analog V") && (Flag == false))
    {
      Due.startTimer();
      if (Due.isTimerEnd())
      {
        MessageBox_AutoTrimmingSaved_Show();
        minCam_Analog_V = minValue/4+1;                                                         minValue = 1024;                dueFlashStorage.write(42, (byte)(minCam_Analog_V));
        if (maxValue%4!=0)maxCam_Analog_V = maxValue/4;else maxCam_Analog_V = maxValue/4-1;     maxValue = 0;                   dueFlashStorage.write(43, (byte)(maxCam_Analog_V));
        centerCam_Analog_V = cenValue/iterations/4;                                             cenValue = 0; iterations = 0;   dueFlashStorage.write(44, (byte)(centerCam_Analog_V)); 
        delay(1500);
        MenuItem_is_Change = " ";
        menu.moveRight();
      }else if (Due.Haw_much_time_elapsed()<5){cenValue += analogValue[5]; iterations++; MessageBox_AutoTrimmingTimerGo_Show();} 
    }
    if ((MenuName == "Auto Cam Analog H") && (Flag == true))
    {
      AutoTrimmingTable1_Show();
      display.print("CamV An    ");if(minCam_Analog_V<250)display.print(" ");if(minCam_Analog_V<25)display.print(" ");if(minCam_Analog_V<=2)display.print(" ");display.print(minCam_Analog_V*4); display.setCursor(99,15); if(maxCam_Analog_V<250 )display.print(" "); display.println(maxCam_Analog_V*4);
      display.setCursor(3,28);
      display.fillRect(64, 24, 128, 14, WHITE); display.drawLine(96, 24, 96, 37, BLACK);
      if (analogValue[4] <= minValue)
      minValue = analogValue[4];
      if (analogValue[4] >= maxValue)
      maxValue = analogValue[4];
      display.print("CamH An    ");display.setTextColor(BLACK); if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,28); if(maxValue<1000)display.print(" "); display.println(maxValue);
      display.setTextColor(WHITE);
      display.setCursor(3,41);
      if (AUX1ModeActive == (B00000010))
      {
        display.print("Aux1' An   ");if(minAux1Analog_<250)display.print(" ");if(minAux1Analog_<25)display.print(" ");if(minAux1Analog_<=2)display.print(" ");display.print(minAux1Analog_*4); display.setCursor(99,41); if(maxAux1Analog_<250 )display.print(" "); display.println(maxAux1Analog_*4);
      }else if (AUX1ModeActive == (B00000100))
      {
        display.print("Aux1 An    ");if(minAux1Analog<250)display.print(" ");if(minAux1Analog<25)display.print(" ");if(minAux1Analog<=2)display.print(" ");display.print(minAux1Analog*4); display.setCursor(99,41); if(maxAux1Analog<250 )display.print(" "); display.println(maxAux1Analog*4);
      }else display.println("Aux1/Aux1'");
      display.setCursor(3,54);
      display.print("Aux2 An    ");if(minAux2Analog<250)display.print(" ");if(minAux2Analog<25)display.print(" ");if(minAux2Analog<=2)display.print(" ");display.print(minAux2Analog*4); display.setCursor(99,54); if(maxAux2Analog<250 )display.print(" "); display.println(maxAux2Analog*4);   
      display.display();
    } else  if ((MenuName == "Auto Cam Analog H") && (Flag == false))
    {
      Due.startTimer();
      if (Due.isTimerEnd())
      {
      MessageBox_AutoTrimmingSaved_Show();
      minCam_Analog_H = minValue/4+1;                                                         minValue = 1024;                dueFlashStorage.write(48, (byte)(minCam_Analog_H));
      if (maxValue%4!=0)maxCam_Analog_H = maxValue/4;else maxCam_Analog_H = maxValue/4-1;     maxValue = 0;                   dueFlashStorage.write(49, (byte)(maxCam_Analog_H));
      centerCam_Analog_H = cenValue/iterations/4;                                             cenValue = 0; iterations = 0;   dueFlashStorage.write(50, (byte)(centerCam_Analog_H));
      delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
      }else if (Due.Haw_much_time_elapsed()<5){cenValue += analogValue[4]; iterations++; MessageBox_AutoTrimmingTimerGo_Show();}
    }
    if ((MenuName == "Auto Aux1") && (Flag == true))
    {
      AutoTrimmingTable1_Show();
      display.print("CamV An    ");if(minCam_Analog_V<250)display.print(" ");if(minCam_Analog_V<25)display.print(" ");if(minCam_Analog_V<=2)display.print(" ");display.print(minCam_Analog_V*4); display.setCursor(99,15); if(maxCam_Analog_V<250 )display.print(" "); display.println(maxCam_Analog_V*4);
      display.setCursor(3,28);
      display.print("CamH An    ");if(minCam_Analog_H<250)display.print(" ");if(minCam_Analog_H<25)display.print(" ");if(minCam_Analog_H<=2)display.print(" ");display.print(minCam_Analog_H*4); display.setCursor(99,28); if(maxCam_Analog_H<250 )display.print(" "); display.println(maxCam_Analog_H*4);
      display.setCursor(3,41);
      display.fillRect(64, 37, 128, 14, WHITE); display.drawLine(96, 37, 96, 50, BLACK);
      if (AUX1ModeActive == (B00000010))
      {
        if (analogValue[1] <= minValue)
        minValue = analogValue[1];
        if (analogValue[1] >= maxValue)
        maxValue = analogValue[1];
      }else if (AUX1ModeActive == (B00000100))
      {
        if (analogValue[3] <= minValue)
        minValue = analogValue[3];
        if (analogValue[3] >= maxValue)
        maxValue = analogValue[3];
      }
      if (AUX1ModeActive == (B00000010))
      {
        display.print("Aux1' An   ");display.setTextColor(BLACK);if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,41); if(maxValue<1000)display.print(" "); display.println(maxValue); 
      }else if (AUX1ModeActive == (B00000100))
      {
        display.print("Aux1 An    ");display.setTextColor(BLACK);if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,41); if(maxValue<1000)display.print(" "); display.println(maxValue); 
      }else display.println("Aux1/Aux1'");
      display.setTextColor(WHITE);
      display.setCursor(3,54);
      display.print("Aux2 An    ");if(minAux2Analog<250)display.print(" ");if(minAux2Analog<25)display.print(" ");if(minAux2Analog<=2)display.print(" ");display.print(minAux2Analog*4); display.setCursor(99,54); if(maxAux2Analog<250 )display.print(" "); display.println(maxAux2Analog*4);   
      display.display();
    } else  if ((MenuName == "Auto Aux1") && (Flag == false))
    {
      if (AUX1ModeActive == (B00000010))
      {
        //Due.startTimer();
        //if (Due.isTimerEnd())
        //{
          cenValue = (minValue + maxValue)/2;
          MessageBox_AutoTrimmingSaved_Show();
          minAux1Analog_ = minValue/4+1;                                                       minValue = 1024;                dueFlashStorage.write(99, (byte)   (minAux1Analog_));
          if (maxValue%4!=0)maxAux1Analog_ = maxValue/4;else maxAux1Analog_ = maxValue/4-1;    maxValue = 0;                   dueFlashStorage.write(100, (byte) (maxAux1Analog_));
          centerAux1Analog_ = cenValue/4;                                                      cenValue = 0; iterations = 0;   dueFlashStorage.write(101, (byte)   (centerAux1Analog_));
          delay(1500);
          MenuItem_is_Change = " ";
          menu.moveRight();
        //}else if (Due.Haw_much_time_elapsed()<5){cenValue += analogValue[1]; iterations++;MessageBox_AutoTrimmingTimerGo_Show();}
      }else if (AUX1ModeActive == (B00000100))
      {
        //Due.startTimer();
      //if (Due.isTimerEnd())
      //{
      cenValue = (minValue + maxValue)/2;
      MessageBox_AutoTrimmingSaved_Show();
      minAux1Analog = minValue/4+1;                                                       minValue = 1024;                dueFlashStorage.write(72, (byte)(minAux1Analog));
      if (maxValue%4!=0)maxAux1Analog = maxValue/4;else maxAux1Analog = maxValue/4-1;     maxValue = 0;                   dueFlashStorage.write(73, (byte)(maxAux1Analog));
      centerAux1Analog = cenValue/4;                                                      cenValue = 0; iterations = 0;   dueFlashStorage.write(74, (byte)(centerAux1Analog));
      delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
      //}else if (Due.Haw_much_time_elapsed()<5){cenValue += analogValue[3]; iterations++;MessageBox_AutoTrimmingTimerGo_Show();}
      }
      
    }
    if ((MenuName == "Auto Aux2") && (Flag == true))
    {
      AutoTrimmingTable1_Show();
      display.print("CamV An    ");if(minCam_Analog_V<250)display.print(" ");if(minCam_Analog_V<25)display.print(" ");if(minCam_Analog_V<=2)display.print(" ");display.print(minCam_Analog_V*4); display.setCursor(99,15); if(maxCam_Analog_V<250 )display.print(" "); display.println(maxCam_Analog_V*4);
      display.setCursor(3,28);
      display.print("CamH An    ");if(minCam_Analog_H<250)display.print(" ");if(minCam_Analog_H<25)display.print(" ");if(minCam_Analog_H<=2)display.print(" ");display.print(minCam_Analog_H*4); display.setCursor(99,28); if(maxCam_Analog_H<250 )display.print(" "); display.println(maxCam_Analog_H*4);
      display.setCursor(3,41);if (AUX1ModeActive == (B00000010))
      {
        display.print("Aux1' An   ");if(minAux1Analog_<250)display.print(" ");if(minAux1Analog_<25)display.print(" ");if(minAux1Analog_<=2)display.print(" ");display.print(minAux1Analog_*4); display.setCursor(99,41); if(maxAux1Analog_<250 )display.print(" "); display.println(maxAux1Analog_*4);
      }else if (AUX1ModeActive == (B00000100))
      {
        display.print("Aux1 An    ");if(minAux1Analog<250)display.print(" ");if(minAux1Analog<25)display.print(" ");if(minAux1Analog<=2)display.print(" ");display.print(minAux1Analog*4); display.setCursor(99,41); if(maxAux1Analog<250 )display.print(" "); display.println(maxAux1Analog*4);
      }else display.println("Aux1/Aux1'");
      display.setCursor(3,54);
      display.fillRect(64, 50, 128, 14, WHITE); display.drawLine(96, 50, 96, 50, BLACK);
      if (analogValue[2] <= minValue)
      minValue = analogValue[2];
      if (analogValue[2] >= maxValue)
      maxValue = analogValue[2];
      display.print("Aux2 An    ");display.setTextColor(BLACK); if(minValue<1000)display.print(" ");if(minValue<100)display.print(" ");if(minValue<=10)display.print(" ");display.print(minValue); display.setCursor(99,54); if(maxValue<1000)display.print(" "); display.println(maxValue); 
      display.setTextColor(WHITE);
      display.display();
    } else  if ((MenuName == "Auto Aux2") && (Flag == false))
    {
      //Due.startTimer();
      //if (Due.isTimerEnd())
      //{
      cenValue = (minValue + maxValue)/2;//линейный закон
      MessageBox_AutoTrimmingSaved_Show();
      minAux2Analog = minValue/4+1;                                                       minValue = 1024;                dueFlashStorage.write(78, (byte)(minAux2Analog));
      if (maxValue%4!=0)maxAux2Analog = maxValue/4;else maxAux2Analog = maxValue/4-1;     maxValue = 0;                   dueFlashStorage.write(79, (byte)(maxAux2Analog));
      centerAux2Analog = cenValue/4;                                                      cenValue = 0; iterations = 0;   dueFlashStorage.write(80, (byte)(centerAux2Analog));
      delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
      //}else if (Due.Haw_much_time_elapsed()<5){cenValue += analogValue[2]; iterations++;MessageBox_AutoTrimmingTimerGo_Show();}
    }
    if ((MenuName == "Trimmer Aileron Center") && (Flag == true))
    {
      int TrimmingValue = (500 + (deltaAileron[dueFlashStorage.read(84)] - 128));
      if ((TrimmingValue + delta) >= (500 + 128)) delta--;
      if ((TrimmingValue + delta) <= (500 - 128)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Aileron Center") && (Flag == false))
    {
      dueFlashStorage.write(3+dueFlashStorage.read(84), (byte)(deltaAileron[dueFlashStorage.read(84)] + delta));
      deltaAileron[dueFlashStorage.read(84)] = dueFlashStorage.read(3+dueFlashStorage.read(84));
      delta = 0;
      MessageBox_TrimmingSaved_Show(500 + (deltaAileron[dueFlashStorage.read(84)] - 128)); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Aileron DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(3+dueFlashStorage.read(84), (byte)(128));
      deltaAileron[dueFlashStorage.read(84)] = dueFlashStorage.read(3+dueFlashStorage.read(84));
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Aileron DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    } 
    if ((MenuName == "Trimmer Elevator Center") && (Flag == true))
    {
      int TrimmingValue = (500 + (deltaElevator[dueFlashStorage.read(84)] - 128));
      if ((TrimmingValue + delta) >= (500 + 128)) delta--;
      if ((TrimmingValue + delta) <= (500 - 128)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Elevator Center") && (Flag == false))
    {
      dueFlashStorage.write(9+dueFlashStorage.read(84), (byte)(deltaElevator[dueFlashStorage.read(84)] + delta));
      deltaElevator[dueFlashStorage.read(84)] = dueFlashStorage.read(9+dueFlashStorage.read(84));
      delta = 0;
      MessageBox_TrimmingSaved_Show(500 + (deltaElevator[dueFlashStorage.read(84)] - 128)); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Elevator DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(9+dueFlashStorage.read(84), (byte)(128));
      deltaElevator[dueFlashStorage.read(84)] = dueFlashStorage.read(9+dueFlashStorage.read(84));
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Elevator DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    } 
    if ((MenuName == "Trimmer Rudder Center") && (Flag == true))
    {
      int TrimmingValue = (500 + (deltaRudder[dueFlashStorage.read(84)] - 128));
      if ((TrimmingValue + delta) >= (500 + 128)) delta--;
      if ((TrimmingValue + delta) <= (500 - 128)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Rudder Center") && (Flag == false))
    {
      dueFlashStorage.write(15+dueFlashStorage.read(84), (byte)(deltaRudder[dueFlashStorage.read(84)] + delta));
      deltaRudder[dueFlashStorage.read(84)] = dueFlashStorage.read(15+dueFlashStorage.read(84));
      delta = 0;
      MessageBox_TrimmingSaved_Show(500 + (deltaRudder[dueFlashStorage.read(84)] - 128)); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Rudder DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(15+dueFlashStorage.read(84), (byte)(128));
      deltaRudder[dueFlashStorage.read(84)] = dueFlashStorage.read(15+dueFlashStorage.read(84));
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Rudder DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    } 
    if ((MenuName == "Trimmer Throttle Min") && (Flag == true))
    {
      int TrimmingValue = (0 + deltaMinThrottle[dueFlashStorage.read(84)]);
      if ((TrimmingValue + delta) >= (0 + 256)) delta--;
      if ((TrimmingValue + delta) < (0)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Throttle Min") && (Flag == false))
    {
      dueFlashStorage.write(19+dueFlashStorage.read(84), (byte)(deltaMinThrottle[dueFlashStorage.read(84)] + delta));
      deltaMinThrottle[dueFlashStorage.read(84)] = dueFlashStorage.read(19+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("       min="); display.print(0 + deltaMinThrottle[dueFlashStorage.read(84)]);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Trimmer Throttle Max") && (Flag == true))
    {
      int TrimmingValue = (1000 - deltaMaxThrottle[dueFlashStorage.read(84)]);
      if ((TrimmingValue + delta) <= (1000 - 256)) delta++;
      if ((TrimmingValue + delta) > (1000)) delta--;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Throttle Max") && (Flag == false))
    {
      dueFlashStorage.write(23+dueFlashStorage.read(84), (byte)(deltaMaxThrottle[dueFlashStorage.read(84)] - delta));
      deltaMaxThrottle[dueFlashStorage.read(84)] = dueFlashStorage.read(23+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("       max="); display.print(1000 - deltaMaxThrottle[dueFlashStorage.read(84)]);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Throttle DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(19+dueFlashStorage.read(84), (byte)(0));
      dueFlashStorage.write(23+dueFlashStorage.read(84), (byte)(0));
      deltaMinThrottle[dueFlashStorage.read(84)] = dueFlashStorage.read(19+dueFlashStorage.read(84));
      deltaMaxThrottle[dueFlashStorage.read(84)] = dueFlashStorage.read(23+dueFlashStorage.read(84));
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Throttle DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    } 
    if ((MenuName == "Trimmer Cam Digital V Down") && (Flag == true))
    {
      int TrimmingValue = (0 + delta_minCam_Digital_V[dueFlashStorage.read(84)]);
      if ((TrimmingValue + delta) >= (0 + 256)) delta--;
      if ((TrimmingValue + delta) < (0)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Cam Digital V Down") && (Flag == false))
    {
      dueFlashStorage.write(26+dueFlashStorage.read(84), (byte)(delta_minCam_Digital_V[dueFlashStorage.read(84)] + delta));
      delta_minCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(26+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("      Down="); display.print(0 + delta_minCam_Digital_V[dueFlashStorage.read(84)]);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Cam Digital V Center") && (Flag == true))
    {
      int TrimmingValue = (500 + (delta_centerCam_Digital_V[dueFlashStorage.read(84)]-128));
      if ((TrimmingValue + delta) >= (628)) delta--;
      if ((TrimmingValue + delta) <= (372)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Cam Digital V Center") && (Flag == false))
    {
      dueFlashStorage.write(93+dueFlashStorage.read(84), (byte)(delta_centerCam_Digital_V[dueFlashStorage.read(84)] + delta));
      delta_centerCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(93+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("     Center="); display.print(500 + (delta_centerCam_Digital_V[dueFlashStorage.read(84)]-128));
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Cam Digital V Up") && (Flag == true))
    {
      int TrimmingValue = (1000 - delta_maxCam_Digital_V[dueFlashStorage.read(84)]);
      if ((TrimmingValue + delta) <= (744)) delta++;
      if ((TrimmingValue + delta) > (1000)) delta--;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Cam Digital V Up") && (Flag == false))
    {
      dueFlashStorage.write(30+dueFlashStorage.read(84), (byte)(delta_maxCam_Digital_V[dueFlashStorage.read(84)] - delta));
      delta_maxCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(30+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("       Up="); display.print(1000 - delta_maxCam_Digital_V[dueFlashStorage.read(84)]);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Cam Digital V DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(26+dueFlashStorage.read(84), (byte)(0));
      dueFlashStorage.write(93+dueFlashStorage.read(84), (byte)(128));
      dueFlashStorage.write(30+dueFlashStorage.read(84), (byte)(0));    
      delta_minCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(26+dueFlashStorage.read(84));
      delta_centerCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(93+dueFlashStorage.read(84));
      delta_maxCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(30+dueFlashStorage.read(84));  
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Cam Digital V DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Trimmer Cam Digital H Down") && (Flag == true))
    {
      int TrimmingValue = (0 + delta_minCam_Digital_H[dueFlashStorage.read(84)]);
      if ((TrimmingValue + delta) >= (0 + 256)) delta--;
      if ((TrimmingValue + delta) < (0)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Cam Digital H Down") && (Flag == false))
    {
      dueFlashStorage.write(33+dueFlashStorage.read(84), (byte)(delta_minCam_Digital_H[dueFlashStorage.read(84)] + delta));
      delta_minCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(33+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("      Down="); display.print(0 + delta_minCam_Digital_H[dueFlashStorage.read(84)]);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Cam Digital H Center") && (Flag == true))
    {
      int TrimmingValue = (500 + (delta_centerCam_Digital_H[dueFlashStorage.read(84)]-128));
      if ((TrimmingValue + delta) >= (628)) delta--;
      if ((TrimmingValue + delta) <= (372)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Cam Digital H Center") && (Flag == false))
    {
      dueFlashStorage.write(39+dueFlashStorage.read(84), (byte)(delta_centerCam_Digital_H[dueFlashStorage.read(84)] + delta));
      delta_centerCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(39+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("     Center="); display.print(500 + (delta_centerCam_Digital_H[dueFlashStorage.read(84)]-128));
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Cam Digital H Up") && (Flag == true))
    {
      int TrimmingValue = (1000 - delta_maxCam_Digital_H[dueFlashStorage.read(84)]);
      if ((TrimmingValue + delta) <= (744)) delta++;
      if ((TrimmingValue + delta) > (1000)) delta--;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Cam Digital H Up") && (Flag == false))
    {
      dueFlashStorage.write(36+dueFlashStorage.read(84), (byte)(delta_maxCam_Digital_H[dueFlashStorage.read(84)] - delta));
      delta_maxCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(36+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("       Up="); display.print(1000 - delta_maxCam_Digital_H[dueFlashStorage.read(84)]);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Cam Digital H DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(33+dueFlashStorage.read(84), (byte)(0));
      dueFlashStorage.write(39+dueFlashStorage.read(84), (byte)(128));
      dueFlashStorage.write(36+dueFlashStorage.read(84), (byte)(0));    
      delta_minCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(33+dueFlashStorage.read(84));
      delta_centerCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(39+dueFlashStorage.read(84));
      delta_maxCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(36+dueFlashStorage.read(84));  
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Cam Digital H DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Trimmer Cam Analog V Center") && (Flag == true))
    {
      int TrimmingValue = (500 + (deltaCam_Analog_V[dueFlashStorage.read(84)] - 128));
      if ((TrimmingValue + delta) >= (500 + 128)) delta--;
      if ((TrimmingValue + delta) <= (500 - 128)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Cam Analog V Center") && (Flag == false))
    {
      dueFlashStorage.write(45+dueFlashStorage.read(84), (byte)(deltaCam_Analog_V[dueFlashStorage.read(84)] + delta));
      deltaCam_Analog_V[dueFlashStorage.read(84)] = dueFlashStorage.read(45+dueFlashStorage.read(84));
      delta = 0;
      MessageBox_TrimmingSaved_Show(500 + (deltaCam_Analog_V[dueFlashStorage.read(84)] - 128)); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Cam Analog V DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(45+dueFlashStorage.read(84), (byte)(128));
      deltaCam_Analog_V[dueFlashStorage.read(84)] = dueFlashStorage.read(45+dueFlashStorage.read(84));
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Cam Analog V DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    } 
    if ((MenuName == "Trimmer Cam Analog H Center") && (Flag == true))
    {
      int TrimmingValue = (500 + (deltaCam_Analog_H[dueFlashStorage.read(84)] - 128));
      if ((TrimmingValue + delta) >= (500 + 128)) delta--;
      if ((TrimmingValue + delta) <= (500 - 128)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Cam Analog H Center") && (Flag == false))
    {
      dueFlashStorage.write(51+dueFlashStorage.read(84), (byte)(deltaCam_Analog_H[dueFlashStorage.read(84)] + delta));
      deltaCam_Analog_H[dueFlashStorage.read(84)] = dueFlashStorage.read(51+dueFlashStorage.read(84));
      delta = 0;
      MessageBox_TrimmingSaved_Show(500 + (deltaCam_Analog_H[dueFlashStorage.read(84)] - 128)); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Cam Analog H DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(51+dueFlashStorage.read(84), (byte)(128));
      deltaCam_Analog_H[dueFlashStorage.read(84)] = dueFlashStorage.read(51+dueFlashStorage.read(84));
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Cam Analog H DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }

    if ((MenuName == "Trimmer Aux1 Digital I") && (Flag == true))
    {
      int TrimmingValue = (Aux1_Digital_I[dueFlashStorage.read(84)]*4);
      if ((TrimmingValue + delta*4) > 1000) delta--;
      if ((TrimmingValue + delta*4) < 0) delta++;
      TrimmingTable_Show(TrimmingValue + delta*4);
    } else  if ((MenuName == "Trimmer Aux1 Digital I") && (Flag == false))
    {
      dueFlashStorage.write(54+dueFlashStorage.read(84), (byte)(Aux1_Digital_I[dueFlashStorage.read(84)] + delta));
      Aux1_Digital_I[dueFlashStorage.read(84)] = dueFlashStorage.read(54+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("        I="); display.print(Aux1_Digital_I[dueFlashStorage.read(84)]*4);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Aux1 Digital O") && (Flag == true))
    {
      int TrimmingValue = (Aux1_Digital_O[dueFlashStorage.read(84)]*4);
      if ((TrimmingValue + delta*4) > 1000) delta--;
      if ((TrimmingValue + delta*4) < 0) delta++;
      TrimmingTable_Show(TrimmingValue + delta*4);
    } else  if ((MenuName == "Trimmer Aux1 Digital O") && (Flag == false))
    {
      dueFlashStorage.write(57+dueFlashStorage.read(84), (byte)(Aux1_Digital_O[dueFlashStorage.read(84)] + delta));
      Aux1_Digital_O[dueFlashStorage.read(84)] = dueFlashStorage.read(57+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("        O="); display.print(Aux1_Digital_O[dueFlashStorage.read(84)]*4);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Aux1 Digital II") && (Flag == true))
    {
       int TrimmingValue = (Aux1_Digital_II[dueFlashStorage.read(84)]*4);
       if ((TrimmingValue + delta*4) > 1000) delta--;
      if ((TrimmingValue + delta*4) < 0) delta++;
      TrimmingTable_Show(TrimmingValue + delta*4);
    } else  if ((MenuName == "Trimmer Aux1 Digital II") && (Flag == false))
    {
       dueFlashStorage.write(60+dueFlashStorage.read(84), (byte)(Aux1_Digital_II[dueFlashStorage.read(84)] + delta));
      Aux1_Digital_II[dueFlashStorage.read(84)] = dueFlashStorage.read(60+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("       II="); display.print(Aux1_Digital_II[dueFlashStorage.read(84)]*4);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Aux1 Digital DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(54+dueFlashStorage.read(84), (byte)(0));
      dueFlashStorage.write(57+dueFlashStorage.read(84), (byte)(0));
      dueFlashStorage.write(60+dueFlashStorage.read(84), (byte)(0));    
      Aux1_Digital_I[dueFlashStorage.read(84)] = dueFlashStorage.read(54+dueFlashStorage.read(84));
      Aux1_Digital_O[dueFlashStorage.read(84)] = dueFlashStorage.read(57+dueFlashStorage.read(84));
      Aux1_Digital_II[dueFlashStorage.read(84)] = dueFlashStorage.read(60+dueFlashStorage.read(84));  
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Aux1 Digital DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Trimmer Aux2 Digital I") && (Flag == true))
    {
      int TrimmingValue = (Aux2_Digital_I[dueFlashStorage.read(84)]*4);
      if ((TrimmingValue + delta*4) > 1000) delta--;
      if ((TrimmingValue + delta*4) < 0) delta++;
      TrimmingTable_Show(TrimmingValue + delta*4);
    } else  if ((MenuName == "Trimmer Aux2 Digital I") && (Flag == false))
    {
      dueFlashStorage.write(63+dueFlashStorage.read(84), (byte)(Aux2_Digital_I[dueFlashStorage.read(84)] + delta));
      Aux2_Digital_I[dueFlashStorage.read(84)] = dueFlashStorage.read(63+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("        I="); display.print(Aux2_Digital_I[dueFlashStorage.read(84)]*4);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Aux2 Digital O") && (Flag == true))
    {
      int TrimmingValue = (Aux2_Digital_O[dueFlashStorage.read(84)]*4);
      if ((TrimmingValue + delta*4) > 1000) delta--;
      if ((TrimmingValue + delta*4) < 0) delta++;
      TrimmingTable_Show(TrimmingValue + delta*4);
    } else  if ((MenuName == "Trimmer Aux2 Digital O") && (Flag == false))
    {
      dueFlashStorage.write(66+dueFlashStorage.read(84), (byte)(Aux2_Digital_O[dueFlashStorage.read(84)] + delta));
      Aux2_Digital_O[dueFlashStorage.read(84)] = dueFlashStorage.read(66+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("        O="); display.print(Aux2_Digital_O[dueFlashStorage.read(84)]*4);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Aux2 Digital II") && (Flag == true))
    {
       int TrimmingValue = (Aux2_Digital_II[dueFlashStorage.read(84)]*4);
       if ((TrimmingValue + delta*4) > 1000) delta--;
      if ((TrimmingValue + delta*4) < 0) delta++;
      TrimmingTable_Show(TrimmingValue + delta*4);
    } else  if ((MenuName == "Trimmer Aux2 Digital II") && (Flag == false))
    {
       dueFlashStorage.write(69+dueFlashStorage.read(84), (byte)(Aux2_Digital_II[dueFlashStorage.read(84)] + delta));
      Aux2_Digital_II[dueFlashStorage.read(84)] = dueFlashStorage.read(69+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("       II="); display.print(Aux2_Digital_II[dueFlashStorage.read(84)]*4);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Aux2 Digital DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(63+dueFlashStorage.read(84), (byte)(0));
      dueFlashStorage.write(66+dueFlashStorage.read(84), (byte)(0));
      dueFlashStorage.write(69+dueFlashStorage.read(84), (byte)(0));    
      Aux2_Digital_I[dueFlashStorage.read(84)] = dueFlashStorage.read(63+dueFlashStorage.read(84));
      Aux2_Digital_O[dueFlashStorage.read(84)] = dueFlashStorage.read(66+dueFlashStorage.read(84));
      Aux2_Digital_II[dueFlashStorage.read(84)] = dueFlashStorage.read(69+dueFlashStorage.read(84));  
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Aux2 Digital DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Trimmer Aux1 Analog Center") && (Flag == true))
    {
      int TrimmingValue;
      if (AUX1ModeActive == (B00000010)) 
      {
        TrimmingValue = (500 + (deltaAux1Analog_[dueFlashStorage.read(84)] - 128));
        if ((TrimmingValue + delta) >= (500 + 128)) delta--;
        if ((TrimmingValue + delta) <= (500 - 128)) delta++;
        TrimmingTable_Show(TrimmingValue + delta);
      }else if (AUX1ModeActive == (B00000100)) 
      {
        TrimmingValue = (500 + (deltaAux1Analog[dueFlashStorage.read(84)] - 128));
        if ((TrimmingValue + delta) >= (500 + 128)) delta--;
        if ((TrimmingValue + delta) <= (500 - 128)) delta++;
        TrimmingTable_Show(TrimmingValue + delta);
      }
    } else  if ((MenuName == "Trimmer Aux1 Analog Center") && (Flag == false))
    {
      if (AUX1ModeActive == (B00000010))
      {
        dueFlashStorage.write(102+dueFlashStorage.read(84), (byte)(deltaAux1Analog_[dueFlashStorage.read(84)] + delta));
        deltaAux1Analog_[dueFlashStorage.read(84)] = dueFlashStorage.read(102+dueFlashStorage.read(84));
        delta = 0;
        MessageBox_TrimmingSaved_Show(500 + (deltaAux1Analog_[dueFlashStorage.read(84)] - 128)); delay(1500);
        MenuItem_is_Change = " ";
        menu.moveUp();
      }else if (AUX1ModeActive == (B00000100))
      {
        dueFlashStorage.write(75+dueFlashStorage.read(84), (byte)(deltaAux1Analog[dueFlashStorage.read(84)] + delta));
        deltaAux1Analog[dueFlashStorage.read(84)] = dueFlashStorage.read(75+dueFlashStorage.read(84));
        delta = 0;
        MessageBox_TrimmingSaved_Show(500 + (deltaAux1Analog[dueFlashStorage.read(84)] - 128)); delay(1500);
        MenuItem_is_Change = " ";
        menu.moveUp();
      }
    }
    if ((MenuName == "Trimmer Aux1 Analog DefaultSettings YES") && (Flag == true))
    {
      if (AUX1ModeActive == (B00000010)) 
      {
        dueFlashStorage.write(102+dueFlashStorage.read(84), (byte)(128));
        deltaAux1Analog_[dueFlashStorage.read(84)] = dueFlashStorage.read(102+dueFlashStorage.read(84));
      }else if (AUX1ModeActive == (B00000100))
      {
        dueFlashStorage.write(75+dueFlashStorage.read(84), (byte)(128));
        deltaAux1Analog[dueFlashStorage.read(84)] = dueFlashStorage.read(75+dueFlashStorage.read(84));
      }
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Aux1 Analog DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Trimmer Aux2 Analog Center") && (Flag == true))
    {
      int TrimmingValue = (500 + (deltaAux2Analog[dueFlashStorage.read(84)] - 128));
      if ((TrimmingValue + delta) >= (500 + 128)) delta--;
      if ((TrimmingValue + delta) <= (500 - 128)) delta++;
      TrimmingTable_Show(TrimmingValue + delta);
    } else  if ((MenuName == "Trimmer Aux2 Analog Center") && (Flag == false))
    {
      dueFlashStorage.write(81+dueFlashStorage.read(84), (byte)(deltaAux2Analog[dueFlashStorage.read(84)] + delta));
      deltaAux2Analog[dueFlashStorage.read(84)] = dueFlashStorage.read(81+dueFlashStorage.read(84));
      delta = 0;
      MessageBox_TrimmingSaved_Show(500 + (deltaAux2Analog[dueFlashStorage.read(84)] - 128)); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Aux2 Analog DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(81+dueFlashStorage.read(84), (byte)(128));
      deltaAux2Analog[dueFlashStorage.read(84)] = dueFlashStorage.read(81+dueFlashStorage.read(84));
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Aux2 Analog DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Trimmer Trigger Digital On") && (Flag == true))
    {
      int TrimmingValue = (Trigger_Digital_On[dueFlashStorage.read(84)]*4);
      if ((TrimmingValue + delta*4) > 1000) delta--;
      if ((TrimmingValue + delta*4) < 0) delta++;
      TrimmingTable_Show(TrimmingValue + delta*4);
    } else  if ((MenuName == "Trimmer Trigger Digital On") && (Flag == false))
    {
      dueFlashStorage.write(105+dueFlashStorage.read(84), (byte)(Trigger_Digital_On[dueFlashStorage.read(84)] + delta));
      Trigger_Digital_On[dueFlashStorage.read(84)] = dueFlashStorage.read(105+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("       On="); display.print(Trigger_Digital_On[dueFlashStorage.read(84)]*4);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Trigger Digital Off") && (Flag == true))
    {
      int TrimmingValue = (Trigger_Digital_Off[dueFlashStorage.read(84)]*4);
      if ((TrimmingValue + delta*4) > 1000) delta--;
      if ((TrimmingValue + delta*4) < 0) delta++;
      TrimmingTable_Show(TrimmingValue + delta*4);
    } else  if ((MenuName == "Trimmer Trigger Digital Off") && (Flag == false))
    {
      dueFlashStorage.write(108+dueFlashStorage.read(84), (byte)(Trigger_Digital_Off[dueFlashStorage.read(84)] + delta));
      Trigger_Digital_Off[dueFlashStorage.read(84)] = dueFlashStorage.read(108+dueFlashStorage.read(84));
      delta = 0;
      display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED"); display.println(); display.setTextSize(1);
      display.print  ("      Off="); display.print(Trigger_Digital_Off[dueFlashStorage.read(84)]*4);
      display.display(); delay(1500);
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "Trimmer Trigger Digital DefaultSettings YES") && (Flag == true))
    {
      dueFlashStorage.write(105+dueFlashStorage.read(84), (byte)(0));
      dueFlashStorage.write(108+dueFlashStorage.read(84), (byte)(0));
      Trigger_Digital_On[dueFlashStorage.read(84)] = dueFlashStorage.read(105+dueFlashStorage.read(84));
      Trigger_Digital_Off[dueFlashStorage.read(84)] = dueFlashStorage.read(108+dueFlashStorage.read(84));
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "Trimmer Trigger Digital DefaultSettings NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
        
    if ((MenuName == "Invert Aileron") && (Flag == true))
    {
      InvertChanelsPart1 = InvertChanelsPart1 ^ (B10000000);
      dueFlashStorage.write(85, (byte)InvertChanelsPart1);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Elevator") && (Flag == true))
    {
      InvertChanelsPart1 = InvertChanelsPart1 ^ (B01000000);
      dueFlashStorage.write(85, (byte)InvertChanelsPart1);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Rudder") && (Flag == true))
    {
      InvertChanelsPart1 = InvertChanelsPart1 ^ (B00100000);
      dueFlashStorage.write(85, (byte)InvertChanelsPart1);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Throttle") && (Flag == true))
    {
      InvertChanelsPart1 = InvertChanelsPart1 ^ (B00010000);
      dueFlashStorage.write(85, (byte)InvertChanelsPart1);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Cam Digital V") && (Flag == true))
    {
      InvertChanelsPart1 = InvertChanelsPart1 ^ (B00001000);
      dueFlashStorage.write(85, (byte)InvertChanelsPart1);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Cam Digital H") && (Flag == true))
    {
      InvertChanelsPart1 = InvertChanelsPart1 ^ (B00000100);
      dueFlashStorage.write(85, (byte)InvertChanelsPart1);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Cam Analog V") && (Flag == true))
    {
      InvertChanelsPart1 = InvertChanelsPart1 ^ (B00000010);
      dueFlashStorage.write(85, (byte)InvertChanelsPart1);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Cam Analog V") && (Flag == true))
    {
      InvertChanelsPart1 = InvertChanelsPart1 ^ (B00000001);
      dueFlashStorage.write(85, (byte)InvertChanelsPart1);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Aux1") && (Flag == true))
    {
      InvertChanelsPart2 = InvertChanelsPart2 ^ (B10000000);
      dueFlashStorage.write(86, (byte)InvertChanelsPart2);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Invert Aux2") && (Flag == true))
    {
      InvertChanelsPart2 = InvertChanelsPart2 ^ (B01000000);
      dueFlashStorage.write(86, (byte)InvertChanelsPart2);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Exponential Aileron") && (Flag == true))
    {
      ExponentialChanels = ExponentialChanels ^ (B10000000);
      dueFlashStorage.write(87, (byte)ExponentialChanels);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Exponential Elevator") && (Flag == true))
    {
      ExponentialChanels = ExponentialChanels ^ (B01000000);
      dueFlashStorage.write(87, (byte)ExponentialChanels);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Exponential Rudder") && (Flag == true))
    {
      ExponentialChanels = ExponentialChanels ^ (B00100000);
      dueFlashStorage.write(87, (byte)ExponentialChanels);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Exponential Throttle") && (Flag == true))
    {
      ExponentialChanels = ExponentialChanels ^ (B00010000);
      dueFlashStorage.write(87, (byte)ExponentialChanels);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Exponential Aux1") && (Flag == true))
    {
      ExponentialChanels = ExponentialChanels ^ (B00001000);
      dueFlashStorage.write(87, (byte)ExponentialChanels);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Exponential Aux2") && (Flag == true))
    {
      ExponentialChanels = ExponentialChanels ^ (B00000100);
      dueFlashStorage.write(87, (byte)ExponentialChanels);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Plane Classic") && (Flag == true))
    {
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED");
      display.println();
      display.setTextSize(1);
      display.println("   Selected Classic"); 
      display.display();
      dueFlashStorage.write(96+dueFlashStorage.read(84), (byte)(0));
      PlaneModeActive[dueFlashStorage.read(84)] = dueFlashStorage.read(96+dueFlashStorage.read(84));
      delay(1500);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Plane Flying Wing") && (Flag == true))
    {
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED");
      display.println();
      display.setTextSize(1);
      display.println(" Selected Flying Wing"); 
      display.display();
      dueFlashStorage.write(96+dueFlashStorage.read(84), (byte)(1));
      PlaneModeActive[dueFlashStorage.read(84)] = dueFlashStorage.read(96+dueFlashStorage.read(84));
      delay(1500);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Plane V-tail") && (Flag == true))
    {
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.println("   SAVED");
      display.println();
      display.setTextSize(1);
      display.println(" Selected V-tail"); 
      display.display();
      dueFlashStorage.write(96+dueFlashStorage.read(84), (byte)(2));
      PlaneModeActive[dueFlashStorage.read(84)] = dueFlashStorage.read(96+dueFlashStorage.read(84));
      delay(1500);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "DefaultSettings for this Plane YES") && (Flag == true))
    {
      
      dueFlashStorage.write(3+dueFlashStorage.read(84), (byte)(128));
      deltaAileron[dueFlashStorage.read(84)] = dueFlashStorage.read(3+dueFlashStorage.read(84));
      
      dueFlashStorage.write(9+dueFlashStorage.read(84), (byte)(128));
      deltaElevator[dueFlashStorage.read(84)] = dueFlashStorage.read(9+dueFlashStorage.read(84));
      
      dueFlashStorage.write(15+dueFlashStorage.read(84), (byte)(128));
      deltaRudder[dueFlashStorage.read(84)] = dueFlashStorage.read(15+dueFlashStorage.read(84));

      dueFlashStorage.write(19+dueFlashStorage.read(84), (byte)(0));
      deltaMinThrottle[dueFlashStorage.read(84)] = dueFlashStorage.read(19+dueFlashStorage.read(84));

      dueFlashStorage.write(23+dueFlashStorage.read(84), (byte)(0));
      deltaMaxThrottle[dueFlashStorage.read(84)] = dueFlashStorage.read(23+dueFlashStorage.read(84));
      
      dueFlashStorage.write(26+dueFlashStorage.read(84), (byte)(0));
      delta_minCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(26+dueFlashStorage.read(84));

      dueFlashStorage.write(93+dueFlashStorage.read(84), (byte)(128));
      delta_centerCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(93+dueFlashStorage.read(84));

      dueFlashStorage.write(30+dueFlashStorage.read(84), (byte)(0));
      delta_maxCam_Digital_V[dueFlashStorage.read(84)] = dueFlashStorage.read(30+dueFlashStorage.read(84));

      dueFlashStorage.write(33+dueFlashStorage.read(84), (byte)(0));
      delta_minCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(33+dueFlashStorage.read(84));

      dueFlashStorage.write(36+dueFlashStorage.read(84), (byte)(128));
      delta_centerCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(36+dueFlashStorage.read(84));

      dueFlashStorage.write(39+dueFlashStorage.read(84), (byte)(0));
      delta_maxCam_Digital_H[dueFlashStorage.read(84)] = dueFlashStorage.read(39+dueFlashStorage.read(84));

      dueFlashStorage.write(45+dueFlashStorage.read(84), (byte)(128));
      deltaCam_Analog_V[dueFlashStorage.read(84)] = dueFlashStorage.read(45+dueFlashStorage.read(84));
      
      dueFlashStorage.write(51+dueFlashStorage.read(84), (byte)(128));
      deltaCam_Analog_H[dueFlashStorage.read(84)] = dueFlashStorage.read(51+dueFlashStorage.read(84));

      dueFlashStorage.write(54+dueFlashStorage.read(84), (byte)(0));
      Aux1_Digital_I[dueFlashStorage.read(84)] = dueFlashStorage.read(54+dueFlashStorage.read(84));

      dueFlashStorage.write(57+dueFlashStorage.read(84), (byte)(0));
      Aux1_Digital_O[dueFlashStorage.read(84)] = dueFlashStorage.read(57+dueFlashStorage.read(84));

      dueFlashStorage.write(60+dueFlashStorage.read(84), (byte)(0));
      Aux1_Digital_II[dueFlashStorage.read(84)] = dueFlashStorage.read(60+dueFlashStorage.read(84));

      dueFlashStorage.write(63+dueFlashStorage.read(84), (byte)(0));
      Aux2_Digital_I[dueFlashStorage.read(84)] = dueFlashStorage.read(63+dueFlashStorage.read(84));

      dueFlashStorage.write(66+dueFlashStorage.read(84), (byte)(0));
      Aux2_Digital_O[dueFlashStorage.read(84)] = dueFlashStorage.read(66+dueFlashStorage.read(84));

      dueFlashStorage.write(69+dueFlashStorage.read(84), (byte)(0));
      Aux2_Digital_II[dueFlashStorage.read(84)] = dueFlashStorage.read(69+dueFlashStorage.read(84));

      dueFlashStorage.write(75+dueFlashStorage.read(84), (byte)(128));
      deltaAux1Analog[dueFlashStorage.read(84)] = dueFlashStorage.read(75+dueFlashStorage.read(84));

      dueFlashStorage.write(81+dueFlashStorage.read(84), (byte)(128));
      deltaAux2Analog[dueFlashStorage.read(84)] = dueFlashStorage.read(81+dueFlashStorage.read(84));

      dueFlashStorage.write(85, (byte)(0));
      InvertChanelsPart1 = dueFlashStorage.read(85);

      dueFlashStorage.write(86, (byte)(0));
      InvertChanelsPart2 = dueFlashStorage.read(86);

      dueFlashStorage.write(87, (byte)(0));
      ExponentialChanels = dueFlashStorage.read(87);

      dueFlashStorage.write(89, (byte)(1));
      InvertPPMActive = dueFlashStorage.read(89);

      dueFlashStorage.write(90, (byte)(0));
      CamModeActive = dueFlashStorage.read(90);

      dueFlashStorage.write(91, (byte)(4));
      AUX1ModeActive = dueFlashStorage.read(91);

      dueFlashStorage.write(92, (byte)(1));
      AUX2ModeActive = dueFlashStorage.read(92);

      dueFlashStorage.write(96+dueFlashStorage.read(84), (byte)(0));
      PlaneModeActive[dueFlashStorage.read(84)] = dueFlashStorage.read(96+dueFlashStorage.read(84));

      dueFlashStorage.write(102+dueFlashStorage.read(84), (byte)(128));
      deltaAux1Analog_[dueFlashStorage.read(84)] = dueFlashStorage.read(102+dueFlashStorage.read(84));

      dueFlashStorage.write(111, (byte)(0));
      CamDigitalDelay = dueFlashStorage.read(111);

      dueFlashStorage.write(112, (byte)(0));
      CamAnalogDelay = dueFlashStorage.read(112);
      
      delta = 0;
      MenuItem_is_Change = " ";
      menu.moveUp();
    }  
    if ((MenuName == "DefaultSettings for this Plane NO") && (Flag == true))
    {
      MenuItem_is_Change = " ";
      menu.moveUp();
    }
    if ((MenuName == "Invert PPM") && (Flag == true))
    {
      InvertPPMActive = InvertPPMActive ^ (B00000001);
      dueFlashStorage.write(89, (byte)InvertPPMActive);
      
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    } 
    if ((MenuName == "Cam Mode") && (Flag == true))
    {
      CamModeActive = CamModeActive ^ (B00000001);
      dueFlashStorage.write(90, (byte)CamModeActive);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "AUX1 Mode") && (Flag == true))
    {
      if (AUX1ModeActive == B00001000) AUX1ModeActive = B00000100;
      else if (AUX1ModeActive == B00000100) AUX1ModeActive = B00000010;
      else if (AUX1ModeActive == B00000010) AUX1ModeActive = B00000001;
      else AUX1ModeActive = B00001000;
      dueFlashStorage.write(91, (byte)AUX1ModeActive);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "AUX2 Mode") && (Flag == true))
    {
      if (AUX2ModeActive == B00000100) AUX2ModeActive = B00000010;
      else if (AUX2ModeActive == B00000010) AUX2ModeActive = B00000001;
      else AUX2ModeActive = B00000100;
      dueFlashStorage.write(92, (byte)AUX2ModeActive);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "CamDigitalDelay Mode") && (Flag == true))
    {
      if (CamDigitalDelay == 10) CamDigitalDelay = 0;
      else CamDigitalDelay++;
      dueFlashStorage.write(111, (byte)CamDigitalDelay);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
    if ((MenuName == "CamAnalogDelay Mode") && (Flag == true))
    {
      if (CamAnalogDelay == 10) CamAnalogDelay = 0;
      else CamAnalogDelay++;
      dueFlashStorage.write(112, (byte)CamAnalogDelay);
      ActiveCange = false;
      MenuItem_is_Change = " ";
      menu.moveRight();
    }
  }
}
//-------------------------------------------------------------------------------------------------------

void menuUsed(MenuUseEvent used)
{
  if (MenuItem_is_Change != used.item.getName()) ActiveCange = false;
/*---First Plane*/if ((used.item.getName() == "First Plane") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "First Plane") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Second Plane*/if ((used.item.getName() == "Second Plane") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Second Plane") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Third Plane*/if ((used.item.getName() == "Third Plane") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Third Plane") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Auto Aileron*/if ((used.item.getName() == "Auto Aileron") && (ActiveCange == false))
  {
    ActiveCange = true;
    
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Auto Aileron") && (ActiveCange == true))
  {
    Due.setTimer(5000);
    ActiveCange = false;
  }
/*---Auto Elevator*/if ((used.item.getName() == "Auto Elevator") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Auto Elevator") && (ActiveCange == true))
  {
    Due.setTimer(5000);
    ActiveCange = false;
  }
/*---Auto Rudder*/if ((used.item.getName() == "Auto Rudder") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Auto Rudder") && (ActiveCange == true))
  {
    Due.setTimer(5000);
    ActiveCange = false;
  }
/*---Auto Throttle*/if ((used.item.getName() == "Auto Throttle") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Auto Throttle") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Auto Cam Analog V*/if ((used.item.getName() == "Auto Cam Analog V") && (ActiveCange == false)) 
  {
    ActiveCange = true;
    
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Auto Cam Analog V") && (ActiveCange == true))
  {
    Due.setTimer(5000);
    ActiveCange = false;
  }
/*---Auto Cam Analog H*/if ((used.item.getName() == "Auto Cam Analog H") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Auto Cam Analog H") && (ActiveCange == true))
  {
    Due.setTimer(5000);
    ActiveCange = false;
  }
/*---Auto Aux1*/if ((used.item.getName() == "Auto Aux1") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Auto Aux1") && (ActiveCange == true))
  {
    Due.setTimer(5000);
    ActiveCange = false;
  }
/*---Auto Aux2*/if ((used.item.getName() == "Auto Aux2") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Auto Aux2") && (ActiveCange == true))
  {
    Due.setTimer(5000);
    ActiveCange = false;
  }
/*---Trimmer Aileron Center*/if ((used.item.getName() == "Trimmer Aileron Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aileron Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aileron DefaultSettings YES*/if ((used.item.getName() == "Trimmer Aileron DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aileron DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aileron DefaultSettings NO*/if ((used.item.getName() == "Trimmer Aileron DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aileron DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Elevator Center*/if ((used.item.getName() == "Trimmer Elevator Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Elevator Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Elevator DefaultSettings YES*/if ((used.item.getName() == "Trimmer Elevator DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Elevator DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Elevator DefaultSettings NO*/if ((used.item.getName() == "Trimmer Elevator DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Elevator DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Rudder Center*/if ((used.item.getName() == "Trimmer Rudder Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Rudder Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Rudder DefaultSettings YES*/if ((used.item.getName() == "Trimmer Rudder DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Rudder DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Rudder DefaultSettings NO*/if ((used.item.getName() == "Trimmer Rudder DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Rudder DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Throttle Min*/if ((used.item.getName() == "Trimmer Throttle Min") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Throttle Min") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Throttle Max*/if ((used.item.getName() == "Trimmer Throttle Max") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Throttle Max") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Throttle DefaultSettings YES*/if ((used.item.getName() == "Trimmer Throttle DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Throttle DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Throttle DefaultSettings NO*/if ((used.item.getName() == "Trimmer Throttle DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Throttle DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital V Down*/if ((used.item.getName() == "Trimmer Cam Digital V Down") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital V Down") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital V Center*/if ((used.item.getName() == "Trimmer Cam Digital V Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital V Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital V Up*/if ((used.item.getName() == "Trimmer Cam Digital V Up") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital V Up") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital V DefaultSettings YES*/if ((used.item.getName() == "Trimmer Cam Digital V DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital V DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital V DefaultSettings NO*/if ((used.item.getName() == "Trimmer Cam Digital V DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital V DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital H Down*/if ((used.item.getName() == "Trimmer Cam Digital H Down") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital H Down") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital H Center*/if ((used.item.getName() == "Trimmer Cam Digital H Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital H Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital H Up*/if ((used.item.getName() == "Trimmer Cam Digital H Up") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital H Up") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital H DefaultSettings YES*/if ((used.item.getName() == "Trimmer Cam Digital H DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital H DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Digital H DefaultSettings NO*/if ((used.item.getName() == "Trimmer Cam Digital H DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Digital H DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Analog V Center*/if ((used.item.getName() == "Trimmer Cam Analog V Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Analog V Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Analog V DefaultSettings YES*/if ((used.item.getName() == "Trimmer Cam Analog V DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Analog V DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Analog V DefaultSettings NO*/if ((used.item.getName() == "Trimmer Cam Analog V DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Analog V DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Analog H Center*/if ((used.item.getName() == "Trimmer Cam Analog H Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Analog H Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Analog H DefaultSettings YES*/if ((used.item.getName() == "Trimmer Cam Analog H DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Analog H DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Cam Analog H DefaultSettings NO*/if ((used.item.getName() == "Trimmer Cam Analog H DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Cam Analog H DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux1 Digital I*/if ((used.item.getName() == "Trimmer Aux1 Digital I") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux1 Digital I") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux1 Digital O*/if ((used.item.getName() == "Trimmer Aux1 Digital O") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux1 Digital O") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux1 Digital II*/if ((used.item.getName() == "Trimmer Aux1 Digital II") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux1 Digital II") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux1 Digital DefaultSettings YES*/if ((used.item.getName() == "Trimmer Aux1 Digital DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux1 Digital DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux1 Digital DefaultSettings NO*/if ((used.item.getName() == "Trimmer Aux1 Digital DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux1 Digital DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux2 Digital I*/if ((used.item.getName() == "Trimmer Aux2 Digital I") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux2 Digital I") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux2 Digital O*/if ((used.item.getName() == "Trimmer Aux2 Digital O") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux2 Digital O") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux2 Digital II*/if ((used.item.getName() == "Trimmer Aux2 Digital II") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux2 Digital II") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux2 Digital DefaultSettings YES*/if ((used.item.getName() == "Trimmer Aux2 Digital DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux2 Digital DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux2 Digital DefaultSettings NO*/if ((used.item.getName() == "Trimmer Aux2 Digital DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux2 Digital DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux1 Analog Center*/if ((used.item.getName() == "Trimmer Aux1 Analog Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux1 Analog Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux1 Analog DefaultSettings YES*/if ((used.item.getName() == "Trimmer Aux1 Analog DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux1 Analog DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux1 Analog DefaultSettings NO*/if ((used.item.getName() == "Trimmer Aux1 Analog DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux1 Analog DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux2 Analog Center*/if ((used.item.getName() == "Trimmer Aux2 Analog Center") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux2 Analog Center") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux2 Analog DefaultSettings YES*/if ((used.item.getName() == "Trimmer Aux2 Analog DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux2 Analog DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Trimmer Aux2 Analog DefaultSettings NO*/if ((used.item.getName() == "Trimmer Aux2 Analog DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Aux2 Analog DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Aileron*/if ((used.item.getName() == "Invert Aileron") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Aileron") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Elevator*/if ((used.item.getName() == "Invert Elevator") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Elevator") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Rudder*/if ((used.item.getName() == "Invert Rudder") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Rudder") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Throttle*/if ((used.item.getName() == "Invert Throttle") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Throttle") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Cam Digital V*/if ((used.item.getName() == "Invert Cam Digital V") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Cam Digital V") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Cam Digital H*/if ((used.item.getName() == "Invert Cam Digital H") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Cam Digital H") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Cam Analog V*/if ((used.item.getName() == "Invert Cam Analog V") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Cam Analog V") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Cam Analog H*/if ((used.item.getName() == "Invert Cam Analog H") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Cam Analog H") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Aux1 Analog*/if ((used.item.getName() == "Invert Aux1") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Aux1") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert Aux2 Analog*/if ((used.item.getName() == "Invert Aux2") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert Aux2") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Exponential Aileron*/if ((used.item.getName() == "Exponential Aileron") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Exponential Aileron") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Exponential Elevator*/if ((used.item.getName() == "Exponential Elevator") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Exponential Elevator") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Exponential Rudder*/if ((used.item.getName() == "Exponential Rudder") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Exponential Rudder") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Exponential Throttle*/if ((used.item.getName() == "Exponential Throttle") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Exponential Throttle") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Exponential Aux1*/if ((used.item.getName() == "Exponential Aux1") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Exponential Aux1") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Exponential Aux2*/if ((used.item.getName() == "Exponential Aux2") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Exponential Aux2") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Plane Classic*/if ((used.item.getName() == "Plane Classic") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Plane Classic") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Plane Flying Wing*/if ((used.item.getName() == "Plane Flying Wing") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Plane Flying Wing") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Plane V-tail*/if ((used.item.getName() == "Plane V-tail") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Plane V-tail") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---DefaultSettings for this Plane YES*/if ((used.item.getName() == "DefaultSettings for this Plane YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "DefaultSettings for this Plane YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---DefaultSettings for this Plane NO*/if ((used.item.getName() == "DefaultSettings for this Plane NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "DefaultSettings for this Plane NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Invert PPM*/if ((used.item.getName() == "Invert PPM") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Invert PPM") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---Cam Mode*/if ((used.item.getName() == "Cam Mode") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Cam Mode") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---AUX1 Mode*/if ((used.item.getName() == "AUX1 Mode") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "AUX1 Mode") && (ActiveCange == true))
  {
    ActiveCange = false;
  }
/*---AUX2 Mode*/if ((used.item.getName() == "AUX2 Mode") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "AUX2 Mode") && (ActiveCange == true))
  {
    ActiveCange = false;
  }     
/*---CamDigitalDelay Mode*/if ((used.item.getName() == "CamDigitalDelay Mode") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "CamDigitalDelay Mode") && (ActiveCange == true))
  {
    ActiveCange = false;
  }   
/*---CamAnalogDelay Mode*/if ((used.item.getName() == "CamAnalogDelay Mode") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "CamAnalogDelay Mode") && (ActiveCange == true))
  {
    ActiveCange = false;
  } 
/*---Trimmer Trigger Digital On*/if ((used.item.getName() == "Trimmer Trigger Digital On") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Trigger Digital On") && (ActiveCange == true))
  {
    ActiveCange = false;
  } 
/*---Trimmer Trigger Digital Off*/if ((used.item.getName() == "Trimmer Trigger Digital Off") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Trigger Digital Off") && (ActiveCange == true))
  {
    ActiveCange = false;
  } 
/*---CamAnalogDelay Mode*/if ((used.item.getName() == "Trimmer Trigger Digital DefaultSettings") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Trigger Digital DefaultSettings") && (ActiveCange == true))
  {
    ActiveCange = false;
  } 
/*---Trimmer Trigger Digital DefaultSettings YES*/if ((used.item.getName() == "Trimmer Trigger Digital DefaultSettings YES") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Trigger Digital DefaultSettings YES") && (ActiveCange == true))
  {
    ActiveCange = false;
  } 
/*---Trimmer Trigger Digital DefaultSettings NO*/if ((used.item.getName() == "Trimmer Trigger Digital DefaultSettings NO") && (ActiveCange == false))
  {
    ActiveCange = true;
    MenuItem_is_Change = used.item.getName();
  }else if ((used.item.getName() == "Trimmer Trigger Digital DefaultSettings NO") && (ActiveCange == true))
  {
    ActiveCange = false;
  } 

MenuItem Trimmer_Trigger_Digital/*......................................*/=MenuItem("Trimmer Trigger Digital");
            MenuItem Trimmer_Trigger_Digital_On/*...................................*/=MenuItem("Trimmer Trigger Digital On");
            MenuItem Trimmer_Trigger_Digital_Off/*..................................*/=MenuItem("Trimmer Trigger Digital Off");
            MenuItem Trimmer_Trigger_Digital_DefaultSettings/*......................*/=MenuItem("Trimmer Trigger Digital DefaultSettings");     
                MenuItem Trimmer_Trigger_Digital_DefaultSettings_YES/*......................*/=MenuItem("Trimmer Trigger Digital DefaultSettings YES");
                MenuItem Trimmer_Trigger_Digital_DefaultSettings_NO/*.......................*/=MenuItem("Trimmer Trigger Digital DefaultSettings NO");                                                               
}
//-------------------------------------------------------------------------------------------------------
  
void Test_Show(String NameItem)
{
  int procent;
  display.clearDisplay();
  int a = 1;//   начало первой строчки
  int b = 13;// расстояние между строчками
  int k = -1; // порядковый номер строчки
  if (NameItem == "Test Aileron") a = 1; else if (NameItem == "Test Elevator") a = 1; else if (NameItem == "Test Rudder") a = 1;else if (NameItem == "Test Throttle") a = -12; else if (NameItem == "Test Cam Analog V") a = -25; else if (NameItem == "Test Cam Analog H") a = -38; else a = -38;
  if ((NameItem == "Test Aileron") || (NameItem == "Test Elevator") || (NameItem == "Test Rudder") || (NameItem == "Test Throttle") || (NameItem == "Test Cam Analog V") || (NameItem == "Test Cam Analog H") || (NameItem == "Test Aux1") || (NameItem == "Test Aux2"))
  {
    //Aileron-------------------------
    k++;
    display.setCursor(3,(a + b*k));
    if (NameItem == "Test Aileron")
    {
          display.setTextColor(BLACK);display.fillRect(0, (a + b*k)-1, 55, 9, WHITE); display.print("Aileron:");display.setTextColor(WHITE);
    }
    else {display.setTextColor(WHITE); display.print("Aileron:");}
    procent = map(ppm_channels_1000_2000[k],1000,2000,0,68);
    display.drawRect(60, (a + b*k)+1, 68, 5, WHITE);
    display.fillRect(60, (a + b*k)+1, procent, 5, WHITE);
    //Elevator-------------------------
    k++;
    display.setCursor(3,(a + b*k));
    if (NameItem == "Test Elevator")
    {
          display.setTextColor(BLACK);display.fillRect(0, (a + b*k)-1, 55, 9, WHITE); display.print("Elevator:");display.setTextColor(WHITE);
    }
    else {display.setTextColor(WHITE); display.print("Elevator:");}
    procent = map(ppm_channels_1000_2000[k],1000,2000,0,68);
    display.drawRect(60, (a + b*k)+1, 68, 5, WHITE);
    display.fillRect(60, (a + b*k)+1, procent, 5, WHITE);
    //Rudder-------------------------
    k++;
    display.setCursor(3,(a + b*k));
    if (NameItem == "Test Rudder")
    {
          display.setTextColor(BLACK);display.fillRect(0, (a + b*k)-1, 55, 9, WHITE); display.print("Rudder:");display.setTextColor(WHITE);
    }
    else {display.setTextColor(WHITE); display.print("Rudder:");}
    procent = map(ppm_channels_1000_2000[k],1000,2000,0,68);
    display.drawRect(60, (a + b*k)+1, 68, 5, WHITE);
    display.fillRect(60, (a + b*k)+1, procent, 5, WHITE);
    //Throttle-------------------------
    k++;
    display.setCursor(3,(a + b*k));
    if (NameItem == "Test Throttle")
    {     
     display.setTextColor(BLACK);display.fillRect(0, (a + b*k)-1, 55, 9, WHITE); display.print("Throttle:");display.setTextColor(WHITE);
    }
    else {display.setCursor(3,(a + b*k));display.setTextColor(WHITE); display.print("Throttle:");}
    procent = map(ppm_channels_1000_2000[k],1000,2000,0,68);
    display.drawRect(60, (a + b*k)+1, 68, 5, WHITE);
    display.fillRect(60, (a + b*k)+1, procent, 5, WHITE);
    //CamV-------------------------
    k++;
    display.setCursor(3,(a + b*k));
    if (NameItem == "Test Cam Analog V")
    { 
     display.setTextColor(BLACK);display.fillRect(0, (a + b*k)-1, 55, 9, WHITE); display.print("CamV:");display.setTextColor(WHITE);
    }
    else {display.setCursor(3,(a + b*k));display.setTextColor(WHITE); display.print("CamV:");}
    procent = map(ppm_channels_1000_2000[k],1000,2000,0,68);
    display.drawRect(60, (a + b*k)+1, 68, 5, WHITE);
    display.fillRect(60, (a + b*k)+1, procent, 5, WHITE);
    //CamH-------------------------
    k++;
    display.setCursor(3,(a + b*k));
    if (NameItem == "Test Cam Analog H")
    { 
     display.setTextColor(BLACK);display.fillRect(0, (a + b*k)-1, 55, 9, WHITE); display.print("CamH:");display.setTextColor(WHITE);
    }
    else {display.setCursor(3,(a + b*k));display.setTextColor(WHITE); display.print("CamH:");}
    procent = map(ppm_channels_1000_2000[k],1000,2000,0,68);
    display.drawRect(60, (a + b*k)+1, 68, 5, WHITE);
    display.fillRect(60, (a + b*k)+1, procent, 5, WHITE);
    //Aux1-------------------------
    k++;
    display.setCursor(3,(a + b*k));
    if (NameItem == "Test Aux1")
    { 
     display.setTextColor(BLACK);display.fillRect(0, (a + b*k)-1, 55, 9, WHITE); display.print("Aux1:");display.setTextColor(WHITE);
    }
    else {display.setCursor(3,(a + b*k));display.setTextColor(WHITE); display.print("Aux1:");}
    procent = map(ppm_channels_1000_2000[k],1000,2000,0,68);
    display.drawRect(60, (a + b*k)+1, 68, 5, WHITE);
    display.fillRect(60, (a + b*k)+1, procent, 5, WHITE);
    //Aux2-------------------------
    k++;
    display.setCursor(3,(a + b*k));
    if (NameItem == "Test Aux2")
    { 
     display.setTextColor(BLACK);display.fillRect(0, (a + b*k)-1, 55, 9, WHITE); display.print("Aux2:");display.setTextColor(WHITE);
    }
    else {display.setCursor(3,(a + b*k));display.setTextColor(WHITE); display.print("Aux2:");}
    procent = map(ppm_channels_1000_2000[k],1000,2000,0,68);
    display.drawRect(60, (a + b*k)+1, 68, 5, WHITE);
    display.fillRect(60, (a + b*k)+1, procent, 5, WHITE);
    
    display.display();
  }
  else isTest_Show = false;
}
void MainScreenPrint()
{
  //Rates-------------------------
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(3,0);
  display.print("Rates:"); 
  display.setCursor(90,0);
  switch (power)
  {
    case 0:
    {
      display.println("HIGH");
      break;    
    }
    case 125:
    {
      display.println("MEDIUM");
      break;    
    }
    case 250:
    {
      display.println("LOW");
      break;    
    }
  }
  //Battery-------------------------
  //  (value / 1023.0) * 3.3; // верно только если Vcc = 3.3 вольт
  //   vin = (analogRead(A7) * 3.3 / 2047.0)/(R2/(R1+R2)); R1=5.7kOm R2=1.083kOm

  display.setCursor(3,11);
  display.print("Battery:");
  display.setCursor(90,11); 
  float vin = (analogRead(A0)*0.025)+0;
  display.print(vin, 1);display.print("V");
  //Throttle-------------------------
  display.setCursor(3,34);
  display.print("Throttle:");
  //display.setCursor(94,33);
  int procent = map(ppm_channels_1000_2000[3],1000,2000,0,68);
  display.drawRect(60, 35, 68, 5, WHITE);
  display.fillRect(60, 35, procent, 5, WHITE);
  //Aux1-------------------------
  display.setCursor(3,45);
  display.print("Aux1:");
  procent = map(ppm_channels_1000_2000[6],1000,2000,0,68);
  display.drawRect(60, 46, 68, 5, WHITE);
  display.fillRect(60, 46, procent, 5, WHITE);
  //Aux2-------------------------
  display.setCursor(3,56);
  display.print("Aux2:");
  procent = map(ppm_channels_1000_2000[7],1000,2000,0,68);
  display.drawRect(60, 57, 68, 5, WHITE);
  display.fillRect(60, 57, procent, 5, WHITE);
  /*//Aux1-------------------------
  display.setCursor(3,56);
  display.print("Aux1:");
  display.setCursor(34,56);
  display.print(map(ppm_channels_1000_2000[6],1000,1996,0,100));display.print("%");
  //Aux2-------------------------
  display.setCursor(72,56);
  display.print("Aux2:");
  display.setCursor(102,56);
  display.print(map(ppm_channels_1000_2000[7],1000,1996,0,100));display.print("%");*/
  
  

  display.display();
}

//-------------------------------------------------------------------------------------------------------
void menuChanged(MenuChangeEvent changed)
{
  MenuItem newMenuItem=changed.to; //get the destination menu
  MenuItem_is_Open = newMenuItem.getName();
  display.clearDisplay();
  display.setCursor(0,0); //set the start position for lcd printing to the second row
  if(newMenuItem == menu.getRoot())
  {
    isMainScreen_Show = true;
  }
  else
  {
    isMainScreen_Show = false;
  if (MenuItem_is_Open == "Plane")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.println(" Plane");
    display.setTextColor(WHITE);
    display.println(" Trimmer mode");
    display.println(" Invert mode"); 
    display.println(" Exponential mode");
    display.println(" Plane mode");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Mode")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Plane");
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.println(" Trimmer mode");
    display.setTextColor(WHITE);
    display.println(" Invert mode"); 
    display.println(" Exponential mode");
    display.println(" Plane mode");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Mode")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Plane");
    display.println(" Trimmer mode");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Invert mode");
    display.setTextColor(WHITE); 
    display.println(" Exponential mode");
    display.println(" Plane mode");
    display.display();
  }
  if (MenuItem_is_Open == "Exponential Mode")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Trimmer mode");
    display.println(" Invert mode");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Exponential mode");
    display.setTextColor(WHITE); 
    display.println(" Plane mode ");
    display.println(" Auto Trimming");
    display.display();
  }  
  if (MenuItem_is_Open == "Plane Mode")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Invert mode");
    display.println(" Exponential mode");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Plane mode");
    display.setTextColor(WHITE); 
    display.println(" Auto Trimming");
    display.println(" Default settings");
    display.display();
  }
  if (MenuItem_is_Open == "Joystick Auto Trimming")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Exponential mode");
    display.println(" Plane mode");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Auto Trimming");
    display.setTextColor(WHITE); 
    display.println(" Default settings");
    display.println(" Other settings");
    display.display();
  }
  if (MenuItem_is_Open == "DefaultSettings for this Plane")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Plane mode");
    display.println(" Auto Trimming");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Default settings");
    display.setTextColor(WHITE); 
    display.println(" Other settings");
    display.println(" Test channels");
    display.display();
  }
  if (MenuItem_is_Open == "Other Settings")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Plane mode");
    display.println(" Auto Trimming");    
    display.println(" Default settings");
    display.fillRect(0, 43, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Other settings");
    display.setTextColor(WHITE);
    display.println(" Test channels");
    display.display();
  }
  if (MenuItem_is_Open == "Test Channel")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Main Menu");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Plane mode");
    display.println(" Auto Trimming");    
    display.println(" Default settings");
    display.println(" Other settings");
    display.fillRect(0, 51, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Test channels");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "First Plane")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Plane");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    if (dueFlashStorage.read(84) == 0)
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   First Plane");
    }
    else display.println("   First Plane");
    display.setTextColor(WHITE);
    if (dueFlashStorage.read(84) == 1)
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Second Plane");
    }
    else display.println("   Second Plane");
    if (dueFlashStorage.read(84) == 2)
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Third Plane"); 
    }
    else display.println("   Third Plane");
    display.display();
  }
  if (MenuItem_is_Open == "Second Plane")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Plane");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 27, display.width(), 9, WHITE); 
    if (dueFlashStorage.read(84) == 0)
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   First Plane");
    }
    else display.println("   First Plane");
    display.setTextColor(BLACK);
    if (dueFlashStorage.read(84) == 1)
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Second Plane");
    }
    else display.println("   Second Plane");
    display.setTextColor(WHITE);
    if (dueFlashStorage.read(84) == 2)
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Third Plane"); 
    }
    else display.println("   Third Plane");
    display.display();
  }
  if (MenuItem_is_Open == "Third Plane")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Plane");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE); 
    if (dueFlashStorage.read(84) == 0)
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   First Plane");
    }
    else display.println("   First Plane");
    if (dueFlashStorage.read(84) == 1)
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Second Plane");
    }
    else display.println("   Second Plane");
    display.setTextColor(BLACK);
    if (dueFlashStorage.read(84) == 2)
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Third Plane"); 
    }
    else display.println("   Third Plane");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aileron")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.println(" Aileron");
    display.setTextColor(WHITE);
    display.println(" Elevator");
    display.println(" Rudder"); 
    display.println(" Throttle");
    display.println(" Cam Digital V");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Elevator")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Aileron");
    display.fillRect(0, 27, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Elevator");
    display.setTextColor(WHITE);
    display.println(" Rudder"); 
    display.println(" Throttle");
    display.println(" Cam Digital V");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Rudder")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Aileron");
    display.println(" Elevator");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Rudder"); 
    display.setTextColor(WHITE);
    display.println(" Throttle");
    display.println(" Cam Digital V");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Throttle")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Elevator");
    display.println(" Rudder");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Throttle"); 
    display.setTextColor(WHITE);
    display.println(" Cam Digital V");
    display.println(" Cam Digital H");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital V")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Rudder");
    display.println(" Throttle");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Cam Digital V"); 
    display.setTextColor(WHITE);
    display.println(" Cam Digital H");
    display.println(" Cam Analog V");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital H")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Throttle");
    display.println(" Cam Digital V");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Cam Digital H"); 
    display.setTextColor(WHITE);
    display.println(" Cam Analog V");
    display.println(" Cam Analog H");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog V")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Cam Digital V");
    display.println(" Cam Digital H");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Cam Analog V"); 
    display.setTextColor(WHITE);
    display.println(" Cam Analog H");
    display.println(" Aux1 Digital");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog H")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Cam Digital H");
    display.println(" Cam Analog V");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Cam Analog H"); 
    display.setTextColor(WHITE);
    display.println(" Aux1 Digital");
    display.println(" Aux2 Digital");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Digital")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Cam Analog V");
    display.println(" Cam Analog H");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Aux1 Digital"); 
    display.setTextColor(WHITE);
    display.println(" Aux2 Digital");
    display.println(" Aux1 Analog");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Digital")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Cam Analog H");
    display.println(" Aux1 Digital");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Aux2 Digital"); 
    display.setTextColor(WHITE);
    display.println(" Aux1 Analog");
    display.println(" Aux2 Analog");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Analog")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Aux1 Digital");
    display.println(" Aux2 Digital");
    display.fillRect(0, 35, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Aux1 Analog"); 
    display.setTextColor(WHITE);
    display.println(" Aux2 Analog");
    display.println(" Trigger");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Analog")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Aux1 Digital");
    display.println(" Aux2 Digital");
    display.println(" Aux1 Analog"); 
    display.fillRect(0, 43, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Aux2 Analog");
    display.setTextColor(WHITE);
    display.println(" Trigger");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Trigger Digital")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trimmer");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.println(" Aux1 Digital");
    display.println(" Aux2 Digital");
    display.println(" Aux1 Analog"); 
    display.println(" Aux2 Analog");
    display.fillRect(0, 51, display.width(), 9, WHITE);
    display.setTextColor(BLACK);
    display.println(" Trigger");
    display.setTextColor(WHITE);
    display.display();
  }  
  if (MenuItem_is_Open == "Auto Aileron")
  {
    AutoTrimmingTable1_Show();
    display.fillRect(0, 11, 64, 14, WHITE);
    display.setTextColor(BLACK);
    display.print("Aileron    ");display.setTextColor(WHITE); if(minAileron<250)display.print(" ");if(minAileron<25)display.print(" ");if(minAileron<=2)display.print(" ");display.print(minAileron*4); display.setCursor(99,15); if(maxAileron<250 )display.print(" "); display.println(maxAileron*4);
    display.setCursor(3,28);
    display.print("Elevator   ");if(minElevator<250)display.print(" ");if(minElevator<25)display.print(" ");if(minElevator<=2)display.print(" ");display.print(minElevator*4); display.setCursor(99,28); if(maxElevator<250 )display.print(" "); display.println(maxElevator*4);
    display.setCursor(3,41);
    display.print("Rudder     ");if(minRudder<250)display.print(" ");if(minRudder<25)display.print(" ");if(minRudder<=2)display.print(" ");display.print(minRudder*4); display.setCursor(99,41); if(maxRudder<250 )display.print(" "); display.println(maxRudder*4);
    display.setCursor(3,54);
    display.print("Throttle   ");if(minThrottle<250)display.print(" ");if(minThrottle<25)display.print(" ");if(minThrottle<=2)display.print(" ");display.print(minThrottle*4); display.setCursor(99,54); if(maxThrottle<250 )display.print(" "); display.println(maxThrottle*4);   
    display.display();
  }
  if (MenuItem_is_Open == "Auto Elevator")
  {
    AutoTrimmingTable1_Show();
    display.print("Aileron    "); if(minAileron<250)display.print(" ");if(minAileron<25)display.print(" ");if(minAileron<=2)display.print(" ");display.print(minAileron*4); display.setCursor(99,15); if(maxAileron<250 )display.print(" "); display.println(maxAileron*4);
    display.setCursor(3,28);
    display.fillRect(0, 24, 64, 14, WHITE);
    display.setTextColor(BLACK);
    display.print("Elevator   ");display.setTextColor(WHITE);if(minElevator<250)display.print(" ");if(minElevator<25)display.print(" ");if(minElevator<=2)display.print(" ");display.print(minElevator*4); display.setCursor(99,28); if(maxElevator<250 )display.print(" "); display.println(maxElevator*4);
    display.setCursor(3,41);
    display.print("Rudder     ");if(minRudder<250)display.print(" ");if(minRudder<25)display.print(" ");if(minRudder<=2)display.print(" ");display.print(minRudder*4); display.setCursor(99,41); if(maxRudder<250 )display.print(" "); display.println(maxRudder*4);
    display.setCursor(3,54);
    display.print("Throttle   ");if(minThrottle<250)display.print(" ");if(minThrottle<25)display.print(" ");if(minThrottle<=2)display.print(" ");display.print(minThrottle*4); display.setCursor(99,54); if(maxThrottle<250 )display.print(" "); display.println(maxThrottle*4);   
    display.display();
  }
  if (MenuItem_is_Open == "Auto Rudder")
  {
    AutoTrimmingTable1_Show();
    display.print("Aileron    "); if(minAileron<250)display.print(" ");if(minAileron<25)display.print(" ");if(minAileron<=2)display.print(" ");display.print(minAileron*4); display.setCursor(99,15); if(maxAileron<250 )display.print(" "); display.println(maxAileron*4);
    display.setCursor(3,28);
    display.print("Elevator   ");if(minElevator<250)display.print(" ");if(minElevator<25)display.print(" ");if(minElevator<=2)display.print(" ");display.print(minElevator*4); display.setCursor(99,28); if(maxElevator<250 )display.print(" "); display.println(maxElevator*4);
    display.setCursor(3,41);
    display.fillRect(0, 37, 64, 14, WHITE);
    display.setTextColor(BLACK);
    display.print("Rudder     ");display.setTextColor(WHITE);if(minRudder<250)display.print(" ");if(minRudder<25)display.print(" ");if(minRudder<=2)display.print(" ");display.print(minRudder*4); display.setCursor(99,41); if(maxRudder<250 )display.print(" "); display.println(maxRudder*4);
    display.setCursor(3,54);
    display.print("Throttle   ");if(minThrottle<250)display.print(" ");if(minThrottle<25)display.print(" ");if(minThrottle<=2)display.print(" ");display.print(minThrottle*4); display.setCursor(99,54); if(maxThrottle<250 )display.print(" "); display.println(maxThrottle*4);   
    display.display();
  }
  if (MenuItem_is_Open == "Auto Throttle")
  {
    AutoTrimmingTable1_Show();
    display.print("Aileron    "); if(minAileron<250)display.print(" ");if(minAileron<25)display.print(" ");if(minAileron<=2)display.print(" ");display.print(minAileron*4); display.setCursor(99,15); if(maxAileron<250 )display.print(" "); display.println(maxAileron*4);
    display.setCursor(3,28);
    display.print("Elevator   ");if(minElevator<250)display.print(" ");if(minElevator<25)display.print(" ");if(minElevator<=2)display.print(" ");display.print(minElevator*4); display.setCursor(99,28); if(maxElevator<250 )display.print(" "); display.println(maxElevator*4);
    display.setCursor(3,41);
    display.print("Rudder     ");if(minRudder<250)display.print(" ");if(minRudder<25)display.print(" ");if(minRudder<=2)display.print(" ");display.print(minRudder*4); display.setCursor(99,41); if(maxRudder<250 )display.print(" "); display.println(maxRudder*4);
    display.setCursor(3,54);
    display.fillRect(0, 50, 64, 14, WHITE);
    display.setTextColor(BLACK);
    display.print("Throttle   ");display.setTextColor(WHITE);if(minThrottle<250)display.print(" ");if(minThrottle<25)display.print(" ");if(minThrottle<=2)display.print(" ");display.print(minThrottle*4); display.setCursor(99,54); if(maxThrottle<250 )display.print(" "); display.println(maxThrottle*4);   
    display.display();
  }
  if (MenuItem_is_Open == "Auto Cam Analog V")
  {
    AutoTrimmingTable1_Show();
    display.fillRect(0, 11, 64, 14, WHITE);
    display.setTextColor(BLACK);
    display.print("CamV An    ");display.setTextColor(WHITE); if(minCam_Analog_V<250)display.print(" ");if(minCam_Analog_V<25)display.print(" ");if(minCam_Analog_V<=2)display.print(" ");display.print(minCam_Analog_V*4); display.setCursor(99,15); if(maxCam_Analog_V<250 )display.print(" "); display.println(maxCam_Analog_V*4);
    display.setCursor(3,28);
    display.print("CamH An    ");if(minCam_Analog_H<250)display.print(" ");if(minCam_Analog_H<25)display.print(" ");if(minCam_Analog_H<=2)display.print(" ");display.print(minCam_Analog_H*4); display.setCursor(99,28); if(maxCam_Analog_H<250 )display.print(" "); display.println(maxCam_Analog_H*4);
    display.setCursor(3,41);
    if (AUX1ModeActive == (B00000010))
    {
      display.print("Aux1' An   ");if(minAux1Analog_<250)display.print(" ");if(minAux1Analog_<25)display.print(" ");if(minAux1Analog_<=2)display.print(" ");display.print(minAux1Analog_*4); display.setCursor(99,41); if(maxAux1Analog_<250 )display.print(" "); display.println(maxAux1Analog_*4);
    }else if (AUX1ModeActive == (B00000100))
    {
      display.print("Aux1 An    ");if(minAux1Analog<250)display.print(" ");if(minAux1Analog<25)display.print(" ");if(minAux1Analog<=2)display.print(" ");display.print(minAux1Analog*4); display.setCursor(99,41); if(maxAux1Analog<250 )display.print(" "); display.println(maxAux1Analog*4);
    }else display.println("Aux1/Aux1'");
    display.setCursor(3,54);
    display.print("Aux2 An    ");if(minAux2Analog<250)display.print(" ");if(minAux2Analog<25)display.print(" ");if(minAux2Analog<=2)display.print(" ");display.print(minAux2Analog*4); display.setCursor(99,54); if(maxAux2Analog<250 )display.print(" "); display.println(maxAux2Analog*4); 
    display.display();
  }
  if (MenuItem_is_Open == "Auto Cam Analog H")
  {
    AutoTrimmingTable1_Show();
    display.print("CamV An    ");if(minCam_Analog_V<250)display.print(" ");if(minCam_Analog_V<25)display.print(" ");if(minCam_Analog_V<=2)display.print(" ");display.print(minCam_Analog_V*4); display.setCursor(99,15); if(maxCam_Analog_V<250 )display.print(" "); display.println(maxCam_Analog_V*4);
    display.setCursor(3,28);
    display.fillRect(0, 24, 64, 14, WHITE);
    display.setTextColor(BLACK);
    display.print("CamH An    ");display.setTextColor(WHITE);if(minCam_Analog_H<250)display.print(" ");if(minCam_Analog_H<25)display.print(" ");if(minCam_Analog_H<=2)display.print(" ");display.print(minCam_Analog_H*4); display.setCursor(99,28); if(maxCam_Analog_H<250 )display.print(" "); display.println(maxCam_Analog_H*4);
    display.setCursor(3,41);
    if (AUX1ModeActive == (B00000010))
    {
      display.print("Aux1' An   ");if(minAux1Analog_<250)display.print(" ");if(minAux1Analog_<25)display.print(" ");if(minAux1Analog_<=2)display.print(" ");display.print(minAux1Analog_*4); display.setCursor(99,41); if(maxAux1Analog_<250 )display.print(" "); display.println(maxAux1Analog_*4);
    }else if (AUX1ModeActive == (B00000100))
    {
      display.print("Aux1 An    ");if(minAux1Analog<250)display.print(" ");if(minAux1Analog<25)display.print(" ");if(minAux1Analog<=2)display.print(" ");display.print(minAux1Analog*4); display.setCursor(99,41); if(maxAux1Analog<250 )display.print(" "); display.println(maxAux1Analog*4);
    }else display.println("Aux1/Aux1'");
    display.setCursor(3,54);
    display.print("Aux2 An    ");if(minAux2Analog<250)display.print(" ");if(minAux2Analog<25)display.print(" ");if(minAux2Analog<=2)display.print(" ");display.print(minAux2Analog*4); display.setCursor(99,54); if(maxAux2Analog<250 )display.print(" "); display.println(maxAux2Analog*4); 
    display.display();
  }
  if (MenuItem_is_Open == "Auto Aux1")
  {
    AutoTrimmingTable1_Show();
    display.print("CamV An    ");if(minCam_Analog_V<250)display.print(" ");if(minCam_Analog_V<25)display.print(" ");if(minCam_Analog_V<=2)display.print(" ");display.print(minCam_Analog_V*4); display.setCursor(99,15); if(maxCam_Analog_V<250 )display.print(" "); display.println(maxCam_Analog_V*4);
    display.setCursor(3,28);
    display.print("CamH An    ");if(minCam_Analog_H<250)display.print(" ");if(minCam_Analog_H<25)display.print(" ");if(minCam_Analog_H<=2)display.print(" ");display.print(minCam_Analog_H*4); display.setCursor(99,28); if(maxCam_Analog_H<250 )display.print(" "); display.println(maxCam_Analog_H*4);
    display.setCursor(3,41);
    display.fillRect(0, 37, 64, 14, WHITE);
    display.setTextColor(BLACK);
    if (AUX1ModeActive == (B00000010))
    {
      display.print("Aux1' An   ");display.setTextColor(WHITE);if(minAux1Analog_<250)display.print(" ");if(minAux1Analog_<25)display.print(" ");if(minAux1Analog_<=2)display.print(" ");display.print(minAux1Analog_*4); display.setCursor(99,41); if(maxAux1Analog_<250 )display.print(" "); display.println(maxAux1Analog_*4);
    }else if (AUX1ModeActive == (B00000100))
    {
      display.print("Aux1 An    ");display.setTextColor(WHITE);if(minAux1Analog<250)display.print(" ");if(minAux1Analog<25)display.print(" ");if(minAux1Analog<=2)display.print(" ");display.print(minAux1Analog*4); display.setCursor(99,41); if(maxAux1Analog<250 )display.print(" "); display.println(maxAux1Analog*4);
    }else {display.println("Aux1/Aux1'"); display.setTextColor(WHITE);}
    display.setCursor(3,54);
    display.print("Aux2 An    ");if(minAux2Analog<250)display.print(" ");if(minAux2Analog<25)display.print(" ");if(minAux2Analog<=2)display.print(" ");display.print(minAux2Analog*4); display.setCursor(99,54); if(maxAux2Analog<250 )display.print(" "); display.println(maxAux2Analog*4); 
    display.display();
  }
  if (MenuItem_is_Open == "Auto Aux2")
  {
    AutoTrimmingTable1_Show();
    display.print("CamV An    ");if(minCam_Analog_V<250)display.print(" ");if(minCam_Analog_V<25)display.print(" ");if(minCam_Analog_V<=2)display.print(" ");display.print(minCam_Analog_V*4); display.setCursor(99,15); if(maxCam_Analog_V<250 )display.print(" "); display.println(maxCam_Analog_V*4);
    display.setCursor(3,28);
    display.print("CamH An    ");if(minCam_Analog_H<250)display.print(" ");if(minCam_Analog_H<25)display.print(" ");if(minCam_Analog_H<=2)display.print(" ");display.print(minCam_Analog_H*4); display.setCursor(99,28); if(maxCam_Analog_H<250 )display.print(" "); display.println(maxCam_Analog_H*4);
    display.setCursor(3,41);
    if (AUX1ModeActive == (B00000010))
    {
      display.print("Aux1' An   "); if(minAux1Analog_<250)display.print(" ");if(minAux1Analog_<25)display.print(" ");if(minAux1Analog_<=2)display.print(" ");display.print(minAux1Analog_*4); display.setCursor(99,41); if(maxAux1Analog_<250 )display.print(" "); display.println(maxAux1Analog_*4);
    }else if (AUX1ModeActive == (B00000100))
    {
      display.print("Aux1 An    ");if(minAux1Analog<250)display.print(" ");if(minAux1Analog<25)display.print(" ");if(minAux1Analog<=2)display.print(" ");display.print(minAux1Analog*4); display.setCursor(99,41); if(maxAux1Analog<250 )display.print(" "); display.println(maxAux1Analog*4);
    }else display.println("Aux1/Aux1'");
    display.setCursor(3,54);
    display.fillRect(0, 50, 64, 14, WHITE);
    display.setTextColor(BLACK);
    display.print("Aux2 An    ");display.setTextColor(WHITE);if(minAux2Analog<250)display.print(" ");if(minAux2Analog<25)display.print(" ");if(minAux2Analog<=2)display.print(" ");display.print(minAux2Analog*4); display.setCursor(99,54); if(maxAux2Analog<250 )display.print(" "); display.println(maxAux2Analog*4);   
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aileron Center")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aileron");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaAileron[dueFlashStorage.read(84)] - 128));
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aileron DefaultSettings")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aileron");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaAileron[dueFlashStorage.read(84)] - 128));
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aileron DefaultSettings YES")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aileron");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aileron DefaultSettings NO")
  {
    display.setCursor(12,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aileron");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Elevator Center")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Elevator");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaElevator[dueFlashStorage.read(84)] - 128));
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Elevator DefaultSettings")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Elevator");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaElevator[dueFlashStorage.read(84)] - 128));
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Elevator DefaultSettings YES")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Elevator");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Elevator DefaultSettings NO")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Elevator");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Rudder Center")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Rudder");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaRudder[dueFlashStorage.read(84)] - 128));
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Rudder DefaultSettings")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Rudder");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaRudder[dueFlashStorage.read(84)] - 128));
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Rudder DefaultSettings YES")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Rudder");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Rudder DefaultSettings NO")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Rudder");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Throttle Min")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Throttle");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Min:"); display.setCursor(103,20);
    display.println(0 + deltaMinThrottle[dueFlashStorage.read(84)]);
    display.setTextColor(WHITE);
    display.print(" Max:"); display.setCursor(103,28);
    display.println(1000 - deltaMaxThrottle[dueFlashStorage.read(84)]);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Throttle Max")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Throttle");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" Min:"); display.setCursor(103,20);
    display.println(0 + deltaMinThrottle[dueFlashStorage.read(84)]);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Max:"); display.setCursor(103,28);
    display.println(1000 - deltaMaxThrottle[dueFlashStorage.read(84)]);
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Throttle DefaultSettings")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Throttle");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" Min:"); display.setCursor(103,20);
    display.println(0 + deltaMinThrottle[dueFlashStorage.read(84)]);
    display.print(" Max:"); display.setCursor(103,28);
    display.println(1000 - deltaMaxThrottle[dueFlashStorage.read(84)]);
    display.fillRect(0, 35, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Throttle DefaultSettings YES")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Throttle");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Throttle DefaultSettings NO")
  {
    display.setCursor(8,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Throttle");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital V Down")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamV Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Down:"); display.setCursor(103,20);
    display.println(0+delta_minCam_Digital_V[dueFlashStorage.read(84)]);
    display.setTextColor(WHITE);
    display.print(" Center:"); display.setCursor(103,28);
    display.println(500 + (delta_centerCam_Digital_V[dueFlashStorage.read(84)]-128));
    display.print(" Up:"); display.setCursor(103,36);
    display.println(1000 - delta_maxCam_Digital_V[dueFlashStorage.read(84)]);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital V Center")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamV Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" Down:"); display.setCursor(103,20);
    display.println(0+delta_minCam_Digital_V[dueFlashStorage.read(84)]);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,28);
    display.println(500 + (delta_centerCam_Digital_V[dueFlashStorage.read(84)]-128));
    display.setTextColor(WHITE);
    display.print(" Up:"); display.setCursor(103,36);
    display.println(1000 - delta_maxCam_Digital_V[dueFlashStorage.read(84)]);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital V Up")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamV Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" Down:"); display.setCursor(103,20);
    display.println(0+delta_minCam_Digital_V[dueFlashStorage.read(84)]);
    display.print(" Center:"); display.setCursor(103,28);
    display.println(500 + (delta_centerCam_Digital_V[dueFlashStorage.read(84)]-128));
    display.fillRect(0, 35, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Up:"); display.setCursor(103,36);
    display.println(1000 - delta_maxCam_Digital_V[dueFlashStorage.read(84)]);
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital V DefaultSettings")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamV Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" Down:"); display.setCursor(103,20);
    display.println(0+delta_minCam_Digital_V[dueFlashStorage.read(84)]);
    display.print(" Center:"); display.setCursor(103,28);
    display.println(500 + (delta_centerCam_Digital_V[dueFlashStorage.read(84)]-128));
    display.print(" Up:"); display.setCursor(103,36);
    display.println(1000 - delta_maxCam_Digital_V[dueFlashStorage.read(84)]);
    display.fillRect(0, 43, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital V DefaultSettings YES")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamV Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital V DefaultSettings NO")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamV Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital H Down")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamH Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Down:"); display.setCursor(103,20);
    display.println(0+delta_minCam_Digital_H[dueFlashStorage.read(84)]);
    display.setTextColor(WHITE);
    display.print(" Center:"); display.setCursor(103,28);
    display.println(500 + (delta_centerCam_Digital_H[dueFlashStorage.read(84)]-128));
    display.print(" Up:"); display.setCursor(103,36);
    display.println(1000 - delta_maxCam_Digital_H[dueFlashStorage.read(84)]);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital H Center")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamH Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" Down:"); display.setCursor(103,20);
    display.println(0+delta_minCam_Digital_H[dueFlashStorage.read(84)]);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,28);
    display.println(500 + (delta_centerCam_Digital_H[dueFlashStorage.read(84)]-128));
    display.setTextColor(WHITE);
    display.print(" Up:"); display.setCursor(103,36);
    display.println(1000 - delta_maxCam_Digital_H[dueFlashStorage.read(84)]);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital H Up")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamH Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" Down:"); display.setCursor(103,20);
    display.println(0+delta_minCam_Digital_H[dueFlashStorage.read(84)]);
    display.print(" Center:"); display.setCursor(103,28);
    display.println(500 + (delta_centerCam_Digital_H[dueFlashStorage.read(84)]-128));
    display.fillRect(0, 35, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Up:"); display.setCursor(103,36);
    display.println(1000 - delta_maxCam_Digital_H[dueFlashStorage.read(84)]);
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital H DefaultSettings")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamH Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" Down:"); display.setCursor(103,20);
    display.println(0+delta_minCam_Digital_H[dueFlashStorage.read(84)]);
    display.print(" Center:"); display.setCursor(103,28);
    display.println(500 + (delta_centerCam_Digital_H[dueFlashStorage.read(84)]-128));
    display.print(" Up:"); display.setCursor(103,36);
    display.println(1000 - delta_maxCam_Digital_H[dueFlashStorage.read(84)]);
    display.fillRect(0, 43, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital H DefaultSettings YES")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamH Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Digital H DefaultSettings NO")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" CamH Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog V Center")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  CamV An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaCam_Analog_V[dueFlashStorage.read(84)] - 128));
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog V DefaultSettings")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  CamV An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaCam_Analog_V[dueFlashStorage.read(84)] - 128));
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog V DefaultSettings YES")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  CamV An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog V DefaultSettings NO")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  CamV An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog H Center")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  CamH An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaCam_Analog_H[dueFlashStorage.read(84)] - 128));
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog H DefaultSettings")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  CamH An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaCam_Analog_H[dueFlashStorage.read(84)] - 128));
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog H DefaultSettings YES")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  CamH An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Cam Analog H DefaultSettings NO")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  CamH An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Digital I")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux1 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" I:"); display.setCursor(103,20);
    display.println(Aux1_Digital_I[dueFlashStorage.read(84)]*4);
    display.setTextColor(WHITE);
    display.print(" O:"); display.setCursor(103,28);
    display.println(Aux1_Digital_O[dueFlashStorage.read(84)]*4);
    display.print(" II:"); display.setCursor(103,36);
    display.println(Aux1_Digital_II[dueFlashStorage.read(84)]*4);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Digital O")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux1 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" I:"); display.setCursor(103,20);
    display.println(Aux1_Digital_I[dueFlashStorage.read(84)]*4);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" O:"); display.setCursor(103,28);
    display.println(Aux1_Digital_O[dueFlashStorage.read(84)]*4);
    display.setTextColor(WHITE);
    display.print(" II:"); display.setCursor(103,36);
    display.println(Aux1_Digital_II[dueFlashStorage.read(84)]*4);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Digital II")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux1 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" I:"); display.setCursor(103,20);
    display.println(Aux1_Digital_I[dueFlashStorage.read(84)]*4);
    display.print(" O:"); display.setCursor(103,28);
    display.println(Aux1_Digital_O[dueFlashStorage.read(84)]*4);
    display.fillRect(0, 35, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" II:"); display.setCursor(103,36);
    display.println(Aux1_Digital_II[dueFlashStorage.read(84)]*4);
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Digital DefaultSettings")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux1 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" I:"); display.setCursor(103,20);
    display.println(Aux1_Digital_I[dueFlashStorage.read(84)]*4);
    display.print(" O:"); display.setCursor(103,28);
    display.println(Aux1_Digital_O[dueFlashStorage.read(84)]*4);
    display.print(" II:"); display.setCursor(103,36);
    display.println(Aux1_Digital_II[dueFlashStorage.read(84)]*4);
    display.fillRect(0, 43, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Digital DefaultSettings YES")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux1 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Digital DefaultSettings NO")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux1 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Digital I")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux2 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" I:"); display.setCursor(103,20);
    display.println(Aux2_Digital_I[dueFlashStorage.read(84)]*4);
    display.setTextColor(WHITE);
    display.print(" O:"); display.setCursor(103,28);
    display.println(Aux2_Digital_O[dueFlashStorage.read(84)]*4);
    display.print(" II:"); display.setCursor(103,36);
    display.println(Aux2_Digital_II[dueFlashStorage.read(84)]*4);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Digital O")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux2 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" I:"); display.setCursor(103,20);
    display.println(Aux2_Digital_I[dueFlashStorage.read(84)]*4);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" O:"); display.setCursor(103,28);
    display.println(Aux2_Digital_O[dueFlashStorage.read(84)]*4);
    display.setTextColor(WHITE);
    display.print(" II:"); display.setCursor(103,36);
    display.println(Aux2_Digital_II[dueFlashStorage.read(84)]*4);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Digital II")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux2 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" I:"); display.setCursor(103,20);
    display.println(Aux2_Digital_I[dueFlashStorage.read(84)]*4);
    display.print(" O:"); display.setCursor(103,28);
    display.println(Aux2_Digital_O[dueFlashStorage.read(84)]*4);
    display.fillRect(0, 35, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" II:"); display.setCursor(103,36);
    display.println(Aux2_Digital_II[dueFlashStorage.read(84)]*4);
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Digital DefaultSettings")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux2 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" I:"); display.setCursor(103,20);
    display.println(Aux2_Digital_I[dueFlashStorage.read(84)]*4);
    display.print(" O:"); display.setCursor(103,28);
    display.println(Aux2_Digital_O[dueFlashStorage.read(84)]*4);
    display.print(" II:"); display.setCursor(103,36);
    display.println(Aux2_Digital_II[dueFlashStorage.read(84)]*4);
    display.fillRect(0, 43, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Digital DefaultSettings YES")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux2 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Digital DefaultSettings NO")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Aux2 Dig");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Analog Center")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Aux1 An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,20);
    if (AUX1ModeActive == B00000010) display.println(500 + (deltaAux1Analog_[dueFlashStorage.read(84)] - 128));
    else display.println(500 + (deltaAux1Analog[dueFlashStorage.read(84)] - 128));
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Analog DefaultSettings")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Aux1 An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.print(" Center:"); display.setCursor(103,20);
    if (AUX1ModeActive == B00000010) display.println(500 + (deltaAux1Analog_[dueFlashStorage.read(84)] - 128));
    else display.println(500 + (deltaAux1Analog[dueFlashStorage.read(84)] - 128));
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Analog DefaultSettings YES")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Aux1 An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux1 Analog DefaultSettings NO")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Aux1 An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Analog Center")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Aux2 An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaAux2Analog[dueFlashStorage.read(84)] - 128));
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Analog DefaultSettings")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Aux2 An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.print(" Center:"); display.setCursor(103,20);
    display.println(500 + (deltaAux2Analog[dueFlashStorage.read(84)] - 128));
    display.setTextColor(BLACK);
    display.println(" Reset Settings");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Analog DefaultSettings YES")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Aux2 An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Aux2 Analog DefaultSettings NO")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Aux2 An");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Trigger Digital On")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trigger");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" On:"); display.setCursor(103,20);
    display.println(Trigger_Digital_On[dueFlashStorage.read(84)]*4);
    display.setTextColor(WHITE);
    display.print(" Off:"); display.setCursor(103,28);
    display.println(Trigger_Digital_Off[dueFlashStorage.read(84)]*4);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Trigger Digital Off")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trigger");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" On:"); display.setCursor(103,20);
    display.println(Trigger_Digital_On[dueFlashStorage.read(84)]*4);
    display.fillRect(0, 27, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Off:"); display.setCursor(103,28);
    display.println(Trigger_Digital_Off[dueFlashStorage.read(84)]*4);
    display.setTextColor(WHITE);
    display.println(" Reset Settings");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Trigger Digital DefaultSettings")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trigger");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.print(" On:"); display.setCursor(103,20);
    display.println(Trigger_Digital_On[dueFlashStorage.read(84)]*4);
    display.print(" Off:"); display.setCursor(103,28);
    display.println(Trigger_Digital_Off[dueFlashStorage.read(84)]*4);
    display.fillRect(0, 35, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    display.print(" Reset Settings"); display.setCursor(103,36);
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Trigger Digital DefaultSettings YES")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trigger");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "Trimmer Trigger Digital DefaultSettings NO")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Trigger");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);   
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }

  
  if (MenuItem_is_Open == "Invert Aileron")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    if ((InvertChanelsPart1&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aileron");
    }
    else display.println("   Aileron");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart1&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    if ((InvertChanelsPart1&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    if ((InvertChanelsPart1&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    if ((InvertChanelsPart1&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 46,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV Dig"); 
    }
    else display.println("   CamV Dig");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Elevator")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 27, display.width(), 9, WHITE); 
    if ((InvertChanelsPart1&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aileron");
    }
    else display.println("   Aileron");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart1&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart1&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    if ((InvertChanelsPart1&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    if ((InvertChanelsPart1&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 46,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV Dig"); 
    }
    else display.println("   CamV Dig");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Rudder")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE); 
    if ((InvertChanelsPart1&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aileron");
    }
    else display.println("   Aileron");
    if ((InvertChanelsPart1&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart1&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart1&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    if ((InvertChanelsPart1&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 46,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV Dig"); 
    }
    else display.println("   CamV Dig");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Throttle")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE); 
    if ((InvertChanelsPart1&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    if ((InvertChanelsPart1&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart1&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart1&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV Dig"); 
    }
    else display.println("   CamV Dig");
    if ((InvertChanelsPart1&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH Dig"); 
    }
    else display.println("   CamH Dig");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Cam Digital V")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE); 
    if ((InvertChanelsPart1&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    if ((InvertChanelsPart1&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart1&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV Dig"); 
    }
    else display.println("   CamV Dig");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart1&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH Dig"); 
    }
    else display.println("   CamH Dig");
    if ((InvertChanelsPart1&(B00000010)) == (B00000010))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV An"); 
    }
    else display.println("   CamV An");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Cam Digital H")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE);
    if ((InvertChanelsPart1&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    if ((InvertChanelsPart1&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV Dig"); 
    }
    else display.println("   CamV Dig");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart1&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH Dig"); 
    }
    else display.println("   CamH Dig");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart1&(B00000010)) == (B00000010))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV An"); 
    }
    else display.println("   CamV An");
    if ((InvertChanelsPart1&(B00000001)) == (B00000001))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH An"); 
    }
    else display.println("   CamH An");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Cam Analog V")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE);
    if ((InvertChanelsPart1&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV Dig"); 
    }
    else display.println("   CamV Dig");
    if ((InvertChanelsPart1&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH Dig"); 
    }
    else display.println("   CamH Dig");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart1&(B00000010)) == (B00000010))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV An"); 
    }
    else display.println("   CamV An");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart1&(B00000001)) == (B00000001))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH An"); 
    }
    else display.println("   CamH An");
    if ((InvertChanelsPart2&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1 An"); 
    }
    else display.println("   Aux1 An");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Cam Analog H")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE);
    if ((InvertChanelsPart1&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH Dig"); 
    }
    else display.println("   CamH Dig");
    if ((InvertChanelsPart1&(B00000010)) == (B00000010))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV An"); 
    }
    else display.println("   CamV An");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart1&(B00000001)) == (B00000001))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH An"); 
    }
    else display.println("   CamH An");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart2&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1 An"); 
    }
    else display.println("   Aux1 An");
    if ((InvertChanelsPart2&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux2 An"); 
    }
    else display.println("   Aux2 An");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Aux1")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 43, display.width(), 9, WHITE);
    if ((InvertChanelsPart1&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH Dig"); 
    }
    else display.println("   CamH Dig");
    if ((InvertChanelsPart1&(B00000010)) == (B00000010))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV An"); 
    }
    else display.println("   CamV An");
    if ((InvertChanelsPart1&(B00000001)) == (B00000001))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH An"); 
    }
    else display.println("   CamH An");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart2&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1 An"); 
    }
    else display.println("   Aux1 An");
    display.setTextColor(WHITE);
    if ((InvertChanelsPart2&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux2 An"); 
    }
    else display.println("   Aux2 An");
    display.display();
  }
  if (MenuItem_is_Open == "Invert Aux2")
  {
    display.setCursor(10,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("  Invert");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 51, display.width(), 9, WHITE);
    if ((InvertChanelsPart1&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH Dig"); 
    }
    else display.println("   CamH Dig");
    if ((InvertChanelsPart1&(B00000010)) == (B00000010))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamV An"); 
    }
    else display.println("   CamV An");
    if ((InvertChanelsPart1&(B00000001)) == (B00000001))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   CamH An"); 
    }
    else display.println("   CamH An");
    if ((InvertChanelsPart2&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1 An"); 
    }
    else display.println("   Aux1 An");
    display.setTextColor(BLACK);
    if ((InvertChanelsPart2&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux2 An"); 
    }
    else display.println("   Aux2 An");
    display.setTextColor(WHITE);
    display.display();
  }

  if (MenuItem_is_Open == "Exponential Aileron")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Exponent");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    if ((ExponentialChanels&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aileron");
    }
    else display.println("   Aileron");
    display.setTextColor(WHITE);
    if ((ExponentialChanels&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    if ((ExponentialChanels&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    if ((ExponentialChanels&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    if ((ExponentialChanels&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 46,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1"); 
    }
    else display.println("   Aux1");
    display.display();
  }
  if (MenuItem_is_Open == "Exponential Elevator")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Exponent");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 27, display.width(), 9, WHITE); 
    if ((ExponentialChanels&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aileron");
    }
    else display.println("   Aileron");
    display.setTextColor(BLACK);
    if ((ExponentialChanels&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    display.setTextColor(WHITE);
    if ((ExponentialChanels&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    if ((ExponentialChanels&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    if ((ExponentialChanels&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 46,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1"); 
    }
    else display.println("   Aux1");
    display.display();
  }
  if (MenuItem_is_Open == "Exponential Rudder")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Exponent");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE); 
    if ((ExponentialChanels&(B10000000)) == (B10000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aileron");
    }
    else display.println("   Aileron");
    if ((ExponentialChanels&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    display.setTextColor(BLACK);
    if ((ExponentialChanels&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    display.setTextColor(WHITE);
    if ((ExponentialChanels&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    if ((ExponentialChanels&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 46,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1"); 
    }
    else display.println("   Aux1");
    display.display();
  }
  if (MenuItem_is_Open == "Exponential Throttle")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Exponent");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE); 
    if ((ExponentialChanels&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    if ((ExponentialChanels&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    display.setTextColor(BLACK);
    if ((ExponentialChanels&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    display.setTextColor(WHITE);
    if ((ExponentialChanels&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1"); 
    }
    else display.println("   Aux1");
    if ((ExponentialChanels&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux2"); 
    }
    else display.println("   Aux2");
    display.display();
  }
  if (MenuItem_is_Open == "Exponential Aux1")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Exponent");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 43, display.width(), 9, WHITE); 
    if ((ExponentialChanels&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    if ((ExponentialChanels&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    
    if ((ExponentialChanels&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    display.setTextColor(BLACK);
    if ((ExponentialChanels&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1"); 
    }
    else display.println("   Aux1");
    display.setTextColor(WHITE);
    if ((ExponentialChanels&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux2"); 
    }
    else display.println("   Aux2");
    display.display();
  }
  if (MenuItem_is_Open == "Exponential Aux2")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Exponent");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 51, display.width(), 9, WHITE); 
    if ((ExponentialChanels&(B01000000)) == (B01000000))
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Elevator");
    }
    else display.println("   Elevator");
    if ((ExponentialChanels&(B00100000)) == (B00100000))
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Rudder"); 
    }
    else display.println("   Rudder");
    
    if ((ExponentialChanels&(B00010000)) == (B00010000))
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Throttle"); 
    }
    else display.println("   Throttle");
    if ((ExponentialChanels&(B00001000)) == (B00001000))
    {
      display.drawBitmap(0, 38,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux1"); 
    }
    else display.println("   Aux1");
    display.setTextColor(BLACK);
    if ((ExponentialChanels&(B00000100)) == (B00000100))
    {
      display.drawBitmap(0, 45,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Aux2"); 
    }
    else display.println("   Aux2");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Plane Classic")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Plane Mode"); display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 19, display.width(), 9, WHITE); 
    display.setTextColor(BLACK);
    if (PlaneModeActive[dueFlashStorage.read(84)] == 0)
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Classic");
    }
    else display.println("   Classic");
    display.setTextColor(WHITE);
    if (PlaneModeActive[dueFlashStorage.read(84)] == 1)
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Flying Wing");
    }
    else display.println("   Flying Wing");
    if (PlaneModeActive[dueFlashStorage.read(84)] == 2)
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   V-tail"); 
    }
    else display.println("   V-tail");
    display.display();
  }
  if (MenuItem_is_Open == "Plane Flying Wing")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Plane Mode");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 27, display.width(), 9, WHITE); 
    if (PlaneModeActive[dueFlashStorage.read(84)] == 0)
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Classic");
    }
    else display.println("   Classic");
    display.setTextColor(BLACK);
    if (PlaneModeActive[dueFlashStorage.read(84)] == 1)
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Flying Wing");
    }
    else display.println("   Flying Wing");
    display.setTextColor(WHITE);
    if (PlaneModeActive[dueFlashStorage.read(84)] == 2)
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   V-tail"); 
    }
    else display.println("   V-tail");
    display.display();
  }
  if (MenuItem_is_Open == "Plane V-tail")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("Plane Mode");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(16, 35, display.width(), 9, WHITE); 
    if (PlaneModeActive[dueFlashStorage.read(84)] == 0)
    {
      display.drawBitmap(0, 14,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Classic");
    }
    else display.println("   Classic");
    if (PlaneModeActive[dueFlashStorage.read(84)] == 1)
    {
      display.drawBitmap(0, 22,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   Flying Wing");
    }
    else display.println("   Flying Wing");
    display.setTextColor(BLACK);
    if (PlaneModeActive[dueFlashStorage.read(84)] == 2)
    {
      display.drawBitmap(0, 30,  Due.CheckItem(), 16, 16, WHITE);
      display.println("   V-tail"); 
    }
    else display.println("   V-tail");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "DefaultSettings for this Plane YES")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("All Option");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 45, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.setTextColor(BLACK);
    display.println("YES");
    display.setTextColor(WHITE);
    display.setCursor(63,55);
    display.println("NO");
    display.display();
  }
  if (MenuItem_is_Open == "DefaultSettings for this Plane NO")
  {
    display.setCursor(0,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println("All Option");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 54, display.width(), 9, WHITE); 
    display.print(" Do you want to resetthe settings for thisplane?"); display.setCursor(60,46);
    display.println("YES");
    display.setCursor(63,55);
    display.setTextColor(BLACK);
    display.println("NO");
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Invert PPM")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Settings");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 19, display.width(), 9, WHITE);  
    display.setTextColor(BLACK);
    display.print(" Invert PPM:"); display.setCursor(90,20); 
    if ((InvertPPMActive&(B00000001)) == 1) display.println("YES"); else if ((InvertPPMActive&(B00000001)) == 0) display.println("NO");
    display.setTextColor(WHITE);
    display.print(" Cam Mode:"); display.setCursor(90,28);
    if ((CamModeActive&(B00000001)) == 0) display.println("An"); else if ((CamModeActive&(B00000001)) == 1) display.println("Dig");
    display.print(" AUX1 Mode:"); display.setCursor(90,36);
    if (AUX1ModeActive == (B00001000)) display.println("Trig"); else if (AUX1ModeActive == (B00000100)) display.println("An"); else if (AUX1ModeActive == (B00000010)) display.println("An'"); else if (AUX1ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.print(" AUX2 Mode:"); display.setCursor(90,44);
    if (AUX2ModeActive == (B00000100)) display.println("Trig"); else if (AUX2ModeActive == (B00000010)) display.println("An"); else if (AUX2ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.print(" CamDigDelay:"); display.setCursor(90,52); display.println(CamDigitalDelay);   
    display.display();
  }
  if (MenuItem_is_Open == "Cam Mode")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Settings");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 27, display.width(), 9, WHITE);  
    display.print(" Invert PPM:"); display.setCursor(90,20); 
    if ((InvertPPMActive&(B00000001)) == 1) display.println("YES"); else if ((InvertPPMActive&(B00000001)) == 0) display.println("NO");
    display.setTextColor(BLACK);
    display.print(" Cam Mode:"); display.setCursor(90,28);
    if ((CamModeActive&(B00000001)) == 0) display.println("An"); else if ((CamModeActive&(B00000001)) == 1) display.println("Dig");
    display.setTextColor(WHITE);
    display.print(" AUX1 Mode:"); display.setCursor(90,36);
    if (AUX1ModeActive == (B00001000)) display.println("Trig"); else if (AUX1ModeActive == (B00000100)) display.println("An"); else if (AUX1ModeActive == (B00000010)) display.println("An'"); else if (AUX1ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.print(" AUX2 Mode:"); display.setCursor(90,44);
    if (AUX2ModeActive == (B00000100)) display.println("Trig"); else if (AUX2ModeActive == (B00000010)) display.println("An"); else if (AUX2ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.print(" CamDigDelay:"); display.setCursor(90,52); display.println(CamDigitalDelay);
    display.display();
  }
  if (MenuItem_is_Open == "AUX1 Mode")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Settings");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 35, display.width(), 9, WHITE);  
    display.print(" Invert PPM:"); display.setCursor(90,20); 
    if ((InvertPPMActive&(B00000001)) == 1) display.println("YES"); else if ((InvertPPMActive&(B00000001)) == 0) display.println("NO");
    display.print(" Cam Mode:"); display.setCursor(90,28);
    if ((CamModeActive&(B00000001)) == 0) display.println("An"); else if ((CamModeActive&(B00000001)) == 1) display.println("Dig");
    display.setTextColor(BLACK);
    display.print(" AUX1 Mode:"); display.setCursor(90,36);
    if (AUX1ModeActive == (B00001000)) display.println("Trig"); else if (AUX1ModeActive == (B00000100)) display.println("An"); else if (AUX1ModeActive == (B00000010)) display.println("An'"); else if (AUX1ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.setTextColor(WHITE);
    display.print(" AUX2 Mode:"); display.setCursor(90,44);
    if (AUX2ModeActive == (B00000100)) display.println("Trig"); else if (AUX2ModeActive == (B00000010)) display.println("An"); else if (AUX2ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.print(" CamDigDelay:"); display.setCursor(90,52); display.println(CamDigitalDelay); 
    display.display();
  }
  if (MenuItem_is_Open == "AUX2 Mode")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Settings");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 35, display.width(), 9, WHITE);  
    display.print(" Cam Mode:"); display.setCursor(90,20);
    if ((CamModeActive&(B00000001)) == 0) display.println("An"); else if ((CamModeActive&(B00000001)) == 1) display.println("Dig");
    display.print(" AUX1 Mode:"); display.setCursor(90,28);
    if (AUX1ModeActive == (B00001000)) display.println("Trig"); else if (AUX1ModeActive == (B00000100)) display.println("An"); else if (AUX1ModeActive == (B00000010)) display.println("An'"); else if (AUX1ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.setTextColor(BLACK);
    display.print(" AUX2 Mode:"); display.setCursor(90,36);
    if (AUX2ModeActive == (B00000100)) display.println("Trig"); else if (AUX2ModeActive == (B00000010)) display.println("An"); else if (AUX2ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.setTextColor(WHITE);
    display.print(" CamDigDelay:"); display.setCursor(90,44); display.println(CamDigitalDelay); 
    display.print(" CamAnDelay:"); display.setCursor(90,52); display.println(CamAnalogDelay); 
    display.display();
  }
  if (MenuItem_is_Open == "CamDigitalDelay Mode")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Settings");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 43, display.width(), 9, WHITE);  
    display.print(" Cam Mode:"); display.setCursor(90,20);
    if ((CamModeActive&(B00000001)) == 0) display.println("An"); else if ((CamModeActive&(B00000001)) == 1) display.println("Dig");
    display.print(" AUX1 Mode:"); display.setCursor(90,28);
    if (AUX1ModeActive == (B00001000)) display.println("Trig"); else if (AUX1ModeActive == (B00000100)) display.println("An"); else if (AUX1ModeActive == (B00000010)) display.println("An'"); else if (AUX1ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.print(" AUX2 Mode:"); display.setCursor(90,36);
    if (AUX2ModeActive == (B00000100)) display.println("Trig"); else if (AUX2ModeActive == (B00000010)) display.println("An"); else if (AUX2ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.setTextColor(BLACK);
    display.print(" CamDigDelay:"); display.setCursor(90,44); display.println(CamDigitalDelay); 
    display.setTextColor(WHITE);
    display.print(" CamAnDelay:"); display.setCursor(90,52); display.println(CamAnalogDelay); 
    display.display();
  }
  if (MenuItem_is_Open == "CamAnalogDelay Mode")
  {
    display.setCursor(4,0);display.setTextSize(2);display.setTextColor(WHITE);
    display.println(" Settings");display.setTextSize(1);display.drawLine(0, 16, display.width()-1, 16, WHITE); display.setCursor(0,20);
    display.fillRect(0, 51, display.width(), 9, WHITE);  
    display.print(" Cam Mode:"); display.setCursor(90,20);
    if ((CamModeActive&(B00000001)) == 0) display.println("An"); else if ((CamModeActive&(B00000001)) == 1) display.println("Dig");
    display.print(" AUX1 Mode:"); display.setCursor(90,28);
    if (AUX1ModeActive == (B00001000)) display.println("Trig"); else if (AUX1ModeActive == (B00000100)) display.println("An"); else if (AUX1ModeActive == (B00000010)) display.println("An'"); else if (AUX1ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.print(" AUX2 Mode:"); display.setCursor(90,36);
    if (AUX2ModeActive == (B00000100)) display.println("Trig"); else if (AUX2ModeActive == (B00000010)) display.println("An"); else if (AUX2ModeActive == (B00000001)) display.println("Dig");else display.println("None");
    display.print(" CamDigDelay:"); display.setCursor(90,44); display.println(CamDigitalDelay); 
    display.setTextColor(BLACK);
    display.print(" CamAnDelay:"); display.setCursor(90,52); display.println(CamAnalogDelay); 
    display.setTextColor(WHITE);
    display.display();
  }
  if (MenuItem_is_Open == "Test Aileron")
  {
    isTest_Show = true;
  }
  if (MenuItem_is_Open == "Test Elevator")
  {
    isTest_Show = true;
  }
  if (MenuItem_is_Open == "Test Rudder")
  {
    isTest_Show = true;
  }
  if (MenuItem_is_Open == "Test Throttle")
  {
    isTest_Show = true;
  }
  if (MenuItem_is_Open == "Test Cam Analog V")
  {
    isTest_Show = true;
  }
  if (MenuItem_is_Open == "Test Cam Analog H")
  {
    isTest_Show = true;
  }
  if (MenuItem_is_Open == "Test Aux1")
  {
    isTest_Show = true;
  }
  if (MenuItem_is_Open == "Test Aux2")
  {
    isTest_Show = true;
  }
  }
}
//-------------------------------------------------------------------------------------------------------
void  isButtonUp(int button, IButton NameButton, bool& flag)
{
  bool ON = true, OFF = false;
  if ((button==0)&&(flag==OFF))
  {
    ButtonUp(NameButton);
    flag = ON; 
  }else if ((button == 1)&&(flag == ON))
  {
    flag = OFF;
    prokrutka = 1;
    Due._TimerSetup1 = false;
    Due._TimeBegin1 = false;
  }
  else if ((button==0)&&(flag==ON))
  {
     if (Due._TimerSetup1 == false)
     {
       prokrutka*=1.5;
       Due.setTimer1(1000/(prokrutka));
       Due.startTimer1();
     }
     if (Due.Haw_much_time_elapsed1()>(1.0/prokrutka))
     {          
       flag = OFF;
       Due._TimerSetup1 = false;
       Due._TimeBegin1 = false;
     }
   }
}
void  MenuNavigate()
{
  isButtonUp(Up, up, Up_flag);
  isButtonUp(Down, down, Down_flag);
  isButtonUp(Left, left, Left_flag);
  isButtonUp(Right, right, Right_flag);
  isButtonUp(Enter, enter, Enter_flag);
  isButtonUp(Cancel, cancel, Cancel_flag);
}
void ButtonUp(IButton NameButton)
{
  MenuItem currentMenu=menu.getCurrent();  
  switch (NameButton){
    case  enter:
    {
      if (OnlineTrimingShow == true)
      {
        isMainScreen_Show = true;
        OnlineTrimingShow = false;
      }
      if(!(currentMenu.moveDown()))
      {  //if the current menu has a child and has been pressed enter then menu navigate to item below
        menu.use();
      }else
      {  //otherwise, if menu has no child and has been pressed enter the current menu is used
        menu.moveDown();
      } 
      break;
    }
    case cancel:
    {
      if (MenuItem_is_Change != " ") {MenuItem_is_Change = " "; delta = 0; minValue = 1024; maxValue = 0; menu.moveRight();}
      else menu.moveUp();  //toRoot() - back to main
      if (OnlineTrimingShow == true)
      {
        isMainScreen_Show = true;
        OnlineTrimingShow = false;
      }
      break;
    }
    case down:
    {
      if (MenuItem_is_Change == " ") menu.moveRight();
      if (isMainScreen_Show || OnlineTrimingShow)
      {
        isMainScreen_Show = false;
        OnlineTrimingShow= true;
        
        if ((InvertChanelsPart1&(B10000000)) == (B10000000))
        {
          if (minAileron*4 + 1 < centerAileron*4 + OnlineTrimAileron)
          OnlineTrimAileron--;
        }
        else if (maxAileron*4 - 1 > centerAileron*4 + OnlineTrimAileron) OnlineTrimAileron++;
        
        display.setCursor(0,0);
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.clearDisplay();
        display.println("  ONLINE");
        display.println();
        display.setTextSize(1);
        display.print  ("  Ail="); display.print(centerAileron*4+OnlineTrimAileron); display.print  ("    Ele="); display.println(centerElevator*4+OnlineTrimElevator); 
        display.setCursor(0,52) ;
        display.display();
      }
      break;     
    } 
    case up:
    {
      if (MenuItem_is_Change == " ") menu.moveLeft();
      if (isMainScreen_Show || OnlineTrimingShow)
      {
        isMainScreen_Show = false;
        OnlineTrimingShow= true;
        
        if ((InvertChanelsPart1&(B10000000)) == (B10000000))
        {
          if (maxAileron*4 - 1 > centerAileron*4 + OnlineTrimAileron)
          OnlineTrimAileron++;
        }
        else if (minAileron*4 + 1 < centerAileron*4 + OnlineTrimAileron) OnlineTrimAileron--;
        
        display.setCursor(0,0);
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.clearDisplay();
        display.println("  ONLINE");
        display.println();
        display.setTextSize(1);
        display.print  ("  Ail="); display.print(centerAileron*4+OnlineTrimAileron); display.print  ("    Ele="); display.println(centerElevator*4+OnlineTrimElevator); 
        display.setCursor(0,52) ;
        display.display();
      }
      break;    
    }
    case right:
    {
      if (ActiveCange)
      delta++;
      if (isMainScreen_Show || OnlineTrimingShow)
      {
        isMainScreen_Show = false;
        OnlineTrimingShow= true;

        if ((InvertChanelsPart1&(B01000000)) == (B01000000))
        {
          if (minElevator*4 + 1 < centerElevator*4 + OnlineTrimElevator)
          OnlineTrimElevator--;
        }
        else if (maxElevator*4 - 1 > centerElevator*4 + OnlineTrimElevator) OnlineTrimElevator++;
        
        display.setCursor(0,0);
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.clearDisplay();
        display.println("  ONLINE");
        display.println();
        display.setTextSize(1);
        display.print  ("  Ail="); display.print(centerAileron*4+OnlineTrimAileron); display.print  ("    Ele="); display.println(centerElevator*4+OnlineTrimElevator); 
        display.setCursor(0,52) ;
        display.display();
      }
      break;  
    }
    case left:
    {
      if (ActiveCange)
      delta--;
      if (isMainScreen_Show || OnlineTrimingShow)
      {
        isMainScreen_Show = false;
        OnlineTrimingShow= true;

        if ((InvertChanelsPart1&(B01000000)) == (B01000000))
        {
          if (maxElevator*4 - 1 > centerElevator*4 + OnlineTrimElevator) 
          OnlineTrimElevator++;
        }
        else if (minElevator*4 + 1 < centerElevator*4 + OnlineTrimElevator) OnlineTrimElevator--;
        
        display.setCursor(0,0);
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.clearDisplay();
        display.println("  ONLINE");
        display.println();
        display.setTextSize(1);
        display.print  ("  Ail="); display.print(centerAileron*4+OnlineTrimAileron); display.print  ("    Ele="); display.println(centerElevator*4+OnlineTrimElevator); 
        display.setCursor(0,52) ;
        display.display();
      }
      break;    
    }
  }
}
//-------------------------------------------------------------------------------------------------------
//*******************************************************************************//
//===============================================================================//



//===============================================================================//
//***************************Инициализация переменных****************************//

void setup() 
{
  Due.Init();
  analogReadResolution(10); //разрешение 0-2048
  Due.PinsInit();
  MenuInit();
  DisplayInit();

  pinMode(2, OUTPUT);        // port B pin 25  
  analogWrite(2, 0);      // sets up some other registers I haven't worked out yet
  REG_PIOB_PDR = 1 << 25;     // disable PIO, enable peripheral
  REG_PIOB_ABSR = 1 << 25;    // select peripheral B
  REG_TC0_WPMR = 0x54494D00;    // enable write to registers

  if ((InvertPPMActive&(B00000001)) == 0)
  {
     REG_TC0_CMR0 = 0b00000000000001101100010000000000; // alternative CMR for inverted output
  }
  else REG_TC0_CMR0 = 0b00000000000010011100010000000000; // set channel mode register (see datasheet)
  
  REG_TC0_RC0 = 100000000;    // counter period
  REG_TC0_CCR0 = 0b101;     // start counter
  REG_TC0_IER0 = 0b00010000;    // enable interrupt on counter = rc
  REG_TC0_IDR0 = 0b11101111;    // disable other interrupts
  REG_TC0_RA0 = PPMPulse_ms*1000*42;     // Pulse lenght    0.3*1000*42=0.3ms
  PPMFrame = PPMFrame_ms*1000*42;          // ppm frame lenght 22.5*1000*42=22.5ms  
  NVIC_EnableIRQ(TC0_IRQn);   // enable TC0 interrupts
}

//*******************************************************************************//
//===============================================================================//



//===============================================================================//
//********************************Основной цикл**********************************//

void loop() 
{
  //if ((millis() > 3300) && (PPM_power == 0)) {PPM_power = 255; pinMode(2, OUTPUT);} 
  readButtons();
  PPM_FORM();
  MenuNavigate();

  CangeAndSave(MenuItem_is_Change, ActiveCange);

  if (isMainScreen_Show) MainScreenPrint();

  //SerialPrint_PPM(); // тест значений PPM

  if(isTest_Show) Test_Show(MenuItem_is_Open);
}

//*******************************************************************************// дошёл до Aux
//===============================================================================//

void PPM_FORM()
{
  if (RatesHigh == 0)
  {
    power = 0;
  }else if (RatesLow == 0)
    {
      power = 250;
    }else 
      {
        power = 125;
      }
 
    int AV = analogValue[8];//aileron
    if (AV < minAileron*4) AV = minAileron*4;
    if (AV > maxAileron*4) AV = maxAileron*4;
    
    int DeltaA;
    if (MenuItem_is_Change == "Trimmer Aileron Center")
    {
      OnlineTrimAileron = 0;
      DeltaA = (deltaAileron[PlaneActive] - 128) + delta;
    }else DeltaA = (deltaAileron[PlaneActive] - 128) + OnlineTrimAileron;
    
    if (DeltaA > 0)
    {
      if (AV < centerAileron*4)
      {
      ppm_channels_1000_2000[0] = map(AV, minAileron*4, centerAileron*4, 1000 + power + (DeltaA)*2, 1500 + DeltaA);
      }else 
      {
        ppm_channels_1000_2000[0] = map(AV, centerAileron*4, maxAileron*4, 1500 + DeltaA, 2000 - power);
      }
    }else
    {
      if (AV < centerAileron*4)
      {
        ppm_channels_1000_2000[0] = map(AV, minAileron*4, centerAileron*4, 1000 + power, 1500 + DeltaA);
      }else 
      {
        ppm_channels_1000_2000[0] = map(AV, centerAileron*4, maxAileron*4, 1500 + DeltaA, 2000 + (DeltaA)*2 - power);
      }
    }

    AV = analogValue[9];//Elevator
    if (AV < minElevator*4) AV = minElevator*4;
    if (AV > maxElevator*4) AV = maxElevator*4;
    
    int DeltaE;
    if (MenuItem_is_Change == "Trimmer Elevator Center")
    {
      OnlineTrimElevator = 0;
      DeltaE = (deltaElevator[PlaneActive] - 128) + delta;
    }else DeltaE = (deltaElevator[PlaneActive] - 128) + OnlineTrimElevator;

    if ((DeltaE) > 0)
    {
      if (AV < centerElevator*4)
      {
      ppm_channels_1000_2000[1] = map(AV, minElevator*4, centerElevator*4, 1000 + power + (DeltaE)*2, 1500 + DeltaE);
      }else 
      {
        ppm_channels_1000_2000[1] = map(AV, centerElevator*4, maxElevator*4, 1500 + DeltaE, 2000 - power);
      }
    }else
    {
      if (AV < centerElevator*4)
      {
        ppm_channels_1000_2000[1] = map(AV, minElevator*4, centerElevator*4, 1000 + power, 1500 + DeltaE);
      }else 
      {
        ppm_channels_1000_2000[1] = map(AV, centerElevator*4, maxElevator*4, 1500 + DeltaE, 2000 + (DeltaE)*2 - power);
      }
    }
    
    int DeltaR;
    if (MenuItem_is_Change == "Trimmer Rudder Center")
    {
      DeltaR = (deltaRudder[PlaneActive] - 128) + delta;
    }else DeltaR = (deltaRudder[PlaneActive] - 128);
    AV = analogValue[6];//Rudder
    if (AV < minRudder*4) AV = minRudder*4;
    if (AV > maxRudder*4) AV = maxRudder*4;
    if ((DeltaR) > 0)
    {
      if (AV < centerRudder*4)
      {
      ppm_channels_1000_2000[2] = map(AV, minRudder*4, centerRudder*4, 1000 + power + (DeltaR)*2, 1500 + DeltaR);
      }else 
      {
        ppm_channels_1000_2000[2] = map(AV, centerRudder*4, maxRudder*4, 1500 + DeltaR, 2000 - power);
      }
    }else
    {
      if (AV < centerRudder*4)
      {
        ppm_channels_1000_2000[2] = map(AV, minRudder*4, centerRudder*4, 1000 + power, 1500 + DeltaR);
      }else 
      {
        ppm_channels_1000_2000[2] = map(AV, centerRudder*4, maxRudder*4, 1500 + DeltaR, 2000 + (DeltaR)*2 - power);
      }
    }
    int DeltaTmin;
    if (MenuItem_is_Change == "Trimmer Throttle Min")
    {
      DeltaTmin = deltaMinThrottle[PlaneActive] + delta;
    }else DeltaTmin = deltaMinThrottle[PlaneActive];
    int DeltaTmax;
    if (MenuItem_is_Change == "Trimmer Throttle Max")
    {
      DeltaTmax = deltaMaxThrottle[PlaneActive] + delta;
    }else DeltaTmax = deltaMaxThrottle[PlaneActive];
    AV = analogValue[7];//Throttle
    if (AV < minThrottle*4) AV = minThrottle*4;
    if (AV > maxThrottle*4) AV = maxThrottle*4;
    ppm_channels_1000_2000[3] = map(AV, minThrottle*4, maxThrottle*4, 1000 + DeltaTmin, 2000 - DeltaTmax);

    if ((CamModeActive&(B00000001)) == 0)
    {
      int DeltaCAV;
      if (MenuItem_is_Change == "Trimmer Cam Analog V Center")
      {
        DeltaCAV = deltaCam_Analog_V[PlaneActive] - 128 + delta;
      }else DeltaCAV = deltaCam_Analog_V[PlaneActive] - 128;
      if (CameraV_old != analogValue[5])
      {
        if (CamAnalogDelay != 0)
        {
          if (CameraV_old < analogValue[5]) CameraV_old += 11 - CamAnalogDelay;
          else CameraV_old -= 11 - CamAnalogDelay;
          AV = CameraV_old;
        }else AV = analogValue[5];
      }
      if (AV < minCam_Analog_V*4) AV = minCam_Analog_V*4;
      if (AV > maxCam_Analog_V*4) AV = maxCam_Analog_V*4;
      if ((DeltaCAV) > 0)
      {
        if (AV < centerCam_Analog_V*4)
        {
        ppm_channels_1000_2000[4] = map(AV, minCam_Analog_V*4, centerCam_Analog_V*4, 1000 + (DeltaCAV)*2, 1500 + DeltaCAV);
        }else 
        {
          ppm_channels_1000_2000[4] = map(AV, centerCam_Analog_V*4, maxCam_Analog_V*4, 1500 + DeltaCAV, 2000);
        }
      }else
      {
        if (AV < centerCam_Analog_V*4)
        {
          ppm_channels_1000_2000[4] = map(AV, minCam_Analog_V*4, centerCam_Analog_V*4, 1000, 1500 + DeltaCAV);
        }else 
        {
          ppm_channels_1000_2000[4] = map(AV, centerCam_Analog_V*4, maxCam_Analog_V*4, 1500 + DeltaCAV, 2000 + (DeltaCAV)*2);
        }
      }

      int DeltaCAH;
      if (MenuItem_is_Change == "Trimmer Cam Analog H Center")
      {
        DeltaCAH = deltaCam_Analog_H[PlaneActive] - 128 + delta;
      }else DeltaCAH = deltaCam_Analog_H[PlaneActive] - 128;
      if (CameraH_old != analogValue[4])
      {
        if (CamAnalogDelay != 0)
        {
          if (CameraH_old < analogValue[4]) CameraH_old += 11 - CamAnalogDelay;
          else CameraH_old -= 11 - CamAnalogDelay;
          AV = CameraH_old;
        }else AV = analogValue[4];
      } 
      if (AV < minCam_Analog_H*4) AV = minCam_Analog_H*4;
      if (AV > maxCam_Analog_H*4) AV = maxCam_Analog_H*4;
      if ((DeltaCAH) > 0)
      {
        if (AV < centerCam_Analog_H*4)
        {
        ppm_channels_1000_2000[5] = map(AV, minCam_Analog_H*4, centerCam_Analog_H*4, 1000 + (DeltaCAH)*2, 1500 + DeltaCAH);
        }else 
        {
          ppm_channels_1000_2000[5] = map(AV, centerCam_Analog_H*4, maxCam_Analog_H*4, 1500 + DeltaCAH, 2000);
        }
      }else
      {
        if (AV < centerCam_Analog_H*4)
        {
          ppm_channels_1000_2000[5] = map(AV, minCam_Analog_H*4, centerCam_Analog_H*4, 1000, 1500 + DeltaCAH);
        }else 
        {
          ppm_channels_1000_2000[5] = map(AV, centerCam_Analog_H*4, maxCam_Analog_H*4, 1500 + DeltaCAH, 2000 + (DeltaCAH)*2);
        }
      }
    }
    else if ((CamModeActive&(B00000001)) == 1) 
    {
      if (CameraUp == 0)
      {
        if (MenuItem_is_Change == "Trimmer Cam Digital V Up")
        ppm_channels_1000_2000[4] = 2000 - delta_maxCam_Digital_V[PlaneActive] + delta;
        else 
        {
          if (CameraV_old != 2000 - delta_maxCam_Digital_V[PlaneActive])
          {
            if (CamDigitalDelay != 0)
            {
              if (CameraV_old < 2000 - delta_maxCam_Digital_V[PlaneActive]) CameraV_old += 11 - CamDigitalDelay;
              if (CameraV_old > 2000 - delta_maxCam_Digital_V[PlaneActive]) ppm_channels_1000_2000[4] = 2000 - delta_maxCam_Digital_V[PlaneActive];
              else ppm_channels_1000_2000[4] = CameraV_old;
            }else ppm_channels_1000_2000[4] = 2000 - delta_maxCam_Digital_V[PlaneActive];
          }         
        }
      }else if (CameraDown == 0)
      {
        if (MenuItem_is_Change == "Trimmer Cam Digital V Down")
        ppm_channels_1000_2000[4] = 1000 + delta_minCam_Digital_V[PlaneActive] + delta;
        else 
        {
          if (CameraV_old != 1000 + delta_minCam_Digital_V[PlaneActive])
          {
            if (CamDigitalDelay != 0)
            {
              if (CameraV_old > 1000 + delta_minCam_Digital_V[PlaneActive]) CameraV_old -= 11 - CamDigitalDelay;
              if (CameraV_old < 1000 + delta_minCam_Digital_V[PlaneActive]) ppm_channels_1000_2000[4] = 1000 + delta_minCam_Digital_V[PlaneActive];
              else ppm_channels_1000_2000[4] = CameraV_old;
            }else ppm_channels_1000_2000[4] = 1000 + delta_minCam_Digital_V[PlaneActive];
          }         
        }
      }
      else if ((CameraUp == 1) && (CameraDown == 1))
      {  
        if (MenuItem_is_Change == "Trimmer Cam Digital V Center")
        ppm_channels_1000_2000[4] = 1500 + delta_centerCam_Digital_V[PlaneActive] - 128 + delta;
        else 
        {
          if (CameraV_old != 1500 + delta_centerCam_Digital_V[PlaneActive] - 128)
          {
            if (CamDigitalDelay != 0)
            {
              if (CameraV_old < 1500 - CamDigitalDelay + delta_centerCam_Digital_V[PlaneActive] - 128) CameraV_old += 11 - CamDigitalDelay;
              if (CameraV_old > 1500 + CamDigitalDelay + delta_centerCam_Digital_V[PlaneActive] - 128) CameraV_old -= 11 - CamDigitalDelay;
              ppm_channels_1000_2000[4] = CameraV_old;
            }else ppm_channels_1000_2000[4] = 1500 + delta_centerCam_Digital_V[PlaneActive] - 128;
          }         
        }
      }

      if (CameraRight == 0)
      {
        if (MenuItem_is_Change == "Trimmer Cam Digital H Up")
        ppm_channels_1000_2000[5] = 2000 - delta_maxCam_Digital_H[PlaneActive] + delta;
        else 
        {
          if (CameraH_old != 2000 - delta_maxCam_Digital_H[PlaneActive])
          {
            if (CamDigitalDelay != 0)
            {
              if (CameraH_old < 2000 - delta_maxCam_Digital_H[PlaneActive]) CameraH_old += 11 - CamDigitalDelay;
              if (CameraH_old > 2000 - delta_maxCam_Digital_H[PlaneActive]) ppm_channels_1000_2000[5] = 2000 - delta_maxCam_Digital_H[PlaneActive];
              else ppm_channels_1000_2000[5] = CameraH_old;
            }else ppm_channels_1000_2000[5] = 2000 - delta_maxCam_Digital_H[PlaneActive];
          }         
        }
      }else if (CameraLeft == 0)
      {
        if (MenuItem_is_Change == "Trimmer Cam Digital H Down")
        ppm_channels_1000_2000[5] = 1000 + delta_minCam_Digital_H[PlaneActive] + delta;
        else
        {
          if (CameraH_old != 1000 + delta_minCam_Digital_H[PlaneActive])
          {
            if (CamDigitalDelay != 0)
            {
              if (CameraH_old > 1000 + delta_minCam_Digital_H[PlaneActive]) CameraH_old -= 11 - CamDigitalDelay;
              if (CameraH_old < 1000 + delta_minCam_Digital_H[PlaneActive]) ppm_channels_1000_2000[5] = 1000 + delta_minCam_Digital_H[PlaneActive];
              else ppm_channels_1000_2000[5] = CameraH_old;
            }else ppm_channels_1000_2000[5] = 1000 + delta_minCam_Digital_H[PlaneActive];
          }         
        }
      }
      else if ((CameraRight == 1) && (CameraLeft == 1))
      {  
        if (MenuItem_is_Change == "Trimmer Cam Digital H Center")
        ppm_channels_1000_2000[5] = 1500 + delta_centerCam_Digital_H[PlaneActive] - 128 + delta;
        else
        {
          if (CameraH_old != 1500 + delta_centerCam_Digital_H[PlaneActive] - 128)
          {
            if (CamDigitalDelay != 0)
            {
              if (CameraH_old < 1500 - CamDigitalDelay + delta_centerCam_Digital_H[PlaneActive] - 128) CameraH_old += 11 - CamDigitalDelay;
              if (CameraH_old > 1500 + CamDigitalDelay + delta_centerCam_Digital_H[PlaneActive] - 128) CameraH_old -= 11 - CamDigitalDelay;
              ppm_channels_1000_2000[5] = CameraH_old;
            }else ppm_channels_1000_2000[5] = 1500 + delta_centerCam_Digital_H[PlaneActive] - 128;
          }         
        }
      } 
    }
//******************************************************************************************************
//******************************************************************************************************
//******************************************************************************************************
//******************************************************************************************************
    if (AUX1ModeActive == B00001000)
    {
      if (Trigger == 0)
      {
        ppm_channels_1000_2000[6] = Trigger_Digital_On[PlaneActive]*4 + 1000;//map(Aux1_Digital_I[PlaneActive]*4,0,1000,1000,2000);
      }
      else if(Trigger == 1)
      {
        ppm_channels_1000_2000[6] = Trigger_Digital_Off[PlaneActive]*4 + 1000;//map(Aux1_Digital_II[PlaneActive]*4,0,1000,1000,2000);
      }
    }
    else if (AUX1ModeActive == B00000100)
    {
      int DeltaAUX1;
      if (MenuItem_is_Change == "Trimmer Aux1 Analog Center")
      {
        DeltaAUX1 = deltaAux1Analog[PlaneActive] - 128 + delta;
      }else DeltaAUX1 = deltaAux1Analog[PlaneActive] - 128;
      AV = analogValue[3]; 
      if (AV < minAux1Analog*4) AV = minAux1Analog*4;
      if (AV > maxAux1Analog*4) AV = maxAux1Analog*4;
      if (DeltaAUX1 > 0)
      {
        if (AV < centerAux1Analog*4)
        {
        ppm_channels_1000_2000[6] = map(AV, minAux1Analog*4, centerAux1Analog*4, 1000 + (DeltaAUX1)*2, 1500 + DeltaAUX1);
        }else 
        {
          ppm_channels_1000_2000[6] = map(AV, centerAux1Analog*4, maxAux1Analog*4, 1500 + DeltaAUX1, 2000);
        }
      }else
      {
        if (AV < centerAux1Analog*4)
        {
          ppm_channels_1000_2000[6] = map(AV, minAux1Analog*4, centerAux1Analog*4, 1000, 1500 + DeltaAUX1);
        }else 
        {
          ppm_channels_1000_2000[6] = map(AV, centerAux1Analog*4, maxAux1Analog*4, 1500 + DeltaAUX1, 2000 + (DeltaAUX1)*2);
        }
      }
    }else if (AUX1ModeActive == B00000010)
    {
      int DeltaAUX1_;
      if (MenuItem_is_Change == "Trimmer Aux1 Analog Center")
      {
        DeltaAUX1_ = deltaAux1Analog_[PlaneActive] - 128 + delta;
      }else DeltaAUX1_ = deltaAux1Analog_[PlaneActive] - 128;
      AV = analogValue[1]; 
      if (AV < minAux1Analog_*4) AV = minAux1Analog_*4;
      if (AV > maxAux1Analog_*4) AV = maxAux1Analog_*4;
      if ((DeltaAUX1_) > 0)
      {
        if (AV < centerAux1Analog_*4)
        {
        ppm_channels_1000_2000[6] = map(AV, minAux1Analog_*4, centerAux1Analog_*4, 1000 + (DeltaAUX1_)*2, 1500 + DeltaAUX1_);
        }else 
        {
          ppm_channels_1000_2000[6] = map(AV, centerAux1Analog_*4, maxAux1Analog_*4, 1500 + DeltaAUX1_, 2000);
        }
      }else
      {
        if (AV < centerAux1Analog_*4)
        {
          ppm_channels_1000_2000[6] = map(AV, minAux1Analog_*4, centerAux1Analog_*4, 1000, 1500 + DeltaAUX1_);
        }else 
        {
          ppm_channels_1000_2000[6] = map(AV, centerAux1Analog_*4, maxAux1Analog_*4, 1500 + DeltaAUX1_, 2000 + (DeltaAUX1_)*2);
        }
      }
    }else if (AUX1ModeActive == B00000001)
    {
      if (digitalRead(14) == LOW)
      {
        ppm_channels_1000_2000[6] = Aux1_Digital_I[PlaneActive]*4 + 1000;//map(Aux1_Digital_I[PlaneActive]*4,0,1000,1000,2000);
      }
      else if(digitalRead(15) == LOW)
      {
        ppm_channels_1000_2000[6] = Aux1_Digital_II[PlaneActive]*4 + 1000;//map(Aux1_Digital_II[PlaneActive]*4,0,1000,1000,2000);
      }else if ((digitalRead(14) == HIGH) && (digitalRead(15) == HIGH))
      {
        ppm_channels_1000_2000[6] = Aux1_Digital_O[PlaneActive]*4 + 1000;//map(Aux1_Digital_O[PlaneActive]*4,0,1000,1000,2000);
      }  
    }

    if (AUX2ModeActive == B00000100)
    {
      if (Trigger == 0)
      {
        ppm_channels_1000_2000[7] = Trigger_Digital_On[PlaneActive]*4 + 1000;//map(Aux1_Digital_I[PlaneActive]*4,0,1000,1000,2000);
      }
      else if(Trigger == 1)
      {
        ppm_channels_1000_2000[7] = Trigger_Digital_Off[PlaneActive]*4 + 1000;//map(Aux1_Digital_II[PlaneActive]*4,0,1000,1000,2000);
      }
    }
    else if (AUX2ModeActive == B00000010)
    {
      AV = analogValue[2]; 
      if (AV < minAux2Analog*4) AV = minAux2Analog*4;
      if (AV > maxAux2Analog*4) AV = maxAux2Analog*4;
      if ((deltaAux2Analog[PlaneActive] - 128) > 0)
      {
        if (AV < centerAux2Analog*4)
        {
          ppm_channels_1000_2000[7] = map(AV, minAux2Analog*4, centerAux2Analog*4, 1000 + (deltaAux2Analog[PlaneActive] - 128)*2, 1500 + deltaAux2Analog[PlaneActive] - 128);
        }else 
        {
          ppm_channels_1000_2000[7] = map(AV, centerAux2Analog*4, maxAux2Analog*4, 1500 + deltaAux2Analog[PlaneActive] - 128, 2000);
        }
      }else
      {
        if (AV < centerAux2Analog*4)
        {
          ppm_channels_1000_2000[7] = map(AV, minAux2Analog*4, centerAux2Analog*4, 1000, 1500 + deltaAux2Analog[PlaneActive] - 128);
        }else 
        {
          ppm_channels_1000_2000[7] = map(AV, centerAux2Analog*4, maxAux2Analog*4, 1500 + deltaAux2Analog[PlaneActive] - 128, 2000 + (deltaAux2Analog[PlaneActive] - 128)*2);
        }
      }
    }else if (AUX2ModeActive == B00000001)
    {
      if (digitalRead(16) == LOW)
      {
        ppm_channels_1000_2000[7] =  Aux2_Digital_I[PlaneActive]*4 + 1000;//map(Aux2_Digital_I[PlaneActive]*4,0,1000,1000,2000);
      }
      else if(digitalRead(17) == LOW)
      {
        ppm_channels_1000_2000[7] = Aux2_Digital_II[PlaneActive]*4 + 1000;//map(Aux2_Digital_II[PlaneActive]*4,0,1000,1000,2000);
      }else if ((digitalRead(16) == HIGH) && (digitalRead(17) == HIGH))
      {
        ppm_channels_1000_2000[7] = Aux2_Digital_O[PlaneActive]*4 + 1000;//map(Aux2_Digital_O[PlaneActive]*4,0,1000,1000,2000);
      }  
    }
        
    if ((InvertChanelsPart1&(B10000000)) == (B10000000))
    ppm_channels_1000_2000[0] = 3000 - ppm_channels_1000_2000[0];
    if ((InvertChanelsPart1&(B01000000)) == (B01000000))
    ppm_channels_1000_2000[1] = 3000 - ppm_channels_1000_2000[1];
    if ((InvertChanelsPart1&(B00100000)) == (B00100000))
    ppm_channels_1000_2000[2] = 3000 - ppm_channels_1000_2000[2];
    if ((InvertChanelsPart1&(B00010000)) == (B00010000))
    ppm_channels_1000_2000[3] = 3000 - ppm_channels_1000_2000[3];
    if (CamModeActive == 1)
    {
      if ((InvertChanelsPart1&(B00001000)) == (B00001000))
      ppm_channels_1000_2000[4] = 3000 - ppm_channels_1000_2000[4];
      if ((InvertChanelsPart1&(B00000100)) == (B00000100))
      ppm_channels_1000_2000[5] = 3000 - ppm_channels_1000_2000[5];
    }else
    {
      if ((InvertChanelsPart1&(B00000010)) == (B00000010))
      ppm_channels_1000_2000[4] = 3000 - ppm_channels_1000_2000[4];
      if ((InvertChanelsPart1&(B00000001)) == (B00000001))
      ppm_channels_1000_2000[5] = 3000 - ppm_channels_1000_2000[5];
    }  
    if ((InvertChanelsPart2&(B10000000)) == (B10000000))
    ppm_channels_1000_2000[6] = 3000 - ppm_channels_1000_2000[6];
    if ((InvertChanelsPart2&(B01000000)) == (B01000000))
    ppm_channels_1000_2000[7] = 3000 - ppm_channels_1000_2000[7];

     if ((ExponentialChanels&(B10000000)) == (B10000000))
    ppm_channels_1000_2000[0] = 1500 + DeltaA + (500 - power - abs(DeltaA))*pow((ppm_channels_1000_2000[0]-(1500 + DeltaA))*1.0/(500 - power - abs(DeltaA)),3);
    if ((ExponentialChanels&(B01000000)) == (B01000000))
    ppm_channels_1000_2000[1] = 1500 + DeltaE + (500 - power - abs(DeltaE))*pow((ppm_channels_1000_2000[1]-(1500 + DeltaE))*1.0/(500 - power - abs(DeltaE)),3);
    if ((ExponentialChanels&(B00100000)) == (B00100000))
    ppm_channels_1000_2000[2] = 1500 + DeltaR + (500 - power - abs(DeltaR))*pow((ppm_channels_1000_2000[2]-(1500 + DeltaR))*1.0/(500 - power - abs(DeltaR)),3);
    if ((ExponentialChanels&(B00010000)) == (B00010000))
    ppm_channels_1000_2000[3] = 1000 + DeltaTmin + (1000 - DeltaTmin - DeltaTmax)*pow((ppm_channels_1000_2000[3]-(1000 + DeltaTmin))*1.0/(1000 - DeltaTmin - DeltaTmax),3);
    
    if (PlaneModeActive[PlaneActive] == 0)
    {
   //PPM остается прежним
   ppm_channels[0] = ppm_channels_1000_2000[0];
   ppm_channels[1] = ppm_channels_1000_2000[1];
   ppm_channels[3] = ppm_channels_1000_2000[2];
   ppm_channels[2] = ppm_channels_1000_2000[3]; //костыль чтобы убрать ошибку удлинения РРМ в 0,5%, иначе не всегда будет стартовать мотор
   ppm_channels[4] = ppm_channels_1000_2000[4];
   ppm_channels[5] = ppm_channels_1000_2000[5];
   ppm_channels[6] = ppm_channels_1000_2000[6];
   ppm_channels[7] = ppm_channels_1000_2000[7];
    }else if (PlaneModeActive[dueFlashStorage.read(84)] == 1)
    {
      int PPM_0, PPM_1;
      PPM_0 = (ppm_channels_1000_2000[0] + ppm_channels_1000_2000[1])/2;
      PPM_1 = (ppm_channels_1000_2000[1] + 3000 - ppm_channels_1000_2000[0])/2;
      ppm_channels[0] = PPM_0;
      ppm_channels[1] = PPM_1;
      ppm_channels[3] = ppm_channels_1000_2000[2];
      ppm_channels[2] = ppm_channels_1000_2000[3];
      ppm_channels[4] = ppm_channels_1000_2000[4];
      ppm_channels[5] = ppm_channels_1000_2000[5];
      ppm_channels[6] = ppm_channels_1000_2000[6];
      ppm_channels[7] = ppm_channels_1000_2000[7];
     //Остальные каналы остаются без изменений
    }else if (PlaneModeActive[dueFlashStorage.read(84)] == 2)
    {
      int PPM_1, PPM_2;
      PPM_1 = (ppm_channels_1000_2000[1] + ppm_channels_1000_2000[2])/2;
      PPM_2 = (ppm_channels_1000_2000[1] + 3000 - ppm_channels_1000_2000[2])/2;
      ppm_channels[0] = ppm_channels_1000_2000[0];
      ppm_channels[1] = PPM_1;
      ppm_channels[3] = PPM_2;
      ppm_channels[2] = ppm_channels_1000_2000[3];
      ppm_channels[4] = ppm_channels_1000_2000[4];
      ppm_channels[5] = ppm_channels_1000_2000[5];
      ppm_channels[6] = ppm_channels_1000_2000[6];
      ppm_channels[7] = ppm_channels_1000_2000[7];
    }  

    for (int i = 0; i < 8; i++)
    {
      if ((ppm_channels[i]<1000) || (ppm_channels[i]>2000))
      {
        ppm_channels[i] = 1500;
      }
    }

int Sum = 0;
for (int i = 0; i < 8; i++) {
    //periods[i] = map(PPMValue[i], 0, 1023, 42000, 84000);
    periods[i] = map(ppm_channels[i], 1000, 2000, 42000, 84000);
    Sum = Sum + periods[i];
  }
  // 3. Calculate the sync frame
  periods[8] = PPMFrame - Sum;
//------------------  ------------------------------------------------------------------
}

