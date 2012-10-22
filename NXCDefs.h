Main Page Related Pages Modules Data Structures Files Examples File List Globals NXCDefs.h
Go to the documentation of this file.
00001 
00028 #ifndef NXCDEFS_H
00029 #define NXCDEFS_H
00030 
00031 #include "NBCCommon.h"
00032 
00040 #define u8 unsigned char  
00041 #define s8 char           
00042 #define u16 unsigned int  
00043 #define s16 int           
00044 #define u32 unsigned long 
00045 #define s32 long            // end of TypeAliases group
00047   // end of MiscConstants group
00048 
00049 
00053 
00054 
00068 #define S1 0 
00069 #define S2 1 
00070 #define S3 2 
00071 #define S4 3  // end of InPorts group
00073 
00083 #define SENSOR_TYPE_NONE            IN_TYPE_NO_SENSOR      
00084 #define SENSOR_TYPE_TOUCH           IN_TYPE_SWITCH         
00085 #define SENSOR_TYPE_TEMPERATURE     IN_TYPE_TEMPERATURE    
00086 #define SENSOR_TYPE_LIGHT           IN_TYPE_REFLECTION     
00087 #define SENSOR_TYPE_ROTATION        IN_TYPE_ANGLE          
00088 #define SENSOR_TYPE_LIGHT_ACTIVE    IN_TYPE_LIGHT_ACTIVE   
00089 #define SENSOR_TYPE_LIGHT_INACTIVE  IN_TYPE_LIGHT_INACTIVE 
00090 #define SENSOR_TYPE_SOUND_DB        IN_TYPE_SOUND_DB       
00091 #define SENSOR_TYPE_SOUND_DBA       IN_TYPE_SOUND_DBA      
00092 #define SENSOR_TYPE_CUSTOM          IN_TYPE_CUSTOM         
00093 #define SENSOR_TYPE_LOWSPEED        IN_TYPE_LOWSPEED       
00094 #define SENSOR_TYPE_LOWSPEED_9V     IN_TYPE_LOWSPEED_9V    
00095 #define SENSOR_TYPE_HIGHSPEED       IN_TYPE_HISPEED        
00096 #if __FIRMWARE_VERSION > 107
00097 #define SENSOR_TYPE_COLORFULL       IN_TYPE_COLORFULL      
00098 #define SENSOR_TYPE_COLORRED        IN_TYPE_COLORRED       
00099 #define SENSOR_TYPE_COLORGREEN      IN_TYPE_COLORGREEN     
00100 #define SENSOR_TYPE_COLORBLUE       IN_TYPE_COLORBLUE      
00101 #define SENSOR_TYPE_COLORNONE       IN_TYPE_COLORNONE      
00102 #endif
00103  // end of SensorTypes group
00104 
00111 #define SENSOR_MODE_RAW         IN_MODE_RAW           
00112 #define SENSOR_MODE_BOOL        IN_MODE_BOOLEAN       
00113 #define SENSOR_MODE_EDGE        IN_MODE_TRANSITIONCNT 
00114 #define SENSOR_MODE_PULSE       IN_MODE_PERIODCOUNTER 
00115 #define SENSOR_MODE_PERCENT     IN_MODE_PCTFULLSCALE  
00116 #define SENSOR_MODE_CELSIUS     IN_MODE_CELSIUS       
00117 #define SENSOR_MODE_FAHRENHEIT  IN_MODE_FAHRENHEIT    
00118 #define SENSOR_MODE_ROTATION    IN_MODE_ANGLESTEP      // end of SensorModes group
00120 
00127 #define _SENSOR_CFG(_type,_mode)        (((_type)<<8)+(_mode))                               
00128 #define SENSOR_TOUCH            _SENSOR_CFG(SENSOR_TYPE_TOUCH, SENSOR_MODE_BOOL)             
00129 #define SENSOR_LIGHT            _SENSOR_CFG(SENSOR_TYPE_LIGHT, SENSOR_MODE_PERCENT)          
00130 #define SENSOR_ROTATION         _SENSOR_CFG(SENSOR_TYPE_ROTATION, SENSOR_MODE_ROTATION)      
00131 #define SENSOR_CELSIUS          _SENSOR_CFG(SENSOR_TYPE_TEMPERATURE, SENSOR_MODE_CELSIUS)    
00132 #define SENSOR_FAHRENHEIT       _SENSOR_CFG(SENSOR_TYPE_TEMPERATURE, SENSOR_MODE_FAHRENHEIT) 
00133 #define SENSOR_PULSE            _SENSOR_CFG(SENSOR_TYPE_TOUCH, SENSOR_MODE_PULSE)            
00134 #define SENSOR_EDGE         _SENSOR_CFG(SENSOR_TYPE_TOUCH, SENSOR_MODE_EDGE)             
00135 #define SENSOR_NXTLIGHT         _SENSOR_CFG(SENSOR_TYPE_LIGHT_ACTIVE, SENSOR_MODE_PERCENT)   
00136 #define SENSOR_SOUND            _SENSOR_CFG(SENSOR_TYPE_SOUND_DB, SENSOR_MODE_PERCENT)       
00137 #define SENSOR_LOWSPEED_9V  _SENSOR_CFG(SENSOR_TYPE_LOWSPEED_9V, SENSOR_MODE_RAW)        
00138 #define SENSOR_LOWSPEED     _SENSOR_CFG(SENSOR_TYPE_LOWSPEED, SENSOR_MODE_RAW)           
00139 #if __FIRMWARE_VERSION > 107
00140 #define SENSOR_COLORFULL        _SENSOR_CFG(SENSOR_TYPE_COLORFULL, SENSOR_MODE_RAW)          
00141 #define SENSOR_COLORRED         _SENSOR_CFG(SENSOR_TYPE_COLORRED, SENSOR_MODE_PERCENT)       
00142 #define SENSOR_COLORGREEN       _SENSOR_CFG(SENSOR_TYPE_COLORGREEN, SENSOR_MODE_PERCENT)     
00143 #define SENSOR_COLORBLUE        _SENSOR_CFG(SENSOR_TYPE_COLORBLUE, SENSOR_MODE_PERCENT)      
00144 #define SENSOR_COLORNONE        _SENSOR_CFG(SENSOR_TYPE_COLORNONE, SENSOR_MODE_PERCENT)      
00145 #endif
00146  // end of SensorModes group // end of InputModuleTypesAndModes group // end of InputModuleConstants group
00149 
00154 #if __FIRMWARE_VERSION > 107
00155 
00163 struct ColorSensorReadType {
00164  char Result;                    
00165  byte Port;                      
00166  int ColorValue;                 
00167  unsigned int RawArray[];        
00168  unsigned int NormalizedArray[]; 
00169  int ScaledArray[];              
00170  bool Invalid;                   
00171 };
00172 #endif
00173 
00174 #if defined(__ENHANCED_FIRMWARE) && (__FIRMWARE_VERSION > 107)
00175 
00181 struct InputValuesType {
00182   byte Port;                    
00183   bool Valid;                   
00184   bool Calibrated;              
00185   byte SensorType;              
00186   byte SensorMode;              
00187   unsigned int RawValue;        
00188   unsigned int NormalizedValue; 
00189   int ScaledValue;              
00190   int CalibratedValue;          
00191 };
00192 
00193 /*
00194 struct InputType {
00195   unsigned int CustomZeroOffset;
00196   unsigned int ADRaw;
00197   unsigned int SensorRaw;
00198   int SensorValue;
00199   byte SensorType;
00200   byte SensorMode;
00201   bool SensorBoolean;
00202   byte DigiPinsDir;
00203   byte DigiPinsIn;
00204   byte DigiPinsOut;
00205   byte CustomPctFullScale;
00206   byte CustomActiveStatus;
00207   bool InvalidData;
00208 };
00209 */
00210 
00211 #endif
00212  // end of InputModuleTypes group
00214 
00225 #define SENSOR_1 Sensor(S1) 
00226 #define SENSOR_2 Sensor(S2) 
00227 #define SENSOR_3 Sensor(S3) 
00228 #define SENSOR_4 Sensor(S4)  // end of BasicSensorValues group
00230 
00241 inline void SetSensorType(const byte & port, byte type) { asm { setin type, port, TypeField } }
00242 
00254 inline void SetSensorMode(const byte & port, byte mode) { asm { setin mode, port, InputModeField } }
00255 
00262 inline void ClearSensor(const byte & port) { asm { setin 0, port, ScaledValueField } }
00263 
00272 inline void ResetSensor(const byte & port) { asm { __ResetSensor(port) } }
00273 
00283 inline void SetSensor(const byte & port, const unsigned int config) {
00284   asm {
00285     setin config>>8, port, TypeField
00286     setin config&0xff, port, InputModeField
00287     __ResetSensor(port)
00288   }
00289 }
00290 
00296 inline void SetSensorTouch(const byte & port) { asm { __SetSensorTouch(port) } }
00297 
00306 inline void SetSensorLight(const byte & port, bool bActive = true) {
00307   SetSensorType(port, bActive ? SENSOR_TYPE_LIGHT_ACTIVE : SENSOR_TYPE_LIGHT_INACTIVE);
00308   SetSensorMode(port, SENSOR_MODE_PERCENT);
00309   ResetSensor(port);
00310 }
00311 
00320 inline void SetSensorSound(const byte & port, bool bdBScaling = true) {
00321   SetSensorType(port, bdBScaling ? SENSOR_TYPE_SOUND_DB : SENSOR_TYPE_SOUND_DBA);
00322   SetSensorMode(port, SENSOR_MODE_PERCENT);
00323   ResetSensor(port);
00324 }
00325 
00335 inline void SetSensorLowspeed(const byte & port, bool bIsPowered = true) {
00336   SetSensorType(port, bIsPowered ? SENSOR_TYPE_LOWSPEED_9V : SENSOR_TYPE_LOWSPEED);
00337   SetSensorMode(port, SENSOR_MODE_RAW);
00338   ResetSensor(port);
00339 }
00340 
00346 inline void SetSensorUltrasonic(const byte & port) { SetSensorLowspeed(port); }
00347 
00353 inline void SetSensorEMeter(const byte & port) { SetSensorLowspeed(port); }
00354 
00362 inline void SetSensorTemperature(const byte & port) {
00363   SetSensorLowspeed(port);
00364   asm {
00365     __MSWriteToRegister(port, LEGO_ADDR_TEMP, TEMP_REG_CONFIG, TEMP_RES_12BIT, __WDSC_LSStatus)
00366   }
00367 }
00368 
00369 
00370 #if __FIRMWARE_VERSION > 107
00371 
00380 inline void SetSensorColorFull(const byte & port) { asm { __SetSensorColorFull(port) } }
00381 
00390 inline void SetSensorColorRed(const byte & port) { asm { __SetSensorColorRed(port) } }
00391 
00400 inline void SetSensorColorGreen(const byte & port) { asm { __SetSensorColorGreen(port) } }
00401 
00410 inline void SetSensorColorBlue(const byte & port) { asm { __SetSensorColorBlue(port) } }
00411 
00420 inline void SetSensorColorNone(const byte & port) { asm { __SetSensorColorNone(port) } }
00421 
00422 #endif
00423 
00424 #ifdef __DOXYGEN_DOCS
00425 
00435 inline variant GetInput(const byte & port, const byte field);
00436 
00447 inline void SetInput(const byte & port, const int field, variant value);
00448 
00459 inline unsigned int Sensor(const byte & port);
00460 
00470 inline bool SensorBoolean(const byte port);
00471 
00479 inline byte SensorDigiPinsDirection(const byte port);
00480 
00488 inline byte SensorDigiPinsOutputLevel(const byte port);
00489 
00497 inline byte SensorDigiPinsStatus(const byte port);
00498 
00507 inline bool SensorInvalid(const byte & port);
00508 
00517 inline byte SensorMode(const byte & port);
00518 
00527 inline unsigned int SensorNormalized(const byte & port);
00528 
00537 inline unsigned int SensorRaw(const byte & port);
00538 
00549 inline unsigned int SensorScaled(const byte & port);
00550 
00559 inline byte SensorType(const byte & port);
00560 
00571 inline unsigned int SensorValue(const byte & port);
00572 
00582 inline bool SensorValueBool(const byte port);
00583 
00592 inline unsigned int SensorValueRaw(const byte & port);
00593 
00601 inline byte CustomSensorActiveStatus(byte port);
00602 
00610 inline byte CustomSensorPercentFullScale(byte port);
00611 
00619 inline unsigned int CustomSensorZeroOffset(byte port);
00620 
00628 inline void SetCustomSensorActiveStatus(byte port, byte activeStatus);
00629 
00637 inline void SetCustomSensorPercentFullScale(byte port, byte pctFullScale);
00638 
00646 inline void SetCustomSensorZeroOffset(byte port, int zeroOffset);
00647 
00655 inline void SetSensorBoolean(byte port, bool value);
00656 
00664 inline void SetSensorDigiPinsDirection(byte port, byte direction);
00665 
00673 inline void SetSensorDigiPinsOutputLevel(byte port, byte outputLevel);
00674 
00682 inline void SetSensorDigiPinsStatus(byte port, byte status);
00683 
00684 
00685 #if __FIRMWARE_VERSION > 107
00686 
00695 inline void SysColorSensorRead(ColorSensorReadType & args);
00696 
00711 inline int ReadSensorColorEx(const byte & port, int & colorval, unsigned int & raw[], unsigned int & norm[], int & scaled[]);
00712 
00723 inline int ReadSensorColorRaw(const byte & port, unsigned int & rawVals[]);
00724 
00735 inline unsigned int ColorADRaw(byte port, byte color);
00736 
00747 inline bool ColorBoolean(byte port, byte color);
00748 
00760 inline long ColorCalibration(byte port, byte point, byte color);
00761 
00771 inline byte ColorCalibrationState(byte port);
00772 
00783 inline unsigned int ColorCalLimits(byte port, byte point);
00784 
00795 inline unsigned int ColorSensorRaw(byte port, byte color);
00796 
00807 inline unsigned int ColorSensorValue(byte port, byte color);
00808 
00809 #endif
00810 
00811 #else
00812 
00813 enum InputFieldNames {
00814   Type,
00815   InputMode,
00816   RawValue,
00817   NormalizedValue,
00818   ScaledValue,
00819   InvalidData
00820 };
00821 
00822 enum OutputFieldNames {
00823   UpdateFlags,
00824   OutputMode,
00825   Power,
00826   ActualSpeed,
00827   TachoCount,
00828   TachoLimit,
00829   RunState,
00830   TurnRatio,
00831   RegMode,
00832   Overload,
00833   RegPValue,
00834   RegIValue,
00835   RegDValue,
00836   BlockTachoCount,
00837   RotationCount,
00838   OutputOptions,
00839   MaxSpeed,
00840   MaxAcceleration
00841 };
00842 
00843 // input fields
00844 #define Sensor(_p) asm { ReadSensor(_p, __RETVAL__) }
00845 #define SensorValue(_p) Sensor(_p)
00846 #define SensorType(_p) GetInput(_p, TypeField)
00847 #define SensorMode(_p) GetInput(_p, InputModeField)
00848 #define SensorRaw(_p) GetInput(_p, RawValueField)
00849 #define SensorNormalized(_p) GetInput(_p, NormalizedValueField)
00850 #define SensorScaled(_p) GetInput(_p, ScaledValueField)
00851 #define SensorInvalid(_p) GetInput(_p, InvalidDataField)
00852 #define SensorValueBool(_p) SensorBoolean(_p)
00853 #define SensorValueRaw(_p) SensorRaw(_p)
00854 
00855 #define CustomSensorZeroOffset(_p) asm { GetInCustomZeroOffset(_p, __TMPWORD__) __RETURN__ __TMPWORD__ }
00856 #define CustomSensorPercentFullScale(_p) asm { GetInCustomPercentFullScale(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
00857 #define CustomSensorActiveStatus(_p) asm { GetInCustomActiveStatus(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
00858 #define SensorBoolean(_p) asm { GetInSensorBoolean(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
00859 #define SensorDigiPinsDirection(_p) asm { GetInDigiPinsDirection(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
00860 #define SensorDigiPinsStatus(_p) asm { GetInDigiPinsStatus(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
00861 #define SensorDigiPinsOutputLevel(_p) asm { GetInDigiPinsOutputLevel(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
00862 
00863 #define SetCustomSensorZeroOffset(_p, _n) asm { __setInCustomZeroOffset(_p, _n) }
00864 #define SetCustomSensorPercentFullScale(_p, _n) asm { __setInCustomPercentFullScale(_p, _n) }
00865 #define SetCustomSensorActiveStatus(_p, _n) asm { __setInCustomActiveStatus(_p, _n) }
00866 #define SetSensorBoolean(_p, _n) asm { __setInSensorBoolean(_p, _n) }
00867 #define SetSensorDigiPinsDirection(_p, _n) asm { __setInDigiPinsDirection(_p, _n) }
00868 #define SetSensorDigiPinsStatus(_p, _n) asm { __setInDigiPinsStatus(_p, _n) }
00869 #define SetSensorDigiPinsOutputLevel(_p, _n) asm { __setInDigiPinsOutputLevel(_p, _n) }
00870 
00871 
00872 #if __FIRMWARE_VERSION > 107
00873 
00874 #define SysColorSensorRead(_args) asm { \
00875   compchktype _args, ColorSensorReadType \
00876   syscall ColorSensorRead, _args \
00877 }
00878 
00879 #define ReadSensorColorRaw(_port, _rawVals) asm { __ReadSensorColorRaw(_port, _rawVals, __RETVAL__) }
00880 #define ReadSensorColorEx(_port, _colorval, _raw, _norm, _scaled) asm { __ReadSensorColorEx(_port, _colorval, _raw, _norm, _scaled, __RETVAL__) }
00881 
00882 #define ColorCalibration(_p, _np, _nc) asm { GetInColorCalibration(_p, _np, _nc, __TMPLONG__) __RETURN__ __TMPLONG__ }
00883 #define ColorCalLimits(_p, _np) asm { GetInColorCalLimits(_p, _np, __TMPWORD__) __RETURN__ __TMPWORD__ }
00884 #define ColorADRaw(_p, _nc) asm { GetInColorADRaw(_p, _nc, __TMPWORD__) __RETURN__ __TMPWORD__ }
00885 #define ColorSensorRaw(_p, _nc) asm { GetInColorSensorRaw(_p, _nc, __TMPWORD__) __RETURN__ __TMPWORD__ }
00886 #define ColorSensorValue(_p, _nc) asm { GetInColorSensorValue(_p, _nc, __TMPWORD__) __RETURN__ __TMPWORD__ }
00887 #define ColorBoolean(_p, _nc) asm { GetInColorBoolean(_p, _nc, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
00888 #define ColorCalibrationState(_p) asm { GetInColorCalibrationState(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
00889 
00890 #endif
00891 
00892 #endif
00893  // end of InputModuleFunctions group // end of InputModule group // end of NXTFirmwareModules group
00896 
00897 
00901 
00902 
00914 #if defined(__ENHANCED_FIRMWARE) && (__FIRMWARE_VERSION > 107)
00915 
00921 struct OutputStateType {
00922   byte Port;                
00923   char Power;               
00924   byte Mode;                
00925   byte RegMode;             
00926   char TurnRatio;           
00927   byte RunState;            
00928   unsigned long TachoLimit; 
00929   long TachoCount;          
00930   long BlockTachoCount;     
00931   long RotationCount;       
00932 };
00933 
00934 #endif
00935  // end of OutputModuleTypes group
00937 
00943 #ifdef __DOXYGEN_DOCS
00944 
00951 inline void SetMotorPwnFreq(byte n);
00952 
00959 inline void SetMotorRegulationTime(byte n);
00960 
00966 inline void SetMotorRegulationOptions(byte n);
00967 
00988 inline void OnFwdSyncPID(byte outputs, char pwr, char turnpct, byte p, byte i, byte d);
00989 
01012 inline void OnFwdSyncExPID(byte outputs, char pwr, char turnpct, const byte reset, byte p, byte i, byte d);
01013 
01034 inline void OnRevSyncPID(byte outputs, char pwr, char turnpct, byte p, byte i, byte d);
01035 
01058 inline void OnRevSyncExPID(byte outputs, char pwr, char turnpct, const byte reset, byte p, byte i, byte d);
01059 
01078 inline void OnFwdRegPID(byte outputs, char pwr, byte regmode, byte p, byte i, byte d);
01079 
01100 inline void OnFwdRegExPID(byte outputs, char pwr, byte regmode, const byte reset, byte p, byte i, byte d);
01101 
01120 inline void OnRevRegPID(byte outputs, char pwr, byte regmode, byte p, byte i, byte d);
01121 
01142 inline void OnRevRegExPID(byte outputs, char pwr, byte regmode, const byte reset, byte p, byte i, byte d);
01143 
01153 inline void Off(byte outputs);
01154 
01166 inline void OffEx(byte outputs, const byte reset);
01167 
01177 inline void Coast(byte outputs);
01178 
01190 inline void CoastEx(byte outputs, const byte reset);
01191 
01201 inline void Float(byte outputs);
01202 
01213 inline void OnFwd(byte outputs, char pwr);
01214 
01227 inline void OnFwdEx(byte outputs, char pwr, const byte reset);
01228 
01239 inline void OnRev(byte outputs, char pwr);
01240 
01253 inline void OnRevEx(byte outputs, char pwr, const byte reset);
01254 
01266 inline void OnFwdReg(byte outputs, char pwr, byte regmode);
01267 
01281 inline void OnFwdRegEx(byte outputs, char pwr, byte regmode, const byte reset);
01282 
01294 inline void OnRevReg(byte outputs, char pwr, byte regmode);
01295 
01309 inline void OnRevRegEx(byte outputs, char pwr, byte regmode, const byte reset);
01310 
01324 inline void OnFwdSync(byte outputs, char pwr, char turnpct);
01325 
01341 inline void OnFwdSyncEx(byte outputs, char pwr, char turnpct, const byte reset);
01342 
01356 inline void OnRevSync(byte outputs, char pwr, char turnpct);
01357 
01373 inline void OnRevSyncEx(byte outputs, char pwr, char turnpct, const byte reset);
01374 
01386 inline void RotateMotor(byte outputs, char pwr, long angle);
01387 
01406 inline void RotateMotorPID(byte outputs, char pwr, long angle, byte p, byte i, byte d);
01407 
01425 inline void RotateMotorEx(byte outputs, char pwr, long angle, char turnpct, bool sync, bool stop);
01426 
01451 inline void RotateMotorExPID(byte outputs, char pwr, long angle, char turnpct, bool sync, bool stop, byte p, byte i, byte d);
01452 
01463 inline void ResetTachoCount(byte outputs);
01464 
01474 inline void ResetBlockTachoCount(byte outputs);
01475 
01485 inline void ResetRotationCount(byte outputs);
01486 
01497 inline void ResetAllTachoCounts(byte outputs);
01498 
01516 inline void SetOutput(byte outputs, byte field1, variant val1, ..., byte fieldN, variant valN);
01517 
01529 inline variant GetOutput(byte output, const byte field);
01530 
01540 inline byte MotorMode(byte output);
01541 
01551 inline char MotorPower(byte output);
01552 
01562 inline char MotorActualSpeed(byte output);
01563 
01573 inline long MotorTachoCount(byte output);
01574 
01584 inline long MotorTachoLimit(byte output);
01585 
01596 inline byte MotorRunState(byte output);
01597 
01607 inline char MotorTurnRatio(byte output);
01608 
01618 inline byte MotorRegulation(byte output);
01619 
01629 inline bool MotorOverload(byte output);
01630 
01640 inline byte MotorRegPValue(byte output);
01641 
01651 inline byte MotorRegIValue(byte output);
01652 
01662 inline byte MotorRegDValue(byte output);
01663 
01673 inline long MotorBlockTachoCount(byte output);
01674 
01684 inline long MotorRotationCount(byte output);
01685 
01695 inline byte MotorOutputOptions(byte output);
01696 
01706 inline byte MotorMaxSpeed(byte output);
01707 
01717 inline byte MotorMaxAcceleration(byte output);
01718 
01724 inline byte MotorPwnFreq();
01725 
01731 inline byte MotorRegulationTime();
01732 
01738 inline byte MotorRegulationOptions();
01739 
01740 #else
01741 
01742 // output fields
01743 #define MotorMode(_p) GetOutput(_p, OutputMode)
01744 #define MotorPower(_p) GetOutput(_p, Power)
01745 #define MotorActualSpeed(_p) GetOutput(_p, ActualSpeed)
01746 #define MotorTachoCount(_p) GetOutput(_p, TachoCount)
01747 #define MotorTachoLimit(_p) GetOutput(_p, TachoLimit)
01748 #define MotorRunState(_p) GetOutput(_p, RunState)
01749 #define MotorTurnRatio(_p) GetOutput(_p, TurnRatio)
01750 #define MotorRegulation(_p) GetOutput(_p, RegMode)
01751 #define MotorOverload(_p) GetOutput(_p, Overload)
01752 #define MotorRegPValue(_p) GetOutput(_p, RegPValue)
01753 #define MotorRegIValue(_p) GetOutput(_p, RegIValue)
01754 #define MotorRegDValue(_p) GetOutput(_p, RegDValue)
01755 #define MotorBlockTachoCount(_p) GetOutput(_p, BlockTachoCount)
01756 #define MotorRotationCount(_p) GetOutput(_p, RotationCount)
01757 #define MotorOutputOptions(_p) GetOutput(_p, OutputOptions)
01758 #define MotorMaxSpeed(_p) GetOutput(_p, MaxSpeed)
01759 #define MotorMaxAcceleration(_p) GetOutput(_p, MaxAcceleration)
01760 
01761 #define MotorPwnFreq() asm { GetOutPwnFreq(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
01762 #define MotorRegulationTime() asm { GetOutRegulationTime(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
01763 #define MotorRegulationOptions() asm { GetOutRegulationOptions(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
01764 
01765 #define SetMotorPwnFreq(_n) asm { __setOutPwnFreq(_n) }
01766 #define SetMotorRegulationTime(_n) asm { __setOutRegulationTime(_n) }
01767 #define SetMotorRegulationOptions(_n) asm { __setOutRegulationOptions(_n) }
01768 
01769 #endif
01770  // end of OutputModuleFunctions group // end of OutputModule group // end of NXTFirmwareModules group
01774 
01775 
01779 
01780 
01799 struct LocationType {
01800   int X;  
01801   int Y;  
01803 };
01804 
01811 struct SizeType {
01812   int Width;  
01813   int Height; 
01814 };
01815 
01824 struct DrawTextType {
01825   char Result;             
01826   LocationType Location;   
01827   string Text;             
01828   unsigned long Options;   
01830 };
01831 
01841 struct DrawPointType {
01842   char Result;             
01843   LocationType Location;   
01844   unsigned long Options;   
01846 };
01847 
01857 struct DrawLineType {
01858   char Result;             
01859   LocationType StartLoc;   
01860   LocationType EndLoc;     
01861   unsigned long Options;   
01863 };
01864 
01874 struct DrawCircleType {
01875   char Result;             
01876   LocationType Center;     
01877   byte Size;               
01878   unsigned long Options;   
01880 };
01881 
01891 struct DrawRectType {
01892   char Result;             
01893   LocationType Location;   
01894   SizeType Size;           
01895   unsigned long Options;   
01897 };
01898 
01909 struct DrawGraphicType {
01910   char Result;             
01912   LocationType Location;   
01913   string Filename;         
01914   long Variables[];         
01915   unsigned long Options;   
01917 };
01918 
01925 struct SetScreenModeType {
01926   char Result;                
01927   unsigned long ScreenMode;   
01936 };
01937 
01938 #ifdef __ENHANCED_FIRMWARE
01939 
01971 struct DisplayExecuteFunctionType {
01972   byte Status;   
01973   byte Cmd;      
01974   bool On;       
01975   byte X1;       
01976   byte Y1;       
01977   byte X2;       
01978   byte Y2;       
01979 };
01980 
01981 #if __FIRMWARE_VERSION > 107
01982 
01992 struct DrawGraphicArrayType {
01993   char Result;            
01994   LocationType Location;  
01995   byte Data[];            
01996   long Variables[];       
01997   unsigned long Options;  
01998 };
01999 
02009 struct DrawPolygonType {
02010   char Result;            
02011   LocationType Points[];  
02012   unsigned long Options;  
02013 };
02014 
02024 struct DrawEllipseType {
02025   char Result;            
02026   LocationType Center;    
02027   byte SizeX;             
02028   byte SizeY;             
02029   unsigned long Options;  
02030 };
02031 
02040 struct DrawFontType {
02041   char Result;             
02042   LocationType Location;   
02043   string Filename;         
02044   string Text;             
02045   unsigned long Options;   
02047 };
02048 #endif
02049 #endif
02050  // end of DisplayModuleTypes group
02051 
02057 #ifdef __DOXYGEN_DOCS
02058 
02063 inline void ResetScreen();
02064 
02079 inline char CircleOut(int x, int y, byte radius, unsigned long options=DRAW_OPT_NORMAL);
02080 
02096 inline char LineOut(int x1, int y1, int x2, int y2, unsigned long options=DRAW_OPT_NORMAL);
02097 
02111 inline char PointOut(int x, int y, unsigned long options=DRAW_OPT_NORMAL);
02112 
02129 inline char RectOut(int x, int y, int width, int height, unsigned long options=DRAW_OPT_NORMAL);
02130 
02147 inline char TextOut(int x, int y, string str, unsigned long options=DRAW_OPT_NORMAL);
02148 
02165 inline char NumOut(int x, int y, variant value, unsigned long options=DRAW_OPT_NORMAL);
02166 
02184 inline char EllipseOut(int x, int y, byte radiusX, byte radiusY, unsigned long options=DRAW_OPT_NORMAL);
02185 
02200 inline char PolyOut(LocationType points[], unsigned long options=DRAW_OPT_NORMAL);
02201 
02221 inline char FontTextOut(int x, int y, string filename, string str, unsigned long options=DRAW_OPT_NORMAL);
02222 
02242 inline char FontNumOut(int x, int y, string filename, variant value, unsigned long options=DRAW_OPT_NORMAL);
02243 
02259 inline char GraphicOut(int x, int y, string filename, unsigned long options=DRAW_OPT_NORMAL);
02260 
02276 inline char GraphicArrayOut(int x, int y, byte data[], unsigned long options=DRAW_OPT_NORMAL);
02277 
02295 inline char GraphicOutEx(int x, int y, string filename, byte vars[], unsigned long options=DRAW_OPT_NORMAL);
02296 
02314 inline char GraphicArrayOutEx(int x, int y, byte data[], byte vars[], unsigned long options=DRAW_OPT_NORMAL);
02315 
02329 inline void GetDisplayNormal(const byte x, const byte line, unsigned int cnt, byte & data[]);
02330 
02344 inline void SetDisplayNormal(const byte x, const byte line, unsigned int cnt, byte data[]);
02345 
02359 inline void GetDisplayPopup(const byte x, const byte line, unsigned int cnt, byte & data[]);
02360 
02374 inline void SetDisplayPopup(const byte x, const byte line, unsigned int cnt, byte data[]);
02375 
02381 inline unsigned long DisplayEraseMask();
02382 
02383 
02389 inline unsigned long DisplayUpdateMask();
02390 
02396 inline unsigned long DisplayFont();
02397 
02403 inline unsigned long DisplayDisplay();
02404 
02411 inline byte DisplayFlags();
02412 
02418 inline byte DisplayTextLinesCenterFlags();
02419 
02427 inline void SysDrawText(DrawTextType & args);
02428 
02436 inline void SysDrawPoint(DrawPointType & args);
02437 
02445 inline void SysDrawLine(DrawLineType & args);
02446 
02454 inline void SysDrawCircle(DrawCircleType & args);
02455 
02463 inline void SysDrawRect(DrawRectType & args);
02464 
02472 inline void SysDrawGraphic(DrawGraphicType & args);
02473 
02481 inline void SysSetScreenMode(SetScreenModeType & args);
02482 
02483 #ifdef __ENHANCED_FIRMWARE
02484 
02493 inline void SysDisplayExecuteFunction(DisplayExecuteFunctionType & args);
02494 
02495 
02496 #if __FIRMWARE_VERSION > 107
02497 
02505 inline byte DisplayContrast();
02506 
02516 inline void SysDrawGraphicArray(DrawGraphicArrayType & args);
02517 
02527 inline void SysDrawPolygon(DrawPolygonType & args);
02528 
02538 inline void SysDrawEllipse(DrawEllipseType & args);
02539 
02549 inline void SysDrawFont(DrawFontType & args);
02550 
02551 #endif
02552 #endif
02553 
02554 #else
02555 
02556 #define GetDisplayNormal(_x, _line, _cnt, _data) asm { __getDisplayNormal(_x, _line, _cnt, _data) }
02557 #define GetDisplayPopup(_x, _line, _cnt, _data) asm { __getDisplayPopup(_x, _line, _cnt, _data) }
02558 
02559 #define DisplayEraseMask() asm { GetDisplayEraseMask(__TMPLONG__) __RETURN__ __TMPLONG__ }
02560 #define DisplayUpdateMask() asm { GetDisplayUpdateMask(__TMPLONG__) __RETURN__ __TMPLONG__ }
02561 #define DisplayFont() asm { GetDisplayFont(__TMPLONG__) __RETURN__ __TMPLONG__ }
02562 #define DisplayDisplay() asm { GetDisplayDisplay(__TMPLONG__) __RETURN__ __TMPLONG__ }
02563 #define DisplayFlags() asm { GetDisplayFlags(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
02564 #define DisplayTextLinesCenterFlags() asm { GetDisplayTextLinesCenterFlags(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
02565 
02566 #define SetDisplayNormal(_x, _line, _cnt, _data) asm { __setDisplayNormal(_x, _line, _cnt, _data) }
02567 #define SetDisplayPopup(_x, _line, _cnt, _data) asm { __setDisplayPopup(_x, _line, _cnt, _data) }
02568 
02569 #define SysDrawText(_args) asm { \
02570   compchktype _args, DrawTextType \
02571   syscall DrawText, _args \
02572 }
02573 #define SysDrawPoint(_args) asm { \
02574   compchktype _args, DrawPointType \
02575   syscall DrawPoint, _args \
02576 }
02577 #define SysDrawLine(_args) asm { \
02578   compchktype _args, DrawLineType \
02579   syscall DrawLine, _args \
02580 }
02581 #define SysDrawCircle(_args) asm { \
02582   compchktype _args, DrawCircleType \
02583   syscall DrawCircle, _args \
02584 }
02585 #define SysDrawRect(_args) asm { \
02586   compchktype _args, DrawRectType \
02587   syscall DrawRect, _args \
02588 }
02589 #define SysDrawGraphic(_args) asm { \
02590   compchktype _args, DrawGraphicType \
02591   syscall DrawGraphic, _args \
02592 }
02593 #define SysSetScreenMode(_args) asm { \
02594   compchktype _args, SetScreenModeType \
02595   syscall SetScreenMode, _args \
02596 }
02597 
02598 #ifdef __ENHANCED_FIRMWARE
02599 
02600 #define SysDisplayExecuteFunction(_args) asm { \
02601   compchktype _args, DisplayExecuteFunctionType \
02602   syscall DisplayExecuteFunction, _args \
02603 }
02604 
02605 #if __FIRMWARE_VERSION > 107
02606 
02607 #define DisplayContrast() asm { GetDisplayContrast(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
02608 
02609 #define SysDrawGraphicArray(_args) asm { \
02610   compchktype _args, DrawGraphicArrayType \
02611   syscall DrawGraphicArray, _args \
02612 }
02613 #define SysDrawPolygon(_args) asm { \
02614   compchktype _args, DrawPolygonType \
02615   syscall DrawPolygon, _args \
02616 }
02617 #define SysDrawEllipse(_args) asm { \
02618   compchktype _args, DrawEllipseType \
02619   syscall DrawEllipse, _args \
02620 }
02621 #define SysDrawFont(_args) asm { \
02622   compchktype _args, DrawFontType \
02623   syscall DrawFont, _args \
02624 }
02625 #endif
02626 #endif
02627 #endif
02628 
02633 inline void ClearScreen() { asm { PointOutEx(200, 200, TRUE) } }
02634 
02640 inline void ClearLine(byte line) { asm { TextOutEx(0, line, __BlankLine, 0) } }
02641 
02648 inline void SetDisplayFont(unsigned long fontaddr) { asm { __setDisplayFont(fontaddr) } }
02649 
02656 inline void SetDisplayDisplay(unsigned long dispaddr) { asm { __setDisplayDisplay(dispaddr) } }
02657 
02664 inline void SetDisplayEraseMask(unsigned long eraseMask) { asm { __setDisplayEraseMask(eraseMask) } }
02665 
02672 inline void SetDisplayFlags(byte flags) { asm { __setDisplayFlags(flags) } }
02673 
02680 inline void SetDisplayTextLinesCenterFlags(byte ctrFlags) { asm { __setDisplayTextLinesCenterFlags(ctrFlags) } }
02681 
02688 inline void SetDisplayUpdateMask(unsigned long updateMask) { asm { __setDisplayUpdateMask(updateMask) } }
02689 
02690 #if (__FIRMWARE_VERSION > 107) && defined(__ENHANCED_FIRMWARE)
02691 
02699 inline void SetDisplayContrast(byte contrast) { asm { __setDisplayContrast(contrast) } }
02700 
02701 #endif
02702  // end of DisplayModuleFunctions group // end of DisplayModule group // end of NXTFirmwareModules group
02706 
02707 
02711 
02712 
02730 struct Tone {
02731   unsigned int Frequency; 
02732   unsigned int Duration;  
02733 };
02734 
02741 struct SoundPlayFileType {
02742   char Result;       
02743   string Filename;   
02744   bool Loop;         
02745   byte SoundLevel;   
02746 };
02747 
02754 struct SoundPlayToneType {
02755   char Result;              
02756   unsigned int Frequency;   
02757   unsigned int Duration;    
02758   bool Loop;                
02759   byte SoundLevel;          
02760 };
02761 
02768 struct SoundGetStateType {
02769   byte State;   
02770   byte Flags;   
02771 };
02772 
02779 struct SoundSetStateType {
02780   byte Result;   
02781   byte State;    
02782   byte Flags;    
02783 };
02784  // end of SoundModuleTypes group
02786 
02792 #ifdef __DOXYGEN_DOCS
02793 
02803 inline char PlayFile(string filename);
02804 
02818 inline char PlayFileEx(string filename, byte volume, bool loop);
02819 
02830 inline char PlayTone(unsigned int frequency, unsigned int duration);
02831 
02845 inline char PlayToneEx(unsigned int frequency, unsigned int duration, byte volume, bool loop);
02846 
02854 inline byte SoundState();
02855 
02863 inline byte SoundFlags();
02864 
02871 inline byte StopSound();
02872 
02880 inline unsigned int SoundFrequency();
02881 
02889 inline unsigned int SoundDuration();
02890 
02898 inline unsigned int SoundSampleRate();
02899 
02907 inline byte SoundMode();
02908 
02916 inline byte SoundVolume();
02917 
02925 inline void SetSoundDuration(unsigned int duration);
02926 
02934 inline void SetSoundFlags(byte flags);
02935 
02943 inline void SetSoundFrequency(unsigned int frequency);
02944 
02952 inline void SetSoundMode(byte mode);
02953 
02961 inline void SetSoundModuleState(byte state);
02962 
02970 inline void SetSoundSampleRate(unsigned int sampleRate);
02971 
02979 inline void SetSoundVolume(byte volume);
02980 
02991 inline void SysSoundPlayFile(SoundPlayFileType & args);
02992 
03001 inline void SysSoundPlayTone(SoundPlayToneType & args);
03002 
03011 inline void SysSoundGetState(SoundGetStateType & args);
03012 
03021 inline void SysSoundSetState(SoundSetStateType & args);
03022 
03023 #else
03024 
03025 #define PlayTone(_f, _d) PlayToneEx(_f, _d, 4, 0)
03026 #define PlayFile(_f) PlayFileEx(_f, 4, 0)
03027 
03028 #define SoundState() asm { GetSoundState(__RETVAL__, __TMPBYTE__) }
03029 #define SoundFlags() asm { GetSoundState(__TMPBYTE__, __RETVAL__) }
03030 #define StopSound() asm { __setSoundState(SOUND_STATE_STOP, 0, __RETVAL__) }
03031 
03032 #define SoundFrequency() asm { GetSoundFrequency(__TMPWORD__) __RETURN__ __TMPWORD__ }
03033 #define SoundDuration() asm { GetSoundDuration(__TMPWORD__) __RETURN__ __TMPWORD__ }
03034 #define SoundSampleRate() asm { GetSoundSampleRate(__TMPWORD__) __RETURN__ __TMPWORD__ }
03035 #define SoundMode() asm { GetSoundMode(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
03036 #define SoundVolume() asm { GetSoundVolume(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
03037 
03038 #define SetSoundFrequency(_n) asm { __setSoundFrequency(_n) }
03039 #define SetSoundDuration(_n) asm { __setSoundDuration(_n) }
03040 #define SetSoundSampleRate(_n) asm { __setSoundSampleRate(_n) }
03041 #define SetSoundFlags(_n) asm { __setSoundFlags(_n) }
03042 #define SetSoundModuleState(_n) asm { __setSoundModuleState(_n) }
03043 #define SetSoundMode(_n) asm { __setSoundMode(_n) }
03044 #define SetSoundVolume(_n) asm { __setSoundVolume(_n) }
03045 
03046 #define SysSoundPlayFile(_args) asm { \
03047   compchktype _args, SoundPlayFileType \
03048   syscall SoundPlayFile, _args \
03049 }
03050 #define SysSoundPlayTone(_args) asm { \
03051   compchktype _args, SoundPlayToneType \
03052   syscall SoundPlayTone, _args \
03053 }
03054 #define SysSoundGetState(_args) asm { \
03055   compchktype _args, SoundGetStateType \
03056   syscall SoundGetState, _args \
03057 }
03058 #define SysSoundSetState(_args) asm { \
03059   compchktype _args, SoundSetStateType \
03060   syscall SoundSetState, _args \
03061 }
03062 
03063 #endif
03064 
03080 void PlaySound(const int &aCode)
03081 {
03082     if (aCode == SOUND_CLICK)
03083         PlayTone(600, MS_200);
03084     else if (aCode == SOUND_DOUBLE_BEEP)
03085     {
03086         PlayTone(600, MS_150);
03087         asm { wait MS_200 };
03088         PlayTone(600, MS_150);
03089         asm { wait MS_150 };
03090     }
03091     else if (aCode == SOUND_UP)
03092         for (int i = 4; i < 8; i++)
03093         {
03094             PlayTone(TONE_C5 * i / 4, MS_100);
03095             asm { wait MS_100 };
03096         }
03097     else if (aCode == SOUND_DOWN)
03098         for (int i = 7; i > 3; i--)
03099         {
03100             PlayTone(TONE_C5 * i / 4, MS_100);
03101             asm { wait MS_100 };
03102         }
03103     else if (aCode == SOUND_LOW_BEEP)
03104     {
03105         PlayTone(100, MS_500);
03106         asm { wait MS_500 };
03107     }
03108     else if (aCode == SOUND_FAST_UP)
03109         for (int i = 4; i < 8; i++)
03110         {
03111             PlayTone(TONE_C5 * i / 4, MS_50);
03112             asm { wait MS_50 };
03113         }
03114 }
03115 
03124 void PlayTones(Tone tones[])
03125 {
03126   for (int i = 0; i <  asm { arrsize __RETVAL__, tones }; i++) {
03127     Tone tmp = tones[i];
03128     PlayTone(tmp.Frequency, tmp.Duration);
03129     asm { waitv tmp.Duration };
03130   }
03131 }
03132  // end of SoundModuleFunctions group // end of SoundModule group // end of NXTFirmwareModules group
03136 
03137 
03141 
03142 
03159 struct CommLSWriteType {
03160   char Result;      
03163   byte Port;        
03164   byte Buffer[];    
03165   byte ReturnLen;   
03167 };
03168 
03175 struct CommLSReadType {
03176   char Result;      
03180   byte Port;        
03181   byte Buffer[];    
03182   byte BufferLen;   
03183 };
03184 
03191 struct CommLSCheckStatusType {
03192   char Result;       
03196   byte Port;         
03197   byte BytesReady;   
03198 };
03199 
03200 #ifdef __ENHANCED_FIRMWARE
03201 
03207 struct CommLSWriteExType {
03208   char Result;          
03211   byte Port;            
03212   byte Buffer[];        
03213   byte ReturnLen;       
03214   bool NoRestartOnRead; 
03215 };
03216 #endif
03217  // end of LowSpeedModuleTypes group
03219 
03225 #ifdef __DOXYGEN_DOCS
03226 
03237 inline byte SensorUS(const byte port);
03238 
03249 inline char ReadSensorUSEx(const byte port, byte & values[]);
03250 
03268 inline char ReadSensorEMeter(const byte & port, float &vIn, float &aIn, float &vOut, float &aOut, int &joules, float &wIn, float &wOut);
03269 
03282 inline char ConfigureTemperatureSensor(const byte & port, const byte & config);
03283 
03295 inline float SensorTemperature(const byte & port);
03296 
03318 inline long LowspeedStatus(const byte port, byte & bytesready);
03319 
03337 inline long LowspeedCheckStatus(const byte port);
03338 
03355 inline byte LowspeedBytesReady(const byte port);
03356 
03378 inline long LowspeedWrite(const byte port, byte retlen, byte buffer[]);
03379 
03399 inline long LowspeedRead(const byte port, byte buflen, byte & buffer[]);
03400 
03422 inline long I2CStatus(const byte port, byte & bytesready);
03423 
03441 inline long I2CCheckStatus(const byte port);
03442 
03459 inline byte I2CBytesReady(const byte port);
03460 
03482 inline long I2CWrite(const byte port, byte retlen, byte buffer[]);
03483 
03503 inline long I2CRead(const byte port, byte buflen, byte & buffer[]);
03504 
03532 inline long I2CBytes(const byte port, byte inbuf[], byte & count, byte & outbuf[]);
03533 
03545 inline char ReadI2CRegister(byte port, byte i2caddr, byte reg, byte & out);
03546 
03558 inline char WriteI2CRegister(byte port, byte i2caddr, byte reg, byte val);
03559 
03574 inline string I2CDeviceInfo(byte port, byte i2caddr, byte info);
03575 
03587 inline string I2CVersion(byte port, byte i2caddr);
03588 
03600 inline string I2CVendorId(byte port, byte i2caddr);
03601 
03613 inline string I2CDeviceId(byte port, byte i2caddr);
03614 
03628 inline long I2CSendCommand(byte port, byte i2caddr, byte cmd);
03629 
03645 inline void GetLSInputBuffer(const byte port, const byte offset, byte cnt, byte & data[]);
03646 
03657 inline void GetLSOutputBuffer(const byte port, const byte offset, byte cnt, byte & data[]);
03658 
03666 inline byte LSInputBufferInPtr(const byte port);
03667 
03675 inline byte LSInputBufferOutPtr(const byte port);
03676 
03684 inline byte LSInputBufferBytesToRx(const byte port);
03685 
03693 inline byte LSOutputBufferInPtr(const byte port);
03694 
03702 inline byte LSOutputBufferOutPtr(const byte port);
03703 
03711 inline byte LSOutputBufferBytesToRx(const byte port);
03712 
03719 inline byte LSMode(const byte port);
03720 
03727 inline byte LSChannelState(const byte port);
03728 
03735 inline byte LSErrorType(const byte port);
03736 
03742 inline byte LSState();
03743 
03750 inline byte LSSpeed();
03751 
03752 #ifdef __ENHANCED_FIRMWARE
03753 
03758 inline byte LSNoRestartOnRead();
03759 
03760 #endif
03761 
03762 /*
03763 // these low speed module IOMap fields are essentially read-only
03764 inline void SetLSInputBuffer(const byte port, const byte offset, byte cnt, byte data[]);
03765 inline void SetLSInputBufferInPtr(const byte port, byte n);
03766 inline void SetLSInputBufferOutPtr(const byte port, byte n);
03767 inline void SetLSInputBufferBytesToRx(const byte port, byte n);
03768 inline void SetLSOutputBuffer(const byte port, const byte offset, byte cnt, byte data[]);
03769 inline void SetLSOutputBufferInPtr(const byte port, byte n);
03770 inline void SetLSOutputBufferOutPtr(const byte port, n);
03771 inline void SetLSOutputBufferBytesToRx(const byte port, byte n);
03772 inline void SetLSMode(const byte port, const byte mode);
03773 inline void SetLSChannelState(const byte port, const byte chState);
03774 inline void SetLSErrorType(const byte port, const byte errType);
03775 inline void SetLSState(const byte lsState);
03776 inline void SetLSSpeed(const byte lsSpeed);
03777 #ifdef __ENHANCED_FIRMWARE
03778 inline void SetLSNoRestartOnRead(const byte lsNoRestart);
03779 #endif
03780 */
03781  // end of LowLevelLowSpeedModuleFunctions group
03783 
03796 inline void SysCommLSWrite(CommLSWriteType & args);
03797 
03805 inline void SysCommLSRead(CommLSReadType & args);
03806 
03816 inline void SysCommLSCheckStatus(CommLSCheckStatusType & args);
03817 
03818 #ifdef __ENHANCED_FIRMWARE
03819 
03829 inline void SysCommLSWriteEx(CommLSWriteExType & args);
03830 
03831 #endif
03832  // end of LowSpeedModuleSystemCallFunctions group
03834 
03835 #else
03836 
03837 // ultrasonic sensor
03838 #define SensorUS(_p) asm { ReadSensorUS(_p, __RETVAL__) }
03839 #define ReadSensorUSEx(_port, _values) asm { __ReadSensorUSEx(_port, _values, __RETVAL__) }
03840 
03841 #define ReadSensorEMeter(_port, _vIn, _aIn, _vOut, _aOut, _joules, _wIn, _wOut) asm { __ReadSensorEMeter(_port, _vIn, _aIn, _vOut, _aOut, _joules, _wIn, _wOut, __RETVAL__) }
03842 
03843 #define ConfigureTemperatureSensor(_port, _config) asm { __TempSendCmd(_port, _config, __RETVAL__) }
03844 #if __FIRMWARE_VERSION > 107
03845 #define SensorTemperature(_port) asm { __ReadSensorTemperature(_port, __FLTRETVAL__) }
03846 #else
03847 #define SensorTemperature(_port) asm { __ReadSensorTemperature(_port, __RETVAL__) }
03848 #endif
03849 
03850 #define ReadI2CRegister(_port, _i2caddr, _reg, _out) asm { __MSReadValue(_port, _i2caddr, _reg, 1, _out, __RETVAL__) }
03851 #define WriteI2CRegister(_port, _i2caddr, _reg, _val) asm { __MSWriteToRegister(_port, _i2caddr, _reg, _val, __RETVAL__) }
03852 
03853 #define LowspeedStatus(_port, _bready) asm { __lowspeedStatus(_port, _bready, __RETVAL__) }
03854 #define LowspeedCheckStatus(_port) asm { __lowspeedStatus(_port, __TMPBYTE__, __RETVAL__) }
03855 #define LowspeedBytesReady(_port) asm { __lowspeedStatus(_port, __RETVAL__, __TMPBYTE__) }
03856 #define LowspeedWrite(_port, _retlen, _buffer) asm { __lowspeedWrite(_port, _retlen, _buffer, __RETVAL__) }
03857 #define LowspeedRead(_port, _buflen, _buffer) asm { __lowspeedRead(_port, _buflen, _buffer, __RETVAL__) }
03858 
03859 #define I2CStatus(_port, _bready) LowspeedStatus(_port, _bready)
03860 #define I2CCheckStatus(_port) LowspeedCheckStatus(_port)
03861 #define I2CBytesReady(_port) LowspeedBytesReady(_port)
03862 #define I2CWrite(_port, _retlen, _buffer) LowspeedWrite(_port, _retlen, _buffer)
03863 #define I2CRead(_port, _buflen, _buffer) LowspeedRead(_port, _buflen, _buffer)
03864 
03865 #define I2CBytes(_port, _inbuf, _count, _outbuf) asm { ReadI2CBytes(_port, _inbuf, _count, _outbuf, __RETVAL__) }
03866 
03867 #define I2CDeviceInfo(_port, _i2caddr, _info) asm { ReadI2CDeviceInfo(_port, _i2caddr, _info, __STRRETVAL__) }
03868 #define I2CVersion(_port, _i2caddr) asm { ReadI2CDeviceInfo(_port, _i2caddr, I2C_REG_VERSION, __STRRETVAL__) }
03869 #define I2CVendorId(_port, _i2caddr) asm { ReadI2CDeviceInfo(_port, _i2caddr, I2C_REG_VENDOR_ID, __STRRETVAL__) }
03870 #define I2CDeviceId(_port, _i2caddr) asm { ReadI2CDeviceInfo(_port, _i2caddr, I2C_REG_DEVICE_ID, __STRRETVAL__) }
03871 
03872 #define I2CSendCommand(_port, _i2caddr, _cmd) asm { __I2CSendCmd(_port, _i2caddr, _cmd, __RETVAL__) }
03873 
03874 #define GetLSInputBuffer(_p, _offset, _cnt, _data) asm { __getLSInputBuffer(_p, _offset, _cnt, _data) }
03875 #define GetLSOutputBuffer(_p, _offset, _cnt, _data) asm { __getLSOutputBuffer(_p, _offset, _cnt, _data) }
03876 
03877 #define LSInputBufferInPtr(_p) asm { GetLSInputBufferInPtr(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03878 #define LSInputBufferOutPtr(_p) asm { GetLSInputBufferOutPtr(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03879 #define LSInputBufferBytesToRx(_p) asm { GetLSInputBufferBytesToRx(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03880 #define LSOutputBufferInPtr(_p) asm { GetLSOutputBufferInPtr(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03881 #define LSOutputBufferOutPtr(_p) asm { GetLSOutputBufferOutPtr(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03882 #define LSOutputBufferBytesToRx(_p) asm { GetLSOutputBufferBytesToRx(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03883 #define LSMode(_p) asm { GetLSMode(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03884 #define LSChannelState(_p) asm { GetLSChannelState(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03885 #define LSErrorType(_p) asm { GetLSErrorType(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
03886 #define LSState() asm { GetLSState(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
03887 #define LSSpeed() asm { GetLSSpeed(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
03888 #ifdef __ENHANCED_FIRMWARE
03889 #define LSNoRestartOnRead(_n) asm { GetLSNoRestartOnRead(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
03890 #endif
03891 
03892 #define SetLSInputBuffer(_p, _offset, _cnt, _data) asm { __setLSInputBuffer(_p, _offset, _cnt, _data) }
03893 
03894 #define SetLSInputBufferInPtr(_p, _n) asm { __setLSInputBufferInPtr(_p, _n) }
03895 #define SetLSInputBufferOutPtr(_p, _n) asm { __setLSInputBufferOutPtr(_p, _n) }
03896 #define SetLSInputBufferBytesToRx(_p, _n) asm { __setLSInputBufferBytesToRx(_p, _n) }
03897 
03898 #define SetLSOutputBuffer(_p, _offset, _cnt, _data) asm { __setLSOutputBuffer(_p, _offset, _cnt, _data) }
03899 
03900 #define SetLSOutputBufferInPtr(_p, _n) asm { __setLSOutputBufferInPtr(_p, _n) }
03901 #define SetLSOutputBufferOutPtr(_p, _n) asm { __setLSOutputBufferOutPtr(_p, _n) }
03902 #define SetLSOutputBufferBytesToRx(_p, _n) asm { __setLSOutputBufferBytesToRx(_p, _n) }
03903 #define SetLSMode(_p, _n) asm { __setLSMode(_p, _n) }
03904 #define SetLSChannelState(_p, _n) asm { __setLSChannelState(_p, _n) }
03905 #define SetLSErrorType(_p, _n) asm { __setLSErrorType(_p, _n) }
03906 #define SetLSState(_n) asm { __setLSState(_n) }
03907 #define SetLSSpeed(_n) asm { __setLSSpeed(_n) }
03908 #ifdef __ENHANCED_FIRMWARE
03909 #define SetLSNoRestartOnRead(_n) asm { __setLSNoRestartOnRead(_n) }
03910 #endif
03911 
03912 #define SysCommLSWrite(_args) asm { \
03913   compchktype _args, CommLSWriteType \
03914   syscall CommLSWrite, _args \
03915 }
03916 #define SysCommLSRead(_args) asm { \
03917   compchktype _args, CommLSReadType \
03918   syscall CommLSRead, _args \
03919 }
03920 #define SysCommLSCheckStatus(_args) asm { \
03921   compchktype _args, CommLSCheckStatusType \
03922   syscall CommLSCheckStatus, _args \
03923 }
03924 #ifdef __ENHANCED_FIRMWARE
03925 #define SysCommLSWriteEx(_args) asm { \
03926   compchktype _args, CommLSWriteExType \
03927   syscall CommLSWriteEx, _args \
03928 }
03929 #endif
03930 
03931 #endif
03932  // end of LowSpeedModuleFunctions group // end of LowSpeedModule group // end of NXTFirmwareModules group
03936 
03937 
03941 
03942 
03960 struct GetStartTickType {
03961   unsigned long Result;   
03962 };
03963 
03970 struct KeepAliveType {
03971   unsigned long Result;   
03972 };
03973 
03980 struct IOMapReadType {
03981   char Result;           
03982   string ModuleName;     
03983   unsigned int Offset;   
03984   unsigned int Count;    
03985   byte Buffer[];         
03986 };
03987 
03994 struct IOMapWriteType {
03995   char Result;           
03996   string ModuleName;     
03997   unsigned int Offset;   
03998   byte Buffer[];         
03999 };
04000 
04001 #ifdef __ENHANCED_FIRMWARE
04002 
04008 struct IOMapReadByIDType {
04009   char Result;            
04010   unsigned long ModuleID; 
04011   unsigned int Offset;    
04012   unsigned int Count;     
04013   byte Buffer[];          
04014 };
04015 
04022 struct IOMapWriteByIDType {
04023   char Result;            
04024   unsigned long ModuleID; 
04025   unsigned int Offset;    
04026   byte Buffer[];          
04027 };
04028 
04029 #endif
04030 
04031 #if __FIRMWARE_VERSION > 107
04032 
04039 struct DatalogWriteType {
04040  char Result;     
04041  byte Message[];  
04042 };
04043 
04050 struct DatalogGetTimesType {
04051  unsigned long SyncTime;  
04052  unsigned long SyncTick;  
04053 };
04054 
04061 struct ReadSemDataType {
04062   byte SemData;  
04063   bool Request;  
04064 };
04065 
04072 struct WriteSemDataType {
04073   byte SemData;   
04074   bool Request;   
04075   byte NewVal;    
04076   bool ClearBits; 
04077 };
04078 
04085 struct UpdateCalibCacheInfoType {
04086   byte Result;          
04087   string Name;          
04088   unsigned int MinVal;  
04089   unsigned int MaxVal;  
04090 };
04091 
04098 struct ComputeCalibValueType {
04099   byte Result;          
04100   string Name;          
04101   unsigned int RawVal;  
04102 };
04103 
04104 #ifdef __ENHANCED_FIRMWARE
04105 
04111 struct MemoryManagerType {
04112   char Result;                
04113   bool Compact;               
04114   unsigned int PoolSize;      
04115   unsigned int DataspaceSize; 
04116 };
04117 
04124 struct ReadLastResponseType {
04125   char Result;   
04126   bool Clear;    
04127   byte Length;   
04128   byte Command;  
04129   byte Buffer[]; 
04130 };
04131 #endif
04132 
04133 #endif
04134  // end of CommandModuleTypes group
04135 
04141 #ifdef __DOXYGEN_DOCS
04142 
04149 inline unsigned long CurrentTick();
04150 
04159 inline unsigned long FirstTick();
04160 
04167 inline long ResetSleepTimer();
04168 
04169 //inline void SpawnProgram(string fname); // not ready to be documented
04170 
04184 inline void SysCall(byte funcID, variant & args);
04185 
04193 inline void SysGetStartTick(GetStartTickType & args);
04194 
04202 inline void SysKeepAlive(KeepAliveType & args);
04203 
04211 inline void SysIOMapRead(IOMapReadType & args);
04212 
04220 inline void SysIOMapWrite(IOMapWriteType & args);
04221 
04222 #ifdef __ENHANCED_FIRMWARE
04223 
04235 inline void SysIOMapReadByID(IOMapReadByIDType & args);
04236 
04249 inline void SysIOMapWriteByID(IOMapWriteByIDType & args);
04250 
04251 #endif
04252 
04253 #if __FIRMWARE_VERSION > 107
04254 
04265 inline void SysDatalogWrite(DatalogWriteType & args);
04266 
04277 inline void SysDatalogGetTimes(DatalogGetTimesType & args);
04278 
04288 inline void SysReadSemData(ReadSemDataType & args);
04289 
04299 inline void SysWriteSemData(WriteSemDataType & args);
04300 
04311 inline void SysUpdateCalibCacheInfo(UpdateCalibCacheInfoType & args);
04312 
04323 inline void SysComputeCalibValue(ComputeCalibValueType & args);
04324 
04325 #endif
04326 
04327 #if defined(__ENHANCED_FIRMWARE) && (__FIRMWARE_VERSION > 107)
04328 
04344 inline char GetMemoryInfo(bool Compact, unsigned int & PoolSize, unsigned int & DataspaceSize);
04345 
04355 inline void SysMemoryManager(MemoryManagerType & args);
04356 
04370 inline char GetLastResponseInfo(bool Clear, byte & Length, byte & Command, byte & Buffer[]);
04371 
04382 inline void SysReadLastResponse(ReadLastResponseType & args);
04383 
04384 #endif
04385 
04386 
04387 #else
04388 
04389 #define CurrentTick() asm { gettick __URETVAL__ }
04390 #define FirstTick() asm { GetFirstTick(__URETVAL__) }
04391 #define ResetSleepTimer() asm { acquire __KeepAliveMutex \
04392   syscall KeepAlive, __KeepAliveArgs \
04393   mov __RETVAL__, __KeepAliveArgs.Result \
04394   release __KeepAliveMutex }
04395 
04396 #define SpawnProgram(_fname) asm { __spawnProgram(_fname) }
04397 
04398 #define SysCall(_func, _args) asm { syscall _func, _args }
04399 
04400 #define SysGetStartTick(_args) asm { \
04401   compchktype _args, GetStartTickType \
04402   syscall GetStartTick, _args \
04403 }
04404 
04405 #define SysKeepAlive(_args) asm { \
04406   compchktype _args, KeepAliveType \
04407   syscall KeepAlive, _args \
04408 }
04409 
04410 #define SysIOMapRead(_args) asm { \
04411   compchktype _args, IOMapReadType \
04412   syscall IOMapRead, _args \
04413 }
04414 #define SysIOMapWrite(_args) asm { \
04415   compchktype _args, IOMapWriteType \
04416   syscall IOMapWrite, _args \
04417 }
04418 
04419 #ifdef __ENHANCED_FIRMWARE
04420 #define SysIOMapReadByID(_args) asm { \
04421   compchktype _args, IOMapReadByIDType \
04422   syscall IOMapReadByID, _args \
04423 }
04424 #define SysIOMapWriteByID(_args) asm { \
04425   compchktype _args, IOMapWriteByIDType \
04426   syscall IOMapWriteByID, _args \
04427 }
04428 #endif
04429 #if __FIRMWARE_VERSION > 107
04430 
04431 #define SysDatalogWrite(_args) asm { \
04432   compchktype _args, DatalogWriteType \
04433   syscall DatalogWrite, _args \
04434 }
04435 #define SysDatalogGetTimes(_args) asm { \
04436   compchktype _args, DatalogGetTimesType \
04437   syscall DatalogGetTimes, _args \
04438 }
04439 #define SysReadSemData(_args) asm { \
04440   compchktype _args, ReadSemDataType \
04441   syscall ReadSemData, _args \
04442 }
04443 #define SysWriteSemData(_args) asm { \
04444   compchktype _args, WriteSemDataType \
04445   syscall WriteSemData, _args \
04446 }
04447 #define SysUpdateCalibCacheInfo(_args) asm { \
04448   compchktype _args, UpdateCalibCacheInfoType \
04449   syscall UpdateCalibCacheInfo, _args \
04450 }
04451 #define SysComputeCalibValue(_args) asm { \
04452   compchktype _args, ComputeCalibValueType \
04453   syscall ComputeCalibValue, _args \
04454 }
04455 #endif
04456 
04457 #if defined(__ENHANCED_FIRMWARE) && (__FIRMWARE_VERSION > 107)
04458 
04459 #define GetMemoryInfo(_Compact,_PoolSize,_DataspaceSize) asm { __GetMemoryInfo(_Compact,_PoolSize,_DataspaceSize,__RETVAL__) }
04460 
04461 #define SysMemoryManager(_args) asm { \
04462   compchktype _args, MemoryManagerType \
04463   syscall MemoryManager, _args \
04464 }
04465 
04466 #define GetLastResponseInfo(_Clear,_Length,_Command,_Buffer) asm { __GetLastResponseInfo(_Clear,_Length,_Command,_Buffer,__RETVAL__) }
04467 
04468 #define SysReadLastResponse(_args) asm { \
04469   compchktype _args, ReadLastResponseType \
04470   syscall ReadLastResponse, _args \
04471 }
04472 
04473 #endif
04474 
04475 #define until(_c) while(!(_c))
04476 
04477 #endif
04478 
04485 inline void Wait(unsigned long ms) { asm { waitv ms } }
04486 
04491 inline void Yield() { asm { wait 1 } }
04492 
04498 inline void StopAllTasks() { Stop(true); }
04499 
04500 
04501 #ifdef __DOXYGEN_DOCS
04502 
04508 inline void Stop(bool bvalue);
04509 
04515 inline void ExitTo(task newTask);
04516 
04528 inline void Precedes(task task1, task task2, ..., task taskN);
04529 
04542 inline void Follows(task task1, task task2, ..., task taskN);
04543 
04554 inline void Acquire(mutex m);
04555 
04564 inline void Release(mutex m);
04565 
04571 inline void StartTask(task t);
04572 
04579 inline void StopTask(task t);
04580 
04599 inline void ArrayBuild(variant & aout[], variant src1, variant src2, ..., variant srcN);
04600 
04608 inline unsigned int ArrayLen(variant data[]);
04609 
04620 inline void ArrayInit(variant & aout[], variant value, unsigned int count);
04621 
04631 inline void ArraySubset(variant & aout[], variant asrc[], unsigned int idx, unsigned int len);
04632 
04633 #ifdef __ENHANCED_FIRMWARE
04634 
04650 inline variant ArraySum(const variant & src[], unsigned int idx, unsigned int len);
04651 
04667 inline variant ArrayMean(const variant & src[], unsigned int idx, unsigned int len);
04668 
04684 inline variant ArraySumSqr(const variant & src[], unsigned int idx, unsigned int len);
04685 
04701 inline variant ArrayStd(const variant & src[], unsigned int idx, unsigned int len);
04702 
04718 inline variant ArrayMin(const variant & src[], unsigned int idx, unsigned int len);
04719 
04735 inline variant ArrayMax(const variant & src[], unsigned int idx, unsigned int len);
04736 
04753 inline void ArraySort(variant & dest[], const variant & src[], unsigned int idx, unsigned int len);
04754 
04770 inline void ArrayOp(const byte op, variant & dest, const variant & src[], unsigned int idx, unsigned int len);
04771 
04772 #endif
04773  // end of ArrayFunctions group
04775 
04776 #else
04777 
04778 #define StartTask(_t) start _t
04779 #define StopTask(_t) stop _t
04780 
04781 #if __FIRMWARE_VERSION <= 107
04782 #define IOMA(_n) asm { mov __RETVAL__, _n }
04783 #define SetIOMA(_n, _val) asm { mov _n, _val }
04784 #endif
04785 
04786 #define ArrayBuild(_aout, ...) asm { arrbuild _aout, __VA_ARGS__ }
04787 #define ArrayLen(_asrc) asm { arrsize __RETVAL__, _asrc }
04788 #define ArrayInit(_aout, _val, _cnt) asm { arrinit _aout, _val, _cnt }
04789 #define ArraySubset(_aout, _asrc, _idx, _len) asm { arrsubset _aout, _asrc, _idx, _len }
04790 
04791 #ifdef __ENHANCED_FIRMWARE
04792 #define ArraySum(_src, _idx, _len) asm { arrop OPARR_SUM, __RETVAL__, _src, _idx, _len }
04793 #define ArrayMean(_src, _idx, _len) asm { arrop OPARR_MEAN, __RETVAL__, _src, _idx, _len }
04794 #define ArraySumSqr(_src, _idx, _len) asm { arrop OPARR_SUMSQR, __RETVAL__, _src, _idx, _len }
04795 #define ArrayStd(_src, _idx, _len) asm { arrop OPARR_STD, __RETVAL__, _src, _idx, _len }
04796 #define ArrayMin(_src, _idx, _len) asm { arrop OPARR_MIN, __RETVAL__, _src, _idx, _len }
04797 #define ArrayMax(_src, _idx, _len) asm { arrop OPARR_MAX, __RETVAL__, _src, _idx, _len }
04798 #define ArraySort(_dest, _src, _idx, _len) asm { arrop OPARR_SORT, _dest, _src, _idx, _len }
04799 #define ArrayOp(_op, _dest, _src, _idx, _len) asm { arrop _op, _dest, _src, _idx, _len }
04800 #endif
04801 
04802 #endif
04803 
04804 
04805 #ifdef __DOXYGEN_DOCS
04806 
04820 inline void SetIOMapBytes(string moduleName, unsigned int offset, unsigned int count, byte data[]);
04821 
04833 inline void SetIOMapValue(string moduleName, unsigned int offset, variant value);
04834 
04848 inline void GetIOMapBytes(string moduleName, unsigned int offset, unsigned int count, byte & data[]);
04849 
04861 inline void GetIOMapValue(string moduleName, unsigned int offset, variant & value);
04862 
04876 inline void GetLowSpeedModuleBytes(unsigned int offset, unsigned int count, byte & data[]);
04877 
04891 inline void GetDisplayModuleBytes(unsigned int offset, unsigned int count, byte & data[]);
04892 
04906 inline void GetCommModuleBytes(unsigned int offset, unsigned int count, byte & data[]);
04907 
04921 inline void GetCommandModuleBytes(unsigned int offset, unsigned int count, byte & data[]);
04922 
04936 inline void SetCommandModuleBytes(unsigned int offset, unsigned int count, byte data[]);
04937 
04951 inline void SetLowSpeedModuleBytes(unsigned int offset, unsigned int count, byte data[]);
04952 
04966 inline void SetDisplayModuleBytes(unsigned int offset, unsigned int count, byte data[]);
04967 
04981 inline void SetCommModuleBytes(unsigned int offset, unsigned int count, byte data[]);
04982 
04983 
04984 #ifdef __ENHANCED_FIRMWARE
04985 
04999 inline void SetIOMapBytesByID(unsigned long moduleId, unsigned int offset, unsigned int count, byte data[]);
05000 
05013 inline void SetIOMapValueByID(unsigned long moduleId, unsigned int offset, variant value);
05014 
05029 inline void GetIOMapBytesByID(unsigned long moduleId, unsigned int offset, unsigned int count, byte & data[]);
05030 
05043 inline void GetIOMapValueByID(unsigned long moduleId, unsigned int offset, variant & value);
05044 
05045 #endif
05046 
05057 inline void SetCommandModuleValue(unsigned int offset, variant value);
05058 
05069 inline void SetIOCtrlModuleValue(unsigned int offset, variant value);
05070 
05081 inline void SetLoaderModuleValue(unsigned int offset, variant value);
05082 
05093 inline void SetUIModuleValue(unsigned int offset, variant value);
05094 
05105 inline void SetSoundModuleValue(unsigned int offset, variant value);
05106 
05117 inline void SetButtonModuleValue(unsigned int offset, variant value);
05118 
05129 inline void SetInputModuleValue(unsigned int offset, variant value);
05130 
05141 inline void SetOutputModuleValue(unsigned int offset, variant value);
05142 
05153 inline void SetLowSpeedModuleValue(unsigned int offset, variant value);
05154 
05165 inline void SetDisplayModuleValue(unsigned int offset, variant value);
05166 
05177 inline void SetCommModuleValue(unsigned int offset, variant value);
05178 
05189 inline void GetCommandModuleValue(unsigned int offset, variant & value);
05190 
05201 inline void GetLoaderModuleValue(unsigned int offset, variant & value);
05202 
05213 inline void GetSoundModuleValue(unsigned int offset, variant & value);
05214 
05225 inline void GetButtonModuleValue(unsigned int offset, variant & value);
05226 
05237 inline void GetUIModuleValue(unsigned int offset, variant & value);
05238 
05249 inline void GetInputModuleValue(unsigned int offset, variant & value);
05250 
05261 inline void GetOutputModuleValue(unsigned int offset, variant & value);
05262 
05273 inline void GetLowSpeedModuleValue(unsigned int offset, variant & value);
05274 
05285 inline void GetDisplayModuleValue(unsigned int offset, variant & value);
05286 
05297 inline void GetCommModuleValue(unsigned int offset, variant & value);
05298 
05299 
05300 #else
05301 
05302 #define SetIOMapBytes(_modName, _offset, _cnt, _arrIn) asm { __SetIOMapBytes(_modName, _offset, _cnt, _arrIn) }
05303 #define SetIOMapValue(_modName, _offset, _n) asm { __SetIOMapValue(_modName, _offset, _n) }
05304 
05305 #define GetIOMapBytes(_modName, _offset, _cnt, _arrOut) asm { __getIOMapBytes(_modName, _offset, _cnt, _arrOut) }
05306 #define GetIOMapValue(_modName, _offset, _n) asm { __getIOMapValue(_modName, _offset, _n) }
05307 
05308 #define GetLowSpeedModuleBytes(_offset, _cnt, _arrOut) asm { __getLowSpeedModuleBytes(_offset, _cnt, _arrOut) }
05309 #define GetDisplayModuleBytes(_offset, _cnt, _arrOut) asm { __getDisplayModuleBytes(_offset, _cnt, _arrOut) }
05310 #define GetCommModuleBytes(_offset, _cnt, _arrOut) asm { __getCommModuleBytes(_offset, _cnt, _arrOut) }
05311 
05312 #ifdef __ENHANCED_FIRMWARE
05313 
05314 #define SetIOMapBytesByID(_modID, _offset, _cnt, _arrIn) asm { __SetIOMapBytesByID(_modID, _offset, _cnt, _arrIn) }
05315 #define SetIOMapValueByID(_modID, _offset, _n) asm { __SetIOMapValueByID(_modID, _offset, _n) }
05316 
05317 #define GetIOMapBytesByID(_modID, _offset, _cnt, _arrOut) asm { __getIOMapBytesByID(_modID, _offset, _cnt, _arrOut) }
05318 #define GetIOMapValueByID(_modID, _offset, _n) asm { __getIOMapValueByID(_modID, _offset, _n) }
05319 
05320 #define SetCommandModuleValue(_offset, _n) SetIOMapValueByID(CommandModuleID, _offset, _n)
05321 #define SetIOCtrlModuleValue(_offset, _n) SetIOMapValueByID(IOCtrlModuleID, _offset, _n)
05322 #define SetLoaderModuleValue(_offset, _n) SetIOMapValueByID(LoaderModuleID, _offset, _n)
05323 #define SetUIModuleValue(_offset, _n) SetIOMapValueByID(UIModuleID, _offset, _n)
05324 #define SetSoundModuleValue(_offset, _n) SetIOMapValueByID(SoundModuleID, _offset, _n)
05325 #define SetButtonModuleValue(_offset, _n) SetIOMapValueByID(ButtonModuleID, _offset, _n)
05326 #define SetInputModuleValue(_offset, _n) SetIOMapValueByID(InputModuleID, _offset, _n)
05327 #define SetOutputModuleValue(_offset, _n) SetIOMapValueByID(OutputModuleID, _offset, _n)
05328 #define SetLowSpeedModuleValue(_offset, _n) SetIOMapValueByID(LowSpeedModuleID, _offset, _n)
05329 #define SetDisplayModuleValue(_offset, _n) SetIOMapValueByID(DisplayModuleID, _offset, _n)
05330 #define SetCommModuleValue(_offset, _n) SetIOMapValueByID(CommModuleID, _offset, _n)
05331 
05332 #define SetCommandModuleBytes(_offset, _cnt, _arrIn) SetIOMapBytesByID(CommandModuleID, _offset, _cnt, _arrIn)
05333 #define SetLowSpeedModuleBytes(_offset, _cnt, _arrIn) SetIOMapBytesByID(LowSpeedModuleID, _offset, _cnt, _arrIn)
05334 #define SetDisplayModuleBytes(_offset, _cnt, _arrIn) SetIOMapBytesByID(DisplayModuleID, _offset, _cnt, _arrIn)
05335 #define SetCommModuleBytes(_offset, _cnt, _arrIn) SetIOMapBytesByID(CommModuleID, _offset, _cnt, _arrIn)
05336 
05337 #define GetCommandModuleValue(_offset, _n) GetIOMapValueByID(CommandModuleID, _offset, _n)
05338 #define GetLoaderModuleValue(_offset, _n) GetIOMapValueByID(LoaderModuleID, _offset, _n)
05339 #define GetSoundModuleValue(_offset, _n) GetIOMapValueByID(SoundModuleID, _offset, _n)
05340 #define GetButtonModuleValue(_offset, _n) GetIOMapValueByID(ButtonModuleID, _offset, _n)
05341 #define GetUIModuleValue(_offset, _n) GetIOMapValueByID(UIModuleID, _offset, _n)
05342 #define GetInputModuleValue(_offset, _n) GetIOMapValueByID(InputModuleID, _offset, _n)
05343 #define GetOutputModuleValue(_offset, _n) GetIOMapValueByID(OutputModuleID, _offset, _n)
05344 #define GetLowSpeedModuleValue(_offset, _n) GetIOMapValueByID(LowSpeedModuleID, _offset, _n)
05345 #define GetDisplayModuleValue(_offset, _n) GetIOMapValueByID(DisplayModuleID, _offset, _n)
05346 #define GetCommModuleValue(_offset, _n) GetIOMapValueByID(CommModuleID, _offset, _n)
05347 
05348 #else
05349 
05350 #define SetCommandModuleValue(_offset, _n) SetIOMapValue(CommandModuleName, _offset, _n)
05351 #define SetIOCtrlModuleValue(_offset, _n) SetIOMapValue(IOCtrlModuleName, _offset, _n)
05352 #define SetLoaderModuleValue(_offset, _n) SetIOMapValue(LoaderModuleName, _offset, _n)
05353 #define SetUIModuleValue(_offset, _n) SetIOMapValue(UIModuleName, _offset, _n)
05354 #define SetSoundModuleValue(_offset, _n) SetIOMapValue(SoundModuleName, _offset, _n)
05355 #define SetButtonModuleValue(_offset, _n) SetIOMapValue(ButtonModuleName, _offset, _n)
05356 #define SetInputModuleValue(_offset, _n) SetIOMapValue(InputModuleName, _offset, _n)
05357 #define SetOutputModuleValue(_offset, _n) SetIOMapValue(OutputModuleName, _offset, _n)
05358 #define SetLowSpeedModuleValue(_offset, _n) SetIOMapValue(LowSpeedModuleName, _offset, _n)
05359 #define SetDisplayModuleValue(_offset, _n) SetIOMapValue(DisplayModuleName, _offset, _n)
05360 #define SetCommModuleValue(_offset, _n) SetIOMapValue(CommModuleName, _offset, _n)
05361 
05362 #define SetCommandModuleBytes(_offset, _cnt, _arrIn) SetIOMapBytes(CommandModuleName, _offset, _cnt, _arrIn)
05363 #define SetLowSpeedModuleBytes(_offset, _cnt, _arrIn) SetIOMapBytes(LowSpeedModuleName, _offset, _cnt, _arrIn)
05364 #define SetDisplayModuleBytes(_offset, _cnt, _arrIn) SetIOMapBytes(DisplayModuleName, _offset, _cnt, _arrIn)
05365 #define SetCommModuleBytes(_offset, _cnt, _arrIn) SetIOMapBytes(CommModuleName, _offset, _cnt, _arrIn)
05366 
05367 #define GetCommandModuleValue(_offset, _n) GetIOMapValue(CommandModuleName, _offset, _n)
05368 #define GetLoaderModuleValue(_offset, _n) GetIOMapValue(LoaderModuleName, _offset, _n)
05369 #define GetSoundModuleValue(_offset, _n) GetIOMapValue(SoundModuleName, _offset, _n)
05370 #define GetButtonModuleValue(_offset, _n) GetIOMapValue(ButtonModuleName, _offset, _n)
05371 #define GetUIModuleValue(_offset, _n) GetIOMapValue(UIModuleName, _offset, _n)
05372 #define GetInputModuleValue(_offset, _n) GetIOMapValue(InputModuleName, _offset, _n)
05373 #define GetOutputModuleValue(_offset, _n) GetIOMapValue(OutputModuleName, _offset, _n)
05374 #define GetLowSpeedModuleValue(_offset, _n) GetIOMapValue(LowSpeedModuleName, _offset, _n)
05375 #define GetDisplayModuleValue(_offset, _n) GetIOMapValue(DisplayModuleName, _offset, _n)
05376 #define GetCommModuleValue(_offset, _n) GetIOMapValue(CommModuleName, _offset, _n)
05377 
05378 #endif
05379 
05380 #endif
05381 
05382  // end of CommandModuleFunctions group // end of CommandModule group // end of NXTFirmwareModules group
05386 
05387 
05391 
05392  // end of IOCtrlModuleTypes group
05414 inline void PowerDown() {
05415   SetIOCtrlModuleValue(IOCtrlOffsetPowerOn, IOCTRL_POWERDOWN);
05416 }
05417 
05423 inline void SleepNow() {
05424   SetIOCtrlModuleValue(IOCtrlOffsetPowerOn, IOCTRL_POWERDOWN);
05425 }
05426 
05432 inline void RebootInFirmwareMode() {
05433   SetIOCtrlModuleValue(IOCtrlOffsetPowerOn, IOCTRL_BOOT);
05434 }
05435  // end of IOCtrlModuleFunctions group // end of IOCtrlModule group // end of NXTFirmwareModules group
05439 
05440 
05444 
05445 
05462 struct MessageWriteType {
05463   char Result;      
05464   byte QueueID;     
05465   string Message;   
05466 };
05467 
05474 struct MessageReadType {
05475   char Result;       
05476   byte QueueID;     
05477   bool Remove;      
05478   string Message;   
05479 };
05480 
05487 struct CommBTCheckStatusType {
05488   char Result;       
05491   byte Connection;   
05492 };
05493 
05500 struct CommBTWriteType {
05501   char Result;       
05504   byte Connection;   
05505   byte Buffer[];     
05506 };
05507 
05508 #ifdef __ENHANCED_FIRMWARE
05509 
05561 struct CommExecuteFunctionType {
05562   unsigned int Result;   
05564   byte Cmd;              
05565   byte Param1;           
05566   byte Param2;           
05567   byte Param3;           
05568   string Name;           
05569   unsigned int RetVal;   
05571 };
05572 
05579 struct CommHSControlType {
05580  char Result;             
05581  byte Command;            
05583  byte BaudRate;           
05584 #if __FIRMWARE_VERSION > 107
05585  unsigned int Mode;       
05588 #endif
05589 };
05590 
05597 struct CommHSCheckStatusType {
05598  bool SendingData;     
05599  bool DataAvailable;   
05600 };
05601 
05608 struct CommHSReadWriteType {
05609  char Status;    
05610  byte Buffer[];  
05612 };
05613 #endif
05614 
05615 #if __FIRMWARE_VERSION > 107
05616 
05622 struct CommBTOnOffType {
05623 #ifdef __ENHANCED_FIRMWARE
05624  unsigned int Result; 
05625 #else
05626  char Result;         
05627 #endif
05628  bool PowerState;     
05629 };
05630 
05637 struct CommBTConnectionType {
05638 #ifdef __ENHANCED_FIRMWARE
05639  unsigned int Result; 
05640 #else
05641  char Result;         
05642 #endif
05643  byte Action;         
05644  string Name;         
05645  byte ConnectionSlot; 
05646 };
05647 #endif
05648  // end of CommModuleTypes group
05655 #ifdef __DOXYGEN_DOCS
05656 
05665 inline char SendMessage(byte queue, string msg);
05666 
05680 inline char ReceiveMessage(byte queue, bool clear, string & msg);
05681 
05690 inline char BluetoothStatus(byte conn);
05691 
05703 inline char BluetoothWrite(byte conn, byte buffer[]);
05704 
05720 inline char RemoteConnectionWrite(byte conn, byte buffer[]);
05721 
05735 inline bool RemoteConnectionIdle(byte conn);
05736 
05750 inline char SendRemoteBool(byte conn, byte queue, bool bval);
05751 
05765 inline char SendRemoteNumber(byte conn, byte queue, long val);
05766 
05780 inline char SendRemoteString(byte conn, byte queue, string str);
05781 
05792 inline char SendResponseBool(byte queue, bool bval);
05793 
05804 inline char SendResponseNumber(byte queue, long val);
05805 
05816 inline char SendResponseString(byte queue, string str);
05817 
05831 inline char ReceiveRemoteBool(byte queue, bool clear, bool & bval);
05832 
05849 inline char ReceiveRemoteMessageEx(byte queue, bool clear, string & str, long & val, bool & bval);
05850 
05864 inline char ReceiveRemoteNumber(byte queue, bool clear, long & val);
05865 
05879 inline char ReceiveRemoteString(byte queue, bool clear, string & str);
05880 
05897 inline char RemoteKeepAlive(byte conn);
05898 
05911 inline char RemoteMessageRead(byte conn, byte queue);
05912 
05926 inline char RemoteMessageWrite(byte conn, byte queue, string msg);
05927 
05940 inline char RemotePlaySoundFile(byte conn, string filename, bool bloop);
05941 
05954 inline char RemotePlayTone(byte conn, unsigned int frequency, unsigned int duration);
05955 
05968 inline char RemoteResetMotorPosition(byte conn, byte port, bool brelative);
05969 
05981 inline char RemoteResetScaledValue(byte conn, byte port);
05982 
05996 inline char RemoteSetInputMode(byte conn, byte port, byte type, byte mode);
05997 
06015 inline char RemoteSetOutputState(byte conn, byte port, char speed, byte mode, byte regmode, char turnpct, byte runstate, unsigned long tacholimit);
06016 
06028 inline char RemoteStartProgram(byte conn, string filename);
06029 
06040 inline char RemoteStopProgram(byte conn);
06041 
06052 inline char RemoteStopSound(byte conn);
06053 
06054 #ifdef __ENHANCED_FIRMWARE
06055 
06068 inline char RemoteGetOutputState(byte conn, OutputStateType & params);
06069 
06082 inline char RemoteGetInputValues(byte conn, InputValuesType & params);
06083 
06097 inline char RemoteGetBatteryLevel(byte conn, int & value);
06098 
06112 inline char RemoteLowspeedGetStatus(byte conn, byte & value);
06113 
06128 inline char RemoteLowspeedRead(byte conn, byte port, byte & bread, byte & data[]);
06129 
06143 inline char RemoteGetCurrentProgramName(byte conn, string & name);
06144 
06159 inline char RemoteDatalogRead(byte conn, bool remove, byte & cnt, byte & log[]);
06160 
06174 inline char RemoteGetContactCount(byte conn, byte & cnt);
06175 
06189 inline char RemoteGetContactName(byte conn, byte idx, string & name);
06190 
06204 inline char RemoteGetConnectionCount(byte conn, byte & cnt);
06205 
06219 inline char RemoteGetConnectionName(byte conn, byte idx, string & name);
06220 
06235 inline char RemoteGetProperty(byte conn, byte property, variant & value);
06236 
06237 #endif
06238 
06250 inline char RemoteResetTachoCount(byte conn, byte port);
06251 
06263 inline char RemoteDatalogSetTimes(byte conn, long synctime);
06264 
06277 inline char RemoteSetProperty(byte conn, byte prop, variant value);
06278 
06293 inline char RemoteLowspeedWrite(byte conn, byte port, byte txlen, byte rxlen, byte data[]);
06294  // end of CommModuleDCFunctions group
06296 
06302 #ifdef __ENHANCED_FIRMWARE
06303 
06318 inline char RemoteOpenRead(byte conn, string filename, byte & handle, long & size);
06319 
06334 inline char RemoteOpenAppendData(byte conn, string filename, byte & handle, long & size);
06335 
06348 inline char RemoteDeleteFile(byte conn, string filename);
06349 
06365 inline char RemoteFindFirstFile(byte conn, string mask, byte & handle, string & name, long & size);
06366 
06383 inline char RemoteGetFirmwareVersion(byte conn, byte & pmin, byte & pmaj, byte & fmin, byte & fmaj);
06384 
06398 inline char RemoteGetBluetoothAddress(byte conn, byte & btaddr[]);
06399 
06416 inline char RemoteGetDeviceInfo(byte conn, string & name, byte & btaddr[], byte & btsignal[], long & freemem);
06417 
06430 inline char RemoteDeleteUserFlash(byte conn);
06431 
06446 inline char RemoteOpenWrite(byte conn, string filename, long size, byte & handle);
06447 
06462 inline char RemoteOpenWriteLinear(byte conn, string filename, long size, byte & handle);
06463 
06478 inline char RemoteOpenWriteData(byte conn, string filename, long size, byte & handle);
06479 
06492 inline char RemoteCloseFile(byte conn, byte handle);
06493 
06508 inline char RemoteFindNextFile(byte conn, byte & handle, string & name, long & size);
06509 
06523 inline char RemotePollCommandLength(byte conn, byte bufnum, byte & length);
06524 
06539 inline char RemoteWrite(byte conn, byte & handle, int & numbytes, byte data[]);
06540 
06556 inline char RemoteRead(byte conn, byte & handle, int & numbytes, byte & data[]);
06557 
06574 inline char RemoteIOMapRead(byte conn, long id, int offset, int & numbytes, byte & data[]);
06575 
06592 inline char RemotePollCommand(byte conn, byte bufnum, byte & len, byte & data[]);
06593 
06609 inline char RemoteRenameFile(byte conn, string oldname, string newname);
06610 
06611 #endif
06612 
06624 inline char RemoteBluetoothFactoryReset(byte conn);
06625 
06640 inline char RemoteIOMapWriteValue(byte conn, long id, int offset, variant value);
06641 
06656 inline char RemoteIOMapWriteBytes(byte conn, long id, int offset, byte data[]);
06657 
06670 inline char RemoteSetBrickName(byte conn, string name);
06671  // end of CommModuleSCFunctions group
06673 
06674 
06680 inline void UseRS485(void);
06681 
06682 #ifdef __ENHANCED_FIRMWARE
06683 
06697 inline char RS485Control(byte cmd, byte baud, unsigned int mode);
06698 
06707 inline bool RS485DataAvailable(void);
06708 
06721 inline char RS485Initialize(void);
06722 
06731 inline char RS485Disable(void);
06732 
06741 inline char RS485Enable(void);
06742 
06752 inline char RS485Read(byte & buffer[]);
06753 
06762 inline bool RS485SendingData(void);
06763 
06773 inline void RS485Status(bool & sendingData, bool & dataAvail);
06774 
06788 inline char RS485Uart(byte baud, unsigned int mode);
06789 
06799 inline char RS485Write(byte buffer[]);
06800 
06810 inline char SendRS485Bool(bool bval);
06811 
06821 inline char SendRS485Number(long val);
06822 
06832 inline char SendRS485String(string str);
06833 
06834 #endif
06835 
06846 inline void GetBTInputBuffer(const byte offset, byte cnt, byte & data[]);
06847 
06858 inline void GetBTOutputBuffer(const byte offset, byte cnt, byte & data[]);
06859 
06870 inline void GetHSInputBuffer(const byte offset, byte cnt, byte & data[]);
06871 
06882 inline void GetHSOutputBuffer(const byte offset, byte cnt, byte & data[]);
06883 
06894 inline void GetUSBInputBuffer(const byte offset, byte cnt, byte & data[]);
06895 
06905 inline void GetUSBOutputBuffer(const byte offset, byte cnt, byte & data[]);
06906 
06916 inline void GetUSBPollBuffer(const byte offset, byte cnt, byte & data[]);
06917 
06925 inline string BTDeviceName(const byte devidx);
06926 
06934 inline string BTConnectionName(const byte conn);
06935 
06943 inline string BTConnectionPinCode(const byte conn);
06944 
06950 inline string BrickDataName(void);
06951 
06959 inline void GetBTDeviceAddress(const byte devidx, byte & data[]);
06960 
06968 inline void GetBTConnectionAddress(const byte conn, byte & data[]);
06969 
06976 inline void GetBrickDataAddress(byte & data[]);
06977 
06985 inline long BTDeviceClass(const byte devidx);
06986 
06994 inline byte BTDeviceStatus(const byte devidx);
06995 
07003 inline long BTConnectionClass(const byte conn);
07004 
07012 inline byte BTConnectionHandleNum(const byte conn);
07013 
07021 inline byte BTConnectionStreamStatus(const byte conn);
07022 
07031 inline byte BTConnectionLinkQuality(const byte conn);
07032 
07038 inline int BrickDataBluecoreVersion(void);
07039 
07045 inline byte BrickDataBtStateStatus(void);
07046 
07052 inline byte BrickDataBtHardwareStatus(void);
07053 
07059 inline byte BrickDataTimeoutValue(void);
07060 
07067 inline byte BTInputBufferInPtr(void);
07068 
07075 inline byte BTInputBufferOutPtr(void);
07076 
07083 inline byte BTOutputBufferInPtr(void);
07084 
07091 inline byte BTOutputBufferOutPtr(void);
07092 
07099 inline byte HSInputBufferInPtr(void);
07100 
07107 inline byte HSInputBufferOutPtr(void);
07108 
07115 inline byte HSOutputBufferInPtr(void);
07116 
07123 inline byte HSOutputBufferOutPtr(void);
07124 
07131 inline byte USBInputBufferInPtr(void);
07132 
07139 inline byte USBInputBufferOutPtr(void);
07140 
07147 inline byte USBOutputBufferInPtr(void);
07148 
07155 inline byte USBOutputBufferOutPtr(void);
07156 
07163 inline byte USBPollBufferInPtr(void);
07164 
07171 inline byte USBPollBufferOutPtr(void);
07172 
07179 inline byte BTDeviceCount(void);
07180 
07188 inline byte BTDeviceNameCount(void);
07189 
07195 inline byte HSFlags(void);
07196 
07202 inline byte HSSpeed(void);
07203 
07209 inline byte HSState(void);
07210 
07211 #if (__FIRMWARE_VERSION > 107) && defined(__ENHANCED_FIRMWARE)
07212 
07222 inline int HSMode(void);
07223 
07231 inline int BTDataMode(void);
07232 
07240 inline int HSDataMode(void);
07241 
07242 #endif
07243 
07249 inline byte USBState(void);
07250 
07258 inline void SetBTInputBuffer(const byte offset, byte cnt, byte data[]);
07259 
07265 inline void SetBTInputBufferInPtr(byte n);
07266 
07272 inline void SetBTInputBufferOutPtr(byte n);
07273 
07281 inline void SetBTOutputBuffer(const byte offset, byte cnt, byte data[]);
07282 
07288 inline void SetBTOutputBufferInPtr(byte n);
07289 
07295 inline void SetBTOutputBufferOutPtr(byte n);
07296 
07304 inline void SetHSInputBuffer(const byte offset, byte cnt, byte data[]);
07305 
07311 inline void SetHSInputBufferInPtr(byte n);
07312 
07318 inline void SetHSInputBufferOutPtr(byte n);
07319 
07327 inline void SetHSOutputBuffer(const byte offset, byte cnt, byte data[]);
07328 
07334 inline void SetHSOutputBufferInPtr(byte n);
07335 
07341 inline void SetHSOutputBufferOutPtr(byte n);
07342 
07350 inline void SetUSBInputBuffer(const byte offset, byte cnt, byte data[]);
07351 
07357 inline void SetUSBInputBufferInPtr(byte n);
07358 
07364 inline void SetUSBInputBufferOutPtr(byte n);
07365 
07373 inline void SetUSBOutputBuffer(const byte offset, byte cnt, byte data[]);
07374 
07380 inline void SetUSBOutputBufferInPtr(byte n);
07381 
07387 inline void SetUSBOutputBufferOutPtr(byte n);
07388 
07396 inline void SetUSBPollBuffer(const byte offset, byte cnt, byte data[]);
07397 
07403 inline void SetUSBPollBufferInPtr(byte n);
07404 
07410 inline void SetUSBPollBufferOutPtr(byte n);
07411 
07417 inline void SetHSFlags(byte hsFlags);
07418 
07424 inline void SetHSSpeed(byte hsSpeed);
07425 
07431 inline void SetHSState(byte hsState);
07432 
07433 #if (__FIRMWARE_VERSION > 107) && defined(__ENHANCED_FIRMWARE)
07434 
07444 inline void SetHSMode(unsigned int hsMode) { asm { __setHSMode(_n) } }
07445 
07453 inline void SetBTDataMode(const byte dataMode);
07454 
07462 inline void SetHSDataMode(const byte dataMode);
07463 
07464 #endif
07465 
07471 inline void SetUSBState(byte usbState);
07472 
07480 void SysMessageWrite(MessageWriteType & args);
07481 
07489 void SysMessageRead(MessageReadType & args);
07490 
07498 void SysCommBTWrite(CommBTWriteType & args);
07499 
07508 void SysCommBTCheckStatus(CommBTCheckStatusType & args);
07509 
07510 #ifdef __ENHANCED_FIRMWARE
07511 
07521 inline void SysCommExecuteFunction(CommExecuteFunctionType & args);
07522 
07533 inline void SysCommHSControl(CommHSControlType & args);
07534 
07545 inline void SysCommHSCheckStatus(CommHSCheckStatusType & args);
07546 
07557 inline void SysCommHSRead(CommHSReadWriteType & args);
07558 
07569 inline void SysCommHSWrite(CommHSReadWriteType & args);
07570 
07571 #endif
07572 
07573 #if __FIRMWARE_VERSION > 107
07574 
07584 inline void SysCommBTOnOff(CommBTOnOffType & args);
07585 
07596 inline void SysCommBTConnection(CommBTConnectionType & args);
07597 
07598 #endif
07599 
07600 /*
07601 // these functions really cannot be used for any useful purpose (read-only)
07602 inline void SetBTDeviceName(const byte devidx, string str);
07603 inline void SetBTDeviceAddress(const byte devidx, const byte btaddr[]);
07604 inline void SetBTConnectionName(const byte conn, string str);
07605 inline void SetBTConnectionPinCode(const byte conn, const byte code[]);
07606 inline void SetBTConnectionAddress(const byte conn, const byte btaddr[]);
07607 inline void SetBrickDataName(string str);
07608 inline void SetBrickDataAddress(const byte p, byte btaddr[]);
07609 inline void SetBTDeviceClass(const byte devidx, unsigned long class);
07610 inline void SetBTDeviceStatus(const byte devidx, const byte status);
07611 inline void SetBTConnectionClass(const byte conn, unsigned long class);
07612 inline void SetBTConnectionHandleNum(const byte conn, const byte handleNum);
07613 inline void SetBTConnectionStreamStatus(const byte conn, const byte status);
07614 inline void SetBTConnectionLinkQuality(const byte conn, const byte quality);
07615 inline void SetBrickDataBluecoreVersion(int version);
07616 inline void SetBrickDataBtStateStatus(byte status);
07617 inline void SetBrickDataBtHardwareStatus(byte status);
07618 inline void SetBrickDataTimeoutValue(const byte timeout);
07619 inline void SetBTDeviceCount(byte count);
07620 inline void SetBTDeviceNameCount(byte count);
07621 */
07622 
07623 #else
07624 
07625 #define SendMessage(_queue, _msg) asm { __sendMessage(_queue, _msg, __RETVAL__) }
07626 #define ReceiveMessage(_queue, _clear, _msg) asm { __receiveMessage(_queue, _clear, _msg, __RETVAL__) }
07627 
07628 #define BluetoothStatus(_conn) asm { __bluetoothStatus(_conn, __RETVAL__) }
07629 #define BluetoothWrite(_conn, _buffer) asm { __bluetoothWrite(_conn, _buffer, __RETVAL__) }
07630 #define RemoteConnectionWrite(_conn, _buffer) asm { __connectionRawWrite(_conn, _buffer, __RETVAL__) }
07631 #define RemoteConnectionIdle(_conn) asm { __remoteConnectionIdle(_conn, __RETVAL__) }
07632 
07633 #define SendRemoteBool(_conn, _queue, _bval) asm { __sendRemoteBool(_conn, _queue, _bval, __RETVAL__) }
07634 #define SendRemoteNumber(_conn, _queue, _val) asm { __sendRemoteNumber(_conn, _queue, _val, __RETVAL__) }
07635 #define SendRemoteString(_conn, _queue, _str) asm { __sendRemoteString(_conn, _queue, _str, __RETVAL__) }
07636 
07637 #define SendResponseBool(_queue, _bval) asm { __sendResponseBool(_queue, _bval, __RETVAL__) }
07638 #define SendResponseNumber(_queue, _val) asm { __sendResponseNumber(_queue, _val, __RETVAL__) }
07639 #define SendResponseString(_queue, _msg) asm { __sendResponseString(_queue, _msg, __RETVAL__) }
07640 
07641 #define ReceiveRemoteBool(_queue, _clear, _bval) asm { __receiveRemoteBool(_queue, _clear, _bval, __RETVAL__) }
07642 #define ReceiveRemoteNumber(_queue, _clear, _val) asm { __receiveRemoteNumber(_queue, _clear, _val, __RETVAL__) }
07643 #define ReceiveRemoteString(_queue, _clear, _str) asm { __receiveMessage(_queue, _clear, _str, __RETVAL__) }
07644 #define ReceiveRemoteMessageEx(_queue, _clear, _str, _val, _bval) asm { __receiveRemoteMessageEx(_queue, _clear, _str, _val, _bval, __RETVAL__) }
07645 
07646 #define RemoteMessageRead(_conn, _queue) asm { __remoteMessageRead(_conn, _queue, __RETVAL__) }
07647 #define RemoteMessageWrite(_conn, _queue, _msg) asm { __sendRemoteString(_conn, _queue, _msg, __RETVAL__) }
07648 #define RemoteStartProgram(_conn, _filename) asm { __remoteStartProgram(_conn, _filename, __RETVAL__) }
07649 #define RemoteStopProgram(_conn) asm { __connectionWrite(_conn, __DCStopProgramPacket, __RETVAL__) }
07650 #define RemotePlaySoundFile(_conn, _filename, _bloop) asm { __remotePlaySoundFile(_conn, _filename, _bloop, __RETVAL__) }
07651 #define RemotePlayTone(_conn, _frequency, _duration) asm { __remotePlayTone(_conn, _frequency, _duration, __RETVAL__) }
07652 #define RemoteStopSound(_conn) asm { __connectionWrite(_conn, __DCStopSoundPacket, __RETVAL__) }
07653 #define RemoteKeepAlive(_conn) asm { __connectionWrite(_conn, __DCKeepAlivePacket, __RETVAL__) }
07654 #define RemoteResetScaledValue(_conn, _port) asm { __remoteResetScaledValue(_conn, _port, __RETVAL__) }
07655 #define RemoteResetMotorPosition(_conn, _port, _brelative) asm { __remoteResetMotorPosition(_conn, _port, _brelative, __RETVAL__) }
07656 #define RemoteSetInputMode(_conn, _port, _type, _mode) asm { __remoteSetInputMode(_conn, _port, _type, _mode, __RETVAL__) }
07657 #define RemoteSetOutputState(_conn, _port, _speed, _mode, _regmode, _turnpct, _runstate, _tacholimit) asm { __remoteSetOutputState(_conn, _port, _speed, _mode, _regmode, _turnpct, _runstate, _tacholimit, __RETVAL__) }
07658 #define RemoteResetTachoCount(_conn, _port) asm { __remoteResetTachoCount(_conn, _port, __RETVAL__) }
07659 #define RemoteDatalogSetTimes(_conn, _synctime) asm { __remoteDatalogSetTimes(_conn, _synctime, __RETVAL__) }
07660 #define RemoteSetProperty(_conn, _prop, _value) asm { __remoteSetProperty(_conn, _prop, _value, __RETVAL__) }
07661 #define RemoteLowspeedWrite(_conn, _port, _txlen, _rxlen, _data) asm { __remoteLowspeedWrite(_conn, _port, _txlen, _rxlen, _data, __RETVAL__) }
07662 
07663 #ifdef __ENHANCED_FIRMWARE
07664 #define RemoteGetOutputState(_conn, _params) asm { \
07665   compchktype _params, OutputStateType \
07666   __remoteGetOutputState(_conn, _params, __RETVAL__) \
07667 }
07668 #define RemoteGetInputValues(_conn, _params) asm { \
07669   compchktype _params, InputValuesType \
07670   __remoteGetInputValues(_conn, _params, __RETVAL__) \
07671 }
07672 #define RemoteGetBatteryLevel(_conn, _value) asm { __remoteGetBatteryLevel(_conn, _value, __RETVAL__) }
07673 #define RemoteLowspeedGetStatus(_conn, _value) asm { __remoteLowspeedGetStatus(_conn, _value, __RETVAL__) }
07674 #define RemoteLowspeedRead(_conn, _port, _bread, _data) asm { __remoteLowspeedRead(_conn, _port, _bread, _data, __RETVAL__) }
07675 #define RemoteGetCurrentProgramName(_conn, _name) asm { __remoteGetCurrentProgramName(_conn, _name, __RETVAL__) }
07676 #define RemoteDatalogRead(_conn, _remove, _cnt, _log) asm { __remoteDatalogRead(_conn, _remove, _cnt, _log, __RETVAL__) }
07677 #define RemoteGetContactCount(_conn, _cnt) asm { __remoteGetContactCount(_conn, _cnt, __RETVAL__) }
07678 #define RemoteGetContactName(_conn, _idx, _name) asm { __remoteGetContactName(_conn, _idx, _name, __RETVAL__) }
07679 #define RemoteGetConnectionCount(_conn, _cnt) asm { __remoteGetConnectionCount(_conn, _cnt, __RETVAL__) }
07680 #define RemoteGetConnectionName(_conn, _idx, _name) asm { __remoteGetConnectionName(_conn, _idx, _name, __RETVAL__) }
07681 
07682 #define RemoteGetProperty(_conn, _property, _value) asm { __remoteGetProperty(_conn, _property, _value, __RETVAL__) }
07683 
07684 #else
07685 
07686 #define RemoteGetOutputState(_conn, _port) asm { __remoteGetOutputState(_conn, _port, __RETVAL__) }
07687 #define RemoteGetInputValues(_conn, _port) asm { __remoteGetInputValues(_conn, _port, __RETVAL__) }
07688 #define RemoteGetBatteryLevel(_conn) asm { __remoteGetBatteryLevel(_conn, __RETVAL__) }
07689 #define RemoteLowspeedGetStatus(_conn) asm { __remoteLowspeedGetStatus(_conn, __RETVAL__) }
07690 #define RemoteLowspeedRead(_conn, _port) asm { __remoteLowspeedRead(_conn, _port, __RETVAL__) }
07691 #define RemoteGetCurrentProgramName(_conn) asm { __remoteGetCurrentProgramName(_conn, __RETVAL__) }
07692 #define RemoteDatalogRead(_conn, _remove) asm { __remoteDatalogRead(_conn, _remove, __RETVAL__) }
07693 #define RemoteGetContactCount(_conn) asm { __remoteGetContactCount(_conn, __RETVAL__) }
07694 #define RemoteGetContactName(_conn, _idx) asm { __remoteGetContactName(_conn, _idx, __RETVAL__) }
07695 #define RemoteGetConnectionCount(_conn) asm { __remoteGetConnectionCount(_conn, __RETVAL__) }
07696 #define RemoteGetConnectionName(_conn, _idx) asm { __remoteGetConnectionName(_conn, _idx, __RETVAL__) }
07697 #define RemoteGetProperty(_conn, _property) asm { __remoteGetProperty(_conn, _property, __RETVAL__) }
07698 
07699 #endif
07700 
07701 #ifdef __ENHANCED_FIRMWARE
07702 
07703 #define RemoteOpenRead(_conn, _filename, _handle, _size) asm { __remoteOpenRead(_conn, _filename, _handle, _size, __RETVAL__) }
07704 #define RemoteOpenWrite(_conn, _filename, _size, _handle) asm { __remoteOpenWrite(_conn, _filename, _size, _handle, __RETVAL__) }
07705 #define RemoteRead(_conn, _handle, _numbytes, _data) asm { __remoteRead(_conn, _handle, _numbytes, _data, __RETVAL__) }
07706 #define RemoteWrite(_conn, _handle, _numbytes, _data) asm { __remoteWrite(_conn, _handle, _numbytes, _data, __RETVAL__) }
07707 #define RemoteCloseFile(_conn, _handle) asm { __remoteCloseFile(_conn, _handle, __RETVAL__) }
07708 #define RemoteDeleteFile(_conn, _filename) asm { __remoteDeleteFile(_conn, _filename, __RETVAL__) }
07709 #define RemoteDeleteUserFlash(_conn) asm { __remoteDeleteUserFlash(_conn, __RETVAL__) }
07710 #define RemoteFindFirstFile(_conn, _mask, _handle, _name, _size) asm { __remoteFindFirstFile(_conn, _mask, _handle, _name, _size, __RETVAL__) }
07711 #define RemoteFindNextFile(_conn, _handle, _name, _size) asm { __remoteFindNextFile(_conn, _handle, _name, _size, __RETVAL__) }
07712 #define RemoteGetFirmwareVersion(_conn, _pmin, _pmaj, _fmin, _fmaj) asm { __remoteGetFirmwareVersion(_conn, _pmin, _pmaj, _fmin, _fmaj, __RETVAL__) }
07713 #define RemoteOpenWriteLinear(_conn, _filename, _size, _handle) asm { __remoteOpenWriteLinear(_conn, _filename, _size, _handle, __RETVAL__) }
07714 #define RemoteOpenWriteData(_conn, _filename, _size, _handle) asm { __remoteOpenWriteData(_conn, _filename, _size, _handle, __RETVAL__) }
07715 #define RemoteOpenAppendData(_conn, _filename, _handle, _size) asm { __remoteOpenAppendData(_conn, _filename, _handle, _size, __RETVAL__) }
07716 #define RemoteGetDeviceInfo(_conn, _name, _btaddr, _btsignal, _freemem) asm { __remoteGetDeviceInfo(_conn, _name, _btaddr, _btsignal, _freemem, __RETVAL__) }
07717 #define RemotePollCommandLength(_conn, _bufnum, _length) asm { __remotePollCommandLength(_conn, _bufnum, _length, __RETVAL__) }
07718 #define RemotePollCommand(_conn, _bufnum, _len, _data) asm { __remotePollCommand(_conn, _bufnum, _len, _data, __RETVAL__) }
07719 #define RemoteIOMapRead(_conn, _id, _offset, _numbytes, _data) asm { __remoteIOMapRead(_conn, _id, _offset, _numbytes, _data, __RETVAL__) }
07720 #define RemoteGetBluetoothAddress(_conn, _btaddr) asm { __remoteGetBluetoothAddress(_conn, _btaddr, __RETVAL__) }
07721 
07722 #define RemoteRenameFile(_conn, _oldname, _newname) asm { __remoteRenameFile(_conn, _oldname, _newname, __RETVAL__) }
07723 
07724 #else
07725 
07726 #define RemoteOpenRead(_conn, _filename) asm { __remoteOpenRead(_conn, _filename, __RETVAL__) }
07727 #define RemoteOpenWrite(_conn, _filename, _size) asm { __remoteOpenWrite(_conn, _filename, _size, __RETVAL__) }
07728 #define RemoteRead(_conn, _handle, _numbytes) asm { __remoteRead(_conn, _handle, _numbytes, __RETVAL__) }
07729 #define RemoteWrite(_conn, _handle, _data) asm { __remoteWrite(_conn, _handle, _data, __RETVAL__) }
07730 #define RemoteCloseFile(_conn, _handle) asm { __remoteCloseFile(_conn, _handle, __RETVAL__) }
07731 #define RemoteDeleteFile(_conn, _filename) asm { __remoteDeleteFile(_conn, _filename, __RETVAL__) }
07732 #define RemoteDeleteUserFlash(_conn) asm { __connectionWrite(_conn, __SCDeleteUserFlashPacket, __RETVAL__) }
07733 #define RemoteFindFirstFile(_conn, _mask) asm { __remoteFindFirstFile(_conn, _mask, __RETVAL__) }
07734 #define RemoteFindNextFile(_conn, _handle) asm { __remoteFindNextFile(_conn, _handle, __RETVAL__) }
07735 #define RemoteGetFirmwareVersion(_conn) asm { __connectionWrite(_conn, __SCGetFirmwareVerPacket, __RETVAL__) }
07736 #define RemoteOpenWriteLinear(_conn, _filename, _size) asm { __remoteOpenWriteLinear(_conn, _filename, _size, __RETVAL__) }
07737 #define RemoteOpenWriteData(_conn, _filename, _size) asm { __remoteOpenWriteData(_conn, _filename, _size, __RETVAL__) }
07738 #define RemoteOpenAppendData(_conn, _filename) asm { __remoteOpenAppendData(_conn, _filename, __RETVAL__) }
07739 #define RemoteGetDeviceInfo(_conn) asm { __connectionWrite(_conn, __SCGetDeviceInfoPacket, __RETVAL__) }
07740 #define RemotePollCommandLength(_conn, _bufnum) asm { __remotePollCommandLength(_conn, _bufnum, __RETVAL__) }
07741 #define RemotePollCommand(_conn, _bufnum, _len) asm { __remotePollCommand(_conn, _bufnum, _len, __RETVAL__) }
07742 #define RemoteIOMapRead(_conn, _id, _offset, _numbytes) asm { __remoteIOMapRead(_conn, _id, _offset, _numbytes, __RETVAL__) }
07743 #define RemoteGetBluetoothAddress(_conn) asm { __connectionWrite(_conn, __SCBTGetAddressPacket, __RETVAL__) }
07744 
07745 #endif
07746 
07747 #define RemoteBluetoothFactoryReset(_conn) asm { __connectionWrite(_conn, __SCBTFactoryResetPacket, __RETVAL__) }
07748 #define RemoteIOMapWriteValue(_conn, _id, _offset, _value) asm { __remoteIOMapWriteValue(_conn, _id, _offset, _value, __RETVAL__) }
07749 #define RemoteIOMapWriteBytes(_conn, _id, _offset, _data) asm { __remoteIOMapWriteBytes(_conn, _id, _offset, _data, __RETVAL__) }
07750 #define RemoteSetBrickName(_conn, _name) asm { __remoteSetBrickName(_conn, _name, __RETVAL__) }
07751 
07752 #define UseRS485() asm { __UseRS485() }
07753 
07754 #ifdef __ENHANCED_FIRMWARE
07755 
07756 #define RS485Status(_sendingData, _dataAvail) asm { __RS485Status(_sendingData, _dataAvail) }
07757 #define RS485SendingData() asm { __RS485Status(__RETVAL__, __TMPBYTE__) }
07758 #define RS485DataAvailable() asm { __RS485Status(__TMPBYTE__, __RETVAL__) }
07759 #define RS485Write(_buffer) asm { __RS485Write(_buffer, __RETVAL__) }
07760 #define RS485Read(_buffer) asm { __RS485Read(_buffer, __RETVAL__) }
07761 
07762 #if __FIRMWARE_VERSION > 107
07763 
07764 #define RS485Control(_cmd, _baud, _mode) asm { __RS485Control(_cmd, _baud, _mode, __RETVAL__) }
07765 #define RS485Uart(_baud, _mode) asm { __RS485Control(HS_CTRL_UART, _baud, _mode, __RETVAL__) }
07766 #define RS485Initialize() asm { __RS485Control(HS_CTRL_UART, HS_BAUD_DEFAULT, HS_MODE_DEFAULT, __RETVAL__) }
07767 #define RS485Enable() asm { __RS485Control(HS_CTRL_INIT, HS_BAUD_DEFAULT, HS_MODE_DEFAULT, __RETVAL__) }
07768 #define RS485Disable() asm { __RS485Control(HS_CTRL_EXIT, HS_BAUD_DEFAULT, HS_MODE_DEFAULT, __RETVAL__) }
07769 
07770 #else
07771 
07772 #define RS485Control(_cmd, _baud) asm { __RS485Control(_cmd, _baud, __RETVAL__) }
07773 #define RS485Uart(_baud) asm { __RS485Control(HS_CTRL_UART, _baud, __RETVAL__) }
07774 #define RS485Initialize() asm { __RS485Control(HS_CTRL_UART, HS_BAUD_DEFAULT, __RETVAL__) }
07775 #define RS485Enable() asm { __RS485Control(HS_CTRL_INIT, HS_BAUD_DEFAULT, __RETVAL__) }
07776 #define RS485Disable() asm { __RS485Control(HS_CTRL_EXIT, HS_BAUD_DEFAULT, __RETVAL__) }
07777 
07778 #endif
07779 
07780 #define SendRS485Bool(_bval) asm { __sendRS485Bool(_bval, __RETVAL__) }
07781 #define SendRS485Number(_val) asm { __sendRS485Number(_val, __RETVAL__) }
07782 #define SendRS485String(_str) asm { __sendRS485String(_str, __RETVAL__) }
07783 
07784 #endif
07785 
07786 #define GetBTInputBuffer(_offset, _cnt, _data) asm { __getBTInputBuffer(_offset, _cnt, _data) }
07787 #define GetBTOutputBuffer(_offset, _cnt, _data) asm { __getBTOutputBuffer(_offset, _cnt, _data) }
07788 #define GetHSInputBuffer(_offset, _cnt, _data) asm { __getHSInputBuffer(_offset, _cnt, _data) }
07789 #define GetHSOutputBuffer(_offset, _cnt, _data) asm { __getHSOutputBuffer(_offset, _cnt, _data) }
07790 #define GetUSBInputBuffer(_offset, _cnt, _data) asm { __getUSBInputBuffer(_offset, _cnt, _data) }
07791 #define GetUSBOutputBuffer(_offset, _cnt, _data) asm { __getUSBOutputBuffer(_offset, _cnt, _data) }
07792 #define GetUSBPollBuffer(_offset, _cnt, _data) asm { __getUSBPollBuffer(_offset, _cnt, _data) }
07793 
07794 #define BTDeviceName(_p) asm { GetBTDeviceName(_p, __STRRETVAL__) }
07795 #define BTConnectionName(_p) asm { GetBTConnectionName(_p, __STRRETVAL__) }
07796 #define BTConnectionPinCode(_p) asm { GetBTConnectionPinCode(_p, __STRRETVAL__) }
07797 #define BrickDataName() asm { GetBrickDataName(__STRRETVAL__) }
07798 
07799 #define GetBTDeviceAddress(_p, _data) asm { __getBTDeviceAddress(_p, _data) }
07800 #define GetBTConnectionAddress(_p, _data) asm { __getBTConnectionAddress(_p, _data) }
07801 #define GetBrickDataAddress(_data) asm { __getCommModuleBytes(CommOffsetBrickDataBdAddr, 7, _data) }
07802 
07803 #define BTDeviceClass(_p) asm { GetBTDeviceClass(_p, __TMPLONG__) __RETURN__ __TMPLONG__ }
07804 #define BTDeviceStatus(_p) asm { GetBTDeviceStatus(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
07805 #define BTConnectionClass(_p) asm { GetBTConnectionClass(_p, __TMPLONG__) __RETURN__ __TMPLONG__ }
07806 #define BTConnectionHandleNum(_p) asm { GetBTConnectionHandleNum(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
07807 #define BTConnectionStreamStatus(_p) asm { GetBTConnectionStreamStatus(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
07808 #define BTConnectionLinkQuality(_p) asm { GetBTConnectionLinkQuality(_p, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
07809 #define BrickDataBluecoreVersion() asm { GetBrickDataBluecoreVersion(__TMPWORD__) __RETURN__ __TMPWORD__ }
07810 #define BrickDataBtStateStatus() asm { GetBrickDataBtStateStatus(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07811 #define BrickDataBtHardwareStatus() asm { GetBrickDataBtHardwareStatus(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07812 #define BrickDataTimeoutValue() asm { GetBrickDataTimeoutValue(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07813 #define BTInputBufferInPtr() asm { GetBTInputBufferInPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07814 #define BTInputBufferOutPtr() asm { GetBTInputBufferOutPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07815 #define BTOutputBufferInPtr() asm { GetBTOutputBufferInPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07816 #define BTOutputBufferOutPtr() asm { GetBTOutputBufferOutPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07817 #define HSInputBufferInPtr() asm { GetHSInputBufferInPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07818 #define HSInputBufferOutPtr() asm { GetHSInputBufferOutPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07819 #define HSOutputBufferInPtr() asm { GetHSOutputBufferInPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07820 #define HSOutputBufferOutPtr() asm { GetHSOutputBufferOutPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07821 #define USBInputBufferInPtr() asm { GetUSBInputBufferInPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07822 #define USBInputBufferOutPtr() asm { GetUSBInputBufferOutPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07823 #define USBOutputBufferInPtr() asm { GetUSBOutputBufferInPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07824 #define USBOutputBufferOutPtr() asm { GetUSBOutputBufferOutPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07825 #define USBPollBufferInPtr() asm { GetUSBPollBufferInPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07826 #define USBPollBufferOutPtr() asm { GetUSBPollBufferOutPtr(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07827 #define BTDeviceCount() asm { GetBTDeviceCount(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07828 #define BTDeviceNameCount() asm { GetBTDeviceNameCount(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07829 #define HSFlags() asm { GetHSFlags(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07830 #define HSSpeed() asm { GetHSSpeed(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07831 #define HSState() asm { GetHSState(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07832 #define USBState() asm { GetUSBState(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07833 
07834 #if (__FIRMWARE_VERSION > 107) && defined(__ENHANCED_FIRMWARE)
07835 #define HSMode() asm { GetHSMode(__TMPWORD__) __RETURN__ __TMPWORD__ }
07836 #define BTDataMode() asm { GetBTDataMode(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07837 #define HSDataMode() asm { GetHSDataMode(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
07838 #endif
07839 
07840 #define SetBTDeviceName(_p, _str) asm { __setBTDeviceName(_p, _str) }
07841 #define SetBTDeviceAddress(_p, _btaddr) asm { __setBTDeviceAddress(_p, _btaddr) }
07842 #define SetBTConnectionName(_p, _str) asm { __setBTConnectionName(_p, _str) }
07843 #define SetBTConnectionPinCode(_p, _code) asm { __setBTConnectionPinCode(_p, _code) }
07844 #define SetBTConnectionAddress(_p, _btaddr) asm { __setBTConnectionAddress(_p, _btaddr) }
07845 #define SetBrickDataName(_str) SetCommModuleBytes(CommOffsetBrickDataName, 16, _str)
07846 #define SetBrickDataAddress(_btaddr) SetCommModuleBytes(CommOffsetBrickDataBdAddr, 7, _btaddr)
07847 
07848 #define SetBTDeviceClass(_p, _n) asm { __setBTDeviceClass(_p, _n) }
07849 #define SetBTDeviceStatus(_p, _n) asm { __setBTDeviceStatus(_p, _n) }
07850 #define SetBTConnectionClass(_p, _n) asm { __setBTConnectionClass(_p, _n) }
07851 #define SetBTConnectionHandleNum(_p, _n) asm { __setBTConnectionHandleNum(_p, _n) }
07852 #define SetBTConnectionStreamStatus(_p, _n) asm { __setBTConnectionStreamStatus(_p, _n) }
07853 #define SetBTConnectionLinkQuality(_p, _n) asm { __setBTConnectionLinkQuality(_p, _n) }
07854 #define SetBrickDataBluecoreVersion(_n) asm { __setBrickDataBluecoreVersion(_n) }
07855 #define SetBrickDataBtStateStatus(_n) asm { __setBrickDataBtStateStatus(_n) }
07856 #define SetBrickDataBtHardwareStatus(_n) asm { __setBrickDataBtHardwareStatus(_n) }
07857 #define SetBrickDataTimeoutValue(_n) asm { __setBrickDataTimeoutValue(_n) }
07858 
07859 #define SetBTDeviceCount(_n) asm { __setBTDeviceCount(_n) }
07860 #define SetBTDeviceNameCount(_n) asm { __setBTDeviceNameCount(_n) }
07861 
07862 #define SetBTInputBuffer(_offset, _cnt, _data) asm { __setBTInputBuffer(_offset, _cnt, _data) }
07863 
07864 #define SetBTInputBufferInPtr(_n) asm { __setBTInputBufferInPtr(_n) }
07865 #define SetBTInputBufferOutPtr(_n) asm { __setBTInputBufferOutPtr(_n) }
07866 
07867 #define SetBTOutputBuffer(_offset, _cnt, _data) asm { __setBTOutputBuffer(_offset, _cnt, _data) }
07868 
07869 #define SetBTOutputBufferInPtr(_n) asm { __setBTOutputBufferInPtr(_n) }
07870 #define SetBTOutputBufferOutPtr(_n) asm { __setBTOutputBufferOutPtr(_n) }
07871 
07872 #define SetHSInputBuffer(_offset, _cnt, _data) asm { __setHSInputBuffer(_offset, _cnt, _data) }
07873 
07874 #define SetHSInputBufferInPtr(_n) asm { __setHSInputBufferInPtr(_n) }
07875 #define SetHSInputBufferOutPtr(_n) asm { __setHSInputBufferOutPtr(_n) }
07876 
07877 #define SetHSOutputBuffer(_offset, _cnt, _data) asm { __setHSOutputBuffer(_offset, _cnt, _data) }
07878 
07879 #define SetHSOutputBufferInPtr(_n) asm { __setHSOutputBufferInPtr(_n) }
07880 #define SetHSOutputBufferOutPtr(_n) asm { __setHSOutputBufferOutPtr(_n) }
07881 
07882 #define SetUSBInputBuffer(_offset, _cnt, _data) asm { __setUSBInputBuffer(_offset, _cnt, _data) }
07883 
07884 #define SetUSBInputBufferInPtr(_n) asm { __setUSBInputBufferInPtr(_n) }
07885 #define SetUSBInputBufferOutPtr(_n) asm { __setUSBInputBufferOutPtr(_n) }
07886 
07887 #define SetUSBOutputBuffer(_offset, _cnt, _data) asm { __setUSBOutputBuffer(_offset, _cnt, _data) }
07888 
07889 #define SetUSBOutputBufferInPtr(_n) asm { __setUSBOutputBufferInPtr(_n) }
07890 #define SetUSBOutputBufferOutPtr(_n) asm { __setUSBOutputBufferOutPtr(_n) }
07891 
07892 #define SetUSBPollBuffer(_offset, _cnt, _data) asm { __setUSBPollBuffer(_offset, _cnt, _data) }
07893 
07894 #define SetUSBPollBufferInPtr(_n) asm { __setUSBPollBufferInPtr(_n) }
07895 #define SetUSBPollBufferOutPtr(_n) asm { __setUSBPollBufferOutPtr(_n) }
07896 
07897 #define SetHSFlags(_n) asm { __setHSFlags(_n) }
07898 #define SetHSSpeed(_n) asm { __setHSSpeed(_n) }
07899 #define SetHSState(_n) asm { __setHSState(_n) }
07900 #define SetUSBState(_n) asm { __setUSBState(_n) }
07901 
07902 #if (__FIRMWARE_VERSION > 107) && defined(__ENHANCED_FIRMWARE)
07903 #define SetBTDataMode(_n) asm { __setBTDataMode(_n) }
07904 #define SetHSDataMode(_n) asm { __setHSDataMode(_n) }
07905 #endif
07906 
07907 #define SysMessageWrite(_args) asm { \
07908   compchktype _args, MessageWriteType \
07909   syscall MessageWrite, _args \
07910 }
07911 #define SysMessageRead(_args) asm { \
07912   compchktype _args, MessageReadType \
07913   syscall MessageRead, _args \
07914 }
07915 #define SysCommBTWrite(_args) asm { \
07916   compchktype _args, CommBTWriteType \
07917   syscall CommBTWrite, _args \
07918 }
07919 #define SysCommBTCheckStatus(_args) asm { \
07920   compchktype _args, CommBTCheckStatusType \
07921   syscall CommBTCheckStatus, _args \
07922 }
07923 #ifdef __ENHANCED_FIRMWARE
07924 #define SysCommExecuteFunction(_args) asm { \
07925   compchktype _args, CommExecuteFunctionType \
07926   syscall CommExecuteFunction, _args \
07927 }
07928 #define SysCommHSControl(_args) asm { \
07929   compchktype _args, CommHSControlType \
07930   syscall CommHSControl, _args \
07931 }
07932 #define SysCommHSCheckStatus(_args) asm { \
07933   compchktype _args, CommHSCheckStatusType \
07934   syscall CommHSCheckStatus, _args \
07935 }
07936 #define SysCommHSRead(_args) asm { \
07937   compchktype _args, CommHSReadWriteType \
07938   syscall CommHSRead, _args \
07939 }
07940 #define SysCommHSWrite(_args) asm { \
07941   compchktype _args, CommHSReadWriteType \
07942   syscall CommHSWrite, _args \
07943 }
07944 #endif
07945 #if __FIRMWARE_VERSION > 107
07946 #define SysCommBTOnOff(_args) asm { \
07947   compchktype _args, CommBTOnOffType \
07948   syscall CommBTOnOff, _args \
07949 }
07950 #define SysCommBTConnection(_args) asm { \
07951   compchktype _args, CommBTConnectionType \
07952   syscall CommBTConnection, _args \
07953 }
07954 #endif
07955 
07956 #endif
07957  // end of CommModuleFunctions group // end of CommModule group // end of NXTFirmwareModules group
07960 
07961 
07965 
07966 
07983 struct ReadButtonType {
07984   char Result;   
07985   byte Index;    
07986   bool Pressed;  
07987   byte Count;    
07988   bool Reset;    
07989 }; // end of ButtonModuleTypes group
07996 #ifdef __DOXYGEN_DOCS
07997 
08007 inline bool ButtonPressed(const byte btn, bool resetCount);
08008 
08019 inline byte ButtonCount(const byte btn, bool resetCount);
08020 
08033 inline char ReadButtonEx(const byte btn, bool reset, bool & pressed, unsigned int & count);
08034 
08041 inline byte ButtonPressCount(const byte btn);
08042 
08050 inline byte ButtonLongPressCount(const byte btn);
08051 
08059 inline byte ButtonShortReleaseCount(const byte btn);
08060 
08068 inline byte ButtonLongReleaseCount(const byte btn);
08069 
08077 inline byte ButtonReleaseCount(const byte btn);
08078 
08086 inline byte ButtonState(const byte btn);
08087 
08095 inline void SetButtonLongPressCount(const byte btn, const byte n);
08096 
08104 inline void SetButtonLongReleaseCount(const byte btn, const byte n);
08105 
08113 inline void SetButtonPressCount(const byte btn, const byte n);
08114 
08122 inline void SetButtonReleaseCount(const byte btn, const byte n);
08123 
08131 inline void SetButtonShortReleaseCount(const byte btn, const byte n);
08132 
08140 inline void SetButtonState(const byte btn, const byte state);
08141 
08149 inline void SysReadButton(ReadButtonType & args);
08150 
08151 #else
08152 
08153 #define ButtonPressCount(_b) asm { GetButtonPressCount(_b, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
08154 #define ButtonLongPressCount(_b) asm { GetButtonLongPressCount(_b, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
08155 #define ButtonShortReleaseCount(_b) asm { GetButtonShortReleaseCount(_b, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
08156 #define ButtonLongReleaseCount(_b) asm { GetButtonLongReleaseCount(_b, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
08157 #define ButtonReleaseCount(_b) asm { GetButtonReleaseCount(_b, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
08158 #define ButtonState(_b) asm { GetButtonState(_b, __TMPBYTE__) __RETURN__ __TMPBYTE__ }
08159 
08160 #define SetButtonPressCount(_b, _n) asm { __setButtonPressCount(_b, _n) }
08161 #define SetButtonLongPressCount(_b, _n) asm { __setButtonLongPressCount(_b, _n) }
08162 #define SetButtonShortReleaseCount(_b, _n) asm { __setButtonShortReleaseCount(_b, _n) }
08163 #define SetButtonLongReleaseCount(_b, _n) asm { __setButtonLongReleaseCount(_b, _n) }
08164 #define SetButtonReleaseCount(_b, _n) asm { __setButtonReleaseCount(_b, _n) }
08165 #define SetButtonState(_b, _n) asm { __setButtonState(_b, _n) }
08166 
08167 #define SysReadButton(_args) asm { \
08168   compchktype _args, ReadButtonType \
08169   syscall ReadButton, _args \
08170 }
08171 #endif
08172  // end of ButtonModuleFunctions group // end of ButtonModule group // end of NXTFirmwareModules group
08175 
08176 
08180 
08181 
08193 #if __FIRMWARE_VERSION > 107
08194 
08200 struct SetSleepTimeoutType {
08201  char Result;                     
08202  unsigned long TheSleepTimeoutMS; 
08203 };
08204 #endif
08205  // end of UiModuleTypes group
08212 #ifdef __DOXYGEN_DOCS
08213 
08219 inline byte CommandFlags(void);
08220 
08226 inline byte UIState(void);
08227 
08233 inline byte UIButton(void);
08234 
08240 inline byte VMRunState(void);
08241 
08247 inline byte BatteryState(void);
08248 
08254 inline byte BluetoothState(void);
08255 
08261 inline byte UsbState(void);
08262 
08269 inline byte SleepTimeout(void);
08270 
08278 inline byte SleepTime(void);
08279 
08287 inline byte SleepTimer(void);
08288 
08294 inline bool RechargeableBattery(void);
08295 
08301 inline byte Volume(void);
08302 
08309 inline byte OnBrickProgramPointer(void);
08310 
08318 inline byte AbortFlag(void);
08319 
08328 inline byte LongAbort(void);
08329 
08335 inline unsigned int BatteryLevel(void);
08336 
08343 inline void SetCommandFlags(const byte cmdFlags);
08344 
08351 inline void SetUIButton(byte btn);
08352 
08359 inline void SetUIState(byte state);
08360 
08370 inline void SetVMRunState(const byte vmRunState);
08371 
08378 inline void SetBatteryState(byte state);
08379 
08386 inline void SetBluetoothState(byte state);
08387 
08394 inline void SetSleepTimeout(const byte n);
08395 
08403 inline void SetSleepTime(const byte n);
08404 
08411 inline void SetSleepTimer(const byte n);
08412 
08419 inline void SetVolume(byte volume);
08420 
08427 inline void SetOnBrickProgramPointer(byte obpStep);
08428 
08434 inline void ForceOff(byte num);
08435 
08446 inline void SetAbortFlag(byte abortFlag);
08447 
08459 inline void SetLongAbort(bool longAbort);
08460 
08461 #if __FIRMWARE_VERSION > 107
08462 
08471 inline void SysSetSleepTimeout(SetSleepTimeoutType & args);
08472 #endif
08473 
08474 #else
08475 
08476 #define CommandFlags() asm { GetCommandFlags(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08477 #define UIState() asm { GetUIState(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08478 #define UIButton() asm { GetUIButton(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08479 #define VMRunState() asm { GetVMRunState(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08480 #define BatteryState() asm { GetBatteryState(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08481 #define BluetoothState() asm { GetBluetoothState(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08482 #define UsbState() asm { GetUsbState(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08483 #define SleepTimeout() asm { GetSleepTimeout(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08484 #define SleepTime() SleepTimeout()
08485 #define SleepTimer() asm { GetSleepTimer(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08486 #define RechargeableBattery() asm { GetRechargeableBattery(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08487 #define Volume() asm { GetVolume(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08488 #define OnBrickProgramPointer() asm { GetOnBrickProgramPointer(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08489 #define AbortFlag() asm { GetAbortFlag(__TMPBYTE__) __RETURN__ __TMPBYTE__ }
08490 #define LongAbort() AbortFlag()
08491 #define BatteryLevel() asm { GetBatteryLevel(__TMPWORD__) __RETURN__ __TMPWORD__ }
08492 
08493 #define SetCommandFlags(_n) asm { __setCommandFlags(_n) }
08494 #define SetUIState(_n) asm { __setUIState(_n) }
08495 #define SetUIButton(_n) asm { __setUIButton(_n) }
08496 #define SetVMRunState(_n) asm { __setVMRunState(_n) }
08497 #define SetBatteryState(_n) asm { __setBatteryState(_n) }
08498 #define SetBluetoothState(_n) asm { __setBluetoothState(_n) }
08499 #define SetUsbState(_n) asm { __setUsbState(_n) }
08500 #define SetSleepTimeout(_n) asm { __setSleepTimeout(_n) }
08501 #define SetSleepTime(_n) SetSleepTimeout(_n)
08502 #define SetSleepTimer(_n) asm { __setSleepTimer(_n) }
08503 #define SetVolume(_n) asm { __setVolume(_n) }
08504 #define SetOnBrickProgramPointer(_n) asm { __setOnBrickProgramPointer(_n) }
08505 #define ForceOff(_n) asm { __forceOff(_n) }
08506 #define SetAbortFlag(_n) asm { __setAbortFlag(_n) }
08507 #define SetLongAbort(_n) do { \
08508   if (_n) { \
08509     asm { __setAbortFlag(BTNSTATE_LONG_PRESSED_EV) } \
08510   } else { \
08511     asm { __setAbortFlag(BTNSTATE_PRESSED_EV) } \
08512   } \
08513 } while(false)
08514 
08515 #if __FIRMWARE_VERSION > 107
08516 #define SysSetSleepTimeout(_args) asm { \
08517   compchktype _args, SetSleepTimeoutType \
08518   syscall SetSleepTimeoutVal, _args \
08519 }
08520 #endif
08521 
08522 #endif
08523  // end of UiModuleFunctions group // end of UiModule group // end of NXTFirmwareModules group
08526 
08527 
08531 
08532 
08552 struct FileOpenType {
08553   unsigned int Result;    
08555   byte FileHandle;        
08557   string Filename;        
08558   unsigned long Length;   
08566 };
08567 
08574 struct FileReadWriteType {
08575   unsigned int Result;    
08577   byte FileHandle;        
08578   string Buffer;          
08580   unsigned long Length;   
08582 };
08583 
08589 struct FileCloseType {
08590   unsigned int Result;   
08592   byte FileHandle;       
08593 };
08594 
08601 struct FileResolveHandleType {
08602   unsigned int Result;   
08604   byte FileHandle;       
08605   bool WriteHandle;      
08606   string Filename;       
08607 };
08608 
08615 struct FileRenameType {
08616   unsigned int Result;   
08618   string OldFilename;    
08619   string NewFilename;    
08620 };
08621 
08628 struct FileDeleteType {
08629   unsigned int Result;   
08631   string Filename;       
08632 };
08633 
08634 #ifdef __ENHANCED_FIRMWARE
08635 
08686 struct LoaderExecuteFunctionType {
08687   unsigned int Result;    
08689   byte Cmd;               
08690   string Filename;        
08691   byte Buffer[];          
08692   unsigned long Length;   
08693 };
08694 
08701 struct FileFindType {
08702   unsigned int Result;    
08704   byte FileHandle;        
08706   string Filename;        
08708   unsigned long Length;   
08709 };
08710 
08711 #if __FIRMWARE_VERSION > 107
08712 
08717 struct FileSeekType {
08718  unsigned int Result; 
08720  byte FileHandle;     
08721  byte Origin;         
08722  long Length;         
08723 };
08724 
08730 struct FileResizeType {
08731  unsigned int Result;   
08733  byte FileHandle;       
08734  unsigned int NewSize;  
08735 };
08736 
08742 struct FileTellType {
08743  unsigned int Result;     
08745  byte FileHandle;         
08746  unsigned long Position;  
08747 };
08748 
08749 #endif
08750 #endif
08751 #if __FIRMWARE_VERSION > 107
08752 
08757 struct ListFilesType {
08758  char Result;       
08760  string Pattern;    
08761  string FileList[]; 
08763 };
08764 #endif
08765  // end of LoaderModuleTypes group
08770 #ifdef __DOXYGEN_DOCS
08771 
08777 inline unsigned int FreeMemory(void);
08778 
08794 inline unsigned int CreateFile(string fname, unsigned int fsize, byte & handle);
08795 
08809 inline unsigned int OpenFileAppend(string fname, unsigned int & fsize, byte & handle);
08810 
08824 inline unsigned int OpenFileRead(string fname, unsigned int & fsize, byte & handle);
08825 
08835 inline unsigned int CloseFile(byte handle);
08836 
08852 inline unsigned int ResolveHandle(string filename, byte & handle, bool & writeable);
08853 
08864 inline unsigned int RenameFile(string oldname, string newname);
08865 
08875 inline unsigned int DeleteFile(string fname);
08876 
08887 inline unsigned int ResizeFile(string fname, const unsigned int newsize);
08888 
08889 #ifdef __ENHANCED_FIRMWARE
08890 
08906 inline unsigned int CreateFileLinear(string fname, unsigned int fsize, byte & handle);
08907 
08924 inline unsigned int CreateFileNonLinear(string fname, unsigned int fsize, byte & handle);
08925 
08940 inline unsigned int OpenFileReadLinear(string fname, unsigned int & fsize, byte & handle);
08941 
08953 inline unsigned int FindFirstFile(string & fname, byte & handle);
08954 
08965 inline unsigned int FindNextFile(string & fname, byte & handle);
08966 
08967 #endif
08968 
08977 inline unsigned int SizeOf(variant & value);
08978 
08990 inline unsigned int Read(byte handle, variant & value);
08991 
09004 inline unsigned int ReadLn(byte handle, variant & value);
09005 
09019 inline unsigned int ReadBytes(byte handle, unsigned int & length, byte & buf[]);
09020 
09033 inline unsigned int ReadLnString(byte handle, string & output);
09034 
09046 inline unsigned int Write(byte handle, const variant & value);
09047 
09060 inline unsigned int WriteBytes(byte handle, const byte & buf[], unsigned int & cnt);
09061 
09076 inline unsigned int WriteBytesEx(byte handle, unsigned int & len, const byte & buf[]);
09077 
09091 inline unsigned int WriteLn(byte handle, const variant & value);
09092 
09107 inline unsigned int WriteLnString(byte handle, const string & str, unsigned int & cnt);
09108 
09121 inline unsigned int WriteString(byte handle, const string & str, unsigned int & cnt);
09122 
09133 inline void SysFileOpenRead(FileOpenType & args);
09134 
09145 inline void SysFileOpenWrite(FileOpenType & args);
09146 
09157 inline void SysFileOpenAppend(FileOpenType & args);
09158 
09167 inline void SysFileRead(FileReadWriteType & args);
09168 
09177 inline void SysFileWrite(FileReadWriteType & args);
09178 
09186 inline void SysFileClose(FileCloseType & args);
09187 
09197 inline void SysFileResolveHandle(FileResolveHandleType & args);
09198 
09206 inline void SysFileRename(FileRenameType & args);
09207 
09215 inline void SysFileDelete(FileDeleteType & args);
09216 
09217 #ifdef __ENHANCED_FIRMWARE
09218 
09229 inline void SysLoaderExecuteFunction(LoaderExecuteFunctionType & args);
09230 
09239 inline void SysFileFindFirst(FileFindType & args);
09240 
09249 inline void SysFileFindNext(FileFindType & args);
09250 
09260 inline void SysFileOpenWriteLinear(FileOpenType & args);
09261 
09271 inline void SysFileOpenWriteNonLinear(FileOpenType & args);
09272 
09282 inline void SysFileOpenReadLinear(FileOpenType & args);
09283 
09284 #if __FIRMWARE_VERSION > 107
09285 
09294 inline void SysFileSeek(FileSeekType & args);
09295 
09306 inline void SysFileResize(FileResizeType & args);
09307 
09317 inline void SysFileTell(FileTellType & args);
09318 
09319 #endif
09320 #endif
09321 #if __FIRMWARE_VERSION > 107
09322 
09329 inline void SysListFiles(ListFilesType & args);
09330 
09331 #endif
09332 
09333 #else
09334 
09335 #define FreeMemory() asm { GetFreeMemory(__RETVAL__) }
09336 
09337 #define CreateFile(_fname, _fsize, _handle) asm { __createFile(_fname, _fsize, _handle, __RETVAL__) }
09338 #define OpenFileAppend(_fname, _fsize, _handle) asm { __openFileAppend(_fname, _fsize, _handle, __RETVAL__) }
09339 #define OpenFileRead(_fname, _fsize, _handle) asm { __openFileRead(_fname, _fsize, _handle, __RETVAL__) }
09340 #define CloseFile(_handle) asm { __closeFile(_handle, __RETVAL__) }
09341 #define ResolveHandle(_fname, _handle, _writeable) asm { __resolveHandle(_fname, _handle, _writeable, __RETVAL__) }
09342 #define RenameFile(_oldname, _newname) asm { __renameFile(_oldname, _newname, __RETVAL__) }
09343 #define DeleteFile(_fname) asm { __deleteFile(_fname, __RETVAL__) }
09344 #define ResizeFile(_fname, _newsize) asm { __fileResize(_fname, _newsize, __RETVAL__) }
09345 
09346 #ifdef __ENHANCED_FIRMWARE
09347 #define CreateFileLinear(_fname, _fsize, _handle) asm { __createFileLinear(_fname, _fsize, _handle, __RETVAL__) }
09348 #define CreateFileNonLinear(_fname, _fsize, _handle) asm { __createFileNonLinear(_fname, _fsize, _handle, __RETVAL__) }
09349 #define OpenFileReadLinear(_fname, _fsize, _handle) asm { __openFileReadLinear(_fname, _fsize, _handle, __RETVAL__) }
09350 #define FindFirstFile(_fname, _handle) asm { __findFirstFile(_fname, _handle, __RETVAL__) }
09351 #define FindNextFile(_fname, _handle) asm { __findNextFile(_fname, _handle, __RETVAL__) }
09352 #endif
09353 
09354 #define SizeOf(_n) asm { __sizeOF(_n, __RETVAL__) }
09355 #define Read(_handle, _n) asm { __readValue(_handle, _n, __RETVAL__) }
09356 #define ReadLn(_handle, _n) asm { __readLnValue(_handle, _n, __RETVAL__) }
09357 #define ReadBytes(_handle, _len, _buf) asm { __readBytes(_handle, _len, _buf, __RETVAL__) }
09358 #define ReadLnString(_handle, _output) asm { __readLnString(_handle, _output, __RETVAL__) }
09359 
09360 #define Write(_handle, _n) asm { __writeValue(_handle, _n, __RETVAL__) }
09361 #define WriteLn(_handle, _n) asm { __writeLnValue(_handle, _n, __RETVAL__) }
09362 #define WriteString(_handle, _str, _cnt) asm { __writeString(_handle, _str, _cnt, __RETVAL__) }
09363 #define WriteLnString(_handle, _str, _cnt) asm { __writeLnString(_handle, _str, _cnt, __RETVAL__) }
09364 #define WriteBytes(_handle, _buf, _cnt) asm { __writeBytes(_handle, _buf, _cnt, __RETVAL__) }
09365 #define WriteBytesEx(_handle, _len, _buf) asm { __writeBytesEx(_handle, _len, _buf, __RETVAL__) }
09366 
09367 #define SysFileOpenRead(_args) asm { \
09368   compchktype _args, FileOpenType \
09369   syscall FileOpenRead, _args \
09370 }
09371 #define SysFileOpenWrite(_args) asm { \
09372   compchktype _args, FileOpenType \
09373   syscall FileOpenWrite, _args \
09374 }
09375 #define SysFileOpenAppend(_args) asm { \
09376   compchktype _args, FileOpenType \
09377   syscall FileOpenAppend, _args \
09378 }
09379 #define SysFileRead(_args) asm { \
09380   compchktype _args, FileReadWriteType \
09381   syscall FileRead, _args \
09382 }
09383 #define SysFileWrite(_args) asm { \
09384   compchktype _args, FileReadWriteType \
09385   syscall FileWrite, _args \
09386 }
09387 #define SysFileClose(_args) asm { \
09388   compchktype _args, FileCloseType \
09389   syscall FileClose, _args \
09390 }
09391 #define SysFileResolveHandle(_args) asm { \
09392   compchktype _args, FileResolveHandleType \
09393   syscall FileResolveHandle, _args \
09394 }
09395 #define SysFileRename(_args) asm { \
09396   compchktype _args, FileRenameType \
09397   syscall FileRename, _args \
09398 }
09399 #define SysFileDelete(_args) asm { \
09400   compchktype _args, FileDeleteType \
09401   syscall FileDelete, _args \
09402 }
09403 
09404 #ifdef __ENHANCED_FIRMWARE
09405 #define SysLoaderExecuteFunction(_args) asm { \
09406   compchktype _args, LoaderExecuteFunctionType \
09407   syscall LoaderExecuteFunction, _args \
09408 }
09409 #define SysFileFindFirst(_args) asm { \
09410   compchktype _args, FileFindType \
09411   syscall FileFindFirst, _args \
09412 }
09413 #define SysFileFindNext(_args) asm { \
09414   compchktype _args, FileFindType \
09415   syscall FileFindNext, _args \
09416 }
09417 #define SysFileOpenWriteLinear(_args) asm { \
09418   compchktype _args, FileOpenType \
09419   syscall FileOpenWriteLinear, _args \
09420 }
09421 #define SysFileOpenWriteNonLinear(_args) asm { \
09422   compchktype _args, FileOpenType \
09423   syscall FileOpenWriteNonLinear, _args \
09424 }
09425 #define SysFileOpenReadLinear(_args) asm { \
09426   compchktype _args, FileOpenType \
09427   syscall FileOpenReadLinear, _args \
09428 }
09429 #if __FIRMWARE_VERSION > 107
09430 #define SysFileSeek(_args) asm { \
09431   compchktype _args, FileSeekType \
09432   syscall FileSeek, _args \
09433 }
09434 #define SysFileResize(_args) asm { \
09435   compchktype _args, FileResizeType \
09436   syscall FileResize, _args \
09437 }
09438 #define SysFileTell(_args) asm { \
09439   compchktype _args, FileTellType \
09440   syscall FileTell, _args \
09441 }
09442 #endif
09443 #endif
09444 #if __FIRMWARE_VERSION > 107
09445 #define SysListFiles(_args) asm { \
09446   compchktype _args, ListFilesType \
09447   syscall ListFiles, _args \
09448 }
09449 #endif
09450 
09451 #endif
09452  // end of LoaderModuleFunctions group // end of LoaderModule group // end of NXTFirmwareModules group
09456 
09457 
09462 
09463 
09464 
09465 
09466 
09481 inline int SensorHTGyro(const byte & port, int offset = 0) {
09482   asm {
09483     getin __RETVAL__, port, RawValueField
09484     sub __RETVAL__, __RETVAL__, 600
09485     sub __RETVAL__, __RETVAL__, offset
09486   }
09487 }
09488 
09499 inline int SensorHTMagnet(const byte & port, int offset = 0) {
09500   asm {
09501     getin __RETVAL__, port, RawValueField
09502     sub __RETVAL__, __RETVAL__, 600
09503     sub __RETVAL__, __RETVAL__, offset
09504   }
09505 }
09506 
09514 inline int SensorHTEOPD(const byte & port) {
09515   asm {
09516     getin __RETVAL__, port, RawValueField
09517     sub __RETVAL__, 1023, __RETVAL__
09518   }
09519 }
09520 
09528 inline void SetSensorHTEOPD(const byte & port, bool bStandard) {
09529   SetSensorType(port, bStandard ? SENSOR_TYPE_LIGHT_INACTIVE : SENSOR_TYPE_LIGHT_ACTIVE);
09530   SetSensorMode(port, SENSOR_MODE_RAW);
09531   ResetSensor(port);
09532 }
09533 
09540 inline void SetSensorHTGyro(const byte & port) {
09541   SetSensorType(port, SENSOR_TYPE_LIGHT_INACTIVE);
09542   SetSensorMode(port, SENSOR_MODE_RAW);
09543   ResetSensor(port);
09544 }
09545 
09552 inline void SetSensorHTMagnet(const byte & port) {
09553   SetSensorType(port, SENSOR_TYPE_LIGHT_INACTIVE);
09554   SetSensorMode(port, SENSOR_MODE_RAW);
09555   ResetSensor(port);
09556 }
09557 
09558 #ifdef __DOXYGEN_DOCS
09559 
09569 inline int SensorHTColorNum(const byte & port);
09570 
09580 inline int SensorHTCompass(const byte & port);
09581 
09591 inline int SensorHTIRSeekerDir(const byte & port);
09592 
09603 inline int SensorHTIRSeeker2Addr(const byte & port, const byte reg);
09604 
09614 inline int SensorHTIRSeeker2DCDir(const byte & port);
09615 
09625 inline int SensorHTIRSeeker2ACDir(const byte & port);
09626 
09637 inline char SetHTColor2Mode(const byte & port, byte mode);
09638 
09649 inline char SetHTIRSeeker2Mode(const byte & port, const byte mode);
09650 
09664 inline bool ReadSensorHTAccel(const byte port, int & x, int & y, int & z);
09665 
09680 inline bool ReadSensorHTColor(const byte port, byte & ColorNum, byte & Red, byte & Green, byte & Blue);
09681 
09698 inline bool ReadSensorHTIRSeeker(const byte port, byte & dir, byte & s1, byte & s3, byte & s5, byte & s7, byte & s9);
09699 
09714 inline bool ReadSensorHTNormalizedColor(const byte port, byte & ColorIdx, byte & Red, byte & Green, byte & Blue);
09715 
09729 inline bool ReadSensorHTRawColor(const byte port, unsigned int & Red, unsigned int & Green, unsigned int & Blue);
09730 
09746 inline bool ReadSensorHTColor2Active(byte port, byte & ColorNum, byte & Red, byte & Green, byte & Blue, byte & White);
09747 
09762 inline bool ReadSensorHTNormalizedColor2Active(const byte port, byte & ColorIdx, byte & Red, byte & Green, byte & Blue);
09763 
09778 inline bool ReadSensorHTRawColor2(const byte port, unsigned int & Red, unsigned int & Green, unsigned int & Blue, unsigned int & White);
09779 
09791 inline bool ReadSensorHTIRReceiver(const byte port, char & pfdata[]);
09792 
09805 inline bool ReadSensorHTIRReceiverEx(const byte port, const byte offset, char & pfchar);
09806 
09823 inline bool ReadSensorHTIRSeeker2AC(const byte port, byte & dir, byte & s1, byte & s3, byte & s5, byte & s7, byte & s9);
09824 
09842 inline bool ReadSensorHTIRSeeker2DC(const byte port, byte & dir, byte & s1, byte & s3, byte & s5, byte & s7, byte & s9, byte & avg);
09843 
09854 inline char ResetSensorHTAngle(const byte port, const byte mode);
09855 
09869 inline bool ReadSensorHTAngle(const byte port, int & Angle, long & AccAngle, int & RPM);
09870 
09881 inline void ReadSensorHTTouchMultiplexer(const byte port, byte & t1, byte & t2, byte & t3, byte & t4);
09882 
09897 inline char HTIRTrain(const byte port, const byte channel, const byte func);
09898 
09913 inline char HTPFComboDirect(const byte port, const byte channel, const byte outa, const byte outb);
09914 
09931 inline char HTPFComboPWM(const byte port, const byte channel, const byte outa, const byte outb);
09932 
09946 inline char HTPFRawOutput(const byte port, const byte nibble0, const byte nibble1, const byte nibble2);
09947 
09960 inline char HTPFRepeat(const byte port, const byte count, const unsigned int delay);
09961 
09979 inline char HTPFSingleOutputCST(const byte port, const byte channel, const byte out, const byte func);
09980 
09998 inline char HTPFSingleOutputPWM(const byte port, const byte channel, const byte out, const byte func);
09999 
10019 inline char HTPFSinglePin(const byte port, const byte channel, const byte out, const byte pin, const byte func, bool cont);
10020 
10035 inline char HTPFTrain(const byte port, const byte channel, const byte func);
10036 
10046 inline void HTRCXSetIRLinkPort(const byte port);
10047 
10054 inline int HTRCXBatteryLevel(void);
10055 
10065 inline int HTRCXPoll(const byte src, const byte value);
10066 
10074 inline int HTRCXPollMemory(const unsigned int address);
10075 
10083 inline void HTRCXAddToDatalog(const byte src, const unsigned int value);
10084 
10089 inline void HTRCXClearAllEvents(void);
10090 
10097 inline void HTRCXClearCounter(const byte counter);
10098 
10103 inline void HTRCXClearMsg(void);
10104 
10111 inline void HTRCXClearSensor(const byte port);
10112 
10117 inline void HTRCXClearSound(void);
10118 
10125 inline void HTRCXClearTimer(const byte timer);
10126 
10133 inline void HTRCXCreateDatalog(const unsigned int size);
10134 
10141 inline void HTRCXDecCounter(const byte counter);
10142 
10149 inline void HTRCXDeleteSub(const byte s);
10150 
10155 inline void HTRCXDeleteSubs(void);
10156 
10163 inline void HTRCXDeleteTask(const byte t);
10164 
10169 inline void HTRCXDeleteTasks(void);
10170 
10177 inline void HTRCXDisableOutput(const byte outputs);
10178 
10185 inline void HTRCXEnableOutput(const byte outputs);
10186 
10194 inline void HTRCXEvent(const byte src, const unsigned int value);
10195 
10202 inline void HTRCXFloat(const byte outputs);
10203 
10210 inline void HTRCXFwd(const byte outputs);
10211 
10218 inline void HTRCXIncCounter(const byte counter);
10219 
10226 inline void HTRCXInvertOutput(const byte outputs);
10227 
10232 inline void HTRCXMuteSound(void);
10233 
10240 inline void HTRCXObvertOutput(const byte outputs);
10241 
10248 inline void HTRCXOff(const byte outputs);
10249 
10256 inline void HTRCXOn(const byte outputs);
10257 
10266 inline void HTRCXOnFor(const byte outputs, const unsigned int ms);
10267 
10275 inline void HTRCXOnFwd(const byte outputs);
10276 
10283 inline void HTRCXOnRev(const byte outputs);
10284 
10289 inline void HTRCXPBTurnOff(void);
10290 
10295 inline void HTRCXPing(void);
10296 
10303 inline void HTRCXPlaySound(const byte snd);
10304 
10312 inline void HTRCXPlayTone(const unsigned int freq, const byte duration);
10313 
10321 inline void HTRCXPlayToneVar(const byte varnum, const byte duration);
10322 
10329 inline void HTRCXRemote(unsigned int cmd);
10330 
10337 inline void HTRCXRev(const byte outputs);
10338 
10346 inline void HTRCXSelectDisplay(const byte src, const unsigned int value);
10347 
10354 inline void HTRCXSelectProgram(const byte prog);
10355 
10363 inline void HTRCXSendSerial(const byte first, const byte count);
10364 
10372 inline void HTRCXSetDirection(const byte outputs, const byte dir);
10373 
10382 inline void HTRCXSetEvent(const byte evt, const byte src, const byte type);
10383 
10391 inline void HTRCXSetGlobalDirection(const byte outputs, const byte dir);
10392 
10400 inline void HTRCXSetGlobalOutput(const byte outputs, const byte mode);
10401 
10410 inline void HTRCXSetMaxPower(const byte outputs, const byte pwrsrc, const byte pwrval);
10411 
10418 inline void HTRCXSetMessage(const byte msg);
10419 
10427 inline void HTRCXSetOutput(const byte outputs, const byte mode);
10428 
10437 inline void HTRCXSetPower(const byte outputs, const byte pwrsrc, const byte pwrval);
10438 
10445 inline void HTRCXSetPriority(const byte p);
10446 
10454 inline void HTRCXSetSensorMode(const byte port, const byte mode);
10455 
10463 inline void HTRCXSetSensorType(const byte port, const byte type);
10464 
10471 inline void HTRCXSetSleepTime(const byte t);
10472 
10479 inline void HTRCXSetTxPower(const byte pwr);
10480 
10488 inline void HTRCXSetWatch(const byte hours, const byte minutes);
10489 
10496 inline void HTRCXStartTask(const byte t);
10497 
10502 inline void HTRCXStopAllTasks(void);
10503 
10510 inline void HTRCXStopTask(const byte t);
10511 
10518 inline void HTRCXToggle(const byte outputs);
10519 
10524 inline void HTRCXUnmuteSound(void);
10525 
10530 inline void HTScoutCalibrateSensor(void);
10531 
10536 inline void HTScoutMuteSound(void);
10537 
10544 inline void HTScoutSelectSounds(const byte grp);
10545 
10553 inline void HTScoutSendVLL(const byte src, const unsigned int value);
10554 
10562 inline void HTScoutSetEventFeedback(const byte src, const unsigned int value);
10563 
10570 inline void HTScoutSetLight(const byte x);
10571 
10578 inline void HTScoutSetScoutMode(const byte mode);
10579 
10587 inline void HTScoutSetSensorClickTime(const byte src, const unsigned int value);
10588 
10596 inline void HTScoutSetSensorHysteresis(const byte src, const unsigned int value);
10597 
10605 inline void HTScoutSetSensorLowerLimit(const byte src, const unsigned int value);
10606 
10614 inline void HTScoutSetSensorUpperLimit(const byte src, const unsigned int value);
10615 
10620 inline void HTScoutUnmuteSound(void);
10621 
10622 #else
10623 
10624 #define SensorHTCompass(_port) asm { ReadSensorHTCompass(_port, __RETVAL__) }
10625 #define ReadSensorHTAccel(_port, _x, _y, _z) asm { __ReadSensorHTAccel(_port, _x, _y, _z, __RETVAL__) }
10626 #define ReadSensorHTColor(_port, _ColorNum, _Red, _Green, _Blue) asm { __ReadSensorHTColor(_port, _ColorNum, _Red, _Green, _Blue, __RETVAL__) }
10627 #define ReadSensorHTRawColor(_port, _Red, _Green, _Blue) asm { __ReadSensorHTRawColor(_port, _Red, _Green, _Blue, __RETVAL__) }
10628 #define ReadSensorHTNormalizedColor(_port, _ColorIdx, _Red, _Green, _Blue) asm { __ReadSensorHTNormalizedColor(_port, _ColorIdx, _Red, _Green, _Blue, __RETVAL__) }
10629 #define ReadSensorHTIRSeeker(_port, _dir, _s1, _s3, _s5, _s7, _s9) asm { __ReadSensorHTIRSeeker(_port, _dir, _s1, _s3, _s5, _s7, _s9, __RETVAL__) }
10630 #define SensorHTIRSeekerDir(_port) asm { ReadSensorHTIRSeekerDir(_port, __RETVAL__) }
10631 #define SensorHTColorNum(_port) asm { ReadSensorHTColorNum(_port, __RETVAL__) }
10632 #define ReadSensorHTTouchMultiplexer(_p, _t1, _t2, _t3, _t4) asm { __ReadSensorHTTouchMultiplexer(_p, _t1, _t2, _t3, _t4) }
10633 #define SensorHTIRSeeker2Addr(_port, _reg) asm { ReadSensorHTIRSeeker2Addr(_port, _reg, __RETVAL__) }
10634 #define SensorHTIRSeeker2DCDir(_port) asm { ReadSensorHTIRSeeker2Addr(_port, HTIR2_REG_DCDIR, __RETVAL__) }
10635 #define SensorHTIRSeeker2ACDir(_port) asm { ReadSensorHTIRSeeker2Addr(_port, HTIR2_REG_ACDIR, __RETVAL__) }
10636 #define ReadSensorHTIRSeeker2DC(_port, _dir, _s1, _s3, _s5, _s7, _s9, _avg) asm { __ReadSensorHTIRSeeker2DC(_port, _dir, _s1, _s3, _s5, _s7, _s9, _avg, __RETVAL__) }
10637 #define ReadSensorHTIRSeeker2AC(_port, _dir, _s1, _s3, _s5, _s7, _s9) asm { __ReadSensorHTIRSeeker2AC(_port, _dir, _s1, _s3, _s5, _s7, _s9, __RETVAL__) }
10638 #define SetHTIRSeeker2Mode(_port, _mode) asm { __SetHTIRSeeker2Mode(_port, _mode, __RETVAL__) }
10639 
10640 #define SetHTColor2Mode(_port, _mode) asm { __SetHTColor2Mode(_port, _mode, __RETVAL__) }
10641 #define ReadSensorHTColor2Active(_port, _ColorNum, _Red, _Green, _Blue, _White) asm { __ReadSensorHTColor2Active(_port, _ColorNum, _Red, _Green, _Blue, _White, __RETVAL__) }
10642 #define ReadSensorHTNormalizedColor2Active(_port, _ColorIdx, _Red, _Green, _Blue) asm { __ReadSensorHTNormalizedColor2Active(_port, _ColorIdx, _Red, _Green, _Blue, __RETVAL__) }
10643 #define ReadSensorHTRawColor2(_port, _Red, _Green, _Blue, _White) asm { __ReadSensorHTRawColor2(_port, _Red, _Green, _Blue, _White, __RETVAL__) }
10644 #define ReadSensorHTIRReceiver(_port, _pfdata) asm { __ReadSensorHTIRReceiver(_port, _pfdata, __RETVAL__) }
10645 #define ReadSensorHTIRReceiverEx(_port, _reg, _pfchar) asm { __ReadSensorHTIRReceiverEx(_port, _reg, _pfchar, __RETVAL__) }
10646 #define ResetSensorHTAngle(_port, _mode) asm { __ResetSensorHTAngle(_port, _mode, __RETVAL__) }
10647 #define ReadSensorHTAngle(_port, _Angle, _AccAngle, _RPM) asm { __ReadSensorHTAngle(_port, _Angle, _AccAngle, _RPM, __RETVAL__) }
10648 
10649 
10650 #define HTPowerFunctionCommand(_port, _channel, _outa, _outb) asm { __HTPFComboDirect(_port, _channel, _outa, _outb, __RETVAL__) }
10651 #define HTPFComboDirect(_port, _channel, _outa, _outb) asm { __HTPFComboDirect(_port, _channel, _outa, _outb, __RETVAL__) }
10652 #define HTPFSinglePin(_port, _channel, _out, _pin, _func, _cont) asm { __HTPFSinglePin(_port, _channel, _out, _pin, _func, _cont, __RETVAL__) }
10653 #define HTPFSingleOutputCST(_port, _channel, _out, _func) asm { __HTPFSingleOutput(_port, _channel, _out, _func, TRUE, __RETVAL__) }
10654 #define HTPFSingleOutputPWM(_port, _channel, _out, _func) asm { __HTPFSingleOutput(_port, _channel, _out, _func, FALSE, __RETVAL__) }
10655 #define HTPFComboPWM(_port, _channel, _outa, _outb) asm { __HTPFComboPWM(_port, _channel, _outa, _outb, __RETVAL__) }
10656 #define HTPFTrain(_port, _channel, _func) asm { __HTIRTrain(_port, _channel, _func, TRUE, __RETVAL__) }
10657 #define HTIRTrain(_port, _channel, _func) asm { __HTIRTrain(_port, _channel, _func, FALSE, __RETVAL__) }
10658 #define HTPFRawOutput(_port, _nibble0, _nibble1, _nibble2) asm { __HTPFRawOutput(_port, _nibble0, _nibble1, _nibble2, __RETVAL__) }
10659 #define HTPFRepeat(_port, _count, _delay) asm { __HTPFRepeatLastCommand(_port, _count, _delay, __RETVAL__) }
10660 
10661 #define HTRCXSetIRLinkPort(_port) asm { __HTRCXSetIRLinkPort(_port) }
10662 #define HTRCXPoll(_src, _value) asm { __HTRCXPoll(_src, _value, __RETVAL__) }
10663 #define HTRCXBatteryLevel() asm { __HTRCXBatteryLevel(__RETVAL__) }
10664 #define HTRCXPing() asm { __HTRCXOpNoArgs(RCX_PingOp) }
10665 #define HTRCXDeleteTasks() asm { __HTRCXOpNoArgs(RCX_DeleteTasksOp) }
10666 #define HTRCXStopAllTasks() asm { __HTRCXOpNoArgs(RCX_StopAllTasksOp) }
10667 #define HTRCXPBTurnOff() asm { __HTRCXOpNoArgs(RCX_PBTurnOffOp) }
10668 #define HTRCXDeleteSubs() asm { __HTRCXOpNoArgs(RCX_DeleteSubsOp) }
10669 #define HTRCXClearSound() asm { __HTRCXOpNoArgs(RCX_ClearSoundOp) }
10670 #define HTRCXClearMsg() asm { __HTRCXOpNoArgs(RCX_ClearMsgOp) }
10671 #define HTRCXMuteSound() asm { __HTRCXOpNoArgs(RCX_MuteSoundOp) }
10672 #define HTRCXUnmuteSound() asm { __HTRCXOpNoArgs(RCX_UnmuteSoundOp) }
10673 #define HTRCXClearAllEvents() asm { __HTRCXOpNoArgs(RCX_ClearAllEventsOp) }
10674 #define HTRCXSetOutput(_outputs, _mode) asm { __HTRCXSetOutput(_outputs, _mode) }
10675 #define HTRCXSetDirection(_outputs, _dir) asm { __HTRCXSetDirection(_outputs, _dir) }
10676 #define HTRCXSetPower(_outputs, _pwrsrc, _pwrval) asm { __HTRCXSetPower(_outputs, _pwrsrc, _pwrval) }
10677 #define HTRCXOn(_outputs) asm { __HTRCXSetOutput(_outputs, RCX_OUT_ON) }
10678 #define HTRCXOff(_outputs) asm { __HTRCXSetOutput(_outputs, RCX_OUT_OFF) }
10679 #define HTRCXFloat(_outputs) asm { __HTRCXSetOutput(_outputs, RCX_OUT_FLOAT) }
10680 #define HTRCXToggle(_outputs) asm { __HTRCXSetDirection(_outputs, RCX_OUT_TOGGLE) }
10681 #define HTRCXFwd(_outputs) asm { __HTRCXSetDirection(_outputs, RCX_OUT_FWD) }
10682 #define HTRCXRev(_outputs) asm { __HTRCXSetDirection(_outputs, RCX_OUT_REV) }
10683 #define HTRCXOnFwd(_outputs) asm { __HTRCXOnFwd(_outputs) }
10684 #define HTRCXOnRev(_outputs) asm { __HTRCXOnRev(_outputs) }
10685 #define HTRCXOnFor(_outputs, _ms) asm { __HTRCXOnFor(_outputs, _ms) }
10686 #define HTRCXSetTxPower(_pwr) asm { __HTRCXSetTxPower(_pwr) }
10687 #define HTRCXPlaySound(_snd) asm { __HTRCXPlaySound(_snd) }
10688 #define HTRCXDeleteTask(_t) asm { __HTRCXDeleteTask(_t) }
10689 #define HTRCXStartTask(_t) asm { __HTRCXStartTask(_t) }
10690 #define HTRCXStopTask(_t) asm { __HTRCXStopTask(_t) }
10691 #define HTRCXSelectProgram(_prog) asm { __HTRCXSelectProgram(_prog) }
10692 #define HTRCXClearTimer(_timer) asm { __HTRCXClearTimer(_timer) }
10693 #define HTRCXSetSleepTime(_t) asm { __HTRCXSetSleepTime(_t) }
10694 #define HTRCXDeleteSub(_s) asm { __HTRCXDeleteSub(_s) }
10695 #define HTRCXClearSensor(_port) asm { __HTRCXClearSensor(_port) }
10696 #define HTRCXPlayToneVar(_varnum, _duration) asm { __HTRCXPlayToneVar(_varnum, _duration) }
10697 #define HTRCXSetWatch(_hours, _minutes) asm { __HTRCXSetWatch(_hours, _minutes) }
10698 #define HTRCXSetSensorType(_port, _type) asm { __HTRCXSetSensorType(_port, _type) }
10699 #define HTRCXSetSensorMode(_port, _mode) asm { __HTRCXSetSensorMode(_port, _mode) }
10700 #define HTRCXCreateDatalog(_size) asm { __HTRCXCreateDatalog(_size) }
10701 #define HTRCXAddToDatalog(_src, _value) asm { __HTRCXAddToDatalog(_src, _value) }
10702 #define HTRCXSendSerial(_first, _count) asm { __HTRCXSendSerial(_first, _count) }
10703 #define HTRCXRemote(_cmd) asm { __HTRCXRemote(_cmd) }
10704 #define HTRCXEvent(_src, _value) asm { __HTRCXEvent(_src, _value) }
10705 #define HTRCXPlayTone(_freq, _duration) asm { __HTRCXPlayTone(_freq, _duration) }
10706 #define HTRCXSelectDisplay(_src, _value) asm { __HTRCXSelectDisplay(_src, _value) }
10707 #define HTRCXPollMemory(_memaddress) asm { __HTRCXPollMemory(_memaddress, __RETVAL__) }
10708 #define HTRCXSetEvent(_evt, _src, _type) asm { __HTRCXSetEvent(_evt, _src, _type) }
10709 #define HTRCXSetGlobalOutput(_outputs, _mode) asm { __HTRCXSetGlobalOutput(_outputs, _mode) }
10710 #define HTRCXSetGlobalDirection(_outputs, _dir) asm { __HTRCXSetGlobalDirection(_outputs, _dir) }
10711 #define HTRCXSetMaxPower(_outputs, _pwrsrc, _pwrval) asm { __HTRCXSetMaxPower(_outputs, _pwrsrc, _pwrval) }
10712 #define HTRCXEnableOutput(_outputs) asm { __HTRCXSetGlobalOutput(_outputs, RCX_OUT_ON) }
10713 #define HTRCXDisableOutput(_outputs) asm { __HTRCXSetGlobalOutput(_outputs, RCX_OUT_OFF) }
10714 #define HTRCXInvertOutput(_outputs) asm { __HTRCXSetGlobalDirection(_outputs, RCX_OUT_REV) }
10715 #define HTRCXObvertOutput(_outputs) asm { __HTRCXSetGlobalDirection(_outputs, RCX_OUT_FWD) }
10716 #define HTRCXIncCounter(_counter) asm { __HTRCXIncCounter(_counter) }
10717 #define HTRCXDecCounter(_counter) asm { __HTRCXDecCounter(_counter) }
10718 #define HTRCXClearCounter(_counter) asm { __HTRCXClearCounter(_counter) }
10719 #define HTRCXSetPriority(_p) asm { __HTRCXSetPriority(_p) }
10720 #define HTRCXSetMessage(_msg) asm { __HTRCXSetMessage(_msg) }
10721 
10722 #define HTScoutCalibrateSensor() asm { __HTRCXOpNoArgs(RCX_LSCalibrateOp) }
10723 #define HTScoutMuteSound() asm { __HTScoutMuteSound() }
10724 #define HTScoutUnmuteSound() asm { __HTScoutUnmuteSound() }
10725 #define HTScoutSelectSounds(_grp) asm { __HTScoutSelectSounds(_grp) }
10726 #define HTScoutSetLight(_x) asm { __HTScoutSetLight(_x) }
10727 #define HTScoutSetSensorClickTime(_src, _value) asm { __HTScoutSetSensorClickTime(_src, _value) }
10728 #define HTScoutSetSensorHysteresis(_src, _value) asm { __HTScoutSetSensorHysteresis(_src, _value) }
10729 #define HTScoutSetSensorLowerLimit(_src, _value) asm { __HTScoutSetSensorLowerLimit(_src, _value) }
10730 #define HTScoutSetSensorUpperLimit(_src, _value) asm { __HTScoutSetSensorUpperLimit(_src, _value) }
10731 #define HTScoutSetEventFeedback(_src, _value) asm { __HTScoutSetEventFeedback(_src, _value) }
10732 #define HTScoutSendVLL(_src, _value) asm { __HTScoutSendVLL(_src, _value) }
10733 #define HTScoutSetScoutMode(_mode) asm { __HTScoutSetScoutMode(_mode) }
10734 
10735 #endif
10736  // end of HiTechnicAPI group
10737 
10738 
10742 
10743 
10754 inline void SetSensorMSPressure(const byte & port ) {
10755   SetSensorType(port, SENSOR_TYPE_LIGHT);
10756   SetSensorMode(port, SENSOR_MODE_RAW);
10757   ResetSensor(port);
10758 }
10759 
10768 inline void SetSensorMSDROD(const byte & port, bool bActive) {
10769   if (bActive)
10770     SetSensorType(port, SENSOR_TYPE_LIGHT_ACTIVE);
10771   else
10772     SetSensorType(port, SENSOR_TYPE_LIGHT_INACTIVE);
10773   SetSensorMode(port, SENSOR_MODE_PERCENT);
10774   ResetSensor(port);
10775 }
10776 
10777 
10786 inline void SetSensorNXTSumoEyes(const byte & port, bool bLong) {
10787   if (bLong)
10788     SetSensorType(port, SENSOR_TYPE_LIGHT_INACTIVE);
10789   else
10790     SetSensorType(port, SENSOR_TYPE_LIGHT_ACTIVE);
10791   SetSensorMode(port, SENSOR_MODE_PERCENT);
10792   ResetSensor(port);
10793   Wait(275);
10794 }
10795 
10804 inline int SensorMSPressure(const byte & port) {
10805   asm {
10806     getin __RETVAL__, port, RawValueField
10807     sub __RETVAL__, 1024, __RETVAL__
10808     div __RETVAL__, __RETVAL__, 25
10809   }
10810 }
10811 
10821 char SensorNXTSumoEyes(const byte & port) {
10822   int value;
10823   asm {
10824     getin value, port, NormalizedValueField
10825     mul value, value, 100
10826     div value, value, 1023
10827   }
10828   if (value > 30 && value < 36)
10829     return NXTSE_ZONE_LEFT;
10830   if (value > 63 && value < 69)
10831     return NXTSE_ZONE_RIGHT;
10832   if (value > 74 && value <= 80)
10833     return NXTSE_ZONE_FRONT;
10834   return NXTSE_ZONE_NONE;
10835 }
10836 
10837 #ifdef __DOXYGEN_DOCS
10838 
10847 inline int SensorMSCompass(const byte & port, const byte i2caddr);
10848 
10856 inline int SensorMSDROD(const byte & port);
10857 
10867 inline int SensorNXTSumoEyesRaw(const byte & port);
10868 
10876 inline int SensorMSPressureRaw(const byte & port);
10877 
10892 inline bool ReadSensorMSAccel(const byte port, const byte i2caddr, int & x, int & y, int & z);
10893 
10911 inline bool ReadSensorMSPlayStation(const byte port, const byte i2caddr, byte & btnset1, byte & btnset2, byte & xleft, byte & yleft, byte & xright, byte & yright);
10912 
10930 inline bool ReadSensorMSRTClock(const byte port, byte & sec, byte & min, byte & hrs, byte & dow, byte & date, byte & month, byte & year);
10931 
10946 inline bool ReadSensorMSTilt(const byte & port, const byte & i2caddr, byte & x, byte & y, byte & z);
10947 
10965 inline bool PFMateSend(const byte & port, const byte & i2caddr, const byte & channel, const byte & motors, const byte & cmdA, const byte & spdA, const byte & cmdB, const byte & spdB);
10966 
10981 inline bool PFMateSendRaw(const byte & port, const byte & i2caddr, const byte & channel, const byte & b1, const byte & b2);
10982 
10996 inline int MSReadValue(const byte port, const byte i2caddr, const byte reg, const byte numbytes);
10997 
11007 inline char MSEnergize(const byte port, const byte i2caddr);
11008 
11018 inline char MSDeenergize(const byte port, const byte i2caddr);
11019 
11029 inline char MSADPAOn(const byte port, const byte i2caddr);
11030 
11040 inline char MSADPAOff(const byte port, const byte i2caddr);
11041 
11051 inline char DISTNxGP2D12(const byte port, const byte i2caddr);
11052 
11062 inline char DISTNxGP2D120(const byte port, const byte i2caddr);
11063 
11073 inline char DISTNxGP2YA02(const byte port, const byte i2caddr);
11074 
11084 inline char DISTNxGP2YA21(const byte port, const byte i2caddr);
11085 
11095 inline int DISTNxDistance(const byte port, const byte i2caddr);
11096 
11106 inline int DISTNxMaxDistance(const byte port, const byte i2caddr);
11107 
11117 inline int DISTNxMinDistance(const byte port, const byte i2caddr);
11118 
11128 inline byte DISTNxModuleType(const byte port, const byte i2caddr);
11129 
11139 inline byte DISTNxNumPoints(const byte port, const byte i2caddr);
11140 
11150 inline int DISTNxVoltage(const byte port, const byte i2caddr);
11151 
11161 inline char ACCLNxCalibrateX(const byte port, const byte i2caddr);
11162 
11172 inline char ACCLNxCalibrateXEnd(const byte port, const byte i2caddr);
11173 
11183 inline char ACCLNxCalibrateY(const byte port, const byte i2caddr);
11184 
11194 inline char ACCLNxCalibrateYEnd(const byte port, const byte i2caddr);
11195 
11205 inline char ACCLNxCalibrateZ(const byte port, const byte i2caddr);
11206 
11216 inline char ACCLNxCalibrateZEnd(const byte port, const byte i2caddr);
11217 
11227 inline char ACCLNxResetCalibration(const byte port, const byte i2caddr);
11228 
11239 inline char SetACCLNxSensitivity(const byte port, const byte i2caddr, byte slevel);
11240 
11250 inline byte ACCLNxSensitivity(const byte port, const byte i2caddr);
11251 
11261 inline int ACCLNxXOffset(const byte port, const byte i2caddr);
11262 
11272 inline int ACCLNxXRange(const byte port, const byte i2caddr);
11273 
11283 inline int ACCLNxYOffset(const byte port, const byte i2caddr);
11284 
11294 inline int ACCLNxYRange(const byte port, const byte i2caddr);
11295 
11305 inline int ACCLNxZOffset(const byte port, const byte i2caddr);
11306 
11316 inline int ACCLNxZRange(const byte port, const byte i2caddr);
11317 
11327 inline char PSPNxDigital(const byte & port, const byte & i2caddr);
11328 
11338 inline char PSPNxAnalog(const byte & port, const byte & i2caddr);
11339 
11350 inline unsigned int NXTServoPosition(const byte & port, const byte & i2caddr, const byte servo);
11351 
11362 inline byte NXTServoSpeed(const byte & port, const byte & i2caddr, const byte servo);
11363 
11373 inline byte NXTServoBatteryVoltage(const byte & port, const byte & i2caddr);
11374 
11387 inline char SetNXTServoSpeed(const byte & port, const byte & i2caddr, const byte servo, const byte & speed);
11388 
11401 inline char SetNXTServoQuickPosition(const byte & port, const byte & i2caddr, const byte servo, const byte & qpos);
11402 
11415 inline char SetNXTServoPosition(const byte & port, const byte & i2caddr, const byte servo, const byte & pos);
11416 
11428 inline char NXTServoReset(const byte & port, const byte & i2caddr);
11429 
11441 inline char NXTServoHaltMacro(const byte & port, const byte & i2caddr);
11442 
11454 inline char NXTServoResumeMacro(const byte & port, const byte & i2caddr);
11455 
11467 inline char NXTServoPauseMacro(const byte & port, const byte & i2caddr);
11468 
11483 inline char NXTServoInit(const byte & port, const byte & i2caddr, const byte servo);
11484 
11497 inline char NXTServoGotoMacroAddress(const byte & port, const byte & i2caddr, const byte & macro);
11498 
11512 inline char NXTServoEditMacro(const byte & port, const byte & i2caddr);
11513 
11524 inline char NXTServoQuitEdit(const byte & port);
11525 
11537 inline char NXTHIDAsciiMode(const byte & port, const byte & i2caddr);
11538 
11550 inline char NXTHIDDirectMode(const byte & port, const byte & i2caddr);
11551 
11562 inline char NXTHIDTransmit(const byte & port, const byte & i2caddr);
11563 
11576 inline char NXTHIDLoadCharacter(const byte & port, const byte & i2caddr, const byte & modifier, const byte & character);
11577 
11588 inline char NXTPowerMeterResetCounters(const byte & port, const byte & i2caddr);
11589 
11599 inline int NXTPowerMeterPresentCurrent(const byte & port, const byte & i2caddr);
11600 
11610 inline int NXTPowerMeterPresentVoltage(const byte & port, const byte & i2caddr);
11611 
11621 inline int NXTPowerMeterCapacityUsed(const byte & port, const byte & i2caddr);
11622 
11632 inline int NXTPowerMeterPresentPower(const byte & port, const byte & i2caddr);
11633 
11643 inline long NXTPowerMeterTotalPowerConsumed(const byte & port, const byte & i2caddr);
11644 
11654 inline int NXTPowerMeterMaxCurrent(const byte & port, const byte & i2caddr);
11655 
11665 inline int NXTPowerMeterMinCurrent(const byte & port, const byte & i2caddr);
11666 
11676 inline int NXTPowerMeterMaxVoltage(const byte & port, const byte & i2caddr);
11677 
11687 inline int NXTPowerMeterMinVoltage(const byte & port, const byte & i2caddr);
11688 
11698 inline long NXTPowerMeterElapsedTime(const byte & port, const byte & i2caddr);
11699 
11709 inline int NXTPowerMeterErrorCount(const byte & port, const byte & i2caddr);
11710 
11724 inline char NXTLineLeaderPowerDown(const byte & port, const byte & i2caddr);
11725 
11737 inline char NXTLineLeaderPowerUp(const byte & port, const byte & i2caddr);
11738 
11750 inline char NXTLineLeaderInvert(const byte & port, const byte & i2caddr);
11751 
11763 inline char NXTLineLeaderReset(const byte & port, const byte & i2caddr);
11764 
11777 inline char NXTLineLeaderSnapshot(const byte & port, const byte & i2caddr);
11778 
11789 inline char NXTLineLeaderCalibrateWhite(const byte & port, const byte & i2caddr);
11790 
11801 inline char NXTLineLeaderCalibrateBlack(const byte & port, const byte & i2caddr);
11802 
11814 inline char NXTLineLeaderSteering(const byte & port, const byte & i2caddr);
11815 
11828 inline char NXTLineLeaderAverage(const byte & port, const byte & i2caddr);
11829 
11842 inline byte NXTLineLeaderResult(const byte & port, const byte & i2caddr);
11843 
11861 inline char SetNXTLineLeaderSetpoint(const byte & port, const byte & i2caddr, const byte & value);
11862 
11878 inline char SetNXTLineLeaderKpValue(const byte & port, const byte & i2caddr, const byte & value);
11879 
11895 inline char SetNXTLineLeaderKiValue(const byte & port, const byte & i2caddr, const byte & value);
11896 
11912 inline char SetNXTLineLeaderKdValue(const byte & port, const byte & i2caddr, const byte & value);
11913 
11926 inline char SetNXTLineLeaderKpFactor(const byte & port, const byte & i2caddr, const byte & value);
11927 
11940 inline char SetNXTLineLeaderKiFactor(const byte & port, const byte & i2caddr, const byte & value);
11941 
11954 inline char SetNXTLineLeaderKdFactor(const byte & port, const byte & i2caddr, const byte & value);
11955 
11965 inline char NRLink2400(const byte port, const byte i2caddr);
11966 
11976 inline char NRLink4800(const byte port, const byte i2caddr);
11977 
11987 inline char NRLinkFlush(const byte port, const byte i2caddr);
11988 
11998 inline char NRLinkIRLong(const byte port, const byte i2caddr);
11999 
12009 inline char NRLinkIRShort(const byte port, const byte i2caddr);
12010 
12020 inline char NRLinkSetPF(const byte port, const byte i2caddr);
12021 
12031 inline char NRLinkSetRCX(const byte port, const byte i2caddr);
12032 
12042 inline char NRLinkSetTrain(const byte port, const byte i2caddr);
12043 
12053 inline char NRLinkTxRaw(const byte port, const byte i2caddr);
12054 
12064 inline byte NRLinkStatus(const byte port, const byte i2caddr);
12065 
12076 inline char RunNRLinkMacro(const byte port, const byte i2caddr, const byte macro);
12077 
12088 inline char WriteNRLinkBytes(const byte port, const byte i2caddr, const byte data[]);
12089 
12100 inline bool ReadNRLinkBytes(const byte port, const byte i2caddr, byte & data[]);
12101 
12117 inline char MSIRTrain(const byte port, const byte i2caddr, const byte channel, const byte func);
12118 
12134 inline char MSPFComboDirect(const byte port, const byte i2caddr, const byte channel, const byte outa, const byte outb);
12135 
12153 inline char MSPFComboPWM(const byte port, const byte i2caddr, const byte channel, const byte outa, const byte outb);
12154 
12169 inline char MSPFRawOutput(const byte port, const byte i2caddr, const byte nibble0, const byte nibble1, const byte nibble2);
12170 
12184 inline char MSPFRepeat(const byte port, const byte i2caddr, const byte count, const unsigned int delay);
12185 
12204 inline char MSPFSingleOutputCST(const byte port, const byte i2caddr, const byte channel, const byte out, const byte func);
12205 
12224 inline char MSPFSingleOutputPWM(const byte port, const byte i2caddr, const byte channel, const byte out, const byte func);
12225 
12246 inline char MSPFSinglePin(const byte port, const byte i2caddr, const byte channel, const byte out, const byte pin, const byte func, bool cont);
12247 
12263 inline char MSPFTrain(const byte port, const byte i2caddr, const byte channel, const byte func);
12264 
12275 inline void MSRCXSetNRLinkPort(const byte port, const byte i2caddr);
12276 
12283 inline int MSRCXBatteryLevel(void);
12284 
12294 inline int MSRCXPoll(const byte src, const byte value);
12295 
12303 inline int MSRCXPollMemory(const unsigned int address);
12304 
12313 inline void MSRCXAbsVar(const byte varnum, const byte byte src, const unsigned int value);
12314 
12322 inline void MSRCXAddToDatalog(const byte src, const unsigned int value);
12323 
12332 inline void MSRCXAndVar(const byte varnum, const byte src, const unsigned int value);
12333 
12338 inline void MSRCXBoot(void);
12339 
12349 inline void MSRCXCalibrateEvent(const byte evt, const byte low, const byte hi, const byte hyst);
12350 
12355 inline void MSRCXClearAllEvents(void);
12356 
12363 inline void MSRCXClearCounter(const byte counter);
12364 
12369 inline void MSRCXClearMsg(void);
12370 
12377 inline void MSRCXClearSensor(const byte port);
12378 
12383 inline void MSRCXClearSound(void);
12384 
12391 inline void MSRCXClearTimer(const byte timer);
12392 
12399 inline void MSRCXCreateDatalog(const unsigned int size);
12400 
12407 inline void MSRCXDecCounter(const byte counter);
12408 
12415 inline void MSRCXDeleteSub(const byte s);
12416 
12421 inline void MSRCXDeleteSubs(void);
12422 
12429 inline void MSRCXDeleteTask(const byte t);
12430 
12435 inline void MSRCXDeleteTasks(void);
12436 
12443 inline void MSRCXDisableOutput(const byte outputs);
12444 
12453 inline void MSRCXDivVar(const byte varnum, const byte src, const unsigned int value);
12454 
12461 inline void MSRCXEnableOutput(const byte outputs);
12462 
12470 inline void MSRCXEvent(const byte src, const unsigned int value);
12471 
12478 inline void MSRCXFloat(const byte outputs);
12479 
12486 inline void MSRCXFwd(const byte outputs);
12487 
12494 inline void MSRCXIncCounter(const byte counter);
12495 
12502 inline void MSRCXInvertOutput(const byte outputs);
12503 
12512 inline void MSRCXMulVar(const byte varnum, const byte src, unsigned int value);
12513 
12518 inline void MSRCXMuteSound(void);
12519 
12526 inline void MSRCXObvertOutput(const byte outputs);
12527 
12534 inline void MSRCXOff(const byte outputs);
12535 
12542 inline void MSRCXOn(const byte outputs);
12543 
12552 inline void MSRCXOnFor(const byte outputs, const unsigned int ms);
12553 
12561 inline void MSRCXOnFwd(const byte outputs);
12562 
12569 inline void MSRCXOnRev(const byte outputs);
12570 
12579 inline void MSRCXOrVar(const byte varnum, const byte src, const unsigned int value);
12580 
12585 inline void MSRCXPBTurnOff(void);
12586 
12591 inline void MSRCXPing(void);
12592 
12599 inline void MSRCXPlaySound(const byte snd);
12600 
12608 inline void MSRCXPlayTone(const unsigned int freq, const byte duration);
12609 
12617 inline void MSRCXPlayToneVar(const byte varnum, const byte duration);
12618 
12625 inline void MSRCXRemote(unsigned int cmd);
12626 
12631 inline void MSRCXReset(void);
12632 
12639 inline void MSRCXRev(const byte outputs);
12640 
12648 inline void MSRCXSelectDisplay(const byte src, const unsigned int value);
12649 
12656 inline void MSRCXSelectProgram(const byte prog);
12657 
12665 inline void MSRCXSendSerial(const byte first, const byte count);
12666 
12676 inline void MSRCXSet(const byte dstsrc, const byte dstval, const byte src, unsigned int value);
12677 
12685 inline void MSRCXSetDirection(const byte outputs, const byte dir);
12686 
12695 inline void MSRCXSetEvent(const byte evt, const byte src, const byte type);
12696 
12704 inline void MSRCXSetGlobalDirection(const byte outputs, const byte dir);
12705 
12713 inline void MSRCXSetGlobalOutput(const byte outputs, const byte mode);
12714 
12723 inline void MSRCXSetMaxPower(const byte outputs, const byte pwrsrc, const byte pwrval);
12724 
12731 inline void MSRCXSetMessage(const byte msg);
12732 
12740 inline void MSRCXSetOutput(const byte outputs, const byte mode);
12741 
12750 inline void MSRCXSetPower(const byte outputs, const byte pwrsrc, const byte pwrval);
12751 
12758 inline void MSRCXSetPriority(const byte p);
12759 
12767 inline void MSRCXSetSensorMode(const byte port, const byte mode);
12768 
12776 inline void MSRCXSetSensorType(const byte port, const byte type);
12777 
12784 inline void MSRCXSetSleepTime(const byte t);
12785 
12792 inline void MSRCXSetTxPower(const byte pwr);
12793 
12802 inline void MSRCXSetUserDisplay(const byte src, const unsigned int value, const byte precision);
12803 
12812 inline void MSRCXSetVar(const byte varnum, const byte src, const unsigned int value);
12813 
12821 inline void MSRCXSetWatch(const byte hours, const byte minutes);
12822 
12831 inline void MSRCXSgnVar(const byte varnum, const byte src, const unsigned int value);
12832 
12839 inline void MSRCXStartTask(const byte t);
12840 
12845 inline void MSRCXStopAllTasks(void);
12846 
12853 inline void MSRCXStopTask(const byte t);
12854 
12863 inline void MSRCXSubVar(const byte varnum, const byte src, const unsigned int value);
12864 
12873 inline void MSRCXSumVar(const byte varnum, const byte src, const unsigned int value);
12874 
12881 inline void MSRCXToggle(const byte outputs);
12882 
12887 inline void MSRCXUnlock(void);
12888 
12893 inline void MSRCXUnmuteSound(void);
12894 
12899 inline void MSScoutCalibrateSensor(void);
12900 
12905 inline void MSScoutMuteSound(void);
12906 
12913 inline void MSScoutSelectSounds(const byte grp);
12914 
12922 inline void MSScoutSendVLL(const byte src, const unsigned int value);
12923 
12932 inline void MSScoutSetCounterLimit(const byte ctr, const byte src, const unsigned int value);
12933 
12941 inline void MSScoutSetEventFeedback(const byte src, const unsigned int value);
12942 
12949 inline void MSScoutSetLight(const byte x);
12950 
12957 inline void MSScoutSetScoutMode(const byte mode);
12958 
12969 inline void MSScoutSetScoutRules(const byte m, const byte t, const byte l, const byte tm, const byte fx);
12970 
12978 inline void MSScoutSetSensorClickTime(const byte src, const unsigned int value);
12979 
12987 inline void MSScoutSetSensorHysteresis(const byte src, const unsigned int value);
12988 
12996 inline void MSScoutSetSensorLowerLimit(const byte src, const unsigned int value);
12997 
13005 inline void MSScoutSetSensorUpperLimit(const byte src, const unsigned int value);
13006 
13015 inline void MSScoutSetTimerLimit(const byte tmr, const byte src, const unsigned int value);
13016 
13021 inline void MSScoutUnmuteSound(void);
13022 
13023 #else
13024 
13025 #define SensorMSDROD(_p) asm { getin __RETVAL__, _p, NormalizedValueField }
13026 #define SensorNXTSumoEyesRaw(_p) asm { getin __RETVAL__, _p, NormalizedValueField }
13027 #define SensorMSPressureRaw(_p) asm { getin __RETVAL__, _p, RawValueField }
13028 #define SensorMSCompass(_port, _i2caddr) asm { ReadSensorMSCompass(_port, _i2caddr, __RETVAL__) }
13029 #define ReadSensorMSRTClock(_port, _sec, _min, _hrs, _dow, _date, _month, _year) asm { __ReadSensorMSRTClock(_port, _sec, _min, _hrs, _dow, _date, _month, _year, __RETVAL__) }
13030 #define ReadSensorMSTilt(_port, _i2caddr, _x, _y, _z) asm { __ReadSensorMSTilt(_port, _i2caddr, _x, _y, _z, __RETVAL__) }
13031 #define ReadSensorMSAccel(_port, _i2caddr, _x, _y, _z) asm { __ReadSensorMSAccel(_port, _i2caddr, _x, _y, _z, __RETVAL__) }
13032 
13033 #define MSReadValue(_port, _i2caddr, _reg, _bytes) asm { __MSReadValue(_port, _i2caddr, _reg, _bytes, __RETVAL__, __TMPBYTE__) }
13034 #define MSEnergize(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, MS_CMD_ENERGIZED, __RETVAL__) }
13035 #define MSDeenergize(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, MS_CMD_DEENERGIZED, __RETVAL__) }
13036 #define MSADPAOn(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, MS_CMD_ADPA_ON, __RETVAL__) }
13037 #define MSADPAOff(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, MS_CMD_ADPA_OFF, __RETVAL__) }
13038 
13039 #define DISTNxGP2D12(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, DIST_CMD_GP2D12, __RETVAL__) }
13040 #define DISTNxGP2D120(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, DIST_CMD_GP2D120, __RETVAL__) }
13041 #define DISTNxGP2YA21(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, DIST_CMD_GP2YA21, __RETVAL__) }
13042 #define DISTNxGP2YA02(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, DIST_CMD_GP2YA02, __RETVAL__) }
13043 #define DISTNxDistance(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, DIST_REG_DIST, 2, __RETVAL__, __TMPBYTE__) }
13044 #define DISTNxVoltage(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, DIST_REG_VOLT, 2, __RETVAL__, __TMPBYTE__) }
13045 #define DISTNxModuleType(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, DIST_REG_MODULE_TYPE, 1, __RETVAL__, __TMPBYTE__) }
13046 #define DISTNxNumPoints(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, DIST_REG_NUM_POINTS, 1, __RETVAL__, __TMPBYTE__) }
13047 #define DISTNxMinDistance(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, DIST_REG_DIST_MIN, 2, __RETVAL__, __TMPBYTE__) }
13048 #define DISTNxMaxDistance(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, DIST_REG_DIST_MAX, 2, __RETVAL__, __TMPBYTE__) }
13049 
13050 #define ACCLNxCalibrateX(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, ACCL_CMD_X_CAL, __RETVAL__) }
13051 #define ACCLNxCalibrateXEnd(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, ACCL_CMD_X_CAL_END, __RETVAL__) }
13052 #define ACCLNxCalibrateY(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, ACCL_CMD_Y_CAL, __RETVAL__) }
13053 #define ACCLNxCalibrateYEnd(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, ACCL_CMD_Y_CAL_END, __RETVAL__) }
13054 #define ACCLNxCalibrateZ(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, ACCL_CMD_Z_CAL, __RETVAL__) }
13055 #define ACCLNxCalibrateZEnd(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, ACCL_CMD_Z_CAL_END, __RETVAL__) }
13056 #define ACCLNxResetCalibration(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, ACCL_CMD_RESET_CAL, __RETVAL__) }
13057 #define SetACCLNxSensitivity(_port, _i2caddr, _slevel) asm { __I2CSendCmd(_port, _i2caddr, _slevel, __RETVAL__) }
13058 #define ACCLNxSensitivity(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, ACCL_REG_sENS_LVL, 1, __RETVAL__, __TMPBYTE__) }
13059 #define ACCLNxXOffset(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, ACCL_REG_X_OFFSET, 2, __RETVAL__, __TMPBYTE__) }
13060 #define ACCLNxXRange(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, ACCL_REG_X_RANGE, 2, __RETVAL__, __TMPBYTE__) }
13061 #define ACCLNxYOffset(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, ACCL_REG_Y_OFFSET, 2, __RETVAL__, __TMPBYTE__) }
13062 #define ACCLNxYRange(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, ACCL_REG_Y_RANGE, 2, __RETVAL__, __TMPBYTE__) }
13063 #define ACCLNxZOffset(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, ACCL_REG_Z_OFFSET, 2, __RETVAL__, __TMPBYTE__) }
13064 #define ACCLNxZRange(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, ACCL_REG_Z_RANGE, 2, __RETVAL__, __TMPBYTE__) }
13065 
13066 #define PFMateSend(_port, _i2caddr, _channel, _motors, _cmdA, _spdA, _cmdB, _spdB) asm { __PFMateSend(_port, _i2caddr, _channel, _motors, _cmdA, _spdA, _cmdB, _spdB, __RETVAL__) }
13067 #define PFMateSendRaw(_port, _i2caddr, _channel, _b1, _b2) asm { __PFMateSendRaw(_port, _i2caddr, _channel, _b1, _b2, __RETVAL__) }
13068 
13069 #define NXTServoPosition(_port, _i2caddr, _servo) asm { __MSReadValue(_port, _i2caddr, NXTSERVO_REG_S1_POS+(_servo*2), 2, __RETVAL__, __TMPBYTE__) }
13070 #define NXTServoSpeed(_port, _i2caddr, _servo) asm { __MSReadValue(_port, _i2caddr, NXTSERVO_REG_S1_SPEED+_servo, 1, __RETVAL__, __TMPBYTE__) }
13071 #define NXTServoBatteryVoltage(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTSERVO_REG_VOLTAGE, 1, __RETVAL__, __TMPBYTE__) }
13072 #define SetNXTServoSpeed(_port, _i2caddr, _servo, _speed) asm { __MSWriteToRegister(_port, _i2caddr, NXTSERVO_REG_S1_SPEED+_servo, _speed, __RETVAL__) }
13073 #define SetNXTServoQuickPosition(_port, _i2caddr, _servo, _qpos) asm { __MSWriteToRegister(_port, _i2caddr, NXTSERVO_REG_S1_QPOS+_servo, _qpos, __RETVAL__) }
13074 #define SetNXTServoPosition(_port, _i2caddr, _servo, _pos) asm { __MSWriteLEIntToRegister(_port, _i2caddr, NXTSERVO_REG_S1_POS+(_servo*2), _pos, __RETVAL__) }
13075 #define NXTServoReset(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTSERVO_CMD_RESET, __RETVAL__) }
13076 #define NXTServoHaltMacro(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTSERVO_CMD_HALT, __RETVAL__) }
13077 #define NXTServoResumeMacro(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTSERVO_CMD_RESUME, __RETVAL__) }
13078 #define NXTServoPauseMacro(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTSERVO_CMD_PAUSE, __RETVAL__) }
13079 #define NXTServoInit(_port, _i2caddr, _servo) asm { __NXTServoInit(_port, _i2caddr, _servo, __RETVAL__) }
13080 #define NXTServoGotoMacroAddress(_port, _i2caddr, _macro) asm { __NXTServoGotoMacroAddress(_port, _i2caddr, _macro, __RETVAL__) }
13081 #define NXTServoEditMacro(_port, _i2caddr) asm { __NXTServoEditMacro(_port, _i2caddr, __RETVAL__) }
13082 #define NXTServoQuitEdit(_port) asm { __MSWriteToRegister(_port, MS_ADDR_NXTSERVO_EM, NXTSERVO_EM_REG_CMD, NXTSERVO_EM_CMD_QUIT, __RETVAL__) }
13083 
13084 #define NXTHIDAsciiMode(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTHID_CMD_ASCII, __RETVAL__) }
13085 #define NXTHIDDirectMode(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTHID_CMD_DIRECT, __RETVAL__) }
13086 #define NXTHIDTransmit(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTHID_CMD_TRANSMIT, __RETVAL__) }
13087 #define NXTHIDLoadCharacter(_port, _i2caddr, _modifier, _character) asm { __NXTHIDLoadCharacter(_port, _i2caddr, _modifier, _character, __RETVAL__) }
13088 
13089 #define NXTPowerMeterResetCounters(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTPM_CMD_RESET, __RETVAL__) }
13090 #define NXTPowerMeterPresentCurrent(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_CURRENT, 2, __RETVAL__, __TMPBYTE__) }
13091 #define NXTPowerMeterPresentVoltage(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_VOLTAGE, 2, __RETVAL__, __TMPBYTE__) }
13092 #define NXTPowerMeterCapacityUsed(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_CAPACITY, 2, __RETVAL__, __TMPBYTE__) }
13093 #define NXTPowerMeterPresentPower(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_POWER, 2, __RETVAL__, __TMPBYTE__) }
13094 #define NXTPowerMeterTotalPowerConsumed(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_POWER, 4, __RETVAL__, __TMPBYTE__) }
13095 #define NXTPowerMeterMaxCurrent(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_MAXCURRENT, 2, __RETVAL__, __TMPBYTE__) }
13096 #define NXTPowerMeterMinCurrent(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_MINCURRENT, 2, __RETVAL__, __TMPBYTE__) }
13097 #define NXTPowerMeterMaxVoltage(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_MAXVOLTAGE, 2, __RETVAL__, __TMPBYTE__) }
13098 #define NXTPowerMeterMinVoltage(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_MINVOLTAGE, 2, __RETVAL__, __TMPBYTE__) }
13099 #define NXTPowerMeterElapsedTime(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_TIME, 4, __RETVAL__, __TMPBYTE__) }
13100 #define NXTPowerMeterErrorCount(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTPM_REG_ERRORCOUNT, 2, __RETVAL__, __TMPBYTE__) }
13101 
13102 #define NXTLineLeaderSteering(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTLL_REG_STEERING, 1, __RETVAL__, __TMPBYTE__) }
13103 #define NXTLineLeaderAverage(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTLL_REG_AVERAGE, 1, __RETVAL__, __TMPBYTE__) }
13104 #define NXTLineLeaderResult(_port, _i2caddr) asm { __MSReadValue(_port, _i2caddr, NXTLL_REG_RESULT, 1, __RETVAL__, __TMPBYTE__) }
13105 #define NXTLineLeaderPowerDown(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTLL_CMD_POWERDOWN, __RETVAL__) }
13106 #define NXTLineLeaderPowerUp(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTLL_CMD_POWERUP, __RETVAL__) }
13107 #define NXTLineLeaderInvert(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTLL_CMD_INVERT, __RETVAL__) }
13108 #define NXTLineLeaderReset(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTLL_CMD_RESET, __RETVAL__) }
13109 #define NXTLineLeaderSnapshot(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTLL_CMD_SNAPSHOT, __RETVAL__) }
13110 #define NXTLineLeaderCalibrateWhite(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTLL_CMD_WHITE, __RETVAL__) }
13111 #define NXTLineLeaderCalibrateBlack(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NXTLL_CMD_BLACK, __RETVAL__) }
13112 #define SetNXTLineLeaderSetpoint(_port, _i2caddr, _value) asm { __MSWriteToRegister(_port, _i2caddr, NXTLL_REG_SETPOINT, _value, __RETVAL__) }
13113 #define SetNXTLineLeaderKpValue(_port, _i2caddr, _value) asm { __MSWriteToRegister(_port, _i2caddr, NXTLL_REG_KP_VALUE, _value, __RETVAL__) }
13114 #define SetNXTLineLeaderKiValue(_port, _i2caddr, _value) asm { __MSWriteToRegister(_port, _i2caddr, NXTLL_REG_KI_VALUE, _value, __RETVAL__) }
13115 #define SetNXTLineLeaderKdValue(_port, _i2caddr, _value) asm { __MSWriteToRegister(_port, _i2caddr, NXTLL_REG_KD_VALUE, _value, __RETVAL__) }
13116 #define SetNXTLineLeaderKpFactor(_port, _i2caddr, _value) asm { __MSWriteToRegister(_port, _i2caddr, NXTLL_REG_KP_FACTOR, _value, __RETVAL__) }
13117 #define SetNXTLineLeaderKiFactor(_port, _i2caddr, _value) asm { __MSWriteToRegister(_port, _i2caddr, NXTLL_REG_KI_FACTOR, _value, __RETVAL__) }
13118 #define SetNXTLineLeaderKdFactor(_port, _i2caddr, _value) asm { __MSWriteToRegister(_port, _i2caddr, NXTLL_REG_KD_FACTOR, _value, __RETVAL__) }
13119 
13120 #define PSPNxDigital(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, PSP_CMD_DIGITAL, __RETVAL__) }
13121 #define PSPNxAnalog(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, PSP_CMD_ANALOG, __RETVAL__) }
13122 
13123 #define ReadSensorMSPlayStation(_port, _i2caddr, _b1, _b2, _xleft, _yleft, _xright, _yright) asm { __ReadSensorMSPlayStation(_port, _i2caddr, _b1, _b2, _xleft, _yleft, _xright, _yright, __RETVAL__) }
13124 
13125 #define NRLink2400(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_2400, __RETVAL__) }
13126 #define NRLink4800(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_4800, __RETVAL__) }
13127 #define NRLinkFlush(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_FLUSH, __RETVAL__) }
13128 #define NRLinkIRLong(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_IR_LONG, __RETVAL__) }
13129 #define NRLinkIRShort(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_IR_SHORT, __RETVAL__) }
13130 #define NRLinkTxRaw(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_TX_RAW, __RETVAL__) }
13131 #define NRLinkSetRCX(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_SET_RCX, __RETVAL__) }
13132 #define NRLinkSetTrain(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_SET_TRAIN, __RETVAL__) }
13133 #define NRLinkSetPF(_port, _i2caddr) asm { __I2CSendCmd(_port, _i2caddr, NRLINK_CMD_SET_PF, __RETVAL__) }
13134 
13135 #define RunNRLinkMacro(_port, _i2caddr, _macro) asm { __RunNRLinkMacro(_port, _i2caddr, _macro, __RETVAL__) }
13136 
13137 #define NRLinkStatus(_port, _i2caddr) asm { ReadNRLinkStatus(_port, _i2caddr, __RETVAL__, __TMPBYTE__) }
13138 
13139 #define WriteNRLinkBytes(_port, _i2caddr, _bytes) asm { __WriteNRLinkBytes(_port, _i2caddr, _bytes, __RETVAL__) }
13140 #define ReadNRLinkBytes(_port, _i2caddr, _bytes) asm { __ReadNRLinkBytes(_port, _i2caddr, _bytes, __RETVAL__) }
13141 
13142 #define MSPFComboDirect(_port, _i2caddr, _channel, _outa, _outb) asm { __MSPFComboDirect(_port, _i2caddr, _channel, _outa, _outb, __RETVAL__) }
13143 #define MSPFSinglePin(_port, _i2caddr, _channel, _out, _pin, _func, _cont) asm { __MSPFSinglePin(_port, _i2caddr, _channel, _out, _pin, _func, _cont, __RETVAL__) }
13144 #define MSPFSingleOutputCST(_port, _i2caddr, _channel, _out, _func) asm { __MSPFSingleOutput(_port, _i2caddr, _channel, _out, _func, TRUE, __RETVAL__) }
13145 #define MSPFSingleOutputPWM(_port, _i2caddr, _channel, _out, _func) asm { __MSPFSingleOutput(_port, _i2caddr, _channel, _out, _func, FALSE, __RETVAL__) }
13146 #define MSPFComboPWM(_port, _i2caddr, _channel, _outa, _outb) asm { __MSPFComboPWM(_port, _i2caddr, _channel, _outa, _outb, __RETVAL__) }
13147 #define MSPFTrain(_port, _i2caddr, _channel, _func) asm { __MSIRTrain(_port, _i2caddr, _channel, _func, TRUE, __RETVAL__) }
13148 #define MSIRTrain(_port, _i2caddr, _channel, _func) asm { __MSIRTrain(_port, _i2caddr, _channel, _func, FALSE, __RETVAL__) }
13149 #define MSPFRawOutput(_port, _i2caddr, _nibble0, _nibble1, _nibble2) asm { __MSPFRawOutput(_port, _i2caddr, _nibble0, _nibble1, _nibble2, __RETVAL__) }
13150 #define MSPFRepeat(_port, _i2caddr, _count, _delay) asm { __MSPFRepeatLastCommand(_port, _i2caddr, _count, _delay, __RETVAL__) }
13151 
13152 #define MSRCXSetNRLinkPort(_port, _i2caddr) asm { __MSRCXSetNRLink(_port, _i2caddr) }
13153 #define MSRCXPoll(_src, _value) asm { __MSRCXPoll(_src, _value, __RETVAL__) }
13154 #define MSRCXBatteryLevel() asm { __MSRCXBatteryLevel(__RETVAL__) }
13155 #define MSRCXPing() asm { __MSRCXOpNoArgs(RCX_PingOp) }
13156 #define MSRCXDeleteTasks() asm { __MSRCXOpNoArgs(RCX_DeleteTasksOp) }
13157 #define MSRCXStopAllTasks() asm { __MSRCXOpNoArgs(RCX_StopAllTasksOp) }
13158 #define MSRCXPBTurnOff() asm { __MSRCXOpNoArgs(RCX_PBTurnOffOp) }
13159 #define MSRCXDeleteSubs() asm { __MSRCXOpNoArgs(RCX_DeleteSubsOp) }
13160 #define MSRCXClearSound() asm { __MSRCXOpNoArgs(RCX_ClearSoundOp) }
13161 #define MSRCXClearMsg() asm { __MSRCXOpNoArgs(RCX_ClearMsgOp) }
13162 #define MSRCXMuteSound() asm { __MSRCXOpNoArgs(RCX_MuteSoundOp) }
13163 #define MSRCXUnmuteSound() asm { __MSRCXOpNoArgs(RCX_UnmuteSoundOp) }
13164 #define MSRCXClearAllEvents() asm { __MSRCXOpNoArgs(RCX_ClearAllEventsOp) }
13165 #define MSRCXSetOutput(_outputs, _mode) asm { __MSRCXSetOutput(_outputs, _mode) }
13166 #define MSRCXSetDirection(_outputs, _dir) asm { __MSRCXSetDirection(_outputs, _dir) }
13167 #define MSRCXSetPower(_outputs, _pwrsrc, _pwrval) asm { __MSRCXSetPower(_outputs, _pwrsrc, _pwrval) }
13168 #define MSRCXOn(_outputs) asm { __MSRCXSetOutput(_outputs, RCX_OUT_ON) }
13169 #define MSRCXOff(_outputs) asm { __MSRCXSetOutput(_outputs, RCX_OUT_OFF) }
13170 #define MSRCXFloat(_outputs) asm { __MSRCXSetOutput(_outputs, RCX_OUT_FLOAT) }
13171 #define MSRCXToggle(_outputs) asm { __MSRCXSetDirection(_outputs, RCX_OUT_TOGGLE) }
13172 #define MSRCXFwd(_outputs) asm { __MSRCXSetDirection(_outputs, RCX_OUT_FWD) }
13173 #define MSRCXRev(_outputs) asm { __MSRCXSetDirection(_outputs, RCX_OUT_REV) }
13174 #define MSRCXOnFwd(_outputs) asm { __MSRCXOnFwd(_outputs) }
13175 #define MSRCXOnRev(_outputs) asm { __MSRCXOnRev(_outputs) }
13176 #define MSRCXOnFor(_outputs, _ms) asm { __MSRCXOnFor(_outputs, _ms) }
13177 #define MSRCXSetTxPower(_pwr) asm { __MSRCXSetTxPower(_pwr) }
13178 #define MSRCXPlaySound(_snd) asm { __MSRCXPlaySound(_snd) }
13179 #define MSRCXDeleteTask(_t) asm { __MSRCXDeleteTask(_t) }
13180 #define MSRCXStartTask(_t) asm { __MSRCXStartTask(_t) }
13181 #define MSRCXStopTask(_t) asm { __MSRCXStopTask(_t) }
13182 #define MSRCXSelectProgram(_prog) asm { __MSRCXSelectProgram(_prog) }
13183 #define MSRCXClearTimer(_timer) asm { __MSRCXClearTimer(_timer) }
13184 #define MSRCXSetSleepTime(_t) asm { __MSRCXSetSleepTime(_t) }
13185 #define MSRCXDeleteSub(_s) asm { __MSRCXDeleteSub(_s) }
13186 #define MSRCXClearSensor(_port) asm { __MSRCXClearSensor(_port) }
13187 #define MSRCXPlayToneVar(_varnum, _duration) asm { __MSRCXPlayToneVar(_varnum, _duration) }
13188 #define MSRCXSetWatch(_hours, _minutes) asm { __MSRCXSetWatch(_hours, _minutes) }
13189 #define MSRCXSetSensorType(_port, _type) asm { __MSRCXSetSensorType(_port, _type) }
13190 #define MSRCXSetSensorMode(_port, _mode) asm { __MSRCXSetSensorMode(_port, _mode) }
13191 #define MSRCXCreateDatalog(_size) asm { __MSRCXCreateDatalog(_size) }
13192 #define MSRCXAddToDatalog(_src, _value) asm { __MSRCXAddToDatalog(_src, _value) }
13193 #define MSRCXSendSerial(_first, _count) asm { __MSRCXSendSerial(_first, _count) }
13194 #define MSRCXRemote(_cmd) asm { __MSRCXRemote(_cmd) }
13195 #define MSRCXEvent(_src, _value) asm { __MSRCXEvent(_src, _value) }
13196 #define MSRCXPlayTone(_freq, _duration) asm { __MSRCXPlayTone(_freq, _duration) }
13197 #define MSRCXSelectDisplay(_src, _value) asm { __MSRCXSelectDisplay(_src, _value) }
13198 #define MSRCXPollMemory(_memaddress) asm { __MSRCXPollMemory(_memaddress, __RETVAL__) }
13199 #define MSRCXSetEvent(_evt, _src, _type) asm { __MSRCXSetEvent(_evt, _src, _type) }
13200 #define MSRCXSetGlobalOutput(_outputs, _mode) asm { __MSRCXSetGlobalOutput(_outputs, _mode) }
13201 #define MSRCXSetGlobalDirection(_outputs, _dir) asm { __MSRCXSetGlobalDirection(_outputs, _dir) }
13202 #define MSRCXSetMaxPower(_outputs, _pwrsrc, _pwrval) asm { __MSRCXSetMaxPower(_outputs, _pwrsrc, _pwrval) }
13203 #define MSRCXEnableOutput(_outputs) asm { __MSRCXSetGlobalOutput(_outputs, RCX_OUT_ON) }
13204 #define MSRCXDisableOutput(_outputs) asm { __MSRCXSetGlobalOutput(_outputs, RCX_OUT_OFF) }
13205 #define MSRCXInvertOutput(_outputs) asm { __MSRCXSetGlobalDirection(_outputs, RCX_OUT_REV) }
13206 #define MSRCXObvertOutput(_outputs) asm { __MSRCXSetGlobalDirection(_outputs, RCX_OUT_FWD) }
13207 #define MSRCXCalibrateEvent(_evt, _low, _hi, _hyst) asm { __MSRCXCalibrateEvent(_evt, _low, _hi, _hyst) }
13208 #define MSRCXSetVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_SetVarOp, _varnum, _src, _value) }
13209 #define MSRCXSumVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_SumVarOp, _varnum, _src, _value) }
13210 #define MSRCXSubVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_SubVarOp, _varnum, _src, _value) }
13211 #define MSRCXDivVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_DivVarOp, _varnum, _src, _value) }
13212 #define MSRCXMulVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_MulVarOp, _varnum, _src, _value) }
13213 #define MSRCXSgnVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_SgnVarOp, _varnum, _src, _value) }
13214 #define MSRCXAbsVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_AbsVarOp, _varnum, _src, _value) }
13215 #define MSRCXAndVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_AndVarOp, _varnum, _src, _value) }
13216 #define MSRCXOrVar(_varnum, _src, _value) asm { __MSRCXVarOp(RCX_OrVarOp, _varnum, _src, _value) }
13217 #define MSRCXSet(_dstsrc, _dstval, _src, _value) asm { __MSRCXSet(_dstsrc, _dstval, _src, _value) }
13218 #define MSRCXUnlock() asm { __MSRCXUnlock() }
13219 #define MSRCXReset() asm { __MSRCXReset() }
13220 #define MSRCXBoot() asm { __MSRCXBoot() }
13221 #define MSRCXSetUserDisplay(_src, _value, _precision) asm { __MSRCXSetUserDisplay(_src, _value, _precision) }
13222 #define MSRCXIncCounter(_counter) asm { __MSRCXIncCounter(_counter) }
13223 #define MSRCXDecCounter(_counter) asm { __MSRCXDecCounter(_counter) }
13224 #define MSRCXClearCounter(_counter) asm { __MSRCXClearCounter(_counter) }
13225 #define MSRCXSetPriority(_p) asm { __MSRCXSetPriority(_p) }
13226 #define MSRCXSetMessage(_msg) asm { __MSRCXSetMessage(_msg) }
13227 
13228 #define MSScoutCalibrateSensor() asm { __MSRCXOpNoArgs(RCX_LSCalibrateOp) }
13229 #define MSScoutMuteSound() asm { __MSScoutMuteSound() }
13230 #define MSScoutUnmuteSound() asm { __MSScoutUnmuteSound() }
13231 #define MSScoutSelectSounds(_grp) asm { __MSScoutSelectSounds(_grp) }
13232 #define MSScoutSetLight(_x) asm { __MSScoutSetLight(_x) }
13233 #define MSScoutSetCounterLimit(_ctr, _src, _value) asm { __MSScoutSetCounterLimit(_ctr, _src, _value) }
13234 #define MSScoutSetTimerLimit(_tmr, _src, _value) asm { __MSScoutSetTimerLimit(_tmr, _src, _value) }
13235 #define MSScoutSetSensorClickTime(_src, _value) asm { __MSScoutSetSensorClickTime(_src, _value) }
13236 #define MSScoutSetSensorHysteresis(_src, _value) asm { __MSScoutSetSensorHysteresis(_src, _value) }
13237 #define MSScoutSetSensorLowerLimit(_src, _value) asm { __MSScoutSetSensorLowerLimit(_src, _value) }
13238 #define MSScoutSetSensorUpperLimit(_src, _value) asm { __MSScoutSetSensorUpperLimit(_src, _value) }
13239 #define MSScoutSetEventFeedback(_src, _value) asm { __MSScoutSetEventFeedback(_src, _value) }
13240 #define MSScoutSendVLL(_src, _value) asm { __MSScoutSendVLL(_src, _value) }
13241 #define MSScoutSetScoutRules(_m, _t, _l, _tm, _fx) asm { __MSScoutSetScoutRules(_m, _t, _l, _tm, _fx) }
13242 #define MSScoutSetScoutMode(_mode) asm { __MSScoutSetScoutMode(_mode) }
13243 
13244 #endif
13245  // end of MindSensorsAPI group
13247 
13251 
13252 
13257 #ifdef __DOXYGEN_DOCS
13258 
13267 inline bool RFIDInit(const byte & port);
13268 
13278 inline bool RFIDMode(const byte & port, const byte & mode);
13279 
13288 inline byte RFIDStatus(const byte & port);
13289 
13299 inline bool RFIDRead(const byte & port, byte & output[]);
13300 
13309 inline bool RFIDStop(const byte & port);
13310 
13320 inline bool RFIDReadSingle(const byte & port, byte & output[]);
13321 
13332 inline bool RFIDReadContinuous(const byte & port, byte & output[]);
13333 
13334 #else
13335 
13336 #define RFIDInit(_port) asm { __RFIDInit(_port, __RETVAL__) }
13337 #define RFIDMode(_port, _mode) asm { __RFIDMode(_port, _mode, __RETVAL__) }
13338 #define RFIDStatus(_port) asm { __RFIDStatus(_port, __RETVAL__) }
13339 #define RFIDRead(_port, _output) asm { __RFIDRead(_port, _output, __RETVAL__) }
13340 #define RFIDStop(_port) asm { __RFIDStop(_port, __RETVAL__) }
13341 #define RFIDReadSingle(_port, _output) asm { __RFIDReadSingle(_port, _output, __RETVAL__) }
13342 #define RFIDReadContinuous(_port, _output) asm { __RFIDReadContinuous(_port, _output, __RETVAL__) }
13343 
13344 #endif
13345  // end of CodatexAPI group
13347  // end of ThirdPartyDevices group
13349 
13350 
13351 
13356 
13357 
13358 
13359 
13360 
13365 #if __FIRMWARE_VERSION > 107
13366 
13376 #define Sqrt(_X) asm { sqrt __FLTRETVAL__, _X }
13377 
13385 inline float sqrt(float x) { asm { sqrt __FLTRETVAL__, x } }
13386 
13387 #ifdef __ENHANCED_FIRMWARE
13388 
13398 #define Sin(_X) asm { sin __FLTRETVAL__, _X }
13399 
13409 #define Cos(_X) asm { cos __FLTRETVAL__, _X }
13410 
13420 #define Asin(_X) asm { asin __FLTRETVAL__, _X }
13421 
13431 #define Acos(_X) asm { acos __FLTRETVAL__, _X }
13432 
13442 #define Atan(_X) asm { atan __FLTRETVAL__, _X }
13443 
13453 #define Ceil(_X) asm { ceil __FLTRETVAL__, _X }
13454 
13465 #define Exp(_X) asm { exp __FLTRETVAL__, _X }
13466 
13476 #define Floor(_X) asm { floor __FLTRETVAL__, _X }
13477 
13487 #define Tan(_X) asm { tan __FLTRETVAL__, _X }
13488 
13498 #define Tanh(_X) asm { tanh __FLTRETVAL__, _X }
13499 
13509 #define Cosh(_X) asm { cosh __FLTRETVAL__, _X }
13510 
13520 #define Sinh(_X) asm { sinh __FLTRETVAL__, _X }
13521 
13533 #define Log(_X) asm { log __FLTRETVAL__, _X }
13534 
13545 #define Log10(_X) asm { log10 __FLTRETVAL__, _X }
13546 
13559 #define Atan2(_Y,_X) asm { atan2 __FLTRETVAL__, _Y, _X }
13560 
13571 #define Pow(_Base,_Exponent) asm { pow __FLTRETVAL__, _Base, _Exponent }
13572 
13582 #define Trunc(_X) asm { trunc __RETVAL__, _X }
13583 
13593 #define Frac(_X) asm { frac __FLTRETVAL__, _X }
13594 
13607 #define MulDiv32(_A,_B,_C) asm { muldiv __RETVAL__, _A, _B, _C }
13608 
13618 #define SinD(_X) asm { sind __FLTRETVAL__, _X }
13619 
13629 #define CosD(_X) asm { cosd __FLTRETVAL__, _X }
13630 
13640 #define AsinD(_X) asm { asind __FLTRETVAL__, _X }
13641 
13651 #define AcosD(_X) asm { acosd __FLTRETVAL__, _X }
13652 
13662 #define AtanD(_X) asm { atand __FLTRETVAL__, _X }
13663 
13673 #define TanD(_X) asm { tand __FLTRETVAL__, _X }
13674 
13684 #define TanhD(_X) asm { tanhd __FLTRETVAL__, _X }
13685 
13695 #define CoshD(_X) asm { coshd __FLTRETVAL__, _X }
13696 
13706 #define SinhD(_X) asm { sinhd __FLTRETVAL__, _X }
13707 
13718 #define Atan2D(_Y,_X) asm { atan2d __FLTRETVAL__, _Y, _X }
13719 
13729 inline float cos(float x) { asm { cos __FLTRETVAL__, x } }
13730 
13740 inline float sin(float x) { asm { sin __FLTRETVAL__, x } }
13741 
13751 inline float tan(float x) { asm { tan __FLTRETVAL__, x } }
13752 
13763 inline float acos(float x) { asm { acos __FLTRETVAL__, x } }
13764 
13775 inline float asin(float x) { asm { asin __FLTRETVAL__, x } }
13776 
13791 inline float atan(float x) { asm { atan __FLTRETVAL__, x } }
13792 
13806 inline float atan2(float y, float x) { asm { atan2 __FLTRETVAL__, y, x } }
13807 
13817 inline float cosh(float x) { asm { cosh __FLTRETVAL__, x } }
13818 
13828 inline float sinh(float x) { asm { sinh __FLTRETVAL__, x } }
13829 
13839 inline float tanh(float x) { asm { tanh __FLTRETVAL__, x } }
13840 
13851 inline float exp(float x) { asm { exp __FLTRETVAL__, x } }
13852 
13865 inline float log(float x) { asm { log __FLTRETVAL__, x } }
13866 
13878 inline float log10(float x) { asm { log10 __FLTRETVAL__, x } }
13879 
13889 inline long trunc(float x) { asm { trunc __RETVAL__, x } }
13890 
13900 inline float frac(float x) { asm { frac __FLTRETVAL__, x } }
13901 
13912 inline float pow(float base, float exponent) { asm { pow __FLTRETVAL__, base, exponent } }
13913 
13923 inline float ceil(float x) { asm { ceil __FLTRETVAL__, x } }
13924 
13934 inline float floor(float x) { asm { floor __FLTRETVAL__, x } }
13935 
13948 inline long muldiv32(long a, long b, long c) { asm { muldiv __RETVAL__, a, b, c } }
13949 
13950 // degree-based trig functions
13951 
13961 inline float cosd(float x) { asm { cosd __FLTRETVAL__, x } }
13962 
13972 inline float sind(float x) { asm { sind __FLTRETVAL__, x } }
13973 
13983 inline float tand(float x) { asm { tand __FLTRETVAL__, x } }
13984 
13995 inline float acosd(float x) { asm { acosd __FLTRETVAL__, x } }
13996 
14007 inline float asind(float x) { asm { asind __FLTRETVAL__, x } }
14008 
14022 inline float atand(float x) { asm { atand __FLTRETVAL__, x } }
14023 
14036 inline float atan2d(float y, float x) { asm { atan2d __FLTRETVAL__, y, x } }
14037 
14047 inline float coshd(float x) { asm { coshd __FLTRETVAL__, x } }
14048 
14058 inline float sinhd(float x) { asm { sinhd __FLTRETVAL__, x } }
14059 
14069 inline float tanhd(float x) { asm { tanhd __FLTRETVAL__, x } }
14070 
14071 #endif
14072 
14073 #else
14074 
14075 // math functions written by Tamas Sorosy (www.sorosy.com)
14076 
14077 // X is any integer; Y is the sqrt value (0->max); if X<0, Y is the sqrt value of absolute X
14078 #define Sqrt(_X) asm { __SQRT(_X,__RETVAL__) }
14079 
14080 #endif
14081 
14082 #if (__FIRMWARE_VERSION <= 107) || !defined(__ENHANCED_FIRMWARE)
14083 
14084 // X is any integer in degrees; Y is 100* the sin value (-100->100)
14085 #define Sin(_X) asm { __SIN(_X,__RETVAL__) }
14086 
14087 // X is any integer in degrees; Y is 100* the cos value (-100->100)
14088 #define Cos(_X) asm { __COS(_X,__RETVAL__) }
14089 
14090 // X is 100* the sin value (-100->100); Y is -90->90; Y is 101 if X is outside -100->100 range
14091 #define Asin(_X) asm { __ASIN(_X,__RETVAL__) }
14092 
14093 // X is 100* the cos value (-100->100); Y is 0->180; Y is -11 if X is outside -100->100 range
14094 #define Acos(_X) asm { __ACOS(_X,__RETVAL__) }
14095 
14096 #endif
14097 
14105 inline byte bcd2dec(byte bcd) { asm { __bcd2dec(bcd, __URETVAL__) } }
14106 
14107 #ifdef __DOXYGEN_DOCS
14108 
14116 inline bool isNAN(float value);
14117 
14127 inline char sign(variant num);
14128 
14129 #else
14130 
14131 #define isNAN(_x) ((_x) != (_x))
14132 
14133 #endif
14134  // end of cmathAPI group
14135 
14136 
14140 
14141 
14154 inline int fclose(byte handle) { return CloseFile(handle); }
14155 
14164 inline int remove(string filename) { return DeleteFile(filename); }
14165 
14175 inline int rename(string old, string new) { return RenameFile(old, new); }
14176 
14187 inline char fgetc(byte handle) {
14188   char ch;
14189   asm {
14190     __readValue(handle, ch, __RETVAL__)
14191     mov __RETVAL__, ch
14192   }
14193 }
14194 
14205 #define getc(_handle) fgetc(_handle)
14206 
14222 inline string fgets(string & str, int num, byte handle) {
14223   asm { __readLnStringEx(handle, str, num, __RETVAL__) };
14224   return str;
14225 }
14226 
14235 inline int feof(byte handle) { return 0; }
14236 
14237 unsigned long __fopen_default_size = 1024;
14238 
14245 inline void set_fopen_size(unsigned long fsize) { __fopen_default_size = fsize; }
14246 
14260 byte fopen(string filename, const string mode) {
14261   byte handle;
14262   int result = LDR_ILLEGALHANDLE;
14263   unsigned long fsize;
14264   switch(mode) {
14265     case "r" :
14266       result = OpenFileRead(filename, fsize, handle);
14267       break;
14268     case "w" :
14269       fsize  = __fopen_default_size;
14270       result = CreateFile(filename, fsize, handle);
14271       break;
14272     case "a" :
14273       result = OpenFileAppend(filename, fsize, handle);
14274       break;
14275   }
14276   if (result != LDR_SUCCESS)
14277     handle = NULL;
14278   return handle;
14279 }
14280 
14288 inline int fflush(byte handle) { return 0; }
14289 
14290 #if defined(__ENHANCED_FIRMWARE) && (__FIRMWARE_VERSION > 107)
14291 
14301 inline unsigned long ftell(byte handle) {
14302   FileTellType ftt;
14303   ftt.FileHandle = handle;
14304   SysFileTell(ftt);
14305   return ftt.Position;
14306 }
14307 #endif
14308 
14321 inline char fputc(char ch, byte handle) {
14322   if (Write(handle, ch) == LDR_SUCCESS)
14323     return ch;
14324   else
14325     return EOF;
14326 }
14327 
14340 #define putc(_ch, _handle) fputc(_ch, _handle)
14341 
14353 inline int fputs(string str, byte handle) {
14354   int cnt;
14355   if (WriteString(handle, str, cnt) == LDR_SUCCESS)
14356     return cnt;
14357   else
14358     return EOF;
14359 }
14360 
14361 #ifdef __ENHANCED_FIRMWARE
14362 
14363 #ifdef __DOXYGEN_DOCS
14364 
14376 inline void printf(string format, variant value);
14377 
14390 inline void fprintf(byte handle, string format, variant value);
14391 
14404 inline void sprintf(string & str, string format, variant value);
14405 
14406 #else
14407 
14408 #define printf(_format, _value) { \
14409   string msg = FormatNum(_format, _value); \
14410   TextOut(0, LCD_LINE1, msg); \
14411 }
14412 #define fprintf(_handle, _format, _value) { \
14413   int cnt = fputs(FormatNum(_format, _value), _handle); \
14414 }
14415 #define sprintf(_str, _format, _value) { \
14416   _str = FormatNum(_format, _value); \
14417 }
14418 
14419 #endif
14420 
14421 #if __FIRMWARE_VERSION > 107
14422 
14427 #define SEEK_SET 0 
14428 #define SEEK_CUR 1 
14429 #define SEEK_END 2  // end of fseekConstants group
14431 
14446 inline int fseek(byte handle, long offset, int origin) {
14447   FileSeekType fst;
14448   fst.FileHandle = handle;
14449   fst.Origin = origin;
14450   fst.Length = offset;
14451   SysFileSeek(fst);
14452   return fst.Result;
14453 }
14454 
14464 inline void rewind(byte handle) { fseek(handle, 0, SEEK_SET); }
14465 
14475 inline int getchar() {
14476   int result = -1;
14477   while (true) {
14478     if (ButtonPressed(BTN1, false))
14479       result = BTN1;
14480     else if (ButtonPressed(BTN2, false))
14481       result = BTN2;
14482     else if (ButtonPressed(BTN3, false))
14483       result = BTN3;
14484     else if (ButtonPressed(BTN4, false))
14485       result = BTN4;
14486     if (result != -1)
14487       break;
14488     else
14489       Yield();
14490   }
14491   while(ButtonPressed(result, false));
14492   return result;
14493 }
14494 
14495 
14496 #endif
14497 #endif
14498 
14499 /*
14500   size_t fread(ptr, size, count, FILE*); // read blocks of data from file; returns number of blocks read
14501   size_t fwrite(ptr, size, count, FILE*); // write blocks of data to stream; returns number of blocks written
14502   int putchar(int character); // write character to stdout
14503 */
14504 
14505 
14506  // end of cstdioAPI group
14508 
14509 
14513 
14514 
14531 struct RandomNumberType {
14532   int Result; 
14533 };
14534 
14543 struct div_t {
14544   int quot;  
14547   int rem;   
14550 };
14551 
14559 struct ldiv_t {
14560   long quot;  
14563   long rem;   
14566 };
14567  // end of cstdlibAPITypes group
14569 
14570 #ifdef __DOXYGEN_DOCS
14571 
14577 inline void abort();
14578 
14587 inline variant abs(variant num);
14588 
14595 inline unsigned int rand();
14596 
14606 inline int Random(unsigned int n = 0);
14607 
14615 inline void SysRandomNumber(RandomNumberType & args);
14616 
14617 #else
14618 
14619 #define abort() Stop(true)
14620 #define rand() Random(RAND_MAX)
14621 
14622 #define SysRandomNumber(_args) asm { \
14623   compchktype _args, RandomNumberType \
14624   syscall RandomNumber, _args \
14625 }
14626 
14627 #endif
14628 
14653 inline int atoi(const string & str) { return StrToNum(str); }
14654 
14679 inline long atol(const string & str) { return StrToNum(str); }
14680 
14688 inline long labs(long n) { return abs(n); }
14689 
14690 #if __FIRMWARE_VERSION > 107
14691 
14719 inline float atof(const string & str) {
14720   float result;
14721   asm { strtonum result, __TMPWORD__, str, NA, NA }
14722   return result;
14723 }
14724 
14755 inline float strtod(const string & str, string & endptr) {
14756   float result;
14757   int offsetpast;
14758   asm {
14759     strtonum result, offsetpast, str, NA, NA
14760     strsubset endptr, str, offsetpast, NA
14761   }
14762   return result;
14763 }
14764 #endif
14765 
14792 inline long strtol(const string & str, string & endptr, int base = 10) {
14793   long result;
14794   int offsetpast;
14795   asm {
14796     strtonum result, offsetpast, str, NA, NA
14797     strsubset endptr, str, offsetpast, NA
14798   }
14799   return result;
14800 }
14801 
14828 inline long strtoul(const string & str, string & endptr, int base = 10) {
14829   unsigned long result;
14830   int offsetpast;
14831   asm {
14832     strtonum result, offsetpast, str, NA, NA
14833     strsubset endptr, str, offsetpast, NA
14834   }
14835   return result;
14836 }
14837 
14850 inline div_t div(int numer, int denom) {
14851   div_t result;
14852   result.quot = numer / denom;
14853   result.rem  = numer % denom;
14854   return result;
14855 }
14856 
14869 inline ldiv_t ldiv(long numer, long denom) {
14870   ldiv_t result;
14871   result.quot = numer / denom;
14872   result.rem  = numer % denom;
14873   return result;
14874 }
14875  // end of cstdlibAPI group
14877 
14878 
14882 
14883 
14889 #ifdef __DOXYGEN_DOCS
14890 
14902 inline variant StrToNum(string str);
14903 
14913 inline unsigned int StrLen(string str);
14914 
14925 inline byte StrIndex(string str, unsigned int idx);
14926 
14934 inline string NumToStr(variant num);
14935 
14948 inline string StrCat(string str1, string str2, string strN);
14949 
14961 inline string SubStr(string str, unsigned int idx, unsigned int len);
14962 
14970 inline string Flatten(variant num);
14971 
14984 inline string StrReplace(string str, unsigned int idx, string strnew);
14985 
14998 inline string FormatNum(string fmt, variant num);
14999 
15008 inline string FlattenVar(variant x);
15009 
15020 inline int UnflattenVar(string str, variant & x);
15021 
15022 #else
15023 
15024 #define FlattenVar(_value) asm { flatten __STRRETVAL__, _value }
15025 #define UnflattenVar(_str, _value) asm { \
15026   unflatten _value, __RETVAL__, _str, _value \
15027   not __RETVAL__, __RETVAL__ \
15028 }
15029 
15030 
15031 #endif
15032 
15046 inline int Pos(string Substr, string S) { asm { __doPos(Substr, S, __RETVAL__) } }
15047 
15058 inline string ByteArrayToStr(byte data[]) { asm { arrtostr __STRBUFFER__, data } }
15059 
15071 inline void ByteArrayToStrEx(byte data[], string & str) { asm { arrtostr str, data } }
15072 
15084 inline void StrToByteArray(string str, byte & data[]) { asm { strtoarr data, str } }
15085 
15095 inline string Copy(string str, unsigned int idx, unsigned int len) {
15096   asm { strsubset __STRBUFFER__, str, idx, len  }
15097 }
15098 
15110 inline string MidStr(string str, unsigned int idx, unsigned int len) {
15111   asm { strsubset __STRBUFFER__, str, idx, len  }
15112 }
15113 
15122 inline string RightStr(string str, unsigned int size) {
15123   unsigned int idx;
15124   asm {
15125     strlen idx, str
15126     sub idx, idx, size
15127     strsubset __STRBUFFER__, str, idx, size
15128   }
15129 }
15130 
15139 inline string LeftStr(string str, unsigned int size) {
15140   asm { strsubset __STRBUFFER__, str, 0, size  }
15141 }
15142 
15143 // cstring functions
15144 
15153 inline int strlen(const string & str) { asm { strlen __RETVAL__, str } }
15154 
15167 inline string strcat(string & dest, const string & src) {
15168   asm {
15169     strcat __STRBUFFER__, dest, src
15170     mov dest, __STRBUFFER__
15171   }
15172 }
15173 
15186 inline string strncat(string & dest, const string & src, unsigned int num) {
15187   asm {
15188     strsubset __STRRETVAL__, src, 0, num
15189     strcat __STRBUFFER__, dest, __STRRETVAL__
15190     mov dest, __STRBUFFER__
15191   }
15192 }
15193 
15203 inline string strcpy(string & dest, const string & src) {
15204   asm {
15205     mov __STRBUFFER__, src
15206     mov dest, __STRBUFFER__
15207   }
15208 }
15209 
15220 inline string strncpy(string & dest, const string & src, unsigned int num) {
15221   asm {
15222     strsubset dest, src, 0, num
15223     mov __STRBUFFER__, dest
15224   }
15225 }
15226 
15239 inline int strcmp(const string & str1, const string & str2) {
15240   int result = -1;
15241   if (str1 == str2)
15242     result = 0;
15243   else if (str1 > str2)
15244     result = 1;
15245   return result;
15246 }
15247 
15261 inline int strncmp(const string & str1, const string & str2, unsigned int num) {
15262   string sub1, sub2;
15263   asm {
15264     strsubset sub1, str1, 0, num
15265     strsubset sub2, str2, 0, num
15266   }
15267   int result = -1;
15268   if (sub1 == sub2)
15269     result = 0;
15270   else if (sub1 > sub2)
15271     result = 1;
15272   return result;
15273 }
15274 
15275 #ifdef __DOXYGEN_DOCS
15276 
15286 inline void memcpy(variant dest, variant src, byte num);
15287 
15297 inline void memmove(variant dest, variant src, byte num);
15298 
15309 inline char memcmp(variant ptr1, variant ptr2, byte num);
15310 
15321 inline unsigned long addressOf(variant data);
15322 
15334 inline unsigned long reladdressOf(variant data);
15335 
15349 inline unsigned long addressOfEx(variant data, bool relative);
15350 
15351 #else
15352 
15353 #define memcpy(_dest, _src, _num) asm { mov _dest, _src }
15354 #define memmove(_dest, _src, _num) asm { mov _dest, _src }
15355 #define memcmp(_ptr1, _ptr2, _num) ( (_ptr1 == _ptr2) ? 0 : ( (_ptr1 > _ptr2) ? 1 : -1 ) )
15356 
15357 #define addressOf(_data) asm { addrof __URETVAL__, _data, 0 }
15358 #define reladdressOf(_data) asm { addrof __URETVAL__, _data, 1 }
15359 #define addressOfEx(_data, _rel) asm { addrof __URETVAL__, _data, _rel }
15360 
15361 #endif
15362 
15363 /*
15364 void * memchr (void * ptr, int value, size_t num ); // Locate character in block of memory
15365 char * strchr (       char * str, int character ); // Locate first occurrence of character in string
15366 size_t strcspn ( const char * str1, const char * str2 ); // Get span until character in string
15367 char * strpbrk ( const char *, const char * ); // Locate character in string
15368 char * strrchr ( const char *, int ); // Locate last occurrence of character in string
15369 size_t strspn ( const char * str1, const char * str2 ); // Get span of character set in string
15370 char * strtok ( char * str, const char * delimiters ); // Split string into tokens
15371 char * strstr ( const char *, const char * ); // Locate substring
15372 
15373 void * memset ( void * ptr, byte value, size_t num ); // Fill block of memory (something like replace)
15374 */
15375 
15376 
15377  // end of cstringAPI group
15379 
15380 
15384 
15385 
15398 inline int isupper(int c) { return ((c >= 'A') && (c <= 'Z')); }
15399 
15408 inline int islower(int c) { return ((c >= 'a') && (c <= 'z')); }
15409 
15418 inline int isalpha(int c) { return isupper(c) || islower(c); }
15419 
15428 inline int isdigit(int c) { return ((c >= '0') && (c <= '9')); }
15429 
15440 inline int isalnum(int c) { return isalpha(c) || isdigit(c); }
15441 
15450 inline int isspace(int c) { return (c == 0x20) || ((c >= 0x09) && (c <= 0x0d)); }
15451 
15460 inline int iscntrl(int c) { return (c <= 0x1f) || (c == 0x7f); }
15461 
15471 inline int isprint(int c) { return !iscntrl(c); }
15472 
15481 inline int isgraph(int c) { return (c != 0x20) && isprint(c); }
15482 
15491 inline int ispunct(int c) { return isgraph(c) && !isalnum(c); }
15492 
15501 inline int isxdigit(int c) {  return isdigit(c) || ((c >= 'A') && (c <= 'F')) || ((c >= 'a') && (c <= 'f')); }
15502 
15513 inline int toupper(int c) { if (islower(c)) c -= 32; return c; }
15514 
15525 inline int tolower(int c) { if (isupper(c)) c += 32; return c; }
15526 
15527  // end of ctypeAPI group
15529  // end of StandardCAPIFunctions group
15531 
15532 
15542 #define RICSetValue(_data, _idx, _newval) _data[(_idx)] = (_newval)&0xFF; _data[(_idx)+1] = (_newval)>>8
15543  // end of RICMacros group
15544 
15548 //------------------------------------------------------------------------------
15549 // File          : nbcGL.nbc
15550 // Description   : Data and subroutines for a very simple 3D engine.
15551 // Programmed by : Arno van der Vegt, legoasimo@gmail.com
15552 //------------------------------------------------------------------------------
15553 
15559 inline void glInit() { asm { __glInit() } }
15560 
15568 inline void glSet(int glType, int glValue) { asm { __glSet(glType, glValue) } }
15569 
15577 inline int glBeginObject() { asm { __glBeginObject(__RETVAL__) } }
15578 
15584 inline void glEndObject() { asm { __glEndObject() } }
15585 
15594 inline void glObjectAction(int glObjectId, int glAction, int glValue) {
15595   asm { __glObjectAction(glObjectId, glAction, glValue) }
15596 }
15597 
15608 inline void glAddVertex(int glX, int glY, int glZ) {
15609   asm { __glAddVertex(glX, glY, glZ) }
15610 }
15611 
15619 inline void glBegin(int glBeginMode) { asm { __glBegin(glBeginMode) } }
15620 
15625 inline void glEnd() { asm { __glEnd() } }
15626 
15631 inline void glBeginRender() { asm { __glBeginRender() } }
15632 
15640 inline void glCallObject(int glObjectId) { asm { __glCallObject(glObjectId) } }
15641 
15647 inline void glFinishRender() { asm { __glFinishRender() } }
15648 
15655 inline void glSetAngleX(int glValue) { asm { __glSetAngleX(glValue) } }
15656 
15663 inline void glAddToAngleX(int glValue) { asm { __glAddToAngleX(glValue) } }
15664 
15671 inline void glSetAngleY(int glValue) { asm { __glSetAngleY(glValue) } }
15672 
15679 inline void glAddToAngleY(int glValue) { asm { __glAddToAngleY(glValue) } }
15680 
15687 inline void glSetAngleZ(int glValue) { asm { __glSetAngleZ(glValue) } }
15688 
15695 inline void glAddToAngleZ(int glValue) { asm { __glAddToAngleZ(glValue) } }
15696 
15705 inline int glSin32768(int glAngle) { asm { __glSin32768(__RETVAL__, glAngle) } }
15706 
15715 inline int glCos32768(int glAngle) { asm { __glCos32768(__RETVAL__, glAngle) } }
15716 
15728 inline int glBox(int glMode, int glSizeX, int glSizeY, int glSizeZ) {
15729   asm { __glBox(glMode, glSizeX, glSizeY, glSizeZ, __RETVAL__) }
15730 }
15731 
15741 inline int glCube(int glMode, int glSize) {
15742   asm { __glBox(glMode, glSize, glSize, glSize, __RETVAL__) }
15743 }
15744 
15756 inline int glPyramid(int glMode, int glSizeX, int glSizeY, int glSizeZ) {
15757   asm { __glPyramid(glMode, glSizeX, glSizeY, glSizeZ, __RETVAL__) }
15758 }
15759  // end of GraphicsLibrary group
15761 
15787 inline void PosRegEnable(byte output, byte p = PID_3, byte i = PID_1, byte d = PID_1)
15788 {
15789     SetOutput(output,
15790                OutputModeField, OUT_MODE_MOTORON+OUT_MODE_BRAKE+OUT_MODE_REGULATED,
15791                RegModeField, OUT_REGMODE_POS,
15792                RunStateField, OUT_RUNSTATE_RUNNING,
15793                PowerField, 0,
15794                TurnRatioField, 0,
15795                RegPValueField, p, RegIValueField, i, RegDValueField, d,
15796                UpdateFlagsField, UF_UPDATE_MODE+UF_UPDATE_SPEED+UF_UPDATE_PID_VALUES+UF_UPDATE_RESET_COUNT);
15797     Wait(MS_2);
15798 }
15799 
15811 inline void PosRegSetAngle(byte output, long angle)
15812 {
15813     SetOutput(output,
15814                TachoLimitField, angle,
15815                UpdateFlagsField, UF_UPDATE_TACHO_LIMIT);
15816 }
15817 
15828 inline void PosRegAddAngle(byte output, long angle_add)
15829 {
15830     long current_angle = GetOutput(output, TachoLimitField);
15831     SetOutput(output,
15832                TachoLimitField, current_angle + angle_add,
15833                UpdateFlagsField, UF_UPDATE_TACHO_LIMIT);
15834 }
15835 
15846 inline void PosRegSetMax(byte output, byte max_speed, byte max_acceleration)
15847 {
15848     SetOutput(output,
15849                MaxSpeedField, max_speed,
15850                MaxAccelerationField, max_acceleration,
15851                UpdateFlagsField, UF_UPDATE_PID_VALUES);
15852     Wait(MS_2);
15853 }
15854  // end of OutputModuleFunctions group // end of OutputModule group // end of NXTFirmwareModules group
15858 
15859 #endif // NXCDEFS_H

--------------------------------------------------------------------------------

Generated by   1.6.2 