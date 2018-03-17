#include <NewPing.h>
#include "C:\Users\kihey\Documents\Arduino\libraries\mavlink\common\mavlink.h"

#define DEBUG_MODE
//#define TELEMETRY_TEST

//==================================INICIALIZACIONES======================================//
/*Initialization of sensor pins. Library "NewPing"
NewPing NAME(Trigger, Echo, MAXDIST);
The value of MAXDIST is the maximum distance measured by the library.
If an Echo returns a higher value of said distance, it is automatically discarded*/
NewPing        sonar0(3, 4, 300);
NewPing       sonar1(5, 6, 300);
NewPing       sonar2(7, 8, 300);
NewPing       sonar3(9, 10, 300);
NewPing       sonar4(11, 12, 100);

//Variable used to control that the HeartBeat is sent every second
unsigned long   HeartbeatTime = 0;

//Variables used to only send one RCOverride each time
//that is modified, and not saturate the controller of redundant orders
uint16_t      Pitch       = 0;
uint16_t      Roll        = 0;
uint16_t      PitchOut      = 0;
uint16_t      RollOut       = 0;
uint16_t      PitchOutTemp    = 0;
uint16_t      RollOutTemp     = 0;
uint8_t       n         = 0;

/////////////////////////////////////////////////////////////////////Modify Here!!!//////////////////////////////////////////////////////////////////////////////////
//Radio Medium Value Roll & Pitch
#define PITCH_MID             1468
#define ROLL_MID              1432
#define COMPENSATION_TIME         1200              //Time that the inertial compensation works in ms

                                      //                Increase Value                Decrease Value
#define AVOIDANCE_POWER_LEVEL_1        400              //  30cm  ex) 1500 + 200 =    1700      or    1500 - 200 =    1300
#define AVOIDANCE_POWER_LEVEL_2        375              //  90cm  ex) 1500 + 175 =    1675      or    1500 - 175 =    1325 
#define AVOIDANCE_POWER_LEVEL_3        350              //  150cm ex) 1500 + 150 =    1650      or    1500 - 150 =    1350
#define COMPENSTATION_POWER          AVOIDANCE_POWER_LEVEL_1    //COMPENSTATION_POWER must equal with AVOIDANCE_POWER_LEVEL_1     

//DIstance Variable
#define DistActMin               100              //Distance at which the control begins to act
#define DistMin                 50              //Minimum difference between two distances of the same axis to move.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef DEBUG_MODE
  #define AltMin                 1              //Height at which the control begins to act
#else
  #define AltMin                70
#endif // DEBUG_MODE


//Variable for Coding
#define DISTANCE_ARRAY_SIZE            5
#define SENSOR_COUNT               5

enum ValorType { PITCH, ROLL };


//Record to save the data of each sensor
struct SensorValues 
{
  uint16_t    distanceArray[DISTANCE_ARRAY_SIZE] = { 0 };
  uint16_t    mediaDistance = 0;
  bool      isDetectedCollision = false;
  bool      isCompensateActivated= false;
  unsigned long leftCompensateTime = 0;
};

//The variables of each sensor are started
SensorValues sensorValues[SENSOR_COUNT];

//====================================PROGRAMA============================================//

void setup()
{
  Serial.begin(57600);
}

void loop()
{
  if ((millis() - HeartbeatTime) > 1000)
  {
    HeartbeatTime = millis();
    FHeartBeat();
  }
  FSensores();
  FRCOverride();
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
    delay(1000);
  #endif // !TELEMETRY_TEST
#endif
}

//===========================================FUNCIONES====================================//
//Task responsible for measuring the sensors
void FSensores()
{
  ShiftArrays();
  MeasureWithSensors();
  CalculateMediaDistance();
  CompareDistance();
}

//Task that sends the movement commands according to the distances detected by the sensors
void FRCOverride()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  Pitch = ComparePitch(Pitch);
  Roll = CompareRoll(Roll);

#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST

  Serial.print("Pitch :");
  Serial.println(Pitch);
  Serial.print("Roll :");
  Serial.println(Roll);
  #endif // !TELEMETRY_TEST
#endif

  CompensacionInercia();

  if (Pitch != PitchOutTemp || Roll != RollOutTemp) 
  {
    n = 0;
    PitchOutTemp = Pitch;
    RollOutTemp = Roll;
  }
  else
  {
    n += 1;
    if (n == 4)
    {
      RollOut = RollOutTemp;
      PitchOut = PitchOutTemp;
      RCOverride(&msg, len, buf, PitchOut, RollOut);
    }
  }
}

//Scrolls each array of Distances in one position
void ShiftArrays()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    for (uint8_t j = DISTANCE_ARRAY_SIZE - 1; j > 0; j--)
    {
      sensorValues[i].distanceArray[j] = sensorValues[i].distanceArray[j - 1];
    }
  }
}

//==================================SENSORES=====================================//
//The sensors are measured, and they are placed in position 0 of each array
void MeasureWithSensors()
{
  sensorValues[0].distanceArray[0] = sonar0.ping_cm();
  sensorValues[1].distanceArray[0] = sonar1.ping_cm();
  sensorValues[2].distanceArray[0] = sonar2.ping_cm();
  sensorValues[3].distanceArray[0] = sonar3.ping_cm();
  sensorValues[4].distanceArray[0] = sonar4.ping_cm();

#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
    Serial.print("Measure values -> ");
    Serial.print(" Front:"); Serial.print(sensorValues[0].distanceArray[0]);
    Serial.print(" Right:"); Serial.print(sensorValues[1].distanceArray[0]);
    Serial.print(" Rear:"); Serial.print(sensorValues[2].distanceArray[0]);
    Serial.print(" Left:"); Serial.print(sensorValues[3].distanceArray[0]);
    Serial.print(" Bottom:"); Serial.println(sensorValues[4].distanceArray[0]);
  #endif // !TELEMETRY_TEST
#endif

}

//The average of all distances is performed. The 0 are discarded
void CalculateMediaDistance()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    int Total = 0;
    uint8_t Num = 0;
    for (uint8_t j = 0; j < DISTANCE_ARRAY_SIZE; j++)
    {
      if (sensorValues[i].distanceArray[j] != 0 && sensorValues[i].distanceArray[j] < 300)
      {
        Total += sensorValues[i].distanceArray[j];
        Num += 1;
      }
    }

    if (Num > 3)    //3개가 넘어가면 그냥 총합에서 평균을 낸뒤에 평균 값으로 저장시킴
    {
      sensorValues[i].mediaDistance = Total / Num;
    }
    else        //거리가 하나도 측정 되지 않은 경우
    {
          
      sensorValues[i].mediaDistance = 0;
    }
  }
}

//It is checked if the average obtained is below the threshold.
void CompareDistance()
{
  //Mínimo de 10 para la distancia. existen errores de medida 
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (sensorValues[i].mediaDistance != 0 && sensorValues[i].mediaDistance < DistActMin)
    {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" collision Detected");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
      sensorValues[i].isDetectedCollision = true;
    }
    else 
    {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" not collision Detected");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
      sensorValues[i].isDetectedCollision = false;
    }
  }
}

//========================MOVIMIENTO=========================//
uint16_t ComparePitch(uint16_t Pitch) 
{
  int16_t Diferencia = sensorValues[0].mediaDistance - sensorValues[2].mediaDistance;
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
  Serial.print("pitch Axis difference =");
  Serial.println(Diferencia);
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE

  if (sensorValues[4].mediaDistance > AltMin || sensorValues[4].mediaDistance == 0)
  {
    if (abs(Diferencia) > DistMin) 
    {
      //Difference greater than 30 between both sensors
      if (sensorValues[0].isDetectedCollision == true)
      {
        //Detecta el frontal
        if (sensorValues[2].isDetectedCollision == true)
        {
          //Detecta el trasero
          if (sensorValues[0].mediaDistance < sensorValues[2].mediaDistance)
          {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
            Serial.println("front O, Rear 0, front<rear,  Increase Pitch for front collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE

            //El sensor frontal tiene una distancia menor
            return(Pitch = ValorRC(sensorValues[0].mediaDistance, 1,ValorType::PITCH));
          }
          else 
          {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
            Serial.println("front O, Rear 0, front>rear, Decrease Pitch for rear distance");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
            //El sensor trasero tiene una distancia menor
            return(Pitch = ValorRC(sensorValues[2].mediaDistance, 0, ValorType::PITCH));
          }
        }
        else 
        {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
          Serial.println("front O, Rear X, Increase Pitch for front collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
          //Detecta el frontal, pero no el trasero
          return(Pitch = ValorRC(sensorValues[0].mediaDistance, 1, ValorType::PITCH));
        }
      }
      else 
      {
        //No detecta el frontal
        if (sensorValues[2].isDetectedCollision == true)
        {
          //Detecta el trasero
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
          Serial.println("front X, Rear O, Decrease Pitch for rear collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
          return(Pitch = ValorRC(sensorValues[2].mediaDistance, 0, ValorType::PITCH));
        }
        else 
        {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
          Serial.println("No value for pitch");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
          //Ambos tienen una distancia mayor de 150
          return(Pitch = 0);
        }
      }
    }
    else if (sensorValues[0].isDetectedCollision == true && sensorValues[2].mediaDistance == 0)
    {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
      Serial.println("Increase Pitch for front collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
      //Detecta el de adelante, y el de detrás al no detectar nada, devuelve 0
      return(Pitch = ValorRC(sensorValues[0].mediaDistance, 1, ValorType::PITCH));
    }
    else if (sensorValues[0].mediaDistance == 0 && sensorValues[2].isDetectedCollision == true)
    {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
      Serial.println("Decrease Pitch for rear collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
      //Lo mismo pero lo contrario
      return(Pitch = ValorRC(sensorValues[2].mediaDistance, 0, ValorType::PITCH));
    }
    else 
    {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
      Serial.println("No value for pitch");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
      //No detecta ninguno. Ambos a 0
      return(Pitch = 0);
    }
  }
  else 
  {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
    Serial.println("No value for pitch");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
    return(Pitch = 0);
  }
}

uint16_t CompareRoll(uint16_t Roll) 
{
  int16_t Diferencia = sensorValues[1].mediaDistance - sensorValues[3].mediaDistance;
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
  Serial.print("Roll Axis difference =");
  Serial.println(Diferencia);
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE

  if (sensorValues[4].mediaDistance > AltMin || sensorValues[4].mediaDistance == 0)
  {
    if (abs(Diferencia) > DistMin) 
    {
      //Diferencia mayor de 20 entre distancias
      if (sensorValues[1].isDetectedCollision == true)
      {
        //Detecta el derecho
        if (sensorValues[3].isDetectedCollision == true)
        {
          //Detecta el izquierdo
          if (sensorValues[1].mediaDistance < sensorValues[3].mediaDistance)
          {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
            Serial.println("right O, left 0, right<left, Decrease Pitch for right collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
            //El sensor derecho tiene una distancia menor
            return(Roll = ValorRC(sensorValues[1].mediaDistance, 0, ValorType::ROLL));
          }
          else 
          {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
            Serial.println("right O, left 0, right>left, Increase Pitch for left collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
            //El sensor izquierdo tiene una distancia menor
            return(Roll = ValorRC(sensorValues[3].mediaDistance, 1, ValorType::ROLL));
          }
        }
        else 
        {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
          Serial.println("right O, left X, Decrease Pitch for right collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
          //Detecta el derecho, pero no el izquierdo
          return(Roll = ValorRC(sensorValues[1].mediaDistance, 0, ValorType::ROLL));
        }
      }
      else 
      {
        //No detecta el derecho
        if (sensorValues[3].isDetectedCollision == true)
        {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
          Serial.println("right X, left 0, Increase Pitch for left collision");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
          //Detecta el izquierdo
          return(Roll = ValorRC(sensorValues[3].mediaDistance, 1, ValorType::ROLL));
        }
        else 
        {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
          Serial.println("No value for roll");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
          //Ambos tienen una distancia mayor de 150
          return(Roll = 0);
        }
      }
    }
    else if (sensorValues[1].isDetectedCollision == true && sensorValues[3].mediaDistance == 0)
    {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
      Serial.println("Decrease Pitch for right collision - ohter case");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE

      //Detecta el derecho, y el izquierdo al no detectar nada, devuelve 0
      return(Roll = ValorRC(sensorValues[1].mediaDistance, 0, ValorType::ROLL));
    }
    else if (sensorValues[1].mediaDistance == 0 && sensorValues[3].isDetectedCollision == true)
    {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
      Serial.println("Increase Pitch for left collision - ohter case");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE

      //Lo mismo pero lo contrario
      return(Roll = ValorRC(sensorValues[3].mediaDistance, 1, ValorType::ROLL));
    }
    else 
    {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
      Serial.println("No value for roll");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
      //No detecta ninguno. Ambos a 0
      return(Roll = 0);
    }
  }
  else 
  {
#ifdef DEBUG_MODE
  #ifndef TELEMETRY_TEST
    Serial.println("No value for roll");
  #endif // !TELEMETRY_TEST
#endif // DEBUG_MODE
    return(Roll = 0);
  }
}

//Devuelve un valor de salida dependiendo de la distancia
//A mayor distancia, menor es la necesidad de movimiento. 
//La variable "Aumentar" es para saber en qué dirección es.
uint16_t ValorRC(uint16_t distance, bool isIncrease,ValorType type) //증가 값인지 확인하고 그에따른 값을 리턴시킴 거리에 따라서 속도를 달리함
{
  if (distance < 30)
  {
    if (isIncrease == true)
    {
      if (type == ValorType::PITCH)
        return(PITCH_MID + AVOIDANCE_POWER_LEVEL_1);

      if (type == ValorType::ROLL)
        return(ROLL_MID + AVOIDANCE_POWER_LEVEL_1);
    }
    else
    {
      if (type == ValorType::PITCH)
        return(PITCH_MID - AVOIDANCE_POWER_LEVEL_1);

      if (type == ValorType::ROLL)
        return(ROLL_MID - AVOIDANCE_POWER_LEVEL_1);
    }
  }
  else if (distance < 90) 
  {
    if (isIncrease == true) 
    {
      if (type == ValorType::PITCH)
        return(PITCH_MID + AVOIDANCE_POWER_LEVEL_2);

      if (type == ValorType::ROLL)
        return(ROLL_MID + AVOIDANCE_POWER_LEVEL_2);
    }
    else 
    {
      if (type == ValorType::PITCH)
        return(PITCH_MID - AVOIDANCE_POWER_LEVEL_2);

      if (type == ValorType::ROLL)
        return(ROLL_MID - AVOIDANCE_POWER_LEVEL_2);
    }
  }
  else if (distance < 150) 
  {
    if (isIncrease == true) 
    {
      if (type == ValorType::PITCH)
        return(PITCH_MID + AVOIDANCE_POWER_LEVEL_3);

      if (type == ValorType::ROLL)
        return(ROLL_MID + AVOIDANCE_POWER_LEVEL_3);
    }
    else 
    {
      if (type == ValorType::PITCH)
        return(PITCH_MID - AVOIDANCE_POWER_LEVEL_3);

      if (type == ValorType::ROLL)
        return(ROLL_MID - AVOIDANCE_POWER_LEVEL_3);
    }
  }
}

void CompensacionInercia() 
{

  if (PitchOut > PITCH_MID && sensorValues[0].isCompensateActivated == false && sensorValues[2].isCompensateActivated == false)
  {
    sensorValues[0].isCompensateActivated = true;
  }
  else if (PitchOut < PITCH_MID && PitchOut != 0 && sensorValues[2].isCompensateActivated == false && sensorValues[0].isCompensateActivated == false)
  {
    sensorValues[2].isCompensateActivated = true;
  }
  else if (PitchOut == 0 && sensorValues[0].isCompensateActivated == true && sensorValues[0].leftCompensateTime == 0)
  {
    sensorValues[0].leftCompensateTime = millis();
  }
  else if (PitchOut == 0 && sensorValues[2].isCompensateActivated == true && sensorValues[2].leftCompensateTime == 0)
  {
    sensorValues[2].leftCompensateTime = millis();
  }

  if (RollOut > ROLL_MID && sensorValues[3].isCompensateActivated == false && sensorValues[1].isCompensateActivated == false)
  {
    sensorValues[3].isCompensateActivated = true;
  }
  else if (RollOut < ROLL_MID && RollOut != 0 && sensorValues[1].isCompensateActivated == false && sensorValues[3].isCompensateActivated == false)
  {
    sensorValues[1].isCompensateActivated = true;
  }
  else if (RollOut == 0 && sensorValues[1].isCompensateActivated == true && sensorValues[1].leftCompensateTime == 0)
  {
    sensorValues[1].leftCompensateTime = millis();
  }
  else if (RollOut == 0 && sensorValues[3].isCompensateActivated == true && sensorValues[3].leftCompensateTime == 0)
  {
    sensorValues[3].leftCompensateTime = millis();
  }

  for (int i = 0; i < 4; i++) 
  {
    if (sensorValues[i].leftCompensateTime != 0 && (sensorValues[i].leftCompensateTime + COMPENSATION_TIME > millis()))
    {
      switch (i)
      {
      case 0:                       //Original Values
        Pitch = PITCH_MID - COMPENSTATION_POWER;    //1300;
        break;
      case 1:
        Roll = ROLL_MID + COMPENSTATION_POWER;      //1700
        break;
      case 2:
        Pitch = PITCH_MID + COMPENSTATION_POWER;    //1700
        break;
      case 3:
        Roll = ROLL_MID - COMPENSTATION_POWER;      //1300;
        break;  
      default:
        break;
      }
    }
    else if (sensorValues[i].leftCompensateTime != 0)
    {
      switch (i) 
      {
      case 0:
      case 2:
        PitchOut = 0;
        sensorValues[i].isCompensateActivated = false;
        sensorValues[i].leftCompensateTime = 0;
        break;
      case 1:
      case 3:
        RollOut = 0;
        sensorValues[i].isCompensateActivated = false;
        sensorValues[i].leftCompensateTime = 0;
        break;
      default:
        break;
      }
    }
  }
}

//============================MAVLINK==========================//
//Task responsible for sending a HeartBeat every second
void FHeartBeat()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  // System ID = 255 = GCS
  mavlink_msg_heartbeat_pack(255, 0, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 1, 0);

  // Copy the message to send buffer
  len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message (.write sends as bytes)

#ifndef DEBUG_MODE
  Serial.write(buf, len);
#endif

#ifdef DEBUG_MODE
  #ifdef TELEMETRY_TEST
    Serial.write(buf, len);
  #endif // TELEMETRY_TEST
#endif // DEBUG_MODE

  //Serial.write("\n\rHeartBeat\n\r");
}

void RCOverride(mavlink_message_t *msg, uint16_t len, uint8_t *buf, uint16_t PitchOut, uint16_t RollOut)
{
  //Pack and send the calculated Pitch and Roll data. Only send if the data is new
  mavlink_msg_rc_channels_override_pack(255, 0, msg, 1, 0, RollOut, PitchOut, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, msg);
#ifndef DEBUG_MODE
  Serial.write(buf, len);
#endif // !DEBUG_MODE

#ifdef DEBUG_MODE
  #ifdef TELEMETRY_TEST
    Serial.write(buf, len);
  #endif // TELEMETRY_TEST
#endif // DEBUG_MODE
/*
  Serial.print("\n\rPitch: ");
  Serial.print(PitchOut);
  Serial.print(",");
  Serial.print(" Roll: ");
  Serial.print(RollOut);*/
}

/*//Armar Dron
//Pack the message
//uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
mavlink_msg_command_long_pack(255, 0, &msg, 1, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);

len = mavlink_msg_to_send_buffer(buf, &msg);

// Send the message (.write sends as bytes)
Serial.write(buf, len);
delay(1000);*/

/*mavlink_msg_rc_channels_override_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw,
uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw)*/

/*Channel 1 = Roll
Channel 2 = Pitch
Channel 3 = Throttle
Channel 4 = Yaw*/

/*Sensor0 = Delantero
Sensor1 = Derecha
Sensor2 = Trasero
Sensor3 = Izquierda*/
