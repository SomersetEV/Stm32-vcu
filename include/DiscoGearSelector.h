#ifndef DiscoGearSelector_h
#define DiscoGearSelector_h

/*  This library supports the Powertrain CAN messages for the BMW E65 for driving dash gauges, putting out malf lights etc
    Also reads gear lever, brake lights etc

*/

#include <stdint.h>
#include "vehicle.h"
#include "my_math.h"

class DiscoGearSelector: public Vehicle
{
public:
//DiscoGearSelector() : terminal15On(false), dashInit(false), gear(PARK) { }
   void SetCanInterface(CanHardware*);
   void Task20Ms();
   void DecodeCAN(int id, uint32_t * data);
   bool GetGear(Vehicle::gear& outGear);
   void SetRevCounter(int speed) { revCounter = speed; }
   void SetTemperatureGauge(float temp) { temperature = temp; }
   bool Ready() { return terminal15On; }


private:
   Vehicle::gear gear;
   bool terminal15On;
   int revCounter;
   float temperature;

   uint8_t Scycle = 0x82;
   uint8_t Fcycle = 0x90;
   uint8_t b0;
   uint8_t b1;
   uint8_t b4;
   uint8_t b5;
   uint8_t b6;
   uint8_t b7;

};

#endif /* DiscoGearSelector_h */

