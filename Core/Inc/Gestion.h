

#ifndef INC_GESTION_H_
#define INC_GESTION_H_
#include <stdbool.h>
typedef struct {
    int maxAmplitudeZ;
    int minAmplitudeZ;
    int duration;
    int changeOfDirectionCount; // Ajout pour compter les changements de direction
} BenchRep;




// Prototypes de fonctions
void MX_MEMS_Init(void);
void MX_MEMS_Process(void);
void Init_Sensors(void);
void ReadSensorData(void);
void CalculateMovementAndSpeed(void);
void CalibrateSensors(void);
void CalibrateAccelerometer(void);
void CalibrateGyroscope(void);
bool recordBenchRep(BenchRep *rep);
bool compareBenchReps(BenchRep refRep, BenchRep newRep, int tolerance);
int _write(int file, char *ptr, int len);


#endif /* INC_GESTION_H_ */
