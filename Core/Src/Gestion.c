

#include "stm32wbxx_hal.h"
#include "iks01a3_mems_control.h"
#include "iks01a3_motion_sensors.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "Gestion.h"
#include <limits.h>

// Définitions pour la configuration des capteurs
#define ALGO_FREQ 50U
#define ACC_ODR ((float)ALGO_FREQ)
#define ACC_FS 4
#define CALIBRATION_SAMPLES 100
#define MOVEMENT_THRESHOLD 100 // Seuil de détection de mouvement pour l'accéléromètre
#define REPETITION_TIME_OUT 2000 // Délai en millisecondes pour déterminer la fin d'une répétition
#define TOLERANCE 500 // Tolérance pour la comparaison des répétitions

// Variables globales
static MOTION_SENSOR_Axes_t AccValue;
static MOTION_SENSOR_Axes_t GyrValue;
static MOTION_SENSOR_Axes_t AccOffset;
static MOTION_SENSOR_Axes_t GyrOffset;
static bool isReferenceMovementRecorded = false;
BenchRep referenceMovement;
BenchRep currentMovement;
static int validMovements = 0;
static int invalidMovements = 0;


int _write(int file, char *ptr, int len) {
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void MX_MEMS_Init(void) {
    Init_Sensors();
    CalibrateSensors();
    // Initialisation des structures BenchRep
    referenceMovement.maxAmplitudeZ = 0;
    referenceMovement.minAmplitudeZ = 0;
    referenceMovement.duration = 0;

    currentMovement.maxAmplitudeZ = 0;
    currentMovement.minAmplitudeZ = 0;
    currentMovement.duration = 0;
}
void MX_MEMS_Process(void) {
    // Réinitialisation des compteurs et de l'état
    validMovements = 0;
    invalidMovements = 0;
    isReferenceMovementRecorded = false;

    while (true) {
        if (!isReferenceMovementRecorded) {
            printf("Enregistrement du mouvement de référence...\n");
            if (!recordBenchRep(&referenceMovement)) {
                printf("Aucun mouvement de référence détecté, fin de la série.\n");
                break; // Sortie si aucun mouvement n'est détecté dans le délai imparti
            }
            isReferenceMovementRecorded = true;
            printf("Mouvement de référence enregistré.\n");
        } else {
            printf("Enregistrement d'une nouvelle répétition...\n");
            if (!recordBenchRep(&currentMovement)) {
                printf("Fin de la série détectée après 5 secondes d'inactivité.\n");
                break; // Sortie si aucun nouveau mouvement n'est détecté dans le délai imparti
            }

            if (compareBenchReps(referenceMovement, currentMovement, TOLERANCE)) {
                validMovements++;
                printf("Répétition valide.\n");
            } else {
                invalidMovements++;
                printf("Répétition non valide.\n");
            }
        }
    }

    // Affichage des résultats à la fin de la série
    printf("Mouvements valides: %d, Mouvements non valides: %d\n", validMovements, invalidMovements);
}





void Init_Sensors(void) {
    BSP_SENSOR_ACC_Init();
    BSP_SENSOR_GYR_Init();

    BSP_SENSOR_ACC_SetOutputDataRate(ACC_ODR);
    BSP_SENSOR_ACC_SetFullScale(ACC_FS);
}

void ReadSensorData(void) {
    BSP_SENSOR_ACC_GetAxes(&AccValue);
    BSP_SENSOR_GYR_GetAxes(&GyrValue);

    // Appliquer le décalage après la calibration
    AccValue.x -= AccOffset.x;
    AccValue.y -= AccOffset.y;
    AccValue.z -= AccOffset.z;

    GyrValue.x -= GyrOffset.x;
    GyrValue.y -= GyrOffset.y;
    GyrValue.z -= GyrOffset.z;
}

void CalculateMovementAndSpeed(void) {
    printf("Accéléromètre [X: %ld, Y: %ld, Z: %ld]\n", (long)AccValue.x, (long)AccValue.y, (long)AccValue.z);
    printf("Gyroscope [X: %ld, Y: %ld, Z: %ld]\n", (long)GyrValue.x, (long)GyrValue.y, (long)GyrValue.z);
    HAL_Delay(2000);
}

void CalibrateSensors(void) {
    CalibrateAccelerometer();
    CalibrateGyroscope();
}

void CalibrateAccelerometer(void) {
    int sumX = 0, sumY = 0, sumZ = 0;
    MOTION_SENSOR_Axes_t tempVal;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        BSP_SENSOR_ACC_GetAxes(&tempVal);
        sumX += tempVal.x;
        sumY += tempVal.y;
        sumZ += tempVal.z;
        HAL_Delay(10);
    }

    AccOffset.x = sumX / CALIBRATION_SAMPLES;
    AccOffset.y = sumY / CALIBRATION_SAMPLES;
    AccOffset.z = sumZ / CALIBRATION_SAMPLES;
}

void CalibrateGyroscope(void) {
    int sumX = 0, sumY = 0, sumZ = 0;
    MOTION_SENSOR_Axes_t tempVal;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        BSP_SENSOR_GYR_GetAxes(&tempVal);
        sumX += tempVal.x;
        sumY += tempVal.y;
        sumZ += tempVal.z;
        HAL_Delay(10);
    }

    GyrOffset.x = sumX / CALIBRATION_SAMPLES;
    GyrOffset.y = sumY / CALIBRATION_SAMPLES;
    GyrOffset.z = sumZ / CALIBRATION_SAMPLES;
}

bool recordBenchRep(BenchRep *rep) {
    int maxValueZ = INT_MIN;
    int minValueZ = INT_MAX;
    uint32_t startTime = HAL_GetTick();
    uint32_t lastMovementTime = startTime;
    uint32_t currentTime;
    bool movementDetected = false;
    bool isAscending = false; // Initialisation différée
    int changeOfDirectionCount = 0;
    bool directionDetermined = false;
    uint32_t inactivityStartTime = HAL_GetTick(); // Pour détecter l'inactivité initiale

    while (true) {
        ReadSensorData();

        // Détecter l'inactivité initiale
        if (!movementDetected && (HAL_GetTick() - inactivityStartTime > 5000)) {
            return false; // Aucun mouvement détecté dans les 5 premières secondes
        }

        if (abs(AccValue.z) > MOVEMENT_THRESHOLD) {
            if (!movementDetected) {
                movementDetected = true;
                startTime = HAL_GetTick();
                lastMovementTime = startTime;
                inactivityStartTime = 0; // Réinitialiser puisqu'un mouvement a été détecté
            } else {
                lastMovementTime = HAL_GetTick();
            }
            maxValueZ = fmax(maxValueZ, AccValue.z);
            minValueZ = fmin(minValueZ, AccValue.z);

            if (!directionDetermined) {
                isAscending = AccValue.z > 0;
                directionDetermined = true;
            } else {
                bool currentAscending = AccValue.z > 0;
                if (currentAscending != isAscending) {
                    changeOfDirectionCount++;
                    isAscending = currentAscending;
                }
            }
        }

        currentTime = HAL_GetTick();
        if (movementDetected && (currentTime - lastMovementTime > REPETITION_TIME_OUT)) {
            break; // Fin de l'enregistrement d'un mouvement
        }
    }

    // Enregistrement des données du mouvement
    rep->maxAmplitudeZ = maxValueZ;
    rep->minAmplitudeZ = minValueZ;
    rep->duration = currentTime - startTime;
    rep->changeOfDirectionCount = changeOfDirectionCount;

    return true; // Mouvement détecté et enregistré
}



bool compareBenchReps(BenchRep refRep, BenchRep newRep, int tolerance) {
    if (abs(refRep.maxAmplitudeZ - newRep.maxAmplitudeZ) <= tolerance &&
        abs(refRep.minAmplitudeZ - newRep.minAmplitudeZ) <= tolerance &&
        abs(refRep.duration - newRep.duration) <= tolerance &&
        refRep.changeOfDirectionCount == newRep.changeOfDirectionCount) {
        return true;
    }
    return false;
}

