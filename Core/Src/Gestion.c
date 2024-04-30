

#include "stm32wbxx_hal.h"
#include "iks01a3_mems_control.h"
#include "iks01a3_motion_sensors.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "Gestion.h"
#include <limits.h>
#include"custom_app.h"

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
bool BLE_CheckNewExerciseSignal=true;
bool BLE_CheckEndOfExerciseSignal=true;

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
    bool isExerciseStarted = false;
    bool isReferenceValidated = false;
    bool exerciseEnded = false;

    while (true) {
    	exerciseEnded = false;
        // Attendre le signal de démarrage d'un nouvel exercice
        if (!isExerciseStarted) {
            if (BLE_CheckNewExerciseSignal) {
                isExerciseStarted = true;
                printf("Nouvel exercice détecté, préparation à l'enregistrement...\n");
            }
        }

        if (isExerciseStarted && !isReferenceMovementRecorded) {
            printf("Enregistrement du mouvement de référence...\n");
            if (!recordBenchRep(&referenceMovement)) {
                printf("Aucun mouvement de référence détecté, fin de la tentative.\n");
                isExerciseStarted = false; // Réinitialiser pour un nouveau signal
                continue;
            }
            isReferenceMovementRecorded = true;
            BLE_MVT_REF();
            printf("Mouvement de référence enregistré. En attente de validation...\n");
        }

        // Phase de validation du mouvement de référence
        if (isReferenceMovementRecorded && !isReferenceValidated) {
            printf("Enregistrement du mouvement pour validation...\n");
            if (!recordBenchRep(&currentMovement)) {
                printf("Aucun mouvement détecté pour validation, veuillez réessayer.\n");
                isReferenceMovementRecorded = false; // Demande de réenregistrer le mouvement de référence
                continue;
            }

            if (compareBenchReps(referenceMovement, currentMovement, TOLERANCE)) {
                isReferenceValidated = true;
                printf("Validation réussie. Commencement des répétitions.\n");
            } else {
                printf("Validation échouée, veuillez réenregistrer le mouvement de référence.\n");
                isReferenceMovementRecorded = false; // Réinitialiser pour enregistrer à nouveau le mouvement de référence
                continue;
            }
        }
        // Réinitialiser les compteurs pour une nouvelle série
               validMovements = 0;
               invalidMovements = 0;

               // Enregistrement des répétitions
               while (!exerciseEnded) {
                   printf("Enregistrement d'une nouvelle répétition...\n");
                   if (!recordBenchRep(&currentMovement)) {
                       printf("Fin de la série détectée après une période d'inactivité.\n");
                       break;  // Sortir de la boucle interne si inactivité détectée
                   }

                   if (compareBenchReps(referenceMovement, currentMovement, TOLERANCE)) {
                       validMovements++;
                       printf("Répétition valide.\n");
                   } else {
                       invalidMovements++;
                       printf("Répétition non valide.\n");
                   }

                   // Vérification de la notification de fin d'exercice
                   if (BLE_CheckEndOfExerciseSignal) {
                       exerciseEnded = true;
                       printf("Fin de l'exercice détectée. Terminaison du programme.\n");
                       validMovements = 0;
                       invalidMovements = 0;
                       isReferenceMovementRecorded = false;
                      isExerciseStarted = false;
                       isReferenceValidated = false;
                       MX_MEMS_Init();

                   }
               }

               // Affichage des résultats à la fin de la série
               printf("Mouvements valides: %d, Mouvements non valides: %d\n", validMovements, invalidMovements);

               // Si la fin de l'exercice n'a pas été signalée, recommencez une nouvelle série
               if (!exerciseEnded) {
                   printf("Préparation pour une nouvelle série...\n");
               }
           }
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
