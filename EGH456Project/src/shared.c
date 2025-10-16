/**
 * @defgroup shared Shared Items
 * @brief Shared resources for inter-task communication and data protection
 * @{
 */
#include <shared.h>

/**
 * @brief Queue handle for sensor data communication between tasks.
 */
QueueHandle_t eventQueue;

/**
 * @brief Creates a queue used for inter-task communication.
 * 
 * @return 0 on success, -1 on failure to create the queue.
 */
uint8_t createQueue(){
    eventQueue = xQueueCreate(16, sizeof(void*));
    if (eventQueue == NULL){
        return -1;
    }
    return 0;
}

/**
 * @brief Retrieves a copy of the shared value in a thread-safe manner.
 * 
 * @param  dataPoint     Pointer to the shared data structure.
 * @param  buff          Pointer to a buffer where the value will be copied.
 * @param  blockingTime  Maximum time to wait for the mutex.
 * 
 * @return 0 on success, -1 on timeout or error acquiring mutex.
 */
uint8_t getter(sharedValues *dataPoint, val* buff, TickType_t blockingTime){
    if (xSemaphoreTake(dataPoint->mutex, blockingTime) != pdPASS){
        return -1;
    }
    *buff = dataPoint->values;
    xSemaphoreGive(dataPoint->mutex);
    return 0;
}

/**
 * @brief Sets the shared value in a thread-safe manner.
 * 
 * @param   dataPoint    Pointer to the shared data structure.
 * @param   values      New values to set.
 * @param   blockingTime Maximum time to wait for the mutex.
 * 
 * @return 0 on success, -1 on timeout or error acquiring mutex.
 */
uint8_t setter(sharedValues *dataPoint, val values, TickType_t blockingTime){
    if (xSemaphoreTake(dataPoint->mutex, blockingTime) != pdPASS){
        return -1;
    }
    dataPoint->values = values;
    xSemaphoreGive(dataPoint->mutex);
    return 0;
}

/**
 * @brief Sets a specific field (filtered or raw) in the shared value.
 * 
 * @param   dataPoint     Pointer to the shared data structure.
 * @param   value        New value to assign.
 * @param   setting      If true, sets filtered; otherwise, sets raw.
 * @param   blockingTime Maximum time to wait for the mutex.
 * 
 * @return 0 on success, -1 on timeout or error acquiring mutex.
 */
uint8_t setterVal(sharedValues *dataPoint, double value, bool setting, TickType_t blockingTime){
    if (xSemaphoreTake(dataPoint->mutex, blockingTime) != pdPASS){
        return -1;
    }
    if (setting)
        dataPoint->values.filtered = value;
    else 
        dataPoint->values.raw = value;
    xSemaphoreGive(dataPoint->mutex);
    return 0;   
}

/** @} */
