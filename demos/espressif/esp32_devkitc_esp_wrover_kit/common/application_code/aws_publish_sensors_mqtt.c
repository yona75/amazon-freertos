/*
 * Amazon FreeRTOS MQTT Echo Demo V1.2.6
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */


/**
 * @file aws_publish_sensors_mqtt.c
 * @brief A simple BME280 (temperature, humidity, barrometric preassure) and 
 * BH1730 (luminocity) sensors MQTT publishing example example.
 *
 * It creates an MQTT client that sends sensors data to AWS IoT
 * Another task reads sensor values and store them in gloval variables
 * - "static" declared variables are reside in global RAM section vs local 
 * variables that stays in the task stack memory that is dynamically allocated 
 * within a heap space.
 *
 */

/* Standard includes. */
#include "string.h"
#include "stdio.h"
#include <driver/i2c.h>
#include <driver/gpio.h>
#include "bh1730.h"
#include "bme280.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

/* MQTT includes. */
#include "aws_mqtt_agent.h"

/* Credentials includes. */
#include "aws_clientcredential.h"

/* Demo includes. */
#include "aws_demo_config.h"
#include "aws_hello_world.h"

#define APP_TAG "APP_MAIN"

#define DEVKIT_C_BOARD
//#define PICO_BOARD

#ifdef DEVKIT_C_BOARD
#warning "Selectig Devkit-C board"
#define GPIO_LED  21
#define GPIO_BTN1 36
#define GPIO_BTN2 39
#define GPIO_BTN3 34
#define GPIO_BTN4 35
#define GPIO_SDA  22
#define GPIO_SCL  23
#else
#ifdef PICO_BOARD
#warning "Selectig Pico board"
#define GPIO_LED  23
#define GPIO_BTN1 36
#define GPIO_BTN2 37
#define GPIO_BTN3 38
#define GPIO_BTN4 39
#define GPIO_SDA  22
#define GPIO_SCL  21
#else
#error "No Board selected"
#endif
#endif


/**
 * @brief MQTT client ID.
 *
 * It must be unique per MQTT broker.
 */
#define echoCLIENT_ID          ( ( const uint8_t * ) "esp32" )

/**
 * @brief The topic that the MQTT client both subscribes and publishes to.
 */
#define sensorTOPIC_NAME         ( ( const uint8_t * ) "freertos/demos/sensors" )

/**
 * @brief Dimension of the character array buffers used to hold data (strings in
 * this case) that is published to and received from the MQTT broker (in the cloud).
 */
#define echoMAX_DATA_LENGTH    128

/**
 * @brief A block time of 0 simply means "don't block".
 */
#define echoDONT_BLOCK         ( ( TickType_t ) 0 )

/*-----------------------------------------------------------*/

/**
 * @brief Implements the task that connects to and then publishes messages to the
 * MQTT broker.
 *
 * Messages are published every five seconds for a minute.
 *
 * @param[in] pvParameters Parameters passed while creating the task. Unused in our
 * case.
 */
static void prvMQTTConnectAndPublishTask( void * pvParameters );

/**
 * @brief Creates an MQTT client and then connects to the MQTT broker.
 *
 * The MQTT broker end point is set by clientcredentialMQTT_BROKER_ENDPOINT.
 *
 * @return pdPASS if everything is successful, pdFAIL otherwise.
 */
static BaseType_t prvCreateClientAndConnectToBroker( void );

/**
 * @brief Publishes the next message to the sensorTOPIC_NAME topic.
 *
 * This is called every five seconds to publish the next message.
 *
 * @param[in] xMessageNumber Appended to the message to make it unique.
 */
static void prvPublishNextMessage( BaseType_t xMessageNumber );

/*-----------------------------------------------------------*/

/**
 * @brief Global variables used to pass the sensor data from pvGetSensorValueTask to
 * prvPublishNextMessage task
 */
static unsigned int uiRandNumber = NULL;
static float lux;
static uint32_t press;
static int32_t temp, hum;

/**
 * @ brief The handle of the MQTT client object used by the MQTT echo demo.
 */
static MQTTAgentHandle_t xMQTTHandle = NULL;

/*-----------------------------------------------------------*/
static esp_err_t i2c_init()
{
    //Initialize I2C port in master mode
    esp_err_t ret;
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode=I2C_MODE_MASTER;
    conf.sda_io_num=GPIO_SDA;
    conf.scl_io_num=GPIO_SCL;
    conf.master.clk_speed=100*1000; //standard I2C speed
    ret = i2c_param_config(I2C_NUM_1, &conf);
    if(ret != ESP_OK) {
        ESP_LOGE(APP_TAG, "I2C param config failed");
        return ESP_FAIL;
    }
    ret = i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
    if(ret != ESP_OK) {
        ESP_LOGE(APP_TAG, "unable to install i2c driver");
        return ESP_FAIL;
    }
    return ESP_OK;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

static BaseType_t prvCreateClientAndConnectToBroker( void )
{
    MQTTAgentReturnCode_t xReturned;
    BaseType_t xReturn = pdFAIL;
    MQTTAgentConnectParams_t xConnectParameters =
    {
        clientcredentialMQTT_BROKER_ENDPOINT, /* The URL of the MQTT broker to connect to. */
        democonfigMQTT_AGENT_CONNECT_FLAGS,   /* Connection flags. */
        pdFALSE,                              /* Deprecated. */
        clientcredentialMQTT_BROKER_PORT,     /* Port number on which the MQTT broker is listening. Can be overridden by ALPN connection flag. */
        echoCLIENT_ID,                        /* Client Identifier of the MQTT client. It should be unique per broker. */
        0,                                    /* The length of the client Id, filled in later as not const. */
        pdFALSE,                              /* Deprecated. */
        NULL,                                 /* User data supplied to the callback. Can be NULL. */
        NULL,                                 /* Callback used to report various events. Can be NULL. */
        NULL,                                 /* Certificate used for secure connection. Can be NULL. */
        0                                     /* Size of certificate used for secure connection. */
    };

    /* Check this function has not already been executed. */
    configASSERT( xMQTTHandle == NULL );

    /* The MQTT client object must be created before it can be used.  The
     * maximum number of MQTT client objects that can exist simultaneously
     * is set by mqttconfigMAX_BROKERS. */
    xReturned = MQTT_AGENT_Create( &xMQTTHandle );

    if( xReturned == eMQTTAgentSuccess )
    {
        /* Fill in the MQTTAgentConnectParams_t member that is not const,
         * and therefore could not be set in the initializer (where
         * xConnectParameters is declared in this function). */
        xConnectParameters.usClientIdLength = ( uint16_t ) strlen( ( const char * ) echoCLIENT_ID );

        /* Connect to the broker. */
        configPRINTF( ( "MQTT echo attempting to connect to %s.\r\n", clientcredentialMQTT_BROKER_ENDPOINT ) );
        xReturned = MQTT_AGENT_Connect( xMQTTHandle,
                                        &xConnectParameters,
                                        democonfigMQTT_ECHO_TLS_NEGOTIATION_TIMEOUT );

        if( xReturned != eMQTTAgentSuccess )
        {
            /* Could not connect, so delete the MQTT client. */
            ( void ) MQTT_AGENT_Delete( xMQTTHandle );
            configPRINTF( ( "ERROR:  MQTT echo failed to connect.\r\n" ) );
        }
        else
        {
            configPRINTF( ( "MQTT echo connected.\r\n" ) );
            xReturn = pdPASS;
        }
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

static void prvPublishNextMessage( BaseType_t xMessageNumber )
{
    MQTTAgentPublishParams_t xPublishParameters;
    MQTTAgentReturnCode_t xReturned;
    char cDataBuffer[ echoMAX_DATA_LENGTH ];

    /* Check this function is not being called before the MQTT client object has
     * been created. */
    configASSERT( xMQTTHandle != NULL );

    /* Create the message that will be published, which is of the form "Hello World n"
     * where n is a monotonically increasing number. Note that snprintf appends
     * terminating null character to the cDataBuffer. */
    ( void ) snprintf( cDataBuffer, echoMAX_DATA_LENGTH, "{\"device\": \"%s\", \"msg\": %d, \"tempC\":  %.2f, \"lux\": %.2f }", echoCLIENT_ID, (int) xMessageNumber, (float) temp/100, (float) lux );

    /* Setup the publish parameters. */
    memset( &( xPublishParameters ), 0x00, sizeof( xPublishParameters ) );
    xPublishParameters.pucTopic = sensorTOPIC_NAME;
    xPublishParameters.pvData = cDataBuffer;
    xPublishParameters.usTopicLength = ( uint16_t ) strlen( ( const char * ) sensorTOPIC_NAME );
    xPublishParameters.ulDataLength = ( uint32_t ) strlen( cDataBuffer );
    xPublishParameters.xQoS = eMQTTQoS1;

    /* Publish the message. */
    xReturned = MQTT_AGENT_Publish( xMQTTHandle,
                                    &( xPublishParameters ),
                                    democonfigMQTT_TIMEOUT );

    if( xReturned == eMQTTAgentSuccess )
    {
        configPRINTF( ( "Echo successfully published '%s'\r\n", cDataBuffer ) );
    }
    else
    {
        configPRINTF( ( "ERROR:  Echo failed to publish '%s'\r\n", cDataBuffer ) );
    }

    /* Remove compiler warnings in case configPRINTF() is not defined. */
    ( void ) xReturned;
}
/*-----------------------------------------------------------*/

static void prvGenRandNumTask(void* pvParams)
{
    (void) pvParams;

    while (1)
    {
        uiRandNumber = rand() % (65 + 1 - 0) + 0;
    }
}


/*-----------------------------------------------------------*/
void prvGetSensorValueTask(void* pvParams)
{       
    
    (void) pvParams;

    ESP_LOGI(APP_TAG, "Initializing light sensor");
    bh1730_t *bh = bh1730_init(I2C_NUM_1, 0x29);
    if(bh == NULL) {
        ESP_LOGE(APP_TAG, "Could not initialize light sensor");
        vTaskDelete(NULL);
    }
    ESP_LOGI(APP_TAG, "Light sensor Initialized");
    vTaskDelay(1667 / portTICK_RATE_MS);
    
    ESP_LOGI(APP_TAG, "Initializing temperature sensor");
    bme280_t *bme = bme280_init(I2C_NUM_1, 0x76);
    if(bme == NULL) {
        ESP_LOGE(APP_TAG, "Could not initialize temperature sensor");
        vTaskDelete(NULL);
    }
    ESP_LOGI(APP_TAG, "Temperature sensor Initialized");
    vTaskDelay(1667 / portTICK_RATE_MS);
    
    while(1) { 
        lux = bh1730_read_lux(bh);
        bme280_read(bme, &temp, &press, &hum);
        vTaskDelay(667 / portTICK_RATE_MS);
    }
}

/*-----------------------------------------------------------*/

static void prvMQTTConnectAndPublishTask( void * pvParameters )
{
    BaseType_t x, xReturned;
    const TickType_t xFiveSeconds = pdMS_TO_TICKS( 5000UL );
    const BaseType_t xIterationsInAMinute = 60 / 5;
    TaskHandle_t xGenRandNumTask = NULL;
    TaskHandle_t xGetSensorValueTask = NULL;

    /* Avoid compiler warnings about unused parameters. */
    ( void ) pvParameters;

    /* Create the MQTT client object and connect it to the MQTT broker. */
    xReturned = prvCreateClientAndConnectToBroker();

    if( xReturned == pdPASS )

    {
        /* Create the task that generates random number */
        xReturned = xTaskCreate( prvGetSensorValueTask,               /* The function that implements the task. */
                                 "ReadSensorsValues",                 /* Human readable name for the task. */
                                 democonfigMQTT_ECHO_TASK_STACK_SIZE, /* Size of the stack to allocate for the task, in words not bytes! */
                                 NULL,                                /* The task parameter is not used. */
                                 tskIDLE_PRIORITY,                    /* Runs at the lowest priority. */
                                 &( xGetSensorValueTask ) );          /* The handle is stored so the created task can be deleted again at the end of the demo. */

        if( xReturned != pdPASS )
        {
            /* The task could not be created because there was insufficient FreeRTOS
             * heap available to create the task's data structures and/or stack. */
            configPRINTF( ( "Random number generation task could not be created - out of heap space?\r\n" ) );
        }
    }
    else
    {
        configPRINTF( ( "Random number canot be generated.\r\n" ) );
    }

    if( xReturned == pdPASS )
    {

        /* MQTT client is now connected to a broker.  Publish a message
         * every five seconds until a minute has elapsed. */
        for( x = 0; x < xIterationsInAMinute; x++ )
        {
            prvPublishNextMessage( x );

            /* Five seconds delay between publishes. */
            vTaskDelay( xFiveSeconds );
        }

    }

    /* Disconnect the client. */
    ( void ) MQTT_AGENT_Disconnect( xMQTTHandle, democonfigMQTT_TIMEOUT );

    /* End the demo by deleting all created resources. */
    configPRINTF( ( "MQTT echo demo finished.\r\n" ) );
    vTaskDelete( NULL ); /* Delete this task. */
}
/*-----------------------------------------------------------*/

//void vStartMQTTEchoDemo( void )
void vStartRandDemo( void )
{
    configPRINTF( ( "Creating MQTT Echo Task...\r\n" ) );
    (void) i2c_init();

    /* Create the task that publishes messages to the MQTT broker every five
     * seconds.  This task, in turn, creates the task that echoes data received
     * from the broker back to the broker. */
    ( void ) xTaskCreate( prvMQTTConnectAndPublishTask,        /* The function that implements the demo task. */
                          "MQTTEcho",                          /* The name to assign to the task being created. */
                          democonfigMQTT_ECHO_TASK_STACK_SIZE, /* The size, in WORDS (not bytes), of the stack to allocate for the task being created. */
                          NULL,                                /* The task parameter is not being used. */
                          democonfigMQTT_ECHO_TASK_PRIORITY,   /* The priority at which the task being created will run. */
                          NULL );                              /* Not storing the task's handle. */
}
/*-----------------------------------------------------------*/
