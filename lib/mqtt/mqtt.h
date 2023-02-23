#ifndef HF_PILOT_MQTT
#define HF_PILOT_MQTT

#include <WiFi.h>
#include <PubSubClient.h>

typedef struct
{
    WiFiClient *wifi_client;
    PubSubClient *mqtt_client;

} mqtt_conn_data;

int wifi_mqtt_init(mqtt_conn_data *conn);
void wifi_reconnect();
void mqtt_reconnect(mqtt_conn_data *conn);
void mqtt_loop(mqtt_conn_data *conn);

#endif