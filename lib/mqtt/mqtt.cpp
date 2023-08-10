#include "mqtt.h"
#include "passwords.h"
#include "Arduino.h"

int wifi_mqtt_init(mqtt_conn_data *conn)
{
    conn->wifi_client = new WiFiClient;
    conn->mqtt_client = new PubSubClient(*conn->wifi_client);

    delay(10);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    conn->mqtt_client->setServer(mqtt_server, 1883); 

    Serial.println("MQTT Initialized");
    return 1;
}

void wifi_reconnect()
{
    if ((WiFi.status() != WL_CONNECTED))
    {
        WiFi.disconnect();
        WiFi.reconnect();
        delay(100);
    }
    Serial.println("WiFi reconnected");
}
void mqtt_reconnect(mqtt_conn_data *conn)
{
    while (!conn->mqtt_client->connected())
    {
        wifi_reconnect();
        // Attempt to connect
        if (conn->mqtt_client->connect("HF_PILOT_ONE"))
        {
            conn->mqtt_client->subscribe("HF_PILOT_ONE/input");
        }
        else
        {
            delay(1000);
        }
    }
    Serial.println("MQTT reconnected");
}

void mqtt_loop(mqtt_conn_data *conn)
{
    if (!conn->mqtt_client->connected())
    {
        mqtt_reconnect(conn);
    }
    conn->mqtt_client->loop();
}