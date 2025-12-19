#include "myServer.h"
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "robot/motors.h"
#include <ArduinoJson.h>

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

void connect_wifi();
void start_server();
void (*posCallback)(float x, float y, float z);
void (*jointCallback)(float t1, float t2, float t3, float t4, float t5, float t6);
void (*zeroCallback)();

// format bytes
String formatBytes(size_t bytes)
{
    if (bytes < 1024)
    {
        return String(bytes) + "B";
    }
    else if (bytes < (1024 * 1024))
    {
        return String(bytes / 1024.0) + "KB";
    }
    else if (bytes < (1024 * 1024 * 1024))
    {
        return String(bytes / 1024.0 / 1024.0) + "MB";
    }
    else
    {
        return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
    }
}

uint8_t startSPIFFS()
{ // Start the SPIFFS and list all contents
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return 1;
    }
    Serial.println("SPIFFS started. Contents:");
    {
        File root = SPIFFS.open("/");
        File file = root.openNextFile();
        while (file)
        { // List the file system contents
            String fileName = file.name();
            size_t fileSize = file.size();
            Serial.printf("\tFS File: /%s, size: %s\r\n", fileName.c_str(), formatBytes(fileSize).c_str());
            file = root.openNextFile();
            delay(5);
        }
        Serial.printf("\n");
    }

    return 0;
}

/**
 * @brief Handles incoming WebSocket messages
 *
 * @param arg Frame info
 * @param data Message data
 * @param len Message length
 */
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
        data[len] = 0;
        // Serial.printf("WebSocket message received: %s\n", (char *)data);
        // Parse JSON message, should be {"x":0.1,"y":0.003,"z":0}
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data);
        if (error)
        {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            return;
        }
        int msgType = doc["type"]; // get message type
        if (msgType == POSITION)
        {
            float x = doc["x"];
            float y = doc["y"];
            float z = doc["z"];
            posCallback(x, y, z);
        }
        else if (msgType == JOINTS)
        {
            float t1 = doc["t1"];
            float t2 = doc["t2"];
            float t3 = doc["t3"];
            float t4 = doc["t4"];
            float t5 = doc["t5"];
            float t6 = doc["t6"];
            jointCallback(t1, t2, t3, t4, t5, t6);
        }
        else if (msgType == ZERO)
        {
            zeroCallback();
        }
    }
}

    /**
     * @brief WebSocket event handler
     *
     */
    void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void *arg, uint8_t *data, size_t len)
    {
        switch (type)
        {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            handleWebSocketMessage(arg, data, len);
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
        }
    }

    /**
     * @brief Connects to wifi
     *
     */
    void connect_wifi()
    {
        // print mac address
        Serial.print("MAC Address: ");
        Serial.println(WiFi.macAddress());

        // change to AP mode
        //  WiFi.mode(WIFI_AP);
        //  WiFi.softAP("ESP32-AP", "123456789");

        // connect to Wi-Fi
#ifdef PASSWORD
        WiFi.begin(mySSID, PASSWORD);
#endif
#ifndef PASSWORD
        WiFi.begin(mySSID);
#endif

        while (WiFi.status() != WL_CONNECTED)
        {
            delay(1000);
            Serial.println("Connecting to WiFi...");
        }
        Serial.println("Connected to " + String(mySSID));
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        // Serial.println(WiFi.softAPIP());

    } // end connect_wifi

    /**
     * @brief Initializes the WebSocket with event handler and adds it to the server
     *
     */
    void initWebSocket()
    {
        ws.onEvent(onEvent);
        server.addHandler(&ws);
    }

    /**
     * @brief Starts the server and the server loop task
     *
     */
    void start_server()
    {

        // Route for root / web page
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send(SPIFFS, "/index.html", "text/html"); });

        server.serveStatic("/", SPIFFS, "/");

        server.begin();
        Serial.println("HTTP server started on port 80");
    } // end start_server

    /**
     * @brief Task to handle server loops
     *
     * @param pvParameters FreeRTOS task parameters
     */
    void server_loop_task(void *pvParameters)
    {
        Serial.println("Server loop task started.");
        while (1)
        {
            ws.cleanupClients();
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Run cleanup every second
        }
    } // end server_loop_task

    /**
     * @brief Connects to wifi and starts the server
     *
     */
    void start_web_services(void (*callback)(float, float, float), void (*jCallback)(float, float, float, float, float, float), void (*zeroCallbackFunc)())
    {
        connect_wifi();
        initWebSocket();
        start_server();

        posCallback = callback;
        jointCallback = jCallback;
        zeroCallback = zeroCallbackFunc;

        // Start task to handle server loops
        xTaskCreate(
            server_loop_task, // Function to implement the task
            "server_loops",   // Name of the task
            16384,            // Stack size in words
            NULL,             // Task input parameter
            1,                // Priority of the task,
            NULL);            // Task handle.

        Serial.println("Web services started.");
    } // end start_web_services
