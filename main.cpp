#include "mbed.h"
#include "MXCHIPInterface.h"
#include "sht2x.h"
#include "http_request.h"

#define HOST_URL "http://192.168.0.252:8080/send"

MXCHIPInterface wifi(D10,D2);
Serial pc(STDIO_UART_TX,STDIO_UART_RX, 115200);
SHT2x sht20(I2C_SDA, I2C_SCL);

int rh, temp;
int err = 0;
float relHumidity, temperature;
int userRegister;


void set_sensor()

{
    sht20.softReset();

    printf("Setting user register...\r\n");
    err |= sht20.readUserRegister(&userRegister);  //get actual user reg
    userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_12_14BIT;
    err |= sht20.writeUserRegister(&userRegister); //write changed user reg 
}

void measure()
{
    int err = 0;
    
    printf("Start sensing...\r\n");
    err |= sht20.measureHM(HUMIDITY, &rh);
    err |= sht20.measureHM(TEMP, &temp);

    if (err > 0)
        printf("Error code %d \r\n", err);

    relHumidity = sht20.calcRH(rh);
    temperature = sht20.calcTemperatureC(temp);

    printf("RH value -> %f \r\n", relHumidity);
    printf("Temp in C -> %f \r\n", temperature);
    printf("Dew point %f \r\n\r\n", sht20.getDewpoint(relHumidity, temperature));

}

void sendHttp(NetworkInterface *network)
{
    char body[200];
    std::sprintf(body, "{\"humidity\":%f,\"temperature\":%f}", relHumidity, temperature);

    // char body[] = "{\"humidity\":\"\", \"temperature\":}";
    HttpRequest* request = new HttpRequest(network, HTTP_GET, HOST_URL);
    request->set_header("Content-Type", "application/json");
    printf("Sending request...\r\n");
    HttpResponse* response = request->send(body, strlen(body));
    printf("status is %d - %s\r\n", response->get_status_code(), response->get_status_message());
    printf("body is:\r\n%s\r\n", response->get_body_as_string().c_str());
}

int main()
{
    printf("Connecting wifi...\r\n");
    int ret = wifi.connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
        printf("\r\nConnection error\r\n");
        return -1;
    }

    printf("Success\r\n\r\n");
    // printf("MAC: %s\r\n", wifi.get_mac_address());
    // printf("IP: %s\r\n", wifi.get_ip_address());
    // printf("Netmask: %s\r\n", wifi.get_netmask());
    // printf("Gateway: %s\r\n", wifi.get_gateway());
    // printf("RSSI: %d\r\n\r\n", wifi.get_rssi());
    set_sensor();


    while(1){
        measure();
        sendHttp(&wifi);
        wait(5);
    }

    wifi.disconnect();

    printf("\r\nDone\r\n");
}
