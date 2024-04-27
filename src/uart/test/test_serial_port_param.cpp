#include "serial_port_param.h"

int main(int, char* [])
{
    //EasyLogger &logger = EasyLogger::GetSingleInstance();
    //logger.Init();
    SerialPortParam serialPortParam;
    SerialPortParam::LoadFromYamlFile("config/sentry/2021-09-05/param/serial_port_param.yaml",
                                      &serialPortParam);

    return 0;
}