#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <iostream>
#include <unistd.h>

// SPI配置
#define SPI_CHANNEL 1    // 选择SPI通道1
#define SPI_SPEED 500000 // 设置SPI速度为500kHz，您可以根据实际需要调整
#define SPI_MODE 0       // 设置SPI模式为0

// GPIO引脚配置
#define SPI_CS0_PIN 18   // SPI1 CS0引脚
#define SPI_CS1_PIN 16   // SPI1 CS1引脚
#define SPI_MISO_PIN 22  // SPI1 MISO引脚
#define SPI_MOSI_PIN 37  // SPI1 MOSI引脚

void setupSPI() {
    // 初始化wiringPi库
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "wiringPi setup failed!" << std::endl;
        exit(1);
    }

    // 设置SPI接口
    if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1) {
        std::cerr << "SPI setup failed!" << std::endl;
        exit(1);
    }

    // 配置GPIO引脚
    pinMode(SPI_CS0_PIN, OUTPUT);
    pinMode(SPI_CS1_PIN, OUTPUT);
    pinMode(SPI_MISO_PIN, INPUT);
    pinMode(SPI_MOSI_PIN, OUTPUT);
    
    digitalWrite(SPI_CS0_PIN, HIGH);  // 设置CS引脚初始为高电平
    digitalWrite(SPI_CS1_PIN, HIGH);  // 设置CS引脚初始为高电平
}

void writeSPI(uint8_t data) {
    // 向SPI总线写入数据
    wiringPiSPIDataRW(SPI_CHANNEL, &data, 1);
}

void initializeGC9A01() {
    // 初始化GC9A01，发送初始化命令
    uint8_t resetCommand = 0x01; // 假设这是一个重置命令，实际根据GC9A01的文档调整
    uint8_t initCommand = 0xFE;  // 假设这是一个初始化命令，实际根据GC9A01的文档调整

    // 发送重置命令
    digitalWrite(SPI_CS0_PIN, LOW); // 使能芯片选择CS0
    writeSPI(resetCommand);
    digitalWrite(SPI_CS0_PIN, HIGH); // 禁用芯片选择CS0

    usleep(10000); // 等待10ms
    
    // 发送初始化命令
    digitalWrite(SPI_CS0_PIN, LOW);
    writeSPI(initCommand);
    digitalWrite(SPI_CS0_PIN, HIGH);
}

void sendDataToScreen(uint8_t* data, int length) {
    // 向屏幕发送数据（假设这是屏幕数据）
    digitalWrite(SPI_CS0_PIN, LOW);
    for (int i = 0; i < length; i++) {
        writeSPI(data[i]);
    }
    digitalWrite(SPI_CS0_PIN, HIGH);
}

int main() {
    // 设置SPI和GPIO
    setupSPI();
    
    // 初始化GC9A01显示屏
    initializeGC9A01();

    // 示例：向屏幕发送一组数据
    uint8_t displayData[] = {0x00, 0x01, 0x02, 0x03}; // 发送示例数据，实际数据需要根据GC9A01控制命令调整
    sendDataToScreen(displayData, sizeof(displayData));

    // 保持程序运行
    while (true) {
        // 您可以根据需要添加更多的控制逻辑
        sleep(1);
    }

    return 0;
}
