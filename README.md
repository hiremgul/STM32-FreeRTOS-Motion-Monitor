# STM32 FreeRTOS Motion Monitor

STM32F407 tabanlı, FreeRTOS kullanan hareket algılama ve veri kayıt projesidir.

Projede LIS302DL ivmeölçer verileri okunur, hareket durumu belirlenir, ölçümler STM32 dahili Flash belleğine kaydedilir ve UART üzerinden Python arayüzüne gönderilir.

OLED ekran henüz mevcut olmadığı için veriler geçici olarak Python arayüzünde gösterilecektir. OLED eklendiğinde aynı veriler fiziksel ekrana da aktarılacaktır.

## Özellikler

- STM32F407 mikrodenetleyici
- FreeRTOS task yapısı
- LIS302DL ivmeölçer haberleşmesi
- SPI üzerinden sensör okuma
- X, Y ve Z ekseni verilerinin alınması
- Hareket algılama
- CMSIS-RTOS Message Queue kullanımı
- STM32 dahili Flash belleğine veri kaydı
- CRC ile Flash kayıt kontrolü
- USART2 üzerinden JSON veri gönderimi
- Python tabanlı OLED ekran simülasyonu
- LED ve buton kontrolü

## Sistem Mimarisi

```text
LIS302DL
    |
    v
Gyro Task
    |
    +----> Display Queue ----> Display Task ----> UART ----> Python
    |
    +----> Flash Queue ------> Flash Task ------> Internal Flash
