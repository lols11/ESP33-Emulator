/*
    ESP_33 Emulator v. 0.2 INDEV

    Copyright (c) 2025 by lols11
    Contact: <contact@matbogucki.pl>
    GitHub: https://github.com/lols11

    Licensed under the MIT License.
    See LICENSE and NOTICE files for details.
*/

/*
    This project aims to make TRW450 ABS friends with PLA 3.0 in
    VW47x(PQ46) platform by emulating missing ESP_33 CAN message.

    CAUTION: This software is intended for OFF-ROAD USE ONLY.
*/

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <avr/wdt.h>

#define DEVELOPMENT_MODE false

// Should be enough time to work and not to overload the canbus or trigger DTC due to missing message
// as timeout for this signal is probably smth. like 2000ms
// Probably can be set even lower as ESP_33 is sent also when onChange event occurs with min. timeout 20ms.
#define WATCHDOG_TIMEOUT WDTO_4S

// Standard time for this signal
#define ESP_33_BROADCAST_TIME_MS 200

// After failed attempts arduino will reboot
#define ESP_33_MAX_RETRY_COUNT 10

const unsigned int ESP_33_CAN_ID = 0x1AB;
// boolean CycleThroughESP_33_BZ = true;
unsigned long currentTime = 0, lastSendTime = 0;
byte canStatus = CAN_FAILINIT;
byte failedRetryCount = 0;

MCP_CAN CAN0(10);

void (*reboot)(void) = 0;

uint8_t esp_33[8] = {
    0x00,       // ESP_33_CHK (XOR-2 CS)
    0b00000000, // ESP_33_BZ (Counter 0-15DEC)
    0x00,       // [UNUSED] ESC_Warnruck_aktiv; ESC_Warnruck_nicht_verfuegbar
    0x00,       // ESC_Prefill_aktiv; ESC_Prefill_nicht_verfuegbar; ESC_HBA_aktiv; ESC_HBA_nicht_verfuegbar; ESC_Verz_Reg_aktiv
    0b00000000, // ESC_Verz_Reg_nicht_verfuegbar; ESC_Verz_Reg_TB_nicht_verfuegbar; ESC_Verz_Reg_ZB_nicht_verfuegbar; ESC_Konsistenz_ACC; ESC_Konsistenz_AWV
    0x00,       // [UNUSED] ESC_Konsistenz_RCTA
    0x00,       // ESC_Fahrer_Bremsdruck_bestimmend; ESC_Konsistenz_MKB
    0x00        // [UNUSED]
};

static boolean getESC_Fahrer_Bremsdruck_bestimmend()
{

    return (esp_33[7] & 0b00000001) != 0;
}

static void setESC_Fahrer_Bremsdruck_bestimmend(boolean value)
{
    if (value)
    {
        esp_33[7] |= 0b00000001;
        return;
    }

    esp_33[7] &= 0b11111110;
}

static boolean getESC_Verz_Reg_nicht_verfuegbar()
{
    return (esp_33[5] & 0b00000100) != 0;
}

static void setESC_Verz_Reg_nicht_verfuegbar(boolean value)
{
    if (value)
    {

        esp_33[5] |= 0b00000100;
        return;
    }

    esp_33[5] &= 0b11111011;
}

static uint8_t getESC_Verz_Reg_aktiv()
{
    // Pobiera wartość bitów 4-7 w bajcie 4
    return (esp_33[4] & 0b11110000) >> 4;
}

static void setESC_Verz_Reg_aktiv(uint8_t value)
{
    if (value > 15) // Maksymalna wartość dla 4-bitowego pola
    {
        Serial.println("ESC_Verz_Reg_aktiv: Value out of range (0-15).");
        return;
    }

    // Wyczyszczenie bitów 4-7
    esp_33[4] &= 0b00001111;

    // Ustawienie nowej wartości na bitach 4-7
    esp_33[4] |= (value << 4);
}

static uint8_t getESP_33_BZ()
{

    return esp_33[1] & 0x0F;
}

static void setESP_33_BZ(uint8_t value)
{
    if (value > 15)
    {
        Serial.println(F("ESP_33_BZ out of range. (0-15DEC)"));
        return;
    }

    esp_33[1] = (esp_33[1] & 0xF0) | (value & 0x0F);
}

static void addToCounterESP_33_BZ()
{
    uint8_t actualValue = getESP_33_BZ();
    if (actualValue == 15)
    {
        setESP_33_BZ(0);
        return;
    }
    setESP_33_BZ(actualValue + 1);
}

static uint8_t getESC_Warnruck_aktiv()
{

    return (esp_33[2] & 0xF0) >> 4;
}

static void setESC_Warnruck_aktiv(uint8_t value)
{
    if (value > 9)
    {
        Serial.println(F("ESC_Warnruck_aktiv out of range. (0-9DEC)"));
        return;
    }

    esp_33[2] = (esp_33[2] & 0x0F) | (value << 4);
    Serial.print(F("ESC_Warnruck_aktiv set to: "));
    Serial.println(value);
}

static boolean getESC_Prefill_aktiv()
{
    return (esp_33[4] & 0b00000001) != 0;
}

static void setESC_Prefill_aktiv(boolean value)
{
    if (value)
    {
        esp_33[4] |= 0b00000001;
        return;
    }

    esp_33[4] &= 0b11111110;
}

uint8_t xor_checksum(const uint8_t *d)
{
    uint8_t checksum = 0;
    for (size_t i = 1; i < 8; i++)
    {
        checksum ^= d[i];
    }
    return checksum;
}

boolean sendESP_33()
{

    addToCounterESP_33_BZ();

    esp_33[0] = xor_checksum(esp_33);
    byte status = CAN0.sendMsgBuf(ESP_33_CAN_ID, 0, 8, esp_33);
    //Serial.println(status);
    return status == CAN_OK;
}

void printHelp()
{
    Serial.println(F("===== ESP_33 Emulator ====="));
    Serial.println(F("====== (c) by lols11 ======"));
    Serial.println(F("WARNING! As settings wait for input it may\ngenerate DTC errors"));
    Serial.println(F("H. Help"));
    Serial.println(F("S. Status"));
    // Serial.println(F("1. Manual set ESP_33_BZ (0-15DEC)"));
    // Serial.println(F("2. Cycle through ESP_33_BZ"));
    //  Serial.println(F("3. Set ESC_Warnruck_aktiv (0-9)"));
    //   Serial.println(F("4. Set ESC_Warnruck_nicht_verfuegbar (0-1)"));
    Serial.println(F("5. Set ESC_Prefill_aktiv (0-1)"));
    // Serial.println(F("6. Set ESC_Prefill_nicht_verfuegbar (0-1)"));
    // Serial.println(F("7. Set ESC_HBA_aktiv (0-1)"));
    //   Serial.println(F("8. Set ESC_HBA_nicht_verfuegbar (0-1)"));
    Serial.println(F("9. Set ESC_Verz_Reg_aktiv (0-15)"));
    Serial.println(F("10. Set ESC_Verz_Reg_nicht_verfuegbar (0-1)"));
    //  Serial.println(F("11. Set ESC_Verz_Reg_TB_nicht_verfuegbar (0-1)"));
    // Serial.println(F("12. Set ESC_Verz_Reg_ZB_nicht_verfuegbar (0-1)"));
    // Serial.println(F("13. Set ESC_Konsistenz_ACC (0-1)"));
    // Serial.println(F("14. Set ESC_Konsistenz_AWV (0-1)"));
    // Serial.println(F("15. Set ESC_Konsistenz_RCTA (0-1)"));
    Serial.println(F("16. Set ESC_Fahrer_Bremsdruck_bestimmend (0-1)"));
    //  Serial.println(F("17. Set ESC_Konsistenz_MKB (0-1)"));
}

void handleSerialInput()
{
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("H") || command.equalsIgnoreCase("Help"))
    {
        printHelp();
        return;
    }

    if (command.equalsIgnoreCase("S"))
    {
        Serial.println(F("===== ESP_33 Status ====="));

        // XOR-checksum
        uint8_t checksum = xor_checksum(esp_33);
        Serial.print(F("ESP_33_CHK: "));
        Serial.println(checksum, DEC); // Wyświetlanie w formacie heksadecymalnym

        // ESP_33_BZ
        uint8_t bzValue = getESP_33_BZ();
        Serial.print(F("ESP_33_BZ: "));
        Serial.println(bzValue);

        // ESC_Warnruck_aktiv
        uint8_t warnruckValue = getESC_Warnruck_aktiv();
        Serial.print(F("ESC_Warnruck_aktiv: "));
        Serial.println(warnruckValue);

        // ESC_Prefill_aktiv
        bool prefillValue = getESC_Prefill_aktiv();
        Serial.print(F("ESC_Prefill_aktiv: "));
        Serial.println(prefillValue);

        // ESC_Verz_Reg_aktiv
        uint8_t verzRegValue = getESC_Verz_Reg_aktiv();
        Serial.print(F("ESC_Verz_Reg_aktiv: "));
        Serial.println(verzRegValue);

        // ESC_Verz_Reg_nicht_verfuegbar
        bool verzNichtValue = getESC_Verz_Reg_nicht_verfuegbar();
        Serial.print(F("ESC_Verz_Reg_nicht_verfuegbar: "));
        Serial.println(verzNichtValue);

        // ESC_Fahrer_Bremsdruck_bestimmend
        bool bremsdruckValue = getESC_Fahrer_Bremsdruck_bestimmend();
        Serial.print(F("ESC_Fahrer_Bremsdruck_bestimmend: "));
        Serial.println(bremsdruckValue);

        return;
    }

    if (command.equalsIgnoreCase("1"))
    {
        Serial.println(F("Podaj wartość dla ESP_33_BZ (0-15):"));
        while (Serial.available() == 0)
            ; // Oczekiwanie na dane
        int value = Serial.parseInt();
        if (value >= 0 && value <= 15)
        {
            setESP_33_BZ(value);
            Serial.println(String("ESP_33_BZ ustawione na: ") + String(value));
        }
        else
        {
            Serial.println(F("Błąd: Wartość poza zakresem (0-15)."));
        }
        return;
    }
    //  if (command.startsWith("2"))
    //  {
    //     CycleThroughESP_33_BZ = !CycleThroughESP_33_BZ;
    //     Serial.println("CycleThroughESP_33_BZ set to: " + CycleThroughESP_33_BZ);
    //    return;
    // }
    if (command.equalsIgnoreCase("3"))
    {
        Serial.println(F("Podaj wartość dla ESC_Warnruck_aktiv (0-9):"));
        while (Serial.available() == 0)
            ; // Oczekiwanie na dane
        int value = Serial.parseInt();
        if (value >= 0 && value <= 9)
        {
            setESC_Warnruck_aktiv(value);
            Serial.println(String("ESC_Warnruck_aktiv ustawione na: ") + String(value));
        }
        else
        {
            Serial.println(F("Błąd: Wartość poza zakresem (0-9)."));
        }
        return;
    }

    if (command.equalsIgnoreCase("5"))
    {
        bool status = !getESC_Prefill_aktiv();
        setESC_Prefill_aktiv(status);
        Serial.println(String("ESC_Prefill_aktiv ustawione na: ") + String(status));
        return;
    }

    // Obsługa komendy "9" (Ustawienie ESC_Verz_Reg_aktiv)
    if (command.equalsIgnoreCase("9"))
    {
        Serial.println(F("Podaj wartość dla ESC_Verz_Reg_aktiv (0-15):"));
        while (Serial.available() == 0)
            ; // Oczekiwanie na dane
        int value = Serial.parseInt();
        if (value >= 0 && value <= 15)
        {
            setESC_Verz_Reg_aktiv(value);
            Serial.println(String("ESC_Verz_Reg_aktiv ustawione na: ") + String(value));
        }
        else
        {
            Serial.println(F("Błąd: Wartość poza zakresem (0-15)."));
        }
        return;
    }

    if (command.equalsIgnoreCase("10"))
    {
        bool status = !getESC_Verz_Reg_nicht_verfuegbar();
        setESC_Verz_Reg_nicht_verfuegbar(status);
        Serial.println(String("ESC_Verz_Reg_nicht_verfuegbar ustawione na: ") + String(status));
        return;
    }

    if (command.equalsIgnoreCase("16"))
    {
        bool status = !getESC_Fahrer_Bremsdruck_bestimmend();
        setESC_Fahrer_Bremsdruck_bestimmend(status);
        Serial.println(String("ESC_Fahrer_Bremsdruck_bestimmend ustawione na: ") + String(status));
        return;
    }
    Serial.println(F("Nieznana komenda. Wpisz 'H' dla pomocy."));
}
void setup()
{

    wdt_enable(WATCHDOG_TIMEOUT);

    canStatus = CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);

    if (DEVELOPMENT_MODE)
    {
        Serial.begin(115200);
        Serial.setTimeout(150);
        if (canStatus == CAN_OK)
            Serial.println(F("MCP2515 Initialized Successfully!"));
        else
        {
            Serial.println(F("Error Initializing MCP2515..."));
            Serial.println(F("Rebooting..."));
        }
    }
    if (canStatus != CAN_OK)
        reboot();

    CAN0.setMode(MCP_NORMAL);
    wdt_reset();
}

void loop()
{
    currentTime = millis();

    // Check if it's time to send the broadcast
    if (currentTime - lastSendTime >= ESP_33_BROADCAST_TIME_MS)
    {
        if (!sendESP_33())
        {
            failedRetryCount++;
        }
        else
        {
            failedRetryCount = 0;
        }

        lastSendTime = currentTime;

        // Reboot if maximum retry count is reached as we are close to the DTC timeout
        if (failedRetryCount >= ESP_33_MAX_RETRY_COUNT)
        {
            Serial.println("Maximum retry count reached. Rebooting...");
            delay(5);
            reboot();
        }
    }

    if (DEVELOPMENT_MODE && Serial.available() > 0)
    {
        handleSerialInput();
    }

    wdt_reset();
}