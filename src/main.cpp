/*
    ESP_33 Emulator

    Copyright (c) 2024 by lols11
    Contact: <contact@matbogucki.pl>
    GitHub: https://github.com/lols11

    Licensed under the MIT License.
    See LICENSE and NOTICE files for details.
*/

/*
    This project aims to make TRW450 ABS friends with PLA 3.0 in
    VW47x(PQ46) platform by emulating missing ESP_33 CAN message.

    NOTE: This software is intended for OFF-ROAD USE ONLY.
*/

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <avr/wdt.h>
#define DEVELOPMENT_MODE true
#define WATCHDOG_TIMEOUT WDTO_500MS

const unsigned int ESP_33_CAN_ID = 0x1AB;
const unsigned int ESP_33_BROADCAST_TIME_MS = 200; // Standard time for this signal
const byte ESP_33_MAX_RETRY_COUNT = 7;             // After failed attempts arduino will reboot

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
    Serial.print(F("ESP_33_BZ set to: "));
    Serial.println(value);
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

unsigned int xor_checksum(const uint8_t *d)
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
        Serial.println("ESP_33_CHK: " + xor_checksum(esp_33));
        Serial.println("ESP_33_BZ: " + getESP_33_BZ());
        Serial.println("ESC_Warnruck_aktiv: " + getESC_Warnruck_aktiv());
        Serial.println("ESC_Prefill_aktiv: " + getESC_Prefill_aktiv());
        Serial.println("ESC_Verz_Reg_aktiv: " + getESC_Verz_Reg_aktiv());
        Serial.println("ESC_Verz_Reg_nicht_verfuegbar: " + getESC_Verz_Reg_nicht_verfuegbar());
        Serial.println("ESC_Fahrer_Bremsdruck_bestimmend: " + getESC_Fahrer_Bremsdruck_bestimmend());
        return;
    }

    if (command.startsWith("1"))
    {
        Serial.println("Wirte value 0-15: ");
        while (Serial.available() == 0)
        {
            ;
        }

        setESP_33_BZ(Serial.read());
        Serial.println("ok");
        return;
    }
    //  if (command.startsWith("2"))
    //  {
    //     CycleThroughESP_33_BZ = !CycleThroughESP_33_BZ;
    //     Serial.println("CycleThroughESP_33_BZ set to: " + CycleThroughESP_33_BZ);
    //    return;
    // }
    if (command.startsWith("3"))
    {
        Serial.println(F("Wirte value 0-9: "));
        Serial.println(F("no_activity"));
        Serial.println(F("Activity_by_AWV"));
        Serial.println(F("Activity_by_vFGS"));
        Serial.println(F("Activity_by_RCTA"));
        Serial.println(F("Activity_by_FCWO"));
        Serial.println(F("Activity_by_FCWP"));
        Serial.println(F("Activity_by_EA"));
        Serial.println(F("Activity_by_PCF"));
        Serial.println(F("Activity_by_KAS"));
        Serial.println(F("Activity_by_AGW"));

        while (Serial.available() == 0)
        {
            ;
        }
        setESC_Warnruck_aktiv(Serial.read());
        return;
    }

    if (command.startsWith("5"))
    {
        bool status = !getESC_Prefill_aktiv();
        Serial.println("ESC_Prefill_aktiv set to: " + status);
        setESC_Prefill_aktiv(status);
        return;
    }

    if (command.startsWith("9"))
    {
        Serial.println(F("0: keine_Aktivitaet"));
        Serial.println(F("1: Aktivitaet_TB_durch_AWV"));
        Serial.println(F("2: Aktivitaet_ZB_durch_AWV"));
        Serial.println(F("3: Aktivitaet_durch_vFGS"));
        Serial.println(F("4: Aktivitaet_durch_TSK"));
        Serial.println(F("5: Aktivitaet_durch_RCTA"));
        Serial.println(F("6: Aktivitaet_durch_PLA_IPA"));
        Serial.println(F("7: Aktivitaet_durch_STA"));
        Serial.println(F("8: Aktivitaet_durch_ARA"));
        Serial.println(F("9: Aktivitaet_durch_MKB"));
        Serial.println(F("10: Aktivitaet_durch_BFF"));
        Serial.println(F("11: Aktivitaet_durch_EA"));
        Serial.println(F("12: Aktivitaet_durch_PCF"));
        Serial.println(F("13: reserviert"));
        Serial.println(F("14: Initialisierung"));

        while (Serial.available() == 0)
        {
            ;
        }
        setESC_Verz_Reg_aktiv(Serial.read());
        return;
    }

    if (command.startsWith("10"))
    {
        bool status = !getESC_Verz_Reg_nicht_verfuegbar();
        Serial.println("ESC_Verz_Reg_TB_nicht_verfuegbar set to: " + status);
        setESC_Verz_Reg_nicht_verfuegbar(status);
        return;
    }

    if (command.startsWith("16"))
    {
        bool status = !getESC_Fahrer_Bremsdruck_bestimmend();
        Serial.println("ESC_Fahrer_Bremsdruck_bestimmend set to: " + status);
        setESC_Fahrer_Bremsdruck_bestimmend(status);
        return;
    }
}
void setup()
{
    // Should be enough time to work and not to overload the canbus or trigger DTC due to missing message
    // as timeout for this signal is probably smth. like 2000ms
    // Probably can be set even lower as ESP_33 is sent also when onChange event occurs with min. timeout 20ms.
    wdt_enable(WATCHDOG_TIMEOUT);

    canStatus = CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);

    if (DEVELOPMENT_MODE)
    {
        wdt_enable(WDTO_1S);
        Serial.begin(115200);
        Serial.setTimeout(150);
        Serial.println("CS: ");
        Serial.print(xor_checksum(esp_33));
        esp_33[0] = xor_checksum(esp_33);
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
        reboot();
    }

    if (DEVELOPMENT_MODE && Serial.available() > 0)
    {
        handleSerialInput();
    }

    wdt_reset();
}
