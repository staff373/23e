#include "bsp_oled.h"

#include "bsp_oled_font.h"
#include "ti_msp_dl_config.h"

#define BSP_OLED_SCL_PORT (GPIOA)
#define BSP_OLED_SCL_PIN (DL_GPIO_PIN_4)
#define BSP_OLED_SCL_IOMUX (IOMUX_PINCM9)

#define BSP_OLED_SDA_PORT (GPIOA)
#define BSP_OLED_SDA_PIN (DL_GPIO_PIN_5)
#define BSP_OLED_SDA_IOMUX (IOMUX_PINCM10)

#define BSP_OLED_I2C_ADDR_WRITE (0x78U)
#define BSP_OLED_CONTROL_COMMAND (0x00U)
#define BSP_OLED_CONTROL_DATA (0x40U)

#define BSP_OLED_I2C_DELAY_CYCLES (80U)
#define BSP_OLED_POWER_ON_DELAY_CYCLES (8000000U)
#define BSP_OLED_MAX_LINE (4U)
#define BSP_OLED_MAX_COLUMN (16U)
#define BSP_OLED_CHAR_WIDTH (8U)
#define BSP_OLED_FONT_BYTES_PER_CHAR (16U)

static void BSP_OLED_Delay(void)
{
    delay_cycles(BSP_OLED_I2C_DELAY_CYCLES);
}

static void BSP_OLED_ReleasePin(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_setPins(port, pin);
    DL_GPIO_disableOutput(port, pin);
    BSP_OLED_Delay();
}

static void BSP_OLED_DrivePinLow(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
    BSP_OLED_Delay();
}

static void BSP_OLED_SetScl(uint8_t high)
{
    if (high != 0U) {
        BSP_OLED_ReleasePin(BSP_OLED_SCL_PORT, BSP_OLED_SCL_PIN);
    } else {
        BSP_OLED_DrivePinLow(BSP_OLED_SCL_PORT, BSP_OLED_SCL_PIN);
    }
}

static void BSP_OLED_SetSda(uint8_t high)
{
    if (high != 0U) {
        BSP_OLED_ReleasePin(BSP_OLED_SDA_PORT, BSP_OLED_SDA_PIN);
    } else {
        BSP_OLED_DrivePinLow(BSP_OLED_SDA_PORT, BSP_OLED_SDA_PIN);
    }
}

static void BSP_OLED_I2cInit(void)
{
    DL_GPIO_initDigitalOutputFeatures(BSP_OLED_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(BSP_OLED_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    BSP_OLED_SetScl(1U);
    BSP_OLED_SetSda(1U);
}

static void BSP_OLED_I2cStart(void)
{
    BSP_OLED_SetSda(1U);
    BSP_OLED_SetScl(1U);
    BSP_OLED_SetSda(0U);
    BSP_OLED_SetScl(0U);
}

static void BSP_OLED_I2cStop(void)
{
    BSP_OLED_SetSda(0U);
    BSP_OLED_SetScl(1U);
    BSP_OLED_SetSda(1U);
}

static void BSP_OLED_I2cSendByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        BSP_OLED_SetSda((uint8_t) ((byte & (uint8_t) (0x80U >> i)) != 0U));
        BSP_OLED_SetScl(1U);
        BSP_OLED_SetScl(0U);
    }

    BSP_OLED_SetSda(1U);
    BSP_OLED_SetScl(1U);
    BSP_OLED_SetScl(0U);
}

static void BSP_OLED_WriteCommand(uint8_t command)
{
    BSP_OLED_I2cStart();
    BSP_OLED_I2cSendByte(BSP_OLED_I2C_ADDR_WRITE);
    BSP_OLED_I2cSendByte(BSP_OLED_CONTROL_COMMAND);
    BSP_OLED_I2cSendByte(command);
    BSP_OLED_I2cStop();
}

static void BSP_OLED_WriteData(uint8_t data)
{
    BSP_OLED_I2cStart();
    BSP_OLED_I2cSendByte(BSP_OLED_I2C_ADDR_WRITE);
    BSP_OLED_I2cSendByte(BSP_OLED_CONTROL_DATA);
    BSP_OLED_I2cSendByte(data);
    BSP_OLED_I2cStop();
}

static void BSP_OLED_SetCursor(uint8_t page, uint8_t x)
{
    BSP_OLED_WriteCommand((uint8_t) (0xB0U | page));
    BSP_OLED_WriteCommand((uint8_t) (0x10U | ((x & 0xF0U) >> 4U)));
    BSP_OLED_WriteCommand((uint8_t) (x & 0x0FU));
}

static uint32_t BSP_OLED_Pow(uint32_t x, uint32_t y)
{
    uint32_t result = 1U;

    while (y > 0U) {
        result *= x;
        y--;
    }

    return result;
}

void BSP_OLED_Clear(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0U; page < 8U; page++) {
        BSP_OLED_SetCursor(page, 0U);
        for (x = 0U; x < 128U; x++) {
            BSP_OLED_WriteData(0x00U);
        }
    }
}

void BSP_OLED_ShowChar(uint8_t line, uint8_t column, char ch)
{
    uint8_t i;
    uint8_t fontIndex;

    if ((line == 0U) || (line > BSP_OLED_MAX_LINE) ||
        (column == 0U) || (column > BSP_OLED_MAX_COLUMN)) {
        return;
    }

    if ((ch < ' ') || (ch > '~')) {
        ch = ' ';
    }

    fontIndex = (uint8_t) (ch - ' ');
    BSP_OLED_SetCursor((uint8_t) ((line - 1U) * 2U),
        (uint8_t) ((column - 1U) * BSP_OLED_CHAR_WIDTH));
    for (i = 0U; i < 8U; i++) {
        BSP_OLED_WriteData(OLED_F8x16[((uint16_t) fontIndex * BSP_OLED_FONT_BYTES_PER_CHAR) + i]);
    }

    BSP_OLED_SetCursor((uint8_t) (((line - 1U) * 2U) + 1U),
        (uint8_t) ((column - 1U) * BSP_OLED_CHAR_WIDTH));
    for (i = 0U; i < 8U; i++) {
        BSP_OLED_WriteData(OLED_F8x16[((uint16_t) fontIndex * BSP_OLED_FONT_BYTES_PER_CHAR) + i + 8U]);
    }
}

void BSP_OLED_ShowString(uint8_t line, uint8_t column, const char *str)
{
    uint8_t offset = 0U;

    if ((str == (const char *) 0) || (column == 0U) || (column > BSP_OLED_MAX_COLUMN)) {
        return;
    }

    while ((str[offset] != '\0') && ((uint8_t) (column + offset) <= BSP_OLED_MAX_COLUMN)) {
        BSP_OLED_ShowChar(line, (uint8_t) (column + offset), str[offset]);
        offset++;
    }
}

void BSP_OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        BSP_OLED_ShowChar(line, (uint8_t) (column + i),
            (char) ((number / BSP_OLED_Pow(10U, (uint32_t) (length - i - 1U))) % 10U + '0'));
    }
}

void BSP_OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length)
{
    uint8_t i;
    uint32_t absNumber;

    if (number >= 0) {
        BSP_OLED_ShowChar(line, column, '+');
        absNumber = (uint32_t) number;
    } else {
        BSP_OLED_ShowChar(line, column, '-');
        absNumber = (uint32_t) (-(number + 1)) + 1U;
    }

    for (i = 0U; i < length; i++) {
        BSP_OLED_ShowChar(line, (uint8_t) (column + i + 1U),
            (char) ((absNumber / BSP_OLED_Pow(10U, (uint32_t) (length - i - 1U))) % 10U + '0'));
    }
}

void BSP_OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i;
    uint8_t digit;

    for (i = 0U; i < length; i++) {
        digit = (uint8_t) ((number / BSP_OLED_Pow(16U, (uint32_t) (length - i - 1U))) % 16U);
        if (digit < 10U) {
            BSP_OLED_ShowChar(line, (uint8_t) (column + i), (char) (digit + '0'));
        } else {
            BSP_OLED_ShowChar(line, (uint8_t) (column + i), (char) (digit - 10U + 'A'));
        }
    }
}

void BSP_OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        BSP_OLED_ShowChar(line, (uint8_t) (column + i),
            (char) ((number / BSP_OLED_Pow(2U, (uint32_t) (length - i - 1U))) % 2U + '0'));
    }
}

void BSP_OLED_Init(void)
{
    delay_cycles(BSP_OLED_POWER_ON_DELAY_CYCLES);

    BSP_OLED_I2cInit();

    BSP_OLED_WriteCommand(0xAEU);
    BSP_OLED_WriteCommand(0xD5U);
    BSP_OLED_WriteCommand(0x80U);
    BSP_OLED_WriteCommand(0xA8U);
    BSP_OLED_WriteCommand(0x3FU);
    BSP_OLED_WriteCommand(0xD3U);
    BSP_OLED_WriteCommand(0x00U);
    BSP_OLED_WriteCommand(0x40U);
    BSP_OLED_WriteCommand(0xA1U);
    BSP_OLED_WriteCommand(0xC8U);
    BSP_OLED_WriteCommand(0xDAU);
    BSP_OLED_WriteCommand(0x12U);
    BSP_OLED_WriteCommand(0x81U);
    BSP_OLED_WriteCommand(0xCFU);
    BSP_OLED_WriteCommand(0xD9U);
    BSP_OLED_WriteCommand(0xF1U);
    BSP_OLED_WriteCommand(0xDBU);
    BSP_OLED_WriteCommand(0x30U);
    BSP_OLED_WriteCommand(0xA4U);
    BSP_OLED_WriteCommand(0xA6U);
    BSP_OLED_WriteCommand(0x8DU);
    BSP_OLED_WriteCommand(0x14U);
    BSP_OLED_WriteCommand(0xAFU);

    BSP_OLED_Clear();
}
