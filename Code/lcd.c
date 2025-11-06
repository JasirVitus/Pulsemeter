#include "lcd/lcd.h"

void DelayMs(uint32_t nTime);
void LCD_LowLevel_Init();
void LCD_WriteCommand(uint8_t n);
void LCD_WriteData(uint8_t n);

void DelayMs(uint32_t ms)
{
	int i, tms;
	tms = 3000*ms;
	for(i=0;i<tms;i++);
}

void LCD_Init(void)
{
	uint8_t ctrl, i;

	ctrl = LCD_PFX_FunctionSet | 15;

	LCD_LowLevel_Init();

	LCD_EN(0);
	LCD_RS(0);

	DelayMs(15);

	for (i = 0; i < 3; i++)
	{
		LCD_SendNibble(3);
		DelayMs(5);
	}

	LCD_SendNibble(2);
	DelayMs(5);

	LCD_SendByte(0, ctrl);

	ctrl = LCD_PFX_DisplayOnOff |
		   LCD_DisplayState_On |
		   LCD_CursorState_Off |
		   LCD_BlinkCursor_Off;

	LCD_SendByte(0, ctrl);

	ctrl = LCD_CMD_ClearDisplay;

	LCD_SendByte(0, ctrl);

	ctrl = LCD_PFX_EntryModeSet |
		   LCD_CursorDirection_Increment |
		   LCD_DisplayShift_NoShift;

	LCD_SendByte(0, ctrl);

}

void LCD_Clear(void)
{
	LCD_SendByte(0, LCD_CMD_ClearDisplay);
	LCD_SendByte(0, LCD_CMD_CursorHome);
}

void LCD_SendNibble(uint8_t n)
{
	GPIO_WriteBit(LCD_CTRL_D4_GPIO_PORT, LCD_CTRL_D4_PIN, GET_BIT(n, 0));
	GPIO_WriteBit(LCD_CTRL_D5_GPIO_PORT, LCD_CTRL_D5_PIN, GET_BIT(n, 1));
	GPIO_WriteBit(LCD_CTRL_D6_GPIO_PORT, LCD_CTRL_D6_PIN, GET_BIT(n, 2));
	GPIO_WriteBit(LCD_CTRL_D7_GPIO_PORT, LCD_CTRL_D7_PIN, GET_BIT(n, 3));

	DelayMs(1);
	LCD_EN_HIGH();
	DelayMs(1);
	LCD_EN_LOW();
}

void LCD_SendByte(uint8_t address, uint8_t n)
{
	LCD_RS(address);
	DelayMs(1);
	LCD_SendNibble(n >> 4);
	LCD_SendNibble(n & 0xf);
}

void LCD_SetDdramAddress(uint8_t address)
{
	LCD_SendByte(0, address | LCD_PFX_DdramSetAddress);
}

void LCD_SetCgramAddress(uint8_t address)
{
	LCD_SendByte(0, address | LCD_PFX_CgramSetAddress);
}

void LCD_Goto(uint8_t cx, uint8_t cy)
{
	uint8_t address;
	if (cy != 1)
		address = LCD_LINE_TWO_ADDR;
	else
		address = 0;

	address += cx - 1;

	LCD_SetDdramAddress(address);
}
void LCD_WriteCustomCharacter(uint8_t num, const uint8_t c[])
{
	uint8_t i;
	if(num < 7)
	{
		LCD_SendByte(0, 0x40 | (num << 3));
		for(i = 0; i < 8; i++)
			LCD_SendByte(1, c[i]);
		LCD_SetDdramAddress(0);
	}
}

void LCD_Putc(char c)
{
	switch (c) {
		case '\a':
			LCD_Goto(1, 1);
			break;
		case '\f':
			LCD_SendByte(0, LCD_CMD_ClearDisplay);
			DelayMs(2);
			break;
		case '\n':
			LCD_Goto(1, 2);
			break;
		case '\b':
			LCD_SendByte(0, 0x10);
			break;
		default:
			LCD_SendByte(1, c);
			break;
	}
}

void LCD_Puts(const char *s)
{
	const char *p = s;
	while(*p != 0)
	{
		LCD_Putc(*p);
		p++;
	}
}

void LCD_PutSignedInt(int32_t n)
{
	char c1[32];

	if(n == 0)
	{
		LCD_Putc('0');
		return;
	}

	signed int value = n;
	unsigned int absolute;

	int i = 0;

	if(value < 0) {
		absolute = -value;
	} else {
		absolute = value;
	}

	while (absolute > 0) {
		c1[i] = '0' + absolute % 10;
		absolute /= 10;
		i++;
	}

	LCD_Putc((value < 0) ? '-' : '+');

	i--;

	while(i >= 0){
		LCD_Putc(c1[i--]);
	}
}

void LCD_PutUnsignedInt(uint32_t n)
{
	char c1[32];
	uint32_t value = n;
	uint32_t i = 0;

	if(n == 0)
	{
		LCD_Putc('0');
		return;
	}

	while (value > 0) {
		c1[i] = '0' + value % 10;
		value /= 10;
		i++;
	}

	while(i-- > 0){
		LCD_Putc(c1[i]);
	}
}

void LCD_LowLevel_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(LCD_CTRL_EN_GPIO_CLK | LCD_CTRL_RS_GPIO_CLK |
			LCD_CTRL_D4_GPIO_CLK | LCD_CTRL_D5_GPIO_CLK |
			LCD_CTRL_D6_GPIO_CLK | LCD_CTRL_D7_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/*  Enable pini ayar�  */
	GPIO_InitStructure.GPIO_Pin = LCD_CTRL_EN_PIN;
	GPIO_Init(LCD_CTRL_EN_GPIO_PORT, &GPIO_InitStructure);

	/*  RS pini ayar�  */
	GPIO_InitStructure.GPIO_Pin = LCD_CTRL_RS_PIN;
	GPIO_Init(LCD_CTRL_RS_GPIO_PORT, &GPIO_InitStructure);

	/*  D4 pini ayar�  */
	GPIO_InitStructure.GPIO_Pin = LCD_CTRL_D4_PIN;
	GPIO_Init(LCD_CTRL_D4_GPIO_PORT, &GPIO_InitStructure);

	/*  D5 pini ayar�  */
	GPIO_InitStructure.GPIO_Pin = LCD_CTRL_D5_PIN;
	GPIO_Init(LCD_CTRL_D5_GPIO_PORT, &GPIO_InitStructure);

	/*  D6 pini ayar�  */
	GPIO_InitStructure.GPIO_Pin = LCD_CTRL_D6_PIN;
	GPIO_Init(LCD_CTRL_D6_GPIO_PORT, &GPIO_InitStructure);

	/*  D7 pini ayar�  */
	GPIO_InitStructure.GPIO_Pin = LCD_CTRL_D7_PIN;
	GPIO_Init(LCD_CTRL_D7_GPIO_PORT, &GPIO_InitStructure);

	/*  Enable pinini pasif et  */
	GPIO_SetBits(LCD_CTRL_EN_GPIO_PORT, LCD_CTRL_EN_PIN);
}

void LCD_EntryModeCommand(LCD_EntryModeCmdTypeDef *LCD_EntryModeCmdStruct)
{
	uint8_t ctrl;
	ctrl = LCD_PFX_EntryModeSet |
		   LCD_EntryModeCmdStruct->CursorDirection |
		   LCD_EntryModeCmdStruct->DisplayShift;
	LCD_SendByte(0, ctrl);
}

void LCD_DisplayOnOffCommand(LCD_DisplayOnOffCmdTypedef *LCD_DisplayOnOffStruct)
{
	uint8_t ctrl;
	ctrl = LCD_PFX_DisplayOnOff |
		   LCD_DisplayOnOffStruct->BlinkCursor |
		   LCD_DisplayOnOffStruct->CursorState |
		   LCD_DisplayOnOffStruct->DisplayState;
	LCD_SendByte(0, ctrl);
}

void LCD_CursorDisplayShiftCommand(
		LCD_CursorDisplayShiftCmdTypeDef *LCD_CursorDisplayShiftStruct)
{
	uint8_t ctrl;
	ctrl = LCD_PFX_CursorDisplayShift |
		   LCD_CursorDisplayShiftStruct->MoveOrShift |
		   LCD_CursorDisplayShiftStruct->ShiftDirection;
	LCD_SendByte(0, ctrl);
}
