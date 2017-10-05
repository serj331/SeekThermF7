#include "stm32f7xx_hal.h"

extern SDRAM_HandleTypeDef hsdram1;

void SDRAM_initSequence() {
	FMC_SDRAM_CommandTypeDef sdramCmd;

	sdramCmd.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;

	sdramCmd.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
	HAL_SDRAM_SendCommand(&hsdram1, &sdramCmd, 1);

	HAL_Delay(1);

	sdramCmd.CommandMode = FMC_SDRAM_CMD_PALL;
	HAL_SDRAM_SendCommand(&hsdram1, &sdramCmd, 1);

	sdramCmd.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	sdramCmd.AutoRefreshNumber = 8;
	HAL_SDRAM_SendCommand(&hsdram1, &sdramCmd, 1);

	sdramCmd.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	sdramCmd.ModeRegisterDefinition = 0 << 0 /* burst length:1 */| \
									  0 << 3 /* burst type: sequential */| \
									  2 << 4 /* latency mode: 2 */| \
									  0 << 7 /* op. mode: standart */| \
									  1 << 9 /* write burst mode: single location access */;
	HAL_SDRAM_SendCommand(&hsdram1, &sdramCmd, 1);

	//((64ms / 4096) * 108 MHz) - 20
	HAL_SDRAM_ProgramRefreshRate(&hsdram1, 1668);
}
