/**
 * @file disp.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_conf.h"
#include "lvgl/lvgl.h"
#include <string.h>

#include "tft.h"
#include "stm32f7xx_hal.h"

#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_sdram.h"

/*********************
 *      DEFINES
 *********************/


#define VSYNC               1
#define VBP                 1
#define VFP                 1
#define VACT                480
#define HSYNC               1
#define HBP                 1
#define HFP                 1
#define HACT                200

#define LAYER0_ADDRESS               (LCD_FB_START_ADDRESS)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/*For LittlevGL*/
static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);
static void my_mem_fill_cb(lv_disp_drv_t *disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
        const lv_area_t * fill_area, lv_color_t color);
static void my_mem_blend_cb(lv_disp_drv_t * disp_drv, lv_color_t * dest, const lv_color_t * src, uint32_t length, lv_opa_t opa);
void my_monitor_cb(lv_disp_drv_t * disp_drv, uint32_t time, uint32_t px);

/*DMA2D*/
static void DMA2D_Config(void);
static void DMA2D_TransferComplete(DMA2D_HandleTypeDef *han);

/*LCD*/
static void LCD_Config(void);
static void LTDC_Init(void);
static void LCD_LayerInit(uint16_t LayerIndex, uint32_t Address);

/*DSI*/
/* Request tear interrupt at specific scanline.*/
void LCD_ReqTear();

/* Configures display to update indicated region of the screen (200pixel wide chunks) - 16bpp mode */
void LCD_SetUpdateRegion(int idx);

/* Configures display to update left half of the screen- 24bpp mode*/
void LCD_SetUpdateRegionLeft();

/* Configures display to update right half of the screen - 24bpp mode*/
void LCD_SetUpdateRegionRight();

/*SD RAM*/
static void CopyBuffer(const uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize);

/**********************
 *  STATIC VARIABLES
 **********************/

extern LTDC_HandleTypeDef hltdc_discovery;
extern DMA2D_HandleTypeDef hdma2d_discovery;
extern DSI_HandleTypeDef hdsi_discovery;
DSI_VidCfgTypeDef hdsivideo_handle;
DSI_CmdCfgTypeDef CmdCfg;
DSI_LPCmdTypeDef LPCmd;
DSI_PLLInitTypeDef dsiPllInit;
static RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;


static uint32_t * my_fb = (uint32_t *)LAYER0_ADDRESS;
uint32_t i=0;
static volatile bool displayRefreshing = false;
static volatile bool refreshRequested = false;
static volatile int updateRegion = 0;

uint8_t pCols[4][4] =
{
    {0x00, 0x00, 0x00, 0xC7}, /*   0 -> 199 */
    {0x00, 0xC8, 0x01, 0x8F}, /* 200 -> 399 */
    {0x01, 0x90, 0x02, 0x57}, /* 400 -> 599 */
    {0x02, 0x58, 0x03, 0x1F}, /* 600 -> 799 */
};

uint8_t pColLeft[]    = {0x00, 0x00, 0x01, 0x8F}; /*   0 -> 399 */
uint8_t pColRight[]   = {0x01, 0x90, 0x03, 0x1F}; /* 400 -> 799 */

uint8_t pPage[] = { 0x00, 0x00, 0x01, 0xDF }; /*   0 -> 479 */

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
static lv_disp_buf_t disp_buf_1;
static lv_color_t buf1_1[LV_HOR_RES_MAX * 10];


/**
 * Initialize your display here
 */
void tft_init(void)
{
	BSP_SDRAM_Init();
	LCD_Config();
	/* Send Display On DCS Command to display */
	/*HAL_DSI_ShortWrite(&(hdsi_discovery),
			0,
			DSI_DCS_SHORT_PKT_WRITE_P1,
			OTM8009A_CMD_DISPON,
			0x00);*/
	LCD_ReqTear();
    lv_disp_buf_init(&disp_buf_1, buf1_1,NULL, LV_HOR_RES_MAX * 10);   /*Initialize the display buffer*/
		/*-----------------------------------
		* Register the display in LittlevGL
		*----------------------------------*/

	lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
	lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/
		/*Used to copy the buffer's content to the display*/
	disp_drv.flush_cb = tft_flush_cb;
	disp_drv.monitor_cb = my_monitor_cb;

		/*Set a display buffer*/
	disp_drv.buffer = &disp_buf_1;
	DMA2D_Config();

#if	LV_USE_GPU
    disp_drv.gpu_blend_cb = my_mem_blend_cb;
    disp_drv.gpu_fill_cb = my_mem_fill_cb;
	DMA2D_Config();
#endif
    lv_disp_t * disp;
    disp=lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static volatile int dma2d_done;

static void DMA2D_TransferComplete(DMA2D_HandleTypeDef *han)
{
	i++;
	dma2d_done = 1;
}


static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
	CopyBuffer((const uint32_t *)color_p, my_fb, area->x1, area->y1, lv_area_get_width(area), lv_area_get_height(area));

    refreshRequested = true ;

	lv_disp_flush_ready(drv);
}
void my_monitor_cb(lv_disp_drv_t * disp_drv, uint32_t time, uint32_t px)
{
  printf("%d px refreshed in %d ms\n",px,time);
}
#if LV_USE_GPU != 0
static void my_mem_blend_cb(lv_disp_drv_t *disp_drv, lv_color_t * dest, const lv_color_t * src, uint32_t length, lv_opa_t opa)
{
    hdma2d_discovery.Init.Mode         = DMA2D_M2M_BLEND;
#if LV_COLOR_DEPTH == 8
    hdma2d_discovery.Init.ColorMode = DMA2D_INPUT_A8;
#elif LV_COLOR_DEPTH == 16
    hdma2d_discovery.Init.ColorMode = DMA2D_INPUT_RGB565;
#elif LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32
    hdma2d_discovery.Init.ColorMode = DMA2D_INPUT_ARGB8888;
#endif
    hdma2d_discovery.LayerCfg[1].InputAlpha = 0xff;
#if LV_COLOR_DEPTH == 8
    hdma2d_discovery.LayerCfg[1].InputColorMode = DMA2D_INPUT_A8;
#elif LV_COLOR_DEPTH == 16
    hdma2d_discovery.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
#elif LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32
    hdma2d_discovery.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
#endif
    hdma2d_discovery.LayerCfg[1].InputOffset = 0x0;
	hdma2d_discovery.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
	hdma2d_discovery.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;

	//   Background Configuration
	hdma2d_discovery.LayerCfg[0].AlphaMode = DMA2D_REGULAR_ALPHA;
	hdma2d_discovery.LayerCfg[0].InputAlpha = 0x0;
#if LV_COLOR_DEPTH == 8
    hdma2d_discovery.LayerCfg[0].InputColorMode = DMA2D_INPUT_A8;
#elif LV_COLOR_DEPTH == 16
    hdma2d_discovery.LayerCfg[0].InputColorMode = DMA2D_INPUT_RGB565;
#elif LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32
    hdma2d_discovery.LayerCfg[0].InputColorMode = DMA2D_INPUT_ARGB8888;
#endif
	hdma2d_discovery.LayerCfg[0].InputOffset = 0x0;

	hdma2d_discovery.XferCpltCallback  = DMA2D_TransferComplete;
	hdma2d_discovery.Instance          = DMA2D;

	HAL_DMA2D_ConfigLayer(&hdma2d_discovery, 0);
	HAL_DMA2D_ConfigLayer(&hdma2d_discovery, 1);

	 if(HAL_DMA2D_Init(&hdma2d_discovery) == HAL_OK)
		{
			if(HAL_DMA2D_ConfigLayer(&hdma2d_discovery, 1) == HAL_OK)
			{
			    refreshRequested = true ;
				dma2d_done = 0;
				if (HAL_DMA2D_BlendingStart_IT(&hdma2d_discovery, (uint32_t) src, (uint32_t) dest, (uint32_t)dest, length, 1)== HAL_OK)
				{
					while(dma2d_done == 0);
				}
			}
		}

}
static void my_mem_fill_cb(lv_disp_drv_t *disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
    const lv_area_t * fill_area, lv_color_t color)
{
	/*Wait for the previous operation*/
//	HAL_DMA2D_PollForTransfer(&hdma2d_discovery, 10);
	hdma2d_discovery.Init.Mode         = DMA2D_R2M;
#if LV_COLOR_DEPTH == 8
hdma2d_discovery.Init.ColorMode = DMA2D_INPUT_A8;
#elif LV_COLOR_DEPTH == 16
	hdma2d_discovery.Init.ColorMode = DMA2D_INPUT_RGB565;
#elif LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32
	hdma2d_discovery.Init.ColorMode = DMA2D_INPUT_ARGB8888;
#endif
	hdma2d_discovery.LayerCfg[1].InputAlpha = 0xff;
#if LV_COLOR_DEPTH == 8
	hdma2d_discovery.LayerCfg[1].InputColorMode = DMA2D_INPUT_A8;
#elif LV_COLOR_DEPTH == 16
	hdma2d_discovery.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
#elif LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32
	hdma2d_discovery.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
#endif
	hdma2d_discovery.XferCpltCallback  = DMA2D_TransferComplete;
	hdma2d_discovery.Instance          = DMA2D;

	HAL_DMA2D_ConfigLayer(&hdma2d_discovery, 1);

    lv_color_t * dest_buf_ofs = dest_buf;
    dest_buf_ofs += dest_width * fill_area->y1;
	dest_buf_ofs += fill_area->x1;
	lv_coord_t area_w = lv_area_get_width(fill_area);

	uint32_t i;
	for(i = fill_area->y1; i <= fill_area->y2; i++) {
		if(HAL_DMA2D_Init(&hdma2d_discovery) == HAL_OK)
		   {
			if(HAL_DMA2D_ConfigLayer(&hdma2d_discovery, 1) == HAL_OK)
				{
				    refreshRequested = true ;
					dma2d_done = 0;
				if (HAL_DMA2D_BlendingStart_IT(&hdma2d_discovery, (uint32_t) lv_color_to32(color), (uint32_t) dest_buf_ofs, (uint32_t)dest_buf_ofs, area_w, 1) == HAL_OK)
				{
					dest_buf_ofs += dest_width;
		     		while(dma2d_done == 0);
			    }
		   }
	   }
	}
}
#endif

static void LCD_LayerInit(uint16_t LayerIndex, uint32_t Address)
{
    LTDC_LayerCfgTypeDef Layercfg;
    /* Layer Init */
    Layercfg.WindowX0 = 0;
    Layercfg.WindowY0 = 0;
    Layercfg.WindowY1 = 480;
    Layercfg.FBStartAdress = Address;
    Layercfg.Alpha = 255;
    Layercfg.Alpha0 = 0;
    Layercfg.Backcolor.Blue = 0;
    Layercfg.Backcolor.Green = 0;
    Layercfg.Backcolor.Red = 0;
    Layercfg.ImageHeight = 480;

#if LV_COLOR_DEPTH == 16
    Layercfg.WindowX1 = 800 / 4; //Note: Div4 due to screen being divided into 4 areas.
    Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    Layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    Layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    Layercfg.ImageWidth = 800 / 4; //Note: Div4 due to screen being divided into 4 areas.
#elif USE_BPP==24
    Layercfg.WindowX1 = LCD_GetXSize() / 2; //Note: Div2 due to screen being divided into 2 areas.
    Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
    Layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    Layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    Layercfg.ImageWidth = LCD_GetXSize() / 2; //Note: Div2 due to screen being divided into 2 areas.

#endif

    HAL_LTDC_ConfigLayer(&hltdc_discovery, &Layercfg, LayerIndex);
}
static void LCD_Config(void)
{
	DSI_PHY_TimerTypeDef  PhyTimings;
	GPIO_InitTypeDef GPIO_Init_Structure;

	    /* Configure DSI TE pin on GPIOJ2 */
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	GPIO_Init_Structure.Pin       = GPIO_PIN_2;
	GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
	GPIO_Init_Structure.Pull      = GPIO_NOPULL;
	GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
	GPIO_Init_Structure.Alternate = GPIO_AF13_DSI;
	HAL_GPIO_Init(GPIOJ, &GPIO_Init_Structure);
	/* Toggle Hardware Reset of the DSI LCD using
	 * its XRES signal (active low) */
	BSP_LCD_Reset();

	/* Call first MSP Initialize only in case of first initialization
	 * This will set IP blocks LTDC, DSI and DMA2D
	 * - out of reset
	 * - clocked
	 * - NVIC IRQ related to IP blocks enabled
	 */
	BSP_LCD_MspInit();

	/* LCD clock configuration */
	/* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
	/* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 417 Mhz */
	/* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 417 MHz / 5 = 83.4 MHz */
	/* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 83.4 / 2 = 41.7 MHz */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 417;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/* Base address of DSI Host/Wrapper registers to be set before calling De-Init */
	hdsi_discovery.Instance = DSI;

	HAL_DSI_DeInit(&(hdsi_discovery));

	dsiPllInit.PLLNDIV  = 100;
	dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV5;
	dsiPllInit.PLLODF   = DSI_PLL_OUT_DIV1;

	hdsi_discovery.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
	hdsi_discovery.Init.TXEscapeCkdiv = 0x4;
	HAL_DSI_Init(&(hdsi_discovery), &(dsiPllInit));

	/* Configure the DSI for Command mode */
	CmdCfg.VirtualChannelID      = 0;
	CmdCfg.HSPolarity            = DSI_HSYNC_ACTIVE_HIGH;
	CmdCfg.VSPolarity            = DSI_VSYNC_ACTIVE_HIGH;
	CmdCfg.DEPolarity            = DSI_DATA_ENABLE_ACTIVE_HIGH;
	CmdCfg.ColorCoding           = DSI_RGB565;
	CmdCfg.CommandSize           = HACT;
	CmdCfg.TearingEffectSource   = DSI_TE_EXTERNAL;
	CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
	CmdCfg.VSyncPol              = DSI_VSYNC_FALLING;
	CmdCfg.AutomaticRefresh      = DSI_AR_DISABLE;
	CmdCfg.TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE;
	HAL_DSI_ConfigAdaptedCommandMode(&hdsi_discovery, &CmdCfg);

	LPCmd.LPGenShortWriteNoP    = DSI_LP_GSW0P_ENABLE;
	LPCmd.LPGenShortWriteOneP   = DSI_LP_GSW1P_ENABLE;
	LPCmd.LPGenShortWriteTwoP   = DSI_LP_GSW2P_ENABLE;
	LPCmd.LPGenShortReadNoP     = DSI_LP_GSR0P_ENABLE;
	LPCmd.LPGenShortReadOneP    = DSI_LP_GSR1P_ENABLE;
	LPCmd.LPGenShortReadTwoP    = DSI_LP_GSR2P_ENABLE;
	LPCmd.LPGenLongWrite        = DSI_LP_GLW_ENABLE;
	LPCmd.LPDcsShortWriteNoP    = DSI_LP_DSW0P_ENABLE;
	LPCmd.LPDcsShortWriteOneP   = DSI_LP_DSW1P_ENABLE;
	LPCmd.LPDcsShortReadNoP     = DSI_LP_DSR0P_ENABLE;
	LPCmd.LPDcsLongWrite        = DSI_LP_DLW_ENABLE;
	HAL_DSI_ConfigCommand(&hdsi_discovery, &LPCmd);

	/* Configure DSI PHY HS2LP and LP2HS timings */
		PhyTimings.ClockLaneHS2LPTime = 35;
		PhyTimings.ClockLaneLP2HSTime = 35;
		PhyTimings.DataLaneHS2LPTime = 35;
		PhyTimings.DataLaneLP2HSTime = 35;
		PhyTimings.DataLaneMaxReadTime = 0;
		PhyTimings.StopWaitTime = 10;
		HAL_DSI_ConfigPhyTimer(&hdsi_discovery, &PhyTimings);

	/* Initialize LTDC */
	LTDC_Init();
	__HAL_LTDC_DISABLE(&hltdc_discovery);

	/* Start DSI */
	HAL_DSI_Start(&(hdsi_discovery));



	/* Initialize the OTM8009A LCD Display IC Driver (KoD LCD IC Driver)
	 *  depending on configuration set in 'hdsivideo_handle'.
	 */
    OTM8009A_Init(OTM8009A_FORMAT_RBG565, 1);

	LPCmd.LPGenShortWriteNoP    = DSI_LP_GSW0P_DISABLE;
	LPCmd.LPGenShortWriteOneP   = DSI_LP_GSW1P_DISABLE;
	LPCmd.LPGenShortWriteTwoP   = DSI_LP_GSW2P_DISABLE;
	LPCmd.LPGenShortReadNoP     = DSI_LP_GSR0P_DISABLE;
	LPCmd.LPGenShortReadOneP    = DSI_LP_GSR1P_DISABLE;
	LPCmd.LPGenShortReadTwoP    = DSI_LP_GSR2P_DISABLE;
	LPCmd.LPGenLongWrite        = DSI_LP_GLW_DISABLE;
	LPCmd.LPDcsShortWriteNoP    = DSI_LP_DSW0P_DISABLE;
	LPCmd.LPDcsShortWriteOneP   = DSI_LP_DSW1P_DISABLE;
	LPCmd.LPDcsShortReadNoP     = DSI_LP_DSR0P_DISABLE;
	LPCmd.LPDcsLongWrite        = DSI_LP_DLW_DISABLE;
	HAL_DSI_ConfigCommand(&hdsi_discovery, &LPCmd);

	//HAL_DSI_ConfigFlowControl(&hdsi_discovery, DSI_FLOW_CONTROL_BTA);

	/* Send Display Off DCS Command to display */
	/*HAL_DSI_ShortWrite(&(hdsi_discovery),
			0,
			DSI_DCS_SHORT_PKT_WRITE_P1,
			OTM8009A_CMD_DISPOFF,
			0x00);*/

	LCD_LayerInit(0, my_fb);
    HAL_LTDC_SetPitch(&hltdc_discovery,800, 0);
    __HAL_LTDC_ENABLE(&hltdc_discovery);
}
static void DMA2D_Config(void)
{
	HAL_NVIC_SetPriority(DMA2D_IRQn, 5, 5);
    HAL_NVIC_EnableIRQ(DMA2D_IRQn);
    HAL_NVIC_SetPriority(DSI_IRQn,5 , 5);
    HAL_NVIC_EnableIRQ(DSI_IRQn);
}
static void LTDC_Init(void)
{
	/* DeInit */
	HAL_LTDC_DeInit(&hltdc_discovery);

	/* LTDC Config */
	/* Timing and polarity */
	hltdc_discovery.Init.HorizontalSync = HSYNC;
	hltdc_discovery.Init.VerticalSync = VSYNC;
	hltdc_discovery.Init.AccumulatedHBP = HSYNC+HBP;
	hltdc_discovery.Init.AccumulatedVBP = VSYNC+VBP;
	hltdc_discovery.Init.AccumulatedActiveH = VSYNC+VBP+VACT;
	hltdc_discovery.Init.AccumulatedActiveW = HSYNC+HBP+HACT;
	hltdc_discovery.Init.TotalHeigh = VSYNC+VBP+VACT+VFP;
	hltdc_discovery.Init.TotalWidth = HSYNC+HBP+HACT+HFP;


	/* background value */
	hltdc_discovery.Init.Backcolor.Blue = 0;
	hltdc_discovery.Init.Backcolor.Green = 0;
	hltdc_discovery.Init.Backcolor.Red = 0;

	/* Polarity */
	hltdc_discovery.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc_discovery.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc_discovery.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc_discovery.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc_discovery.Instance = LTDC;

	HAL_LTDC_Init(&hltdc_discovery);
}


static void CopyBuffer(const uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize)
{
	/**********************
	 * Using CPU
	 **********************/

/*	uint32_t row;
		uint16_t *src16, *dst16;
		src16 = (uint16_t*)pSrc;
		dst16 = (uint16_t*)pDst;
		for(row = y; row < y + ysize; row++) {
			memcpy(&dst16[row * 800 + x], src16, xsize * 2);
			src16 += xsize;
	}*/

	/**********************
	 * Using DMA2D
	 **********************/
#if 1
	uint32_t destination = (uint32_t)pDst + (y * 800 + x) * 2;
	uint32_t source      = (uint32_t)pSrc;

	/*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
	hdma2d_discovery.Init.Mode         = DMA2D_M2M;
	hdma2d_discovery.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
	hdma2d_discovery.Init.OutputOffset = 800 - xsize;
	hdma2d_discovery.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
	hdma2d_discovery.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */

	/*##-2- DMA2D Callbacks Configuration ######################################*/
	hdma2d_discovery.XferCpltCallback  = DMA2D_TransferComplete;

	/*##-3- Foreground Configuration ###########################################*/
	hdma2d_discovery.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d_discovery.LayerCfg[1].InputAlpha = 0xFF;
	hdma2d_discovery.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d_discovery.LayerCfg[1].InputOffset = 0;
	hdma2d_discovery.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
	hdma2d_discovery.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

	hdma2d_discovery.Instance          = DMA2D;
	/* DMA2D Initialization */
   if(HAL_DMA2D_Init(&hdma2d_discovery) == HAL_OK)
	{
		if(HAL_DMA2D_ConfigLayer(&hdma2d_discovery, 1) == HAL_OK)
		{
			dma2d_done = 0;
			if (HAL_DMA2D_Start_IT(&hdma2d_discovery, source, destination, xsize, ysize) == HAL_OK)
			{
				while(dma2d_done == 0);
			}
		}
	}
#endif
}

void LCD_ReqTear(void)
   {
       uint8_t ScanLineParams[2];
       uint16_t scanline = 533;

       ScanLineParams[0] = scanline >> 8;
       ScanLineParams[1] = scanline & 0x00FF;

       HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 2, OTM8009A_CMD_WRTESCN, ScanLineParams);
       HAL_DSI_ShortWrite(&hdsi_discovery, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_TEEON, OTM8009A_TEEON_TELOM_VBLANKING_INFO_ONLY);
   }

void LCD_SetUpdateRegion(int idx)
    {
        HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[idx]);
    }

void LCD_SetUpdateRegionLeft()
   {
       HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pColLeft);
   }

void LCD_SetUpdateRegionRight()
   {
       HAL_DSI_LongWrite(&hdsi_discovery, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pColRight);
   }

void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef* hdsi_discovery)
{
	    if (refreshRequested && !displayRefreshing)
	    {
	        // Update region 0 = first area of display (First quarter for 16bpp, first half for 24bpp)
	        updateRegion = 0;

	        //Set update region based on bit depth of framebuffer. 16pp or 24bpp.
	        if (LV_COLOR_DEPTH == 32)
	        {
	            LCD_SetUpdateRegionLeft();
	        }
	        //Default to 16 bpp
	        else
	        {
	            LCD_SetUpdateRegion(updateRegion);
	        }

	        // Transfer a quarter screen of pixel data.
	        HAL_DSI_Refresh(hdsi_discovery);
	        displayRefreshing = true;
	    }
	    else
	    {

	    }
}
void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef* hdsi_discovery)
{
    if (displayRefreshing)
    {
        if (LV_COLOR_DEPTH == 32)
        {
            if (updateRegion == 0)
            {
                // If we transferred the left half, also transfer right half.
                __HAL_DSI_WRAPPER_DISABLE(hdsi_discovery);
               LTDC_LAYER(&hltdc_discovery, 0)->CFBAR = ((uint32_t)my_fb) + (800 / 2) * 3  ;  // ajouter la moitié de buffer
               __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
                LCD_SetUpdateRegionRight(); //Set display column to 400-799
                __HAL_DSI_WRAPPER_ENABLE(hdsi_discovery);
                updateRegion = 1;
                HAL_DSI_Refresh(hdsi_discovery);
            }
            else
            {
                // Otherwise we are done refreshing.
                __HAL_DSI_WRAPPER_DISABLE(hdsi_discovery);
                LTDC_LAYER(&hltdc_discovery, 0)->CFBAR = (uint32_t)my_fb; // buffer complet
                __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
                LCD_SetUpdateRegionLeft(); //Set display column to 0-399
                __HAL_DSI_WRAPPER_ENABLE(hdsi_discovery);
                displayRefreshing = false;
            }
        }
        else   //Default to 16bpp
        {
            updateRegion++;
            if (updateRegion < 4)  // 0 ou 1 ou 2 ou 3
            {
                __HAL_DSI_WRAPPER_DISABLE(hdsi_discovery);
                LTDC_LAYER(&hltdc_discovery, 0)->CFBAR = (uint32_t)my_fb + (updateRegion * 800) /2  ;
                __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);

                LCD_SetUpdateRegion(updateRegion);
                __HAL_DSI_WRAPPER_ENABLE(hdsi_discovery);
                HAL_DSI_Refresh(hdsi_discovery);
            }
            else   // =4
            {
                __HAL_DSI_WRAPPER_DISABLE(hdsi_discovery);
                LTDC_LAYER(&hltdc_discovery, 0)->CFBAR = (uint32_t)my_fb;  //buffer complet
                __HAL_LTDC_RELOAD_CONFIG(&hltdc_discovery);
                LCD_SetUpdateRegion(0);
                __HAL_DSI_WRAPPER_ENABLE(hdsi_discovery);

                displayRefreshing = false;

            }
        }
    }
}

