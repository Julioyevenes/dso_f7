/**
  ******************************************************************************
  * @file    ${file_name} 
  * @author  ${user}
  * @version 
  * @date    ${date}
  * @brief   
  ******************************************************************************
  * @attention
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dso.h"
#include "main.h"

/* Private types ------------------------------------------------------------*/
typedef enum _DSO_STATE
{
	DSO_TRIGGER,
	DSO_SAMPLE,
	DSO_MATH,
	DSO_DRAW,
	DSO_DISPLAY
} DSO_STATE;

typedef enum _DSO_DRAW_STATE
{
	DSO_STATE_SET,
	DSO_STATE_DRAW,
	DSO_STATE_VERLINE,
	DSO_STATE_HORLINE,
	DSO_STATE_BUTTONS
} DSO_DRAW_STATE;

typedef enum _BUFFER_STATE
{
	BUFFER_OFFSET_NONE = 0,
	BUFFER_OFFSET_HALF,
	BUFFER_OFFSET_FULL,
} BUFFER_STATE;

typedef struct _BufferTypeDef
{
	WORD 		*rptr;
	WORD 		*wptr;
	WORD 		len;
	WORD 		pos;

	BUFFER_STATE buffstate;
} BufferTypeDef;

typedef struct _DSO_ADCTypeDef
{
	uint32_t Channel;
	uint32_t Rank;
	uint32_t SamplingTime;
	uint32_t Offset;
} DSO_ADCTypeDef;

typedef struct _DSO_ReScaleTypeDef
{
	int16_t	x1;
	int16_t	x2;
	int16_t	y1;
	int16_t	y2;
} DSO_ReScaleTypeDef;

typedef struct _DSO_TriggerTypeDef
{
	uint16_t	nSamples;
	uint16_t	vThreshold;
} DSO_TriggerTypeDef;

typedef struct _DSO_HandleTypeDef
{
	DSO_STATE 			DsoState;
	DSO_DRAW_STATE 		DsoDrawState;
	BufferTypeDef 		DsoBuffer;
	DSO_ADCTypeDef  	DsoADC;
	DSO_ReScaleTypeDef	DsoReScale;
	DSO_TriggerTypeDef	DsoTrigger;
} DSO_HandleTypeDef;

typedef struct _DSO_ButtonTypeDef
{
	BUTTON  *pBtn;
	WORD	BtnState;
	XCHAR	*BtnString;
	BOOL	BtnSetClr;
} DSO_ButtonTypeDef;

/* Private constants --------------------------------------------------------*/
#define GR_CLR_GRID                 LIGHTGRAY
#define GR_CLR_BACKGROUND           BLACK
#define GR_CLR_POINTS               BRIGHTGREEN

#define DSO_DISPLAY_DELAY           1

#define LCD_LAYER0_ADD				((uint32_t)0xC0000000)
#define LCD_LAYER1_ADD				((uint32_t)0xC0500000)
#define LCD_RENDER_ADD				((uint32_t)0xC0200000)

#define DSO_ADC_BITDEPTH			12
#define DSO_ADC_MAXVALUE 			(1 << DSO_ADC_BITDEPTH)

#define DSO_RESCALER_STEP			((GR_BOTTOM - GR_TOP) >> 4)

#define DSO_TRIGGER_HOR_STEP		(DSO_BUFFER_SIZE >> 8)
#define DSO_TRIGGER_VER_STEP		(DSO_ADC_MAXVALUE >> 4)

#define ID_BUTTON_A					101
#define ID_BUTTON_B					102
#define ID_BUTTON_C					103
#define ID_BUTTON_D					104

#define NUM_CTRL_BUTTONS 			4

const XCHAR                 		IncTimeStr[] = {'T','i','m','e',' ','+',0};
const XCHAR                 		DecTimeStr[] = {'T','i','m','e',' ','-',0};
const XCHAR                 		IncAmpStr[] = {'A','m','p','l',' ','+',0};
const XCHAR                 		DecAmpStr[] = {'A','m','p','l',' ','-',0};

/* Private macro ------------------------------------------------------------*/
#define WAIT_UNTIL_FINISH(x)    	while(!x)
	
/* Private variables --------------------------------------------------------*/
GOL_SCHEME              			*dsoScheme;

DSO_HandleTypeDef 					hdso = {DSO_TRIGGER, DSO_STATE_SET, 0, 0, 0, 0};

WORD 								adcBuffer[DSO_BUFFER_SIZE];
WORD 								dsoBuffer[DSO_BUFFER_SIZE];

uint32_t							aSamplingTime[] = {	ADC_SAMPLETIME_3CYCLES,
														ADC_SAMPLETIME_15CYCLES,
														ADC_SAMPLETIME_28CYCLES,
														ADC_SAMPLETIME_56CYCLES,
														ADC_SAMPLETIME_84CYCLES,
														ADC_SAMPLETIME_112CYCLES,
														ADC_SAMPLETIME_144CYCLES,
														ADC_SAMPLETIME_480CYCLES};

uint8_t								iSamplingTime = 0;

DSO_ButtonTypeDef					hbutton[NUM_CTRL_BUTTONS];

/* Private function prototypes ----------------------------------------------*/
void DSO_Process(void);
void DSO_ReScaleBuffer(WORD *Src, WORD *Dst, WORD len, SHORT x1, SHORT x2, SHORT y1, SHORT y2);
void DSO_Plot(WORD *data, WORD len, WORD pos);
WORD DSO_CreateCtrlButtons(XCHAR *pTextA, XCHAR *pTextB, XCHAR *pTextC, XCHAR *pTextD);
void DSO_SetCtrlButtons(DSO_ButtonTypeDef *hb);
void DSO_LCDLayerPutPixel(DWORD dst, DWORD color, WORD x, WORD y, WORD ImageWidth);
void DSO_LCDClear(DWORD dst, DWORD color, WORD ImageWidth, WORD ImageHeight);
void ADC_ChannelConfig(uint32_t Channel, uint32_t Rank, uint32_t SamplingTime, uint32_t Offset);
void DMA2D_Init(uint32_t ImageWidth, uint32_t ImageHeight);
void DMA2D_CopyBuffer(uint32_t *pSrc, uint32_t *pDst, uint16_t xPos, uint16_t yPos, uint16_t ImageWidth, uint16_t ImageHeight);

/**
  * @brief  
  * @param  
  * @retval 
  */
WORD DSO_Create(void)
{
    static SHORT    		pos;

    switch(hdso.DsoDrawState)
    {
        case DSO_STATE_SET:
		
			// Free memory for the objects in the previous linked list and start new list to display
			GOLFree();

			// initialize the dso buffer handler
			hdso.DsoBuffer.rptr = (WORD *) &adcBuffer;
			hdso.DsoBuffer.wptr = (WORD *) &dsoBuffer;
			hdso.DsoBuffer.len = DSO_BUFFER_SIZE;
			hdso.DsoBuffer.pos = 0;
			hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;

			// initialize the dso adc handler
			hdso.DsoADC.Channel = ADC_CHANNEL_6;
			hdso.DsoADC.Rank = 1;
			hdso.DsoADC.SamplingTime = ADC_SAMPLETIME_3CYCLES;
			hdso.DsoADC.Offset = 0;

			// initialize the dso rescaler handler
			hdso.DsoReScale.x1 = 0;
			hdso.DsoReScale.x2 = DSO_ADC_MAXVALUE;
			hdso.DsoReScale.y1 = GR_BOTTOM;
			hdso.DsoReScale.y2 = GR_TOP;

			// initialize the dso trigger handler
			hdso.DsoTrigger.nSamples = DSO_TRIGGER_HOR_STEP;
			hdso.DsoTrigger.vThreshold = DSO_TRIGGER_VER_STEP;

			// initialize the screen
			DMA2D_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

			SetColor(GR_CLR_BACKGROUND);
			ClearDevice();		

            // set parameters for panel
            GOLPanelDraw(	DSO_PANEL_LEFT,
            				DSO_PANEL_TOP,
							DSO_PANEL_RIGHT,
							DSO_PANEL_BOTTOM,
							0,
							GR_CLR_BACKGROUND,
							dsoScheme->EmbossDkColor,
							dsoScheme->EmbossLtColor,
							NULL,
							GOL_EMBOSS_SIZE);

            hdso.DsoDrawState = DSO_STATE_DRAW;     // change state

            break;

        case DSO_STATE_DRAW:
            if(!GOLPanelDrawTsk())      // draw panel for graph
                return (0);             // drawing is not completed
            SetColor(GR_CLR_GRID);
            SetLineType(DOTTED_LINE);
            pos = GR_LEFT + ((GR_RIGHT - GR_LEFT) >> 3);
            hdso.DsoDrawState = DSO_STATE_VERLINE;  // change state

            break;

        case DSO_STATE_VERLINE:
            while(pos < GR_RIGHT)
            {                           // draw vertical grid lines
                if(IsDeviceBusy())
                    return (0);         // drawing is not completed
                WAIT_UNTIL_FINISH(Line(pos, GR_TOP, pos, GR_BOTTOM));
                pos += (GR_RIGHT - GR_LEFT) >> 3;
            }

            pos = GR_TOP + ((GR_BOTTOM - GR_TOP) >> 3);
            hdso.DsoDrawState = DSO_STATE_HORLINE;  // change state

            break;

        case DSO_STATE_HORLINE:
            while(pos < GR_BOTTOM)
            {                           // draw vertical grid lines
                if(IsDeviceBusy())
                    return (0);         // drawing is not completed
                WAIT_UNTIL_FINISH(Line(GR_LEFT, pos, GR_RIGHT, pos));
                pos += (GR_BOTTOM - GR_TOP) >> 3;
            }

            SetLineType(SOLID_LINE);

            hdso.DsoDrawState = DSO_STATE_BUTTONS;  // change state

            break;

        case DSO_STATE_BUTTONS:
            // create the control buttons at the bottom of the screen
            if(!DSO_CreateCtrlButtons(NULL, NULL, NULL, NULL))
        		return (0);

            hbutton[0].pBtn = (BUTTON *)GOLFindObject(ID_BUTTON_A);
            hbutton[0].BtnState = BTN_DISABLED;
            hbutton[0].BtnString = &DecAmpStr;
            hbutton[0].BtnSetClr = 0;

            hbutton[1].pBtn = (BUTTON *)GOLFindObject(ID_BUTTON_B);
            hbutton[1].BtnState = BTN_DISABLED;
            hbutton[1].BtnString = &IncAmpStr;
            hbutton[1].BtnSetClr = 0;

            hbutton[2].pBtn = (BUTTON *)GOLFindObject(ID_BUTTON_C);
            hbutton[2].BtnState = BTN_DISABLED;
            hbutton[2].BtnString = &DecTimeStr;
            hbutton[2].BtnSetClr = 0;

            hbutton[3].pBtn = (BUTTON *)GOLFindObject(ID_BUTTON_D);
            hbutton[3].BtnState = BTN_DISABLED;
            hbutton[3].BtnString = &IncTimeStr;
            hbutton[3].BtnSetClr = 0;

            DSO_SetCtrlButtons(&hbutton[0]);

            hdso.DsoDrawState = DSO_STATE_SET;      // change to initial state
            return (1);                 // drawing is done
    }	

    return (0);         // drawing is not completed
}

/**
  * @brief  
  * @param  
  * @retval 
  */
WORD DSO_MsgCallback(WORD objMsg, OBJ_HEADER *pObj, GOL_MSG *pMsg)
{
	switch(GetObjID(pObj))
	{
		case ID_BUTTON_A:
			if(objMsg == BTN_MSG_RELEASED)
			{
				hdso.DsoReScale.y1 -= DSO_RESCALER_STEP;
				hdso.DsoReScale.y2 += DSO_RESCALER_STEP;
			}

			return (1);

		case ID_BUTTON_B:
			if(objMsg == BTN_MSG_RELEASED)
			{
				hdso.DsoReScale.y1 += DSO_RESCALER_STEP;
				hdso.DsoReScale.y2 -= DSO_RESCALER_STEP;
			}

			return (1);

		case ID_BUTTON_C:
			if(objMsg == BTN_MSG_RELEASED)
			{
				if(iSamplingTime > 0)
					iSamplingTime--;

				hdso.DsoADC.SamplingTime = aSamplingTime[iSamplingTime];

				HAL_ADC_Stop_DMA(&AdcHandle);

				ADC_ChannelConfig(	hdso.DsoADC.Channel,
									hdso.DsoADC.Rank,
									hdso.DsoADC.SamplingTime,
									hdso.DsoADC.Offset);

				hdso.DsoState = DSO_TRIGGER;
				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;
			}

			return (1);

		case ID_BUTTON_D:
			if(objMsg == BTN_MSG_RELEASED)
			{
				if(iSamplingTime < sizeof(aSamplingTime)/4 - 1)
					iSamplingTime++;

				hdso.DsoADC.SamplingTime = aSamplingTime[iSamplingTime];

				HAL_ADC_Stop_DMA(&AdcHandle);

				ADC_ChannelConfig(	hdso.DsoADC.Channel,
									hdso.DsoADC.Rank,
									hdso.DsoADC.SamplingTime,
									hdso.DsoADC.Offset);

				hdso.DsoState = DSO_TRIGGER;
				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;
			}

			return (1);
	}

	return (1);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
WORD DSO_DrawCallback(void)
{
	DSO_Process();

	return (1);
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_Process(void)
{
    static DWORD    prevTick = 0;

	switch(hdso.DsoState)
	{
		case DSO_TRIGGER:
			if(hdso.DsoBuffer.buffstate == BUFFER_OFFSET_NONE)
				HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*) &adcBuffer, hdso.DsoTrigger.nSamples);

			if(hdso.DsoBuffer.buffstate == BUFFER_OFFSET_FULL)
			{
				if(	(adcBuffer[0] < (DSO_ADC_MAXVALUE >> 1) - hdso.DsoTrigger.vThreshold) \
					&& (adcBuffer[hdso.DsoTrigger.nSamples - 1] > (DSO_ADC_MAXVALUE >> 1) + hdso.DsoTrigger.vThreshold))
				{
					hdso.DsoState = DSO_SAMPLE;
				}

				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;
			}

			break;

		case DSO_SAMPLE:
			HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*) &adcBuffer, DSO_BUFFER_SIZE);

			hdso.DsoState = DSO_MATH;

			break;

		case DSO_MATH:
			DSO_ReScaleBuffer(	hdso.DsoBuffer.rptr,
								hdso.DsoBuffer.wptr,
								hdso.DsoBuffer.len,
								hdso.DsoReScale.x1,
								hdso.DsoReScale.x2,
								hdso.DsoReScale.y1,
								hdso.DsoReScale.y2);

			hdso.DsoState = DSO_DRAW;

			break;

		case DSO_DRAW:
			if(hdso.DsoBuffer.buffstate == BUFFER_OFFSET_HALF)
			{
				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;
			}

			if(hdso.DsoBuffer.buffstate == BUFFER_OFFSET_FULL)
			{
				DSO_Plot(	hdso.DsoBuffer.wptr,
							hdso.DsoBuffer.len,
							0);

				while(hdma2d.State != HAL_DMA2D_STATE_READY);

				DMA2D_CopyBuffer(	(uint32_t *) LCD_RENDER_ADD,
									(uint32_t *) LCD_LAYER1_ADD,
									0,
									0,
									BSP_LCD_GetXSize(),
									BSP_LCD_GetYSize());

				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;
				hdso.DsoState = DSO_DISPLAY;
			}

			break;

		case DSO_DISPLAY:
			if((tick - prevTick) > DSO_DISPLAY_DELAY)
			{
				DSO_LCDClear(	LCD_RENDER_ADD,
								GR_CLR_BACKGROUND,
								BSP_LCD_GetXSize(),
								BSP_LCD_GetYSize());

				prevTick = tick;
				hdso.DsoState = DSO_TRIGGER;
			}

			break;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_ReScaleBuffer(WORD *Src, WORD *Dst, WORD len, SHORT x1, SHORT x2, SHORT y1, SHORT y2)
{
	while(len--)
	{
		*Dst = ((*Src - x1)*(y2 - y1))/(x2 - x1) + y1;

		Src++;
		Dst++;
	}
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void DSO_Plot(WORD *data, WORD len, WORD pos)
{
	while(len--)
	{
		if((*data > GR_TOP) && (*data < GR_BOTTOM))
			DSO_LCDLayerPutPixel(	LCD_RENDER_ADD,
									GR_CLR_POINTS,
									GR_LEFT + pos,
									*data,
									BSP_LCD_GetXSize());

		pos++;
		data++;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_InitStyleScheme(GOL_SCHEME *pScheme)
{
    pScheme->Color0 = RGB565CONVERT(0x4C, 0x8E, 0xFF);
    pScheme->Color1 = RGB565CONVERT(0xFF, 0xBB, 0x4C);
    pScheme->EmbossDkColor = RGB565CONVERT(0xD4, 0xED, 0xF7);
    pScheme->EmbossLtColor = RGB565CONVERT(0xD4, 0xED, 0xF7);
    pScheme->ColorDisabled = RGB565CONVERT(0xD4, 0xE1, 0xF7);

    pScheme->TextColor1 = BRIGHTBLUE;
    pScheme->TextColor0 = RGB565CONVERT(0xFF, 0xBB, 0x4C);
    pScheme->TextColorDisabled = RGB565CONVERT(0xB8, 0xB9, 0xBC);
}

/**
  * @brief
  * @param
  * @retval
  */
WORD DSO_CreateCtrlButtons(XCHAR *pTextA, XCHAR *pTextB, XCHAR *pTextC, XCHAR *pTextD)
{
    WORD    state;
    BUTTON  *pObj;


    state = BTN_DRAW;
    if(pTextA == NULL)
        state = BTN_DRAW | BTN_DISABLED;
    pObj = BtnCreate
    (
        ID_BUTTON_A,
        CtrlBtnLeft(0),
        CtrlBtnTop(),
        CtrlBtnRight(0),
        CtrlBtnBottom(),
        0,
        state,
        NULL,
        pTextA,
        NULL
    );
    if (pObj == NULL)
        return (0);

    state = BTN_DRAW;
    if(pTextB == NULL)
        state = BTN_DRAW | BTN_DISABLED;
    pObj = BtnCreate
    (
        ID_BUTTON_B,
        CtrlBtnLeft(1),
        CtrlBtnTop(),
        CtrlBtnRight(1),
        CtrlBtnBottom(),
        0,
        state,
        NULL,
        pTextB,
        NULL
    );
    if (pObj == NULL)
        return (0);

    state = BTN_DRAW;
    if(pTextC == NULL)
        state = BTN_DRAW | BTN_DISABLED;
    pObj = BtnCreate
    (
        ID_BUTTON_C,
        CtrlBtnLeft(2),
        CtrlBtnTop(),
        CtrlBtnRight(2),
        CtrlBtnBottom(),
        0,
        state,
        NULL,
        pTextC,
        NULL
    );
    if (pObj == NULL)
        return (0);

    state = BTN_DRAW;
    if(pTextD == NULL)
        state = BTN_DRAW | BTN_DISABLED;
    pObj = BtnCreate
    (
        ID_BUTTON_D,
        CtrlBtnLeft(3),
        CtrlBtnTop(),
        CtrlBtnRight(3),
        CtrlBtnBottom(),
        0,
        state,
        NULL,
        pTextD,
        NULL
    );
    if (pObj == NULL)
        return (0);

    return (1);
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_SetCtrlButtons(DSO_ButtonTypeDef *hb)
{
	uint8_t i;

	for(i = 0 ; i < NUM_CTRL_BUTTONS ; i++)
	{
		BtnSetText(hb[i].pBtn, hb[i].BtnString);

		if(hb[i].BtnSetClr)
			SetState(hb[i].pBtn, hb[i].BtnState);
		else
			ClrState(hb[i].pBtn, hb[i].BtnState);
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_LCDLayerPutPixel(DWORD dst, DWORD color, WORD x, WORD y, WORD ImageWidth)
{
	*(__IO uint16_t*) (dst + (2*(y*ImageWidth + x))) = color;
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_LCDClear(DWORD dst, DWORD color, WORD ImageWidth, WORD ImageHeight)
{
	WORD y;

	for(y = 0 ; y < ImageHeight ; y++)
	{
		memset(dst + 2*y*ImageWidth, color, 2*ImageWidth);
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void ADC_ChannelConfig(uint32_t Channel, uint32_t Rank, uint32_t SamplingTime, uint32_t Offset)
{
	ADC_ChannelConfTypeDef sConfig;

  	sConfig.Channel = Channel;
  	sConfig.Rank = Rank;
  	sConfig.SamplingTime = SamplingTime;
  	sConfig.Offset = Offset;

  	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
}

/**
  * @brief  Initialize the DMA2D in memory to memory with PFC.
  * @param  ImageWidth: image width
  * @param  ImageHeight: image Height
  * @retval None
  */
void DMA2D_Init(uint32_t ImageWidth, uint32_t ImageHeight)
{
	/* Init DMA2D */
	/* Configure the DMA2D Mode, Color Mode and output offset */
	hdma2d.Init.Mode          = DMA2D_M2M_PFC;
	hdma2d.Init.ColorMode     = DMA2D_OUTPUT_RGB565;
	hdma2d.Init.OutputOffset  = BSP_LCD_GetXSize() - ImageWidth;
	hdma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
	hdma2d.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */

	/* DMA2D Callbacks Configuration */
	hdma2d.XferCpltCallback  	= NULL;

	/* Foreground Configuration */
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0xFF;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
	hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

	hdma2d.Instance          = DMA2D;

	/* DMA2D Initialization */
	HAL_DMA2D_Init(&hdma2d);
	HAL_DMA2D_ConfigLayer(&hdma2d, 1);
}

/**
  * @brief  Copy the Decoded image to the display Frame buffer.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Pointer to destination buffer
  * @param  ImageWidth: image width
  * @param  ImageHeight: image Height
  * @retval None
  */
void DMA2D_CopyBuffer(uint32_t *pSrc, uint32_t *pDst, uint16_t xPos, uint16_t yPos, uint16_t ImageWidth, uint16_t ImageHeight)
{
	HAL_DMA2D_Start_IT(&hdma2d, pSrc, pDst + (2*(yPos*BSP_LCD_GetXSize() + xPos)), ImageWidth, ImageHeight);
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	hdso.DsoBuffer.buffstate = BUFFER_OFFSET_FULL;
}

/**
  * @brief  Regular conversion half DMA transfer callback in non blocking mode
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	hdso.DsoBuffer.buffstate = BUFFER_OFFSET_HALF;
}
