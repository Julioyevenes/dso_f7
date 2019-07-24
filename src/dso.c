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
    DSO_SAMPLE,
    DSO_DISPLAY
} DSO_STATE;

typedef enum _DSO_DRAW_STATE
{
	DSO_STATE_SET,
	DSO_STATE_DRAW,
	DSO_STATE_VERLINE,
	DSO_STATE_HORLINE
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

typedef struct _DSO_HandleTypeDef
{
	DSO_STATE 		DsoState;
	DSO_DRAW_STATE 	DsoDrawState;
	BufferTypeDef 	DsoBuffer;
} DSO_HandleTypeDef;

/* Private constants --------------------------------------------------------*/
#define GR_CLR_GRID                 LIGHTGRAY
#define GR_CLR_BACKGROUND           BLACK
#define GR_CLR_POINTS               BRIGHTGREEN

#define DSO_DISPLAY_DELAY           1

#define LCD_LAYER0_ADD				((uint32_t)0xC0000000)
#define LCD_LAYER1_ADD				((uint32_t)0xC0500000)
#define LCD_RENDER_ADD				((uint32_t)0xC0200000)

/* Private macro ------------------------------------------------------------*/
#define WAIT_UNTIL_FINISH(x)    	while(!x)
	
/* Private variables --------------------------------------------------------*/
GOL_SCHEME              			*dsoScheme;

DSO_HandleTypeDef 					hdso = {DSO_SAMPLE, DSO_STATE_SET, 0};

WORD 								adcBuffer[DSO_BUFFER_SIZE];
WORD 								dsoBuffer[DSO_BUFFER_SIZE];

/* Private function prototypes ----------------------------------------------*/
void DSO_Graph(WORD *data, WORD len, WORD pos);
void DSO_LCDLayerPutPixel(DWORD dst, DWORD color, WORD x, WORD y);
void DSO_LCDClear(DWORD dst, DWORD color);
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

			// initialize the screen
			DMA2D_Init(GR_RIGHT, GR_BOTTOM);

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
	return (1);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
WORD DSO_DrawCallback(void)
{
    static DWORD    prevTick = 0;

	switch(hdso.DsoState)
	{
		case DSO_SAMPLE:
			if(hdso.DsoBuffer.buffstate == BUFFER_OFFSET_HALF)
			{
				DSO_Graph(	hdso.DsoBuffer.wptr,
							hdso.DsoBuffer.len/2,
							0);

				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;
			}

			if(hdso.DsoBuffer.buffstate == BUFFER_OFFSET_FULL)
			{
				DSO_Graph(	hdso.DsoBuffer.wptr + hdso.DsoBuffer.len/4,
							hdso.DsoBuffer.len/2,
							hdso.DsoBuffer.len/2);

				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;

				DMA2D_CopyBuffer(	(uint32_t *) LCD_RENDER_ADD,
									(uint32_t *) LCD_LAYER1_ADD,
									GR_LEFT,
									GR_TOP,
									GR_RIGHT,
									GR_BOTTOM);

				hdso.DsoState = DSO_DISPLAY;
			}

			break;

		case DSO_DISPLAY:
			if((tick - prevTick) > DSO_DISPLAY_DELAY)
			{
				DSO_LCDClear(LCD_RENDER_ADD, GR_CLR_BACKGROUND);

				prevTick = tick;
				hdso.DsoState = DSO_SAMPLE;
			}

			break;
	}

	return (1);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void DSO_Graph(WORD *data, WORD len, WORD pos)
{
	while(len--)
	{
		pos++;

		DSO_LCDLayerPutPixel(LCD_RENDER_ADD, GR_CLR_POINTS, GR_LEFT + pos, *data);

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
void DSO_LCDLayerPutPixel(DWORD dst, DWORD color, WORD x, WORD y)
{
	*(__IO uint16_t*) (dst + (2*(y*GetMaxX() + x))) = color;
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_LCDClear(DWORD dst, DWORD color)
{
	WORD x, y;

	for(y = 0 ; y < GetMaxY() ; y++)
	{
		memset(dst + y*GetMaxX()*2, color, GetMaxX()*2);
	}
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
	hdma2d.Init.OutputOffset  = GetMaxX() - ImageWidth;
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
	HAL_DMA2D_Start_IT(&hdma2d, pSrc, pDst + (2*(yPos*GetMaxX() + xPos)), ImageWidth, ImageHeight);
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	volatile WORD len = hdso.DsoBuffer.len/2;
	volatile WORD *sptr = hdso.DsoBuffer.rptr + len/2;
	volatile WORD *dptr = hdso.DsoBuffer.wptr + len/2;

	while(len--)
	{
		*dptr = (GR_BOTTOM - GR_TOP) - ((*sptr * (GR_BOTTOM - GR_TOP)) >> 12);

		if((*dptr + GR_TOP) > GR_BOTTOM)
			*dptr = GR_BOTTOM - GR_TOP;

		sptr++;
		dptr++;
	}

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
	volatile WORD len = hdso.DsoBuffer.len/2;
	volatile WORD *sptr = hdso.DsoBuffer.rptr;
	volatile WORD *dptr = hdso.DsoBuffer.wptr;

	while(len--)
	{
		*dptr = (GR_BOTTOM - GR_TOP) - ((*sptr * (GR_BOTTOM - GR_TOP)) >> 12);

		if((*dptr + GR_TOP) > GR_BOTTOM)
			*dptr = GR_BOTTOM - GR_TOP;

		sptr++;
		dptr++;
	}

	hdso.DsoBuffer.buffstate = BUFFER_OFFSET_HALF;
}
