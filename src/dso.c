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
#include "arm_math.h"

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
	DSO_STATE_BUTTONS,
	DSO_STATE_STATICTEXT
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
	WORD 		*cptr;
	WORD 		len;
	WORD 		pos;

	BUFFER_STATE buffstate;
} BufferTypeDef;

typedef struct _DSO_ADCTypeDef
{
	DWORD Channel;
	DWORD Rank;
	DWORD SamplingTime;
	DWORD Offset;
} DSO_ADCTypeDef;

typedef struct _DSO_ReScaleTypeDef
{
	SHORT	x1;
	SHORT	x2;
	SHORT	y1;
	SHORT	y2;
} DSO_ReScaleTypeDef;

typedef struct _DSO_TriggerTypeDef
{
	WORD	nSamples;
	WORD	vThreshold;
} DSO_TriggerTypeDef;

typedef struct _DSO_MathOutTypeDef
{
	float 						XScale;
	float 						YScale;
	float 						MaxY;
	float 						MinY;
	float 						Freq;

	arm_rfft_instance_q15 		hrfft;
	BufferTypeDef				fftBuffer;
} DSO_MathOutTypeDef;

typedef struct _DSO_HandleTypeDef
{
	DSO_STATE 			DsoState;
	DSO_DRAW_STATE 		DsoDrawState;
	BufferTypeDef 		DsoBuffer;
	DSO_ADCTypeDef  	DsoADC;
	DSO_ReScaleTypeDef	DsoReScale;
	DSO_TriggerTypeDef	DsoTrigger;
	DSO_MathOutTypeDef	DsoMathOut;
} DSO_HandleTypeDef;

typedef struct _DSO_ButtonTypeDef
{
	BUTTON  *pBtn;
	XCHAR	*BtnString;
	WORD	BtnId;
	SHORT	BtnLeft;
	SHORT	BtnRight;
	SHORT	BtnTop;
	SHORT	BtnBottom;
	WORD	BtnState;
	BOOL	BtnSetClr;
} DSO_ButtonTypeDef;

typedef struct _DSO_TextTypeDef
{
	STATICTEXT  *pText;
	XCHAR		*TextString;
	WORD		TextId;
	SHORT		TextLeft;
	SHORT		TextRight;
	SHORT		TextTop;
	SHORT		TextBottom;
	WORD		TextState;
	BOOL		TextSetClr;
} DSO_TextTypeDef;

/* Private constants --------------------------------------------------------*/
#define GR_CLR_GRID                 LIGHTGRAY
#define GR_CLR_BACKGROUND           BLACK
#define GR_CLR_POINTS               BRIGHTGREEN
#define GR_CLR_FFT 					BRIGHTMAGENTA

#define DSO_HOR_GRID 				8
#define DSO_VER_GRID 				8

#define DSO_DISPLAY_DELAY           1

#define LCD_LAYER0_ADD				((DWORD)0xC0000000)
#define LCD_LAYER1_ADD				((DWORD)0xC0500000)
#define LCD_RENDER_ADD				((DWORD)0xC0200000)

#define DSO_ADC_MAXVOLT				3300
#define DSO_ADC_BITDEPTH			12
#define DSO_ADC_MAXVAL 				(1 << DSO_ADC_BITDEPTH)
#define DSO_ADC_CLOCK				36

#define DSO_RESCALER_STEP			((GR_BOTTOM - GR_TOP) >> 4)

#define DSO_TRIGGER_HOR_STEP		(DSO_BUFFER_SIZE >> 8)
#define DSO_TRIGGER_VER_STEP		(DSO_ADC_MAXVAL >> 6)

#define DSO_FFT_MAXVAL 				(1 << 10)
#define DSO_FFT_FREQ_OFFSET 		1

#define DSO_NUM_BUFFERS				2

#define ID_BUTTON_A					101
#define ID_BUTTON_B					102
#define ID_BUTTON_C					103
#define ID_BUTTON_D					104
#define ID_BUTTON_E					105
#define ID_BUTTON_F					106
#define ID_BUTTON_G					107
#define ID_BUTTON_H					108

#define ID_STATICTEXT1 				201
#define ID_STATICTEXT2 				202
#define ID_STATICTEXT3 				203
#define ID_STATICTEXT4 				204
#define ID_STATICTEXT5 				205

#define NUM_CTRL_BUTTONS 			8
#define NUM_STATICTEXT 				5

#define STRING_SIZE 				12

const XCHAR							aBtnStr[][STRING_SIZE] = {	{'A','m','p','l',' ','-',0},
																{'A','m','p','l',' ','+',0},
																{'T','i','m','e',' ','-',0},
																{'T','i','m','e',' ','+',0},
																{'X','T','r','i','g',' ','-',0},
																{'X','T','r','i','g',' ','+',0},
																{'Y','T','r','i','g',' ','-',0},
																{'Y','T','r','i','g',' ','+',0}};

const XCHAR							aTextStr[][STRING_SIZE] = {	{0},
																{0},
																{0},
																{0},
																{0}};

/* Private macro ------------------------------------------------------------*/
#define WAIT_UNTIL_FINISH(x)    					while(!x)

#define DSO_BTN_SET_STATE(HANDLER, STATE, SETCLR)	HANDLER.BtnState = STATE;   \
													HANDLER.BtnSetClr = SETCLR

#define DSO_BTN_SET_STRING(HANDLER, STRING)			HANDLER.BtnString = STRING
#define DSO_TEXT_SET_STRING(HANDLER, STRING)		HANDLER.TextString = STRING; \
													HANDLER.TextState = ST_DRAW | ST_FRAME | ST_CENTER_ALIGN; \
													HANDLER.TextSetClr = 1;
	
/* Private variables --------------------------------------------------------*/
GOL_SCHEME              			*dsoScheme;

DSO_HandleTypeDef 					hdso = {DSO_TRIGGER, DSO_STATE_SET, 0, 0, 0, 0, 0};

WORD 								adcBuffer[DSO_BUFFER_SIZE];
WORD 								dsoBuffer[DSO_NUM_BUFFERS][DSO_BUFFER_SIZE];

q15_t 								fftBuffer[2 * DSO_BUFFER_SIZE];

DWORD								aSTime[] = {	ADC_SAMPLETIME_3CYCLES,
													ADC_SAMPLETIME_15CYCLES,
													ADC_SAMPLETIME_28CYCLES,
													ADC_SAMPLETIME_56CYCLES,
													ADC_SAMPLETIME_84CYCLES,
													ADC_SAMPLETIME_112CYCLES,
													ADC_SAMPLETIME_144CYCLES,
													ADC_SAMPLETIME_480CYCLES};

DWORD								aAdcCycles[] = {3, 15, 28, 56, 84, 112, 144, 480};

WORD 								aBtnId[] = {	ID_BUTTON_A,
													ID_BUTTON_B,
													ID_BUTTON_C,
													ID_BUTTON_D,
													ID_BUTTON_E,
													ID_BUTTON_F,
													ID_BUTTON_G,
													ID_BUTTON_H};

WORD 								aTextId[] = {	ID_STATICTEXT1,
													ID_STATICTEXT2,
													ID_STATICTEXT3,
													ID_STATICTEXT4,
													ID_STATICTEXT5};

BYTE								iSTime = 0;

DSO_ButtonTypeDef					hButton[NUM_CTRL_BUTTONS];
DSO_TextTypeDef 					hText[NUM_STATICTEXT];

CHAR 								TempStr[NUM_STATICTEXT][STRING_SIZE];

/* Private function prototypes ----------------------------------------------*/
void DSO_Process(void);
void DSO_ReScaleBuffer(WORD *Src, WORD *Dst, WORD len, SHORT x1, SHORT x2, SHORT y1, SHORT y2);
void DSO_BufferFFT(DSO_HandleTypeDef *hdso);
SHORT DSO_GetMax(WORD *Src, WORD len, WORD Offset);
SHORT DSO_GetMin(WORD *Src, WORD len, WORD Offset);
DWORD DSO_GetFreq(DSO_HandleTypeDef *hdso, WORD Offset);
void DSO_Plot(WORD *data, WORD len, WORD pos, DWORD color);
WORD DSO_CreateCtrlButtons(DSO_ButtonTypeDef *hb, BYTE nb, WORD *BtnId, XCHAR *BtnStr, BYTE StrSize);
WORD DSO_CreateText(DSO_TextTypeDef *ht, BYTE nt, WORD *TextId, XCHAR *TextStr, BYTE StrSize);
void DSO_SetCtrlButtons(DSO_ButtonTypeDef *hb, BYTE nb);
void DSO_SetText(DSO_TextTypeDef *ht, BYTE nt);
void DSO_LCDLayerPutPixel(DWORD dst, DWORD color, WORD x, WORD y, WORD ImageWidth);
void DSO_LCDClear(DWORD dst, DWORD color, WORD ImageWidth, WORD ImageHeight);
void ADC_ChannelConfig(DWORD Channel, DWORD Rank, DWORD SamplingTime, DWORD Offset);
void DMA2D_Init(DWORD ImageWidth, DWORD ImageHeight);
void DMA2D_CopyBuffer(DWORD *pSrc, DWORD *pDst, WORD xPos, WORD yPos, WORD ImageWidth, WORD ImageHeight);

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
			hdso.DsoBuffer.wptr = (WORD *) &dsoBuffer[0][0];
			hdso.DsoBuffer.cptr = NULL;
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
			hdso.DsoReScale.x2 = DSO_ADC_MAXVAL;
			hdso.DsoReScale.y1 = GR_BOTTOM;
			hdso.DsoReScale.y2 = GR_TOP;

			// initialize the dso trigger handler
			hdso.DsoTrigger.nSamples = DSO_TRIGGER_HOR_STEP;
			hdso.DsoTrigger.vThreshold = DSO_TRIGGER_VER_STEP;

			// initialize the dso math handler
			hdso.DsoMathOut.fftBuffer.rptr = (WORD *) &adcBuffer;
			hdso.DsoMathOut.fftBuffer.wptr = (WORD *) &dsoBuffer[1][0];
			hdso.DsoMathOut.fftBuffer.cptr = (WORD *) &fftBuffer;
			hdso.DsoMathOut.fftBuffer.len = DSO_BUFFER_SIZE;
			hdso.DsoMathOut.fftBuffer.pos = 0;
			hdso.DsoMathOut.fftBuffer.buffstate = BUFFER_OFFSET_NONE;
			arm_rfft_init_q15(&hdso.DsoMathOut.hrfft, DSO_BUFFER_SIZE, 0, 1);

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
            pos = GR_LEFT + ((GR_RIGHT - GR_LEFT) / DSO_HOR_GRID);
            hdso.DsoDrawState = DSO_STATE_VERLINE;  // change state

            break;

        case DSO_STATE_VERLINE:
            while(pos < GR_RIGHT)
            {                           // draw vertical grid lines
                if(IsDeviceBusy())
                    return (0);         // drawing is not completed
                WAIT_UNTIL_FINISH(Line(pos, GR_TOP, pos, GR_BOTTOM));
                pos += (GR_RIGHT - GR_LEFT) / DSO_HOR_GRID;
            }

            pos = GR_TOP + ((GR_BOTTOM - GR_TOP) / DSO_VER_GRID);
            hdso.DsoDrawState = DSO_STATE_HORLINE;  // change state

            break;

        case DSO_STATE_HORLINE:
            while(pos < GR_BOTTOM)
            {                           // draw vertical grid lines
                if(IsDeviceBusy())
                    return (0);         // drawing is not completed
                WAIT_UNTIL_FINISH(Line(GR_LEFT, pos, GR_RIGHT, pos));
                pos += (GR_BOTTOM - GR_TOP) / DSO_VER_GRID;
            }

            SetLineType(SOLID_LINE);

            hdso.DsoDrawState = DSO_STATE_BUTTONS;  // change state

            break;

        case DSO_STATE_BUTTONS:
            // create the control buttons at the bottom of the screen
            if(!DSO_CreateCtrlButtons(&hButton[0], NUM_CTRL_BUTTONS, &aBtnId[0], &aBtnStr[0][0], STRING_SIZE))
        		return (0);

            DSO_BTN_SET_STATE(hButton[0], BTN_DISABLED, 0);
            DSO_BTN_SET_STATE(hButton[1], BTN_DISABLED, 0);
            DSO_BTN_SET_STATE(hButton[2], BTN_DISABLED, 0);
            DSO_BTN_SET_STATE(hButton[3], BTN_DISABLED, 0);
            DSO_BTN_SET_STATE(hButton[4], BTN_DISABLED, 0);
            DSO_BTN_SET_STATE(hButton[5], BTN_DISABLED, 0);
            DSO_BTN_SET_STATE(hButton[6], BTN_DISABLED, 0);
            DSO_BTN_SET_STATE(hButton[7], BTN_DISABLED, 0);

            DSO_SetCtrlButtons(&hButton[0], NUM_CTRL_BUTTONS);

            hdso.DsoDrawState = DSO_STATE_STATICTEXT;      // change to initial state

            break;

        case DSO_STATE_STATICTEXT:
            if(!DSO_CreateText(&hText[0], NUM_STATICTEXT, &aTextId[0], &aTextStr[0][0], STRING_SIZE))
            	return (0);

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
				if(iSTime > 0)
					iSTime--;

				hdso.DsoADC.SamplingTime = aSTime[iSTime];

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
				if(iSTime < sizeof(aSTime)/4 - 1)
					iSTime++;

				hdso.DsoADC.SamplingTime = aSTime[iSTime];

				HAL_ADC_Stop_DMA(&AdcHandle);

				ADC_ChannelConfig(	hdso.DsoADC.Channel,
									hdso.DsoADC.Rank,
									hdso.DsoADC.SamplingTime,
									hdso.DsoADC.Offset);

				hdso.DsoState = DSO_TRIGGER;
				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;
			}

			return (1);

		case ID_BUTTON_E:
			if(objMsg == BTN_MSG_RELEASED)
			{
				if(hdso.DsoTrigger.nSamples > DSO_TRIGGER_HOR_STEP)
					hdso.DsoTrigger.nSamples -= DSO_TRIGGER_HOR_STEP;
			}

			return (1);

		case ID_BUTTON_F:
			if(objMsg == BTN_MSG_RELEASED)
			{
				if(hdso.DsoTrigger.nSamples < DSO_BUFFER_SIZE)
					hdso.DsoTrigger.nSamples += DSO_TRIGGER_HOR_STEP;
			}

			return (1);

		case ID_BUTTON_G:
			if(objMsg == BTN_MSG_RELEASED)
			{
				if(hdso.DsoTrigger.vThreshold > DSO_TRIGGER_VER_STEP)
					hdso.DsoTrigger.vThreshold -= DSO_TRIGGER_VER_STEP;
			}

			return (1);

		case ID_BUTTON_H:
			if(objMsg == BTN_MSG_RELEASED)
			{
				if(hdso.DsoTrigger.vThreshold < DSO_ADC_MAXVAL >> 1)
					hdso.DsoTrigger.vThreshold += DSO_TRIGGER_VER_STEP;
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
				HAL_ADC_Start_DMA(&AdcHandle, (DWORD*) &adcBuffer, hdso.DsoTrigger.nSamples);

			if(hdso.DsoBuffer.buffstate == BUFFER_OFFSET_FULL)
			{
				if(	(adcBuffer[0] < (DSO_ADC_MAXVAL >> 1) - hdso.DsoTrigger.vThreshold) \
					&& (adcBuffer[hdso.DsoTrigger.nSamples - 1] > (DSO_ADC_MAXVAL >> 1) + hdso.DsoTrigger.vThreshold))
				{
					hdso.DsoState = DSO_SAMPLE;
				}

				hdso.DsoBuffer.buffstate = BUFFER_OFFSET_NONE;
			}

			break;

		case DSO_SAMPLE:
			HAL_ADC_Start_DMA(&AdcHandle, (DWORD*) &adcBuffer, DSO_BUFFER_SIZE);

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

			DSO_BufferFFT(&hdso);

			DSO_ReScaleBuffer(	hdso.DsoMathOut.fftBuffer.cptr,
								hdso.DsoMathOut.fftBuffer.wptr,
								hdso.DsoMathOut.fftBuffer.len,
								0,
								DSO_FFT_MAXVAL,
								GR_BOTTOM,
								GR_TOP);

			hdso.DsoMathOut.XScale = (2 * (aAdcCycles[iSTime] + 12) * ((GR_RIGHT - GR_LEFT)/DSO_HOR_GRID)) / DSO_ADC_CLOCK;
			hdso.DsoMathOut.YScale = ((DSO_ADC_MAXVOLT/DSO_VER_GRID) * (GR_BOTTOM - GR_TOP)) / (hdso.DsoReScale.y1 - hdso.DsoReScale.y2);
			hdso.DsoMathOut.MaxY = (DSO_ADC_MAXVOLT * DSO_GetMax(hdso.DsoBuffer.rptr, DSO_BUFFER_SIZE, DSO_ADC_MAXVAL >> 1)) / DSO_ADC_MAXVAL;
			hdso.DsoMathOut.MinY = (DSO_ADC_MAXVOLT * DSO_GetMin(hdso.DsoBuffer.rptr, DSO_BUFFER_SIZE ,DSO_ADC_MAXVAL >> 1)) / DSO_ADC_MAXVAL;
			hdso.DsoMathOut.Freq = DSO_GetFreq(&hdso, DSO_FFT_FREQ_OFFSET);

			sprintf(&TempStr[0][0], "%3.0f [us]\0", hdso.DsoMathOut.XScale);
			sprintf(&TempStr[1][0], "%3.0f [mV]\0", hdso.DsoMathOut.YScale);
			sprintf(&TempStr[2][0], "%3.0f [mV]\0", hdso.DsoMathOut.MaxY);
			sprintf(&TempStr[3][0], "%3.0f [mV]\0", hdso.DsoMathOut.MinY);
			sprintf(&TempStr[4][0], "%3.0f [Hz]\0", hdso.DsoMathOut.Freq);

			DSO_TEXT_SET_STRING(hText[0], &TempStr[0][0]);
			DSO_TEXT_SET_STRING(hText[1], &TempStr[1][0]);
			DSO_TEXT_SET_STRING(hText[2], &TempStr[2][0]);
			DSO_TEXT_SET_STRING(hText[3], &TempStr[3][0]);
			DSO_TEXT_SET_STRING(hText[4], &TempStr[4][0]);

			DSO_SetText(&hText[0], NUM_STATICTEXT);

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
							GR_RIGHT - GR_LEFT,
							0,
							GR_CLR_POINTS);

				DSO_Plot(	hdso.DsoMathOut.fftBuffer.wptr,
							GR_RIGHT - GR_LEFT,
							0,
							GR_CLR_FFT);

				while(hdma2d.State != HAL_DMA2D_STATE_READY);

				DMA2D_CopyBuffer(	(DWORD *) LCD_RENDER_ADD,
									(DWORD *) LCD_LAYER1_ADD,
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
void DSO_BufferFFT(DSO_HandleTypeDef *hdso)
{
	WORD 	i;
	q15_t 	InBuffer[hdso->DsoMathOut.fftBuffer.len];
	q15_t 	OutBuffer[2 * hdso->DsoMathOut.fftBuffer.len];

	for(i = 0 ; i < hdso->DsoMathOut.fftBuffer.len ; i++)
		InBuffer[i] = (q15_t) hdso->DsoMathOut.fftBuffer.rptr[i];

	arm_rfft_q15(	&(hdso->DsoMathOut.hrfft),
					(q15_t *) &InBuffer,
					(q15_t *) &OutBuffer);

	arm_abs_q15(	(q15_t *) &OutBuffer,
					(q15_t *) &OutBuffer,
					hdso->DsoMathOut.fftBuffer.len);

	for(i = 0 ; i < 2 * hdso->DsoMathOut.fftBuffer.len ; i++)
		hdso->DsoMathOut.fftBuffer.cptr[i] = (WORD) OutBuffer[i];
}

/**
  * @brief
  * @param
  * @retval
  */
SHORT DSO_GetMax(WORD *Src, WORD len, WORD Offset)
{
	SHORT ret = Offset;

	while(len--)
	{
		if(*Src > ret)
			ret = *Src;

		Src++;
	}

	return(ret);
}

/**
  * @brief
  * @param
  * @retval
  */
SHORT DSO_GetMin(WORD *Src, WORD len, WORD Offset)
{
	SHORT ret = Offset;

	while(len--)
	{
		if(*Src < ret)
			ret = *Src;

		Src++;
	}

	return(ret);
}

/**
  * @brief
  * @param
  * @retval
  */
DWORD DSO_GetFreq(DSO_HandleTypeDef *hdso, WORD Offset)
{
	WORD	i, idx;
	WORD	len = hdso->DsoMathOut.fftBuffer.len;
	WORD	*Src = &(hdso->DsoMathOut.fftBuffer.cptr[Offset]);
	SHORT	temp = 0;
	DWORD	freq;

	for(i = 0 ; i < len - Offset ; i++)
	{
		if(*Src > temp)
		{
			temp = *Src;
			idx = i/2;
		}

		Src++;
	}

	freq = (1000000 * DSO_ADC_CLOCK * idx) / (2 * (aAdcCycles[iSTime] + 12) * len);

	return(freq);
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void DSO_Plot(WORD *data, WORD len, WORD pos, DWORD color)
{
	while(len--)
	{
		if((*data > GR_TOP) && (*data < GR_BOTTOM))
			DSO_LCDLayerPutPixel(	LCD_RENDER_ADD,
									color,
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
WORD DSO_CreateCtrlButtons(DSO_ButtonTypeDef *hb, BYTE nb, WORD *BtnId, XCHAR *BtnStr, BYTE StrSize)
{
	BYTE i;

	for(i = 0 ; i < nb ; i++)
	{
		hb[i].BtnString = &BtnStr[i * StrSize];
		hb[i].BtnId = BtnId[i];
		hb[i].BtnLeft = CtrlBtnLeft(i, nb);
		hb[i].BtnRight = CtrlBtnRight(i, nb);
		hb[i].BtnTop = CtrlBtnTop();
		hb[i].BtnBottom = CtrlBtnBottom();

		hb[i].BtnState = BTN_DRAW;
		if(BtnStr[i] == NULL)
			hb[i].BtnState |= BTN_DISABLED;

    		hb[i].pBtn = BtnCreate
    		(
        		hb[i].BtnId,
        		hb[i].BtnLeft,
        		hb[i].BtnTop,
        		hb[i].BtnRight,
        		hb[i].BtnBottom,
        		0,
        		hb[i].BtnState,
        		NULL,
        		hb[i].BtnString,
        		NULL
    		);
    		if (hb[i].pBtn == NULL)
        		return (0);
	}

	return (1);
}

/**
  * @brief
  * @param
  * @retval
  */
WORD DSO_CreateText(DSO_TextTypeDef *ht, BYTE nt, WORD *TextId, XCHAR *TextStr, BYTE StrSize)
{
	BYTE i;

	for(i = 0 ; i < nt ; i++)
	{
		ht[i].TextString = &TextStr[i * StrSize];
		ht[i].TextId = TextId[i];
		ht[i].TextLeft = InfoTexLeft(i, nt);
		ht[i].TextRight = InfoTexRight(i, nt);
		ht[i].TextTop = InfoTextTop();
		ht[i].TextBottom = InfoTextBottom();
		ht[i].TextState = ST_DRAW | ST_FRAME | ST_CENTER_ALIGN;

    		ht[i].pText = StCreate
    		(
        		ht[i].TextId,
        		ht[i].TextLeft,
        		ht[i].TextTop,
        		ht[i].TextRight,
        		ht[i].TextBottom,
        		ht[i].TextState,
        		ht[i].TextString,
        		NULL
    		);
    		if (ht[i].pText == NULL)
        		return (0);
	}

	return (1);
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_SetCtrlButtons(DSO_ButtonTypeDef *hb, BYTE nb)
{
	BYTE i;

	for(i = 0 ; i < nb ; i++)
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
void DSO_SetText(DSO_TextTypeDef *ht, BYTE nt)
{
	BYTE i;

	for(i = 0 ; i < nt ; i++)
	{
		StSetText(ht[i].pText, ht[i].TextString);

		if(ht[i].TextSetClr)
			SetState(ht[i].pText, ht[i].TextState);
		else
			ClrState(ht[i].pText, ht[i].TextState);
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void DSO_LCDLayerPutPixel(DWORD dst, DWORD color, WORD x, WORD y, WORD ImageWidth)
{
	*(__IO WORD*) (dst + (2*(y*ImageWidth + x))) = color;
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
void ADC_ChannelConfig(DWORD Channel, DWORD Rank, DWORD SamplingTime, DWORD Offset)
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
void DMA2D_Init(DWORD ImageWidth, DWORD ImageHeight)
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
void DMA2D_CopyBuffer(DWORD *pSrc, DWORD *pDst, WORD xPos, WORD yPos, WORD ImageWidth, WORD ImageHeight)
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
