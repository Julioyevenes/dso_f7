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
typedef enum _DSO_STATES
{
    DSO_SAMPLE,
    DSO_DISPLAY
} DSO_STATES;

typedef enum _DSO_DRAW_STATE
{
	DSO_STATE_SET,
	DSO_STATE_DRAW,
	DSO_STATE_VERLINE,
	DSO_STATE_HORLINE
} DSO_DRAW_STATE;

/* Private constants --------------------------------------------------------*/
#define GR_CLR_GRID                 LIGHTGRAY
#define GR_CLR_BACKGROUND           BLACK
#define GR_CLR_POINTS               BRIGHTGREEN

#define DSO_DISPLAY_DELAY           40

// Dimensions for DSO graph area
#define DSO_ORIGIN_X    			0
#define DSO_ORIGIN_Y    			0   

#define DSO_PANEL_LEFT   			DSO_ORIGIN_X
#define DSO_PANEL_RIGHT  			DSO_ORIGIN_X + GetMaxX()
#define DSO_PANEL_TOP    			DSO_ORIGIN_Y
#define DSO_PANEL_BOTTOM 			DSO_ORIGIN_Y + GetMaxY()

// Graph area borders
#define GR_LEFT     				(DSO_PANEL_LEFT + GOL_EMBOSS_SIZE)
#define GR_RIGHT    				(DSO_PANEL_RIGHT - GOL_EMBOSS_SIZE)
#define GR_TOP      				(DSO_PANEL_TOP + GOL_EMBOSS_SIZE)
#define GR_BOTTOM   				(DSO_PANEL_BOTTOM - GOL_EMBOSS_SIZE)

// Scanning window size
#define DSO_WINDOW_SIZE 			8

// DSO data circular buffer size
#define DSO_BUFFER_SIZE 			(GR_RIGHT - GR_LEFT)

#define DSO_DELAY					1

/* Private macro ------------------------------------------------------------*/
#define WAIT_UNTIL_FINISH(x)    	while(!x)
	
/* Private variables --------------------------------------------------------*/
// style scheme for DSO
GOL_SCHEME              			*dsoScheme;

// DSO data circular buffer
SHORT                       		dsoBuffer[DSO_BUFFER_SIZE];

// Temporary buffer for graph
SHORT                       		tempBuffer[10];

// DSO scan/calculation states
DSO_STATES                  		dsoStates = DSO_SAMPLE;

// DSO sample variable
extern WORD							DSOSample;

/* Private function prototypes ----------------------------------------------*/
void DSO_Graph(void);
WORD DSO_GetSamples(WORD number);

/**
  * @brief  
  * @param  
  * @retval 
  */
WORD DSO_Create(void)
{
    static DSO_DRAW_STATE	state = DSO_STATE_SET;
    static SHORT    		pos;

    switch(state)
    {
        case DSO_STATE_SET:
		
			// Free memory for the objects in the previous linked list and start new list to display
			GOLFree();

			// initialize the screen	
			SetColor(GR_CLR_BACKGROUND);
			ClearDevice();		

            // set parameters for panel
            GOLPanelDraw
            (
                DSO_PANEL_LEFT,
                DSO_PANEL_TOP,
                DSO_PANEL_RIGHT,
                DSO_PANEL_BOTTOM,
                0,
                GR_CLR_BACKGROUND,
                dsoScheme->EmbossDkColor,
                dsoScheme->EmbossLtColor,
                NULL,
                GOL_EMBOSS_SIZE
            );

            state = DSO_STATE_DRAW;     // change state

            break;

        case DSO_STATE_DRAW:
            if(!GOLPanelDrawTsk())      // draw panel for graph
                return (0);             // drawing is not completed
            SetColor(GR_CLR_GRID);
            SetLineType(DOTTED_LINE);
            pos = GR_LEFT + ((GR_RIGHT - GR_LEFT) >> 3);
            state = DSO_STATE_VERLINE;  // change state

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
            state = DSO_STATE_HORLINE;  // change state

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

            state = DSO_STATE_SET;      // change to initial state
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

	switch(dsoStates)
	{
        case DSO_SAMPLE:
            if((tick - prevTick) > DSO_DELAY)
            {
				if(DSO_GetSamples(DSO_WINDOW_SIZE))
					DSO_Graph(); // redraw graph
						
                prevTick = tick;
            }

            break;

        case DSO_DISPLAY:
            if((tick - prevTick) > DSO_DISPLAY_DELAY)
            {
                prevTick = tick;
                dsoStates = DSO_SAMPLE;
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
void DSO_Graph(void)
{
    SHORT           x, y;
    static SHORT    sy = 0;
    static SHORT    tsy = 0;
    SHORT           ey;
    static SHORT    *ptr = dsoBuffer;
    static SHORT    pos = 0;
    SHORT           counter;
    SHORT           *pTemp;
    SHORT           temp;

    // remove graph
    SetColor(GR_CLR_BACKGROUND);

    pTemp = ptr;
    temp = pos;

    for(x = 0; x < DSO_WINDOW_SIZE; x++)
    {
        ey = *ptr++;
        pos++;

        if(ey > sy)
        {
            for(y = sy + GR_TOP; y < ey + GR_TOP + 1; y++)
                PutPixel(GR_LEFT + pos, y);
        }
        else
        {
            for(y = ey + GR_TOP; y < sy + GR_TOP + 1; y++)
                PutPixel(GR_LEFT + pos, y);
        }

        if(ptr == (dsoBuffer + DSO_BUFFER_SIZE))
        {
            ptr = dsoBuffer;
            pos = 0;
        }

        sy = ey;
    }

    // copy new data from temporary buffer
    ptr = pTemp;
    pos = temp;

    for(counter = 0; counter < DSO_WINDOW_SIZE; counter++)
    {
        *ptr++ = tempBuffer[counter];
        pos++;
        if(ptr == (dsoBuffer + DSO_BUFFER_SIZE))
        {
            ptr = dsoBuffer;
            pos = 0;
        }
    }

    // draw graph
    SetColor(GR_CLR_POINTS);

    ptr = pTemp;
    pos = temp;

    for(x = 0; x < DSO_WINDOW_SIZE; x++)
    {
        ey = *ptr++;
        pos++;

        if(ey > tsy)
        {
            for(y = tsy + GR_TOP; y < ey + GR_TOP + 1; y++)
                PutPixel(GR_LEFT + pos, y);
        }
        else
        {
            for(y = ey + GR_TOP; y < tsy + GR_TOP + 1; y++)
                PutPixel(GR_LEFT + pos, y);
        }

        if(ptr == (dsoBuffer + DSO_BUFFER_SIZE))
        {
            ptr = dsoBuffer;
            pos = 0;
            dsoStates = DSO_DISPLAY;
        }

        tsy = ey;
    }

    // draw grid
    SetColor(LIGHTGRAY);
    SetLineType(DOTTED_LINE);
    for(x = GR_LEFT + ((GR_RIGHT - GR_LEFT) >> 3); x < GR_RIGHT; x += (GR_RIGHT - GR_LEFT) >> 3)
    {
        if((x >= GR_LEFT + temp) && (x <= GR_LEFT + DSO_WINDOW_SIZE + temp))
            WAIT_UNTIL_FINISH(Line(x, GR_TOP, x, GR_BOTTOM));
    }

    for(y = GR_TOP + ((GR_BOTTOM - GR_TOP) >> 3); y < GR_BOTTOM; y += (GR_BOTTOM - GR_TOP) >> 3)
        WAIT_UNTIL_FINISH(Line(GR_LEFT + temp, y, temp + GR_LEFT + DSO_WINDOW_SIZE, y));
    SetLineType(SOLID_LINE);	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
WORD DSO_GetSamples(WORD number)
{
    static BYTE     counter = 0;
    volatile SHORT  temp;

    temp = (GR_BOTTOM - GR_TOP) - ((DSOSample * (GR_BOTTOM - GR_TOP)) >> 12);

    if((temp + GR_TOP) > GR_BOTTOM)
        temp = GR_BOTTOM - GR_TOP;

    tempBuffer[counter++] = temp;

    if(counter >= number)
    {
        counter = 0;
        return (1);
    }

    return (0);
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
