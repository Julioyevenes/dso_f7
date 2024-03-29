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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DSO_H
#define __DSO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "GenericTypeDefs.h"
#include "Graphics.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define CTRLBTN_XINDENT         			0
#define CTRLBTN_HEIGHT          			45
#define CTRLBTN_WIDTH(nButton)      		(((GetMaxX() + 1) - (CTRLBTN_XINDENT * 2)) / nButton)
#define CtrlBtnTop()            			(GetMaxY() - CTRLBTN_HEIGHT)
#define CtrlBtnBottom()         			GetMaxY()
#define CtrlBtnLeft(column, nButton)     	(((column + 1) * CTRLBTN_XINDENT) + (column * CTRLBTN_WIDTH(nButton)))
#define CtrlBtnRight(column, nButton)    	((column + 1) * (CTRLBTN_XINDENT + CTRLBTN_WIDTH(nButton)))

#define InfoTextIndent         				0
#define InfoTextWidth(nText)  				(((GetMaxX() + 1) - (InfoTextIndent * 2)) / nText)
#define InfoTextTop()						0
#define InfoTextBottom()					InfoTextTop() + GetTextHeight((void *) &GOLFontDefault)
#define InfoTexLeft(col, nText)     		(((col + 1) * InfoTextIndent) + (col * InfoTextWidth(nText)))
#define InfoTexRight(col, nText)    		((col + 1) * (InfoTextIndent + InfoTextWidth(nText)))

// Dimensions for DSO graph area
#define DSO_ORIGIN_X    					0
#define DSO_ORIGIN_Y    					InfoTextBottom()

#define DSO_PANEL_LEFT   					DSO_ORIGIN_X
#define DSO_PANEL_RIGHT  					DSO_ORIGIN_X + GetMaxX()
#define DSO_PANEL_TOP    					DSO_ORIGIN_Y
#define DSO_PANEL_BOTTOM 					CtrlBtnTop()

// Graph area borders
#define GR_LEFT     						(DSO_PANEL_LEFT + GOL_EMBOSS_SIZE)
#define GR_RIGHT    						(DSO_PANEL_RIGHT - GOL_EMBOSS_SIZE)
#define GR_TOP      						(DSO_PANEL_TOP + GOL_EMBOSS_SIZE)
#define GR_BOTTOM   						(DSO_PANEL_BOTTOM - GOL_EMBOSS_SIZE)

// DSO data circular buffer size
#define DSO_BUFFER_SIZE 					(4 * 1024)

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern GOL_SCHEME 							*dsoScheme;
extern WORD 								adcBuffer[DSO_BUFFER_SIZE];

/* Exported functions ------------------------------------------------------- */
WORD DSO_Create(void);
WORD DSO_MsgCallback(WORD objMsg, OBJ_HEADER *pObj, GOL_MSG *pMsg);
WORD DSO_DrawCallback(void);
void DSO_InitStyleScheme(GOL_SCHEME *pScheme);

#ifdef __cplusplus
}
#endif

#endif /* __DSO_H */
