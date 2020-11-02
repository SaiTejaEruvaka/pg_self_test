/**
 * \file ST7565R.c
 * \brief Functions relating to ST7565R.
 * \author Andy Gock
 * \see glcd.h
 */
#include "main.h"
#include "glcd.h"
#include "LCDTest.h"

// extern uint8_t left_scroll_array[128];
#if defined(GLCD_CONTROLLER_ST7565R)

#if 0
void glcd_Scroll_write(uint8_t page)
{
    uint8_t bank = page;
    uint8_t column;
//	for (bank = 0; bank < GLCD_NUMBER_OF_BANKS; bank++) {
    /* Each bank is a single row 8 bits tall */


//		if (glcd_bbox_selected->y_min >= (bank+1)*8) {
//			continue; /* Skip the entire bank */
//		}

//		if (glcd_bbox_selected->y_max < bank*8) {
//			break;    /* No more banks need updating */
//		}

    glcd_set_y_address(bank);
    glcd_set_x_address(0);
    // Aug20 - cppcheck warning - removed 128th location access
    for (column = 0; column < 128; column++)
    {
        glcd_data(left_scroll_array[column]);
    }
    // }

//	glcd_reset_bbox();
}
#endif
void glcd_spi_write(uint8_t c)
{
    PLATFORM_spi1_write(&c, 1);
}

void GLCD_CS_HIGH(void)
{
	 HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin,GPIO_PIN_SET);
}

void GLCD_CS_LOW(void)
{
	 HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin,GPIO_PIN_RESET);
}

void GLCD_A0_HIGH(void)
{
   HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin,GPIO_PIN_SET);
}

void GLCD_A0_LOW(void)
{
    HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin,GPIO_PIN_RESET);
}

void glcd_command(uint8_t c)
{
    GLCD_A0_LOW();
    glcd_spi_write(c);
}

void glcd_data(uint8_t c)
{
    GLCD_A0_HIGH();
    glcd_spi_write(c);
}

void glcd_set_contrast(uint8_t val)
{
    /* Can set a 6-bit value (0 to 63)  */

    /* Must send this command byte before setting the contrast */
    glcd_command(0x81);

    /* Set the contrat value ("electronic volumne register") */
    if (val > 63)
    {
        glcd_command(63);
    }
    else
    {
        glcd_command(val);
    }
    return;
}
#if 0
void glcd_power_down(void)
{
    /* Not applicable */
    return;
    // asm("break");
}

void glcd_power_up(void)
{
    /* Not applicable */
    return;
    // asm("break");
}
#endif
void glcd_set_y_address(uint8_t y)
{
    glcd_command(ST7565R_PAGE_ADDRESS_SET | (0x0F & y));
}

void glcd_set_x_address(uint8_t x)
{
    glcd_set_column_upper(x);
    glcd_set_column_lower(x);
}

#if 0
void glcd_all_on(void)
{
    glcd_command(ST7565R_DISPLAY_ALL_ON);
}
#endif

void glcd_normal(void)
{
    glcd_command(ST7565R_DISPLAY_NORMAL);
}

void glcd_set_column_upper(uint8_t addr)
{
    glcd_command(ST7565R_COLUMN_ADDRESS_SET_UPPER | (addr >> 4));
}

void glcd_set_column_lower(uint8_t addr)
{
    glcd_command(ST7565R_COLUMN_ADDRESS_SET_LOWER | (0x0f & addr));
}
#if 0
void glcd_set_start_line(uint8_t addr)
{
    glcd_command(ST7565R_SET_START_LINE | (0b00111111 & addr));
}
#endif
/** Clear the display immediately, does not buffer */
void glcd_clear_now(void)
{
    uint8_t page;
    uint8_t col;
    for (page = 0; page < GLCD_NUMBER_OF_BANKS; page++)
    {
        glcd_set_y_address(page);
        glcd_set_x_address(0);

        for (col = 0; col < GLCD_NUMBER_OF_COLS; col++)
        {
            glcd_data(0);
        }
    }
}
#if 0
void glcd_pattern(void)
{
    uint8_t page;
    for (page = 0; page < GLCD_NUMBER_OF_BANKS; page++)
    {
        glcd_set_y_address(page);
        glcd_set_x_address(0);
        uint8_t col;
        for (col = 0; col < GLCD_NUMBER_OF_COLS; col++)
        {
            glcd_data((col / 8 + 2) % 2 == 1 ? 0xff : 0x00);
        }
    }
}
#endif

void glcd_write(void)
{
    uint8_t bank;

    for (bank = 0; bank < GLCD_NUMBER_OF_BANKS; bank++)
    {
        /* Each bank is a single row 8 bits tall */
        uint8_t column;

        if (glcd_bbox_selected->y_min >= (bank + 1) * 8)
        {
            continue; /* Skip the entire bank */
        }

        if (glcd_bbox_selected->y_max < bank * 8)
        {
            break;    /* No more banks need updating */
        }

        glcd_set_y_address(bank);
        glcd_set_x_address(glcd_bbox_selected->x_min);

        for (column = glcd_bbox_selected->x_min; column <= glcd_bbox_selected->x_max; column++)
        {
            glcd_data(glcd_buffer_selected[GLCD_NUMBER_OF_COLS * bank + column]);
        }
    }

    glcd_reset_bbox();
}

void glcd_ST7565R_init(void)
{
    int i = 0;

#if defined(GLCD_INIT_NHD_C12832A1Z_FSW_FBW_3V3)
    /* Init sequence based on datasheet example code for NHD-C12832A1Z-FSW-FBW-3V3 */
    /* Datasheet says max SCK frequency 20MHz for this LCD */
    /* We use "reverse direction" for common output mode, as opposed to datasheet specifying "normal direction" */

    glcd_command(0xa0); /* ADC select in normal mode */
    glcd_command(0xae); /* Display OFF */
    glcd_command(0xc8); /* Common output mode select: reverse direction (last 3 bits are ignored) */
    glcd_command(0xa2); /* LCD bias set at 1/9 */
    glcd_command(0x2f); /* Power control set to operating mode: 7 */
    glcd_command(0x21); /* Internal resistor ratio, set to: 1 */
    glcd_set_contrast(40); /* Set contrast, value experimentally determined, can set to 6-bit value, 0 to 63 */
    glcd_command(0xaf); /* Display on */

#elif defined(GLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT)
    /* Init sequence based on datasheet example code for NHD-C12864A1Z-FSW-FBW-HTT */
    /* Datasheet says max SCK frequency 2.5MHz for this LCD */
    /* We use "reverse direction" for common output mode, as opposed to datasheet specifying "normal direction" */

    glcd_command(0xa0); /* ADC select in normal mode */
    glcd_command(0xae); /* Display OFF */
    glcd_command(0xc8); /* Common output mode select: reverse direction (last 3 bits are ignored) */
    glcd_command(0xa2); /* LCD bias set at 1/9 */
    glcd_command(0x2f); /* Power control set to operating mode: 7 */
    glcd_command(0x26); /* Internal resistor ratio, set to: 6 */
    glcd_set_contrast(20); /* Set contrast, value experimentally determined */
    glcd_command(0xaf); /* Display on */

#elif defined(GLCD_INIT_NHD_C12864WC_FSW_FBW_3V3_M)
    /* Init sequence for NHD-C12864WC-FSW-FBW-3V3-M */

    glcd_command(ST7565R_RESET); /* Internal reset */
    glcd_command(0xa2); /* 1/9 bias */
    glcd_command(0xa0); /* ADC select, normal */
    glcd_command(0xc8); /* Com output reverse */
    glcd_command(0xa4); /* Display all points normal */
    glcd_command(0x40); /* Display start line set */
    glcd_command(0x25); /* Internal resistor ratio */
    glcd_set_contrast(45); /* Set contrast value, experimentally determined, value 0 to 63 */
    glcd_command(0x2f); /* Power controller set */
    glcd_command(0xaf); /* Display on */

#else
    GLCD_CS_LOW();

    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin,GPIO_PIN_RESET);

    for (i = 0; i < 100; i++)
        ;
    //  digitalWrite(rst, HIGH);
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin,GPIO_PIN_SET);
    glcd_command(0xa3);
    glcd_command(0xa0);
    glcd_command(0xc8);
    // glcd_command(0xC0);
    glcd_command(0x40);
    glcd_command(0x28 | 0x4);
    glcd_command(0x28 | 0x6);
    glcd_command(0x28 | 0x7);
    glcd_command(0x20 | 0x6);
    glcd_set_contrast(0); /* Set contrast, value experimentally determined, value 0 to 63 */
    glcd_command(0xaf); /* Display on */
    glcd_normal();
    // glcd_all_on();
    // glcd_select_screen(glcd_buffer,&glcd_bbox);


    /* Default init sequence */
    /* Currently just set the same as GLCD_INIT_NHD_C12864A1Z_FSW_FBW_HTT */

//	glcd_command(0xa0); /* ADC select in normal mode */
//	glcd_command(0xae); /* Display OFF */
//	glcd_command(0xc8); /* Common output mode select: reverse direction (last 3 bits are ignored) */
//	glcd_command(0xa2); /* LCD bias set at 1/9 */
//	glcd_command(0x2f); /* Power control set to operating mode: 7 */
//	glcd_command(0x26); /* Internal resistor ratio, set to: 6 */
//	glcd_set_contrast(20); /* Set contrast, value experimentally determined, value 0 to 63 */
//	glcd_command(0xaf); /* Display on */
#endif
}
#endif
