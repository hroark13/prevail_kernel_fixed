#ifndef KEY_EXPANDER_ADP5587_H
#define KEY_EXPANDER_ADP5587_H


#define ADP5587_I2C_WRITE_ADDR              0x68
#define ADP5587_I2C_READ_ADDR               0x69


/* ADP5587 Register */
// Device ID
#define ADP5587_REG_DEV_ID                  0x00

// Configuration Register 1
#define ADP5587_REG_CFG                     0x01

// Interrupt Status Register
#define ADP5587_REG_INT_STAT                0x02

// Keylock and event counter register
#define ADP5587_REG_KEY_LCK_EC_STAT         0x03

// Key Event Register A~H
#define ADP5587_REG_KEY_EVENTA              0x04
#define ADP5587_REG_KEY_EVENTB              0x05
#define ADP5587_REG_KEY_EVENTC              0x06
#define ADP5587_REG_KEY_EVENTD              0x07
#define ADP5587_REG_KEY_EVENTE              0x08
#define ADP5587_REG_KEY_EVENTF              0x09
#define ADP5587_REG_KEY_EVENTG              0x0A
#define ADP5587_REG_KEY_EVENTH              0x0B
#define ADP5587_REG_KEY_EVENTI              0x0C
#define ADP5587_REG_KEY_EVENTJ              0x0D

// Keypad Unlock 1 to Keypad Unlock 2 timer
#define ADP5587_REG_KP_LCK_TMR              0x0E

// Unlock Key 1
#define ADP5587_REG_UNLOCK1                 0x0F

// Unlock Key 2
#define ADP5587_REG_UNLOCK2                 0x10

// GPIO interrupt status
#define ADP5587_REG_GPIO_INT_STAT1          0x11
#define ADP5587_REG_GPIO_INT_STAT2          0x12
#define ADP5587_REG_GPIO_INT_STAT3          0x13

// GPIO data status, read twice to clear
#define ADP5587_REG_DAT_STAT1               0x14
#define ADP5587_REG_DAT_STAT2               0x15
#define ADP5587_REG_DAT_STAT3               0x16

// GPIO data out
#define ADP5587_REG_DAT_OUT1                0x17
#define ADP5587_REG_DAT_OUT2                0x18
#define ADP5587_REG_DAT_OUT3                0x19

// GPIO interrupt enable
#define ADP5587_REG_INT_EN1                 0x1A
#define ADP5587_REG_INT_EN2                 0x1B
#define ADP5587_REG_INT_EN3                 0x1C

// Keypad or GPIO selection
#define ADP5587_REG_KP_GPIO1                0x1D
#define ADP5587_REG_KP_GPIO2                0x1E
#define ADP5587_REG_KP_GPIO3                0x1F

// GPI Event Mode
#define ADP5587_REG_GPI_EM_REG1             0x20
#define ADP5587_REG_GPI_EM_REG2             0x21
#define ADP5587_REG_GPI_EM_REG3             0x22

// GPIO data direction
#define ADP5587_REG_GPIO_DIR1               0x23
#define ADP5587_REG_GPIO_DIR2               0x24
#define ADP5587_REG_GPIO_DIR3               0x25

// GPIO edge/level detect
#define ADP5587_REG_GPIO_INT_LVL1           0x26
#define ADP5587_REG_GPIO_INT_LVL2           0x27
#define ADP5587_REG_GPIO_INT_LVL3           0x28

// Debounce disable
#define ADP5587_REG_DEBOUNCE_DIS1           0x29
#define ADP5587_REG_DEBOUNCE_DIS2           0x2A
#define ADP5587_REG_DEBOUNCE_DIS3           0x2B

// GPIO pull disable
#define ADP5587_REG_GPIO_PULL1              0x2C
#define ADP5587_REG_GPIO_PULL2              0x2D
#define ADP5587_REG_GPIO_PULL3              0x2E

#define ADP5587_REG_MAX                     0xFF

extern unsigned char hw_version;

// GPIO
#if defined(CONFIG_MACH_CHIEF)
#define KEY_INT                 40
#define KEY_RST                 124
#define KEY_SCL                 ((hw_version>=1)?(1):(37))
#define KEY_SDA                 41

/* GPIOs on ADP5587 */
/* -------------------------------------------------------------------*/
/* |1111 1111     |1        |11            |1          |1111         |*/
/* |expander gpio |not used |register name |col or row |assigned bit |*/
/* -------------------------------------------------------------------*/
#define GPIO_BT_WLAN_REG_ON     0xFF43  /* COL3 */
#define ACC_INT                 0xFF44  /* COL4 */
#define SVC_LED_R               0xFF45  /* COL5 */
#define SVC_LED_B               0xFF46  /* COL6 */
#define CAM_EN                  0xFF47  /* COL7 */
#define CAM_VT_nSTBY            0xFF60  /* COL8 */
#define POPUP_SW_EN             0xFF61  /* COL9 */

#define PS_OUT                  0xFF33  /* ROW3 */
#define T_FLASH_DET             0xFF34  /* ROW4 */
#define UART_SEL                0xFF35  /* ROW5 */
#define KEY_LED_EN              0xFF36  /* ROW6 */

int expandergpio_get_value(unsigned int gpio);
int expandergpio_get_value(unsigned int gpio);
void expandergpio_set_value(unsigned int gpio, int on);
int expandergpio_configure(unsigned int gpio, unsigned long flags);
unsigned int expandergpio_int_en(unsigned int gpio);
extern void keyexpander_gpio_control();
#else

#define KEY_INT                 40
#define KEY_RST                 72
#define KEY_SCL                 73
#define KEY_SDA                 74
#endif


struct adp5587_kpad_platform_data {
	/* code map for the keys */
  unsigned int rows;
  unsigned int cols;
  unsigned int en_keylock;
  unsigned int unlock_key1;
  unsigned int unlock_key2;
	unsigned int *keymap;
	unsigned int keymapsize;
  unsigned int repeat;
};


#endif /* KEY_EXPANDER_ADP5588_H */
