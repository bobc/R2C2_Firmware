// CMSIS
#include    "LPC17xx.h"
#include    "core_cm3.h"

// NXP
#include    "lpc17xx_nvic.h"
#include    "lpc17xx_pinsel.h"
#include    "lpc17xx_gpio.h"

// FatFs
#include    "ff.h"

// LPCUSB
#include    "usbapi.h"

// Local
#include    "blockdev.h"
#include    "sbl_iap.h"
#include    "sbl_config.h"
#include    "msc_scsi.h"
#include    "spi.h"
#include    "uart.h"
#include    "debug.h"
#include        "ios.h"

#include "config_pins.h"


#define extruder_heater_off()   digital_write(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, LOW);
#define heated_bed_off()        digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, LOW);
#define extruder_fan_off()      digital_write(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, LOW);
#define buzzer_off()            digital_write(BUZZER_PORT, BUZZER_PIN, LOW);

#define x_disable()         digital_write(X_ENABLE_PORT, X_ENABLE_PIN, 1)
#define x_step()            digital_write(X_STEP_PORT, X_STEP_PIN, 1)
#define y_disable()         digital_write(Y_ENABLE_PORT, Y_ENABLE_PIN, 1)
#define y_step()            digital_write(Y_STEP_PORT, Y_STEP_PIN, 1)
#define z_disable()         digital_write(Z_ENABLE_PORT, Z_ENABLE_PIN, 1)
#define z_step()            digital_write(Z_STEP_PORT, Z_STEP_PIN, 1)
#define e_disable()         digital_write(E_ENABLE_PORT, E_ENABLE_PIN, 1)
#define e_step()            digital_write(E_STEP_PORT, E_STEP_PIN, 1)



FATFS fatfs;
FIL f;
FRESULT r;

// in msc_usb_start.c
void init_usb_msc_device(void);

void reboot (void)
{
    NVIC_SystemReset();
}

// use variable from msc_scsi.c
BOOL usb_msc_not_ejected(void)
{
    return (ejected == 0)?TRUE:FALSE;
}

// from sbl_iap
void configure_boot_button (void)
{
    /* Configure bootloader IO button */
    PINSEL_CFG_Type pin;
    pin.Portnum = BOOT_BUTTON_PORT;
    pin.Pinnum = BOOT_BUTTON_PIN;
    pin.Funcnum = PINSEL_FUNC_0;
    pin.Pinmode = PINSEL_PINMODE_PULLUP;
    pin.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&pin);

    /* set as input */
    GPIO_SetDir(BOOT_BUTTON_PORT, (1<<BOOT_BUTTON_PIN), 0);
}

BOOL bootloader_button_pressed(void)
{
    /* Verify if bootloader pin is activated */
    if(GPIO_ReadValue(BOOT_BUTTON_PORT) & (1<<BOOT_BUTTON_PIN))
        return FALSE;
    return TRUE;
}

void set_safe_outputs(void)
{
    /*
     * Disable some pins like the ones for heaters while on bootloader, if not, the heaters would be ON
     */
    /* Extruder 0 Heater pin */
    pin_mode(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, OUTPUT);
    extruder_heater_off();
    /* Heated Bed 0 Heater pin */
    pin_mode(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, OUTPUT);
    heated_bed_off();
    /* Extruder fan pin */
    pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, OUTPUT);
    extruder_fan_off();
    if (BUZZER_PORT != 0xFF)
    {
        /* Buzzer fan pin */
        pin_mode(BUZZER_PORT, BUZZER_PIN, OUTPUT);
        buzzer_off();
    }
    if (STEPPERS_RESET_PORT != 0xFF)
    {
        /* Disable reset for all stepper motors */
        pin_mode(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, OUTPUT);
        digital_write(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, 1);
    }
    /* Disable all motors BUT enable step pin, so each LED will be ON */
    pin_mode(X_STEP_PORT, X_STEP_PIN, OUTPUT);
    pin_mode(X_ENABLE_PORT, X_ENABLE_PIN, OUTPUT);
    x_disable();
    x_step();
    pin_mode(Y_STEP_PORT, Y_STEP_PIN, OUTPUT);
    pin_mode(Y_ENABLE_PORT, Y_ENABLE_PIN, OUTPUT);
    y_disable();
    y_step();
    pin_mode(Z_STEP_PORT, Z_STEP_PIN, OUTPUT);
    pin_mode(Z_ENABLE_PORT, Z_ENABLE_PIN, OUTPUT);
    z_disable();
    z_step();
    pin_mode(E_STEP_PORT, E_STEP_PIN, OUTPUT);
    pin_mode(E_ENABLE_PORT, E_ENABLE_PIN, OUTPUT);
    e_disable();
    e_step();
}

// try to mount SD card and flash new firmware if present
BOOL flash_new_firmware (void)
{
    BOOL result = FALSE;
    
    if ((r = f_mount(0, &fatfs)) == FR_OK) 
    {
        if ((r = f_open(&f, "/firmware.bin", FA_READ | FA_OPEN_EXISTING)) == FR_OK) 
        {
            unsigned int fs = f_size(&f);
            DBG("found firmware.bin with %u bytes", fs);
            if ((fs > 0) && (fs <= USER_FLASH_SIZE)) {
                U8 buffer[FLASH_BUF_SIZE];
                for (unsigned int i = 0; i < fs; i += FLASH_BUF_SIZE) {
                    unsigned int j = FLASH_BUF_SIZE;
                    if (i + j > fs)
                      j = fs - i;
                    // DBG("writing %d-%d", i, i+j);
                    if ((r = f_read(&f, buffer, j, &j)) == FR_OK) {
                        // pad last block to a full sector size
                        while (j < FLASH_BUF_SIZE) {
                            buffer[j++] = 0xFF;
                        }
                        write_flash((unsigned int *) (USER_FLASH_START + i), (char *) &buffer, j);
                    }
                    else {
                        DBG("read failed: %d", r);
                        i = fs;
                    }
                }
                r = f_close(&f);
                r = f_unlink("/firmware.bck");
                r = f_rename("/firmware.bin", "/firmware.bck");
                
                result = TRUE;
            }
        }
        else 
        {
            DBG("open \"/firmware.bin\" failed: %d", r);
        }
        
#ifdef  GENERATE_FIRMWARE_CUR
        if (f_open(&f, "/firmware.bck", FA_READ | FA_OPEN_EXISTING)) 
        {
            f_close(&f);
        }
        else 
        {
            // no firmware.bck, generate one!
            if (f_open(&f, "/firmware.bck", FA_WRITE | FA_CREATE_NEW) == FR_OK) 
            {
                f_close(&f);
            }
        }
#endif
        // elm-chan's fatfs doesn't have an unmount function
        // f_umount(&fatfs);
    }
    else 
    {
        DBG("mount failed: %d", r);
    }
    spi_close();
    
    return result;
}

int main() 
{
    // DeInit NVIC and SCBNVIC
    NVIC_DeInit();
    NVIC_SCBDeInit();

    /* Configure the NVIC Preemption Priority Bits:
     * two (2) bits of preemption priority, six (6) bits of sub-priority.
     * Since the Number of Bits used for Priority Levels is five (5), so the
     * actual bit number of sub-priority is three (3)
     */
    NVIC_SetPriorityGrouping(0x05);

    NVIC_SetVTOR(0x00000000);

    set_safe_outputs();

    configure_boot_button();
    
    #ifdef  UARTDEBUG
        hal_uart_init();
    #endif

    DBG("== USB_Bootloader V1.1 ==" );
        
    if (BlockDevInit() == 0)
    {
        /* uSDCard is present */
#ifdef  UARTDEBUG
        if (1) {
            U32 size;
            BlockDevGetSize(&size);
            DBG("Found SD card of size %d", size);
            BlockDevGetBlockLength(&size);
            DBG("block length %d", size);
        }
#endif

        if (bootloader_button_pressed())
        {
            DBG("entering bootloader");
            init_usb_msc_device();

            for (;usb_msc_not_ejected();)
              USBHwISR();

            DBG("usb ejected, rebooting");

            USBHwConnect(FALSE);
            spi_close();
        }
        else 
        {
            if (flash_new_firmware())
            {
                DBG("flashed ok, rebooting");
                reboot ();
            }
            else
            {
                // 
                if (user_code_present()) 
                {
                    DBG("starting user code...");
                    execute_user_code();
                }
                else 
                {
                    DBG("user code invalid");
//                    reboot();
                    // oops
                    while (1);
                }
            }        
        }
        
        reboot();
    }
    else
    {
        /* probably uSDCard is not on the socket so we should execute user code/firmware */
        
        
        DBG("uSDCard init failed...");
        
        // 
        if (user_code_present()) 
        {
            DBG("starting user code...");
            execute_user_code();
        }
        else 
        {
            DBG("user code invalid");
            
//            reboot();
            // oops
            while (1);
        }
    }
}