#include "vl53l5cx.h"
#include "debug.h"

//#include "vl53l5cx_api.h"

// 0x29 is without w/r bit
#define VL53L5CX_DEFAULT_I2C_ADDRESS_NO_W_R_BIT 0x29


bool vl53l5cxInit(VL53L5CX_Dev_t *pdev, I2C_Dev *I2Cx)
{
  static uint8_t newI2cAddress = 0x45;  // TODO: Change this...

  uint8_t status;
  uint8_t isAlive;

  pdev->dev.platform.I2Cx = I2Cx;
  pdev->dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS_NO_W_R_BIT;

	status = vl53l5cx_is_alive(&pdev->dev, &isAlive);

	if(!isAlive || status)
	{
    // Failed to find device at default address, let's check the other one.
    //pdev->dev.platform.address = newI2cAddress;
    //status = vl53l5cx_is_alive(&pdev->dev, &isAlive);
    //if(!isAlive || status)
    //{
      DEBUG_PRINT("VL53L5CX not detected at address %02x or %02x, status: %d\n", VL53L5CX_DEFAULT_I2C_ADDRESS_NO_W_R_BIT, pdev->dev.platform.address, status);
		  return false;
    //}
	}

  status = vl53l5cx_init(&pdev->dev);
  if (status) {
    DEBUG_PRINT("VL53L5CX failed to init: %d\n", status);
    return false;
  }

  // TODO: fix i2c address set
  return true;

  // Set new I2C address
  status = vl53l5cx_set_i2c_address(&pdev->dev, newI2cAddress);
  if (status) {
    DEBUG_PRINT("VL53L5CX Failed to set i2c address: %d\n", status);
    return false;
  }

  pdev->dev.platform.address = newI2cAddress & 0b01111111;
  DEBUG_PRINT("VL5 init OK with i2c address %d\n", pdev->dev.platform.address);
  return true;
}

bool vl53l5cTestConnection(VL53L5CX_Dev_t* pdev)
{
    return true;
}