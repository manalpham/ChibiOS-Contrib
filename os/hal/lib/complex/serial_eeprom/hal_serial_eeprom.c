/*
    Serial eeprom interface for ChibiOS-Contrib
        Copyright (C) 2020 E. Bernet-Rollande aeroman@alpham.eu

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_serial_eeprom.c
 * @brief   Generic serial eeprom driver class code.
 *
 * @addtogroup HAL_SERIAL_EEPROM
 * @{
 */

#include "hal.h"
#include "hal_serial_eeprom.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static size_t seeprom_get_size(void *instance);
static size_t seeprom_get_page_size(void *instance);
static uint32_t seeprom_get_page_number(void *instance);
static eeprom_error_t seeprom_read(void *instance, eeprom_offset_t offset, size_t n, uint8_t *rp);
static eeprom_error_t seeprom_read_page(void *instance, eeprom_offset_t offset, size_t n, uint8_t *rp);
static eeprom_error_t seeprom_write(void *instance, eeprom_offset_t offset, size_t n, const uint8_t *wp);
static eeprom_error_t seeprom_write_page(void *instance, eeprom_offset_t offset, size_t n, const uint8_t *wp);

/**
 * @brief   Virtual methods table.
 */
static const struct SEEPROMDriverVMT seeprom_vmt = {
  (size_t)0,
  seeprom_get_size,
  seeprom_read,
  seeprom_write,
  seeprom_get_page_size,
  seeprom_get_page_number,
  seeprom_read_page,
  seeprom_write_page
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if ((SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_I2C) &&                      \
     (SEEPROM_SHARED_BUS == TRUE)) || defined(__DOXYGEN__)

static void bus_acquire(BUSDriver *busp, const BUSConfig *config) {
  i2cAcquireBus(busp);
  if (busp->config != config) {
    i2cStart(busp, config);
  }
}

static void bus_release(BUSDriver *busp) {
  i2cReleaseBus(busp);
}

#elif (SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_SPI) &&                     \
      (SEEPROM_SHARED_BUS == TRUE)

static void bus_acquire(BUSDriver *busp, const BUSConfig *config) {
  spiAcquireBus(busp);
  if (busp->config != config) {
    spiStart(busp, config);
  }
}

static void bus_release(BUSDriver *busp) {
  spiReleaseBus(busp);
}

#else
/* No bus sharing, empty macros.*/
#define bus_acquire(busp, config)
#define bus_release(busp)

#endif

static size_t seeprom_get_size(void *instance) {
	SEEPROMDriver * devp = (SEEPROMDriver *) instance;

	osalDbgCheck(devp != NULL);
	osalDbgAssert(devp->state == EEPROM_READY, "invalid state");

	return devp->config->size;
}

static size_t seeprom_get_page_size(void *instance) {
	SEEPROMDriver * devp = (SEEPROMDriver *) instance;

	osalDbgCheck(devp != NULL);
	osalDbgAssert(devp->state == EEPROM_READY, "invalid state");

	return devp->config->page_size;
}

static uint32_t seeprom_get_page_number(void *instance) {
	SEEPROMDriver * devp = (SEEPROMDriver *) instance;

	osalDbgCheck(devp != NULL);
	osalDbgAssert(devp->state == EEPROM_READY, "invalid state");

	return devp->page_number;
}

static eeprom_error_t seeprom_read(void *instance, eeprom_offset_t offset, size_t n, uint8_t *rp) {
	SEEPROMDriver * devp = (SEEPROMDriver *) instance;
	eeprom_error_t err;

	osalDbgCheck((devp != NULL) && (rp != NULL) && (n > 0U));
	osalDbgCheck((size_t) offset + n <= devp->config->size);

	osalDbgAssert(devp->state == EEPROM_READY, "invalid state");

	/* Bus acquired.*/
	bus_acquire(devp->config->busp, devp->config->buscfg);

	/* EEPROM_READ state while the operation is performed.*/
	devp->state = EEPROM_READ;

	/* Actual read implementation.*/
	err = seeprom_device_read(devp, offset, n, rp);

	/* Ready state again.*/
	devp->state = EEPROM_READY;

	/* Bus released.*/
	bus_release(devp->config->busp);

	return err;
}

static eeprom_error_t seeprom_read_page(void *instance, eeprom_offset_t offset, size_t n, uint8_t *rp) {
	SEEPROMDriver * devp = (SEEPROMDriver *) instance;
	eeprom_error_t err;

	osalDbgCheck((devp != NULL) && (rp != NULL) && (n > 0U));
	osalDbgCheck((size_t) offset + n <= devp->page_number);

	osalDbgAssert(devp->state == EEPROM_READY, "invalid state");

	/* Bus acquired.*/
	bus_acquire(devp->config->busp, devp->config->buscfg);

	/* EEPROM_READ state while the operation is performed.*/
	devp->state = EEPROM_READ;

	/* Actual read page implementation.*/
	err = seeprom_device_read_page(devp, offset, n, rp);

	/* Ready state again.*/
	devp->state = EEPROM_READY;

	/* Bus released.*/
	bus_release(devp->config->busp);

	return err;
}

static eeprom_error_t seeprom_write(void *instance, eeprom_offset_t offset, size_t n, const uint8_t *wp) {
	SEEPROMDriver * devp = (SEEPROMDriver *) instance;
	eeprom_error_t err;

	osalDbgCheck((devp != NULL) && (wp != NULL) && (n > 0U));
	osalDbgCheck((size_t) offset + n <= devp->config->size);

	osalDbgAssert(devp->state == EEPROM_READY, "invalid state");

	/* Bus acquired.*/
	bus_acquire(devp->config->busp, devp->config->buscfg);

	/* EEPROM_WRITE state while the operation is performed.*/
	devp->state = EEPROM_WRITE;

	/* Actual write implementation.*/
	err = seeprom_device_write(devp, offset, n, wp);

	/* Ready state again.*/
	devp->state = EEPROM_READY;

	/* Bus released.*/
	bus_release(devp->config->busp);

	return err;
}

static eeprom_error_t seeprom_write_page(void *instance, eeprom_offset_t offset, size_t n, const uint8_t *wp) {
	SEEPROMDriver * devp = (SEEPROMDriver *) instance;
	eeprom_error_t err;

	osalDbgCheck((devp != NULL) && (wp != NULL) && (n > 0U));
	osalDbgCheck((size_t) offset + n <= devp->page_number);

	osalDbgAssert(devp->state == EEPROM_READY, "invalid state");

	/* Bus acquired.*/
	bus_acquire(devp->config->busp, devp->config->buscfg);

	/* EEPROM_WRITE state while the operation is performed.*/
	devp->state = EEPROM_WRITE;

	/* Actual write page implementation.*/
	err = seeprom_device_write_page(devp, offset, n, wp);

	/* Ready state again.*/
	devp->state = EEPROM_READY;

	/* Bus released.*/
	bus_release(devp->config->busp);

	return err;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void seepromObjectInit(SEEPROMDriver *devp) {
	  osalDbgCheck(devp != NULL);

	  devp->vmt         = &seeprom_vmt;
	  devp->state       = EEPROM_STOP;
	  devp->config      = NULL;
}

void seepromStart(SEEPROMDriver *devp, const SEEPROMConfig *config) {
	  osalDbgCheck((devp != NULL) && (config != NULL));
	  osalDbgAssert(devp->state != EEPROM_UNINIT, "invalid state");

	  devp->config = config;

	  osalDbgCheck(devp->config->page_size > 0U);
	  devp->page_number = devp->config->size / devp->config->page_size;

	  if (devp->state == EEPROM_STOP) {
#if !SEEPROM_SHARED_BUS
#if SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_I2C
		  i2cStart(devp->config->busp, devp->config->buscfg);
#elif SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_SPI
		  spiStart(devp->config->busp, devp->config->buscfg);
#endif
		  /* Device initialization.*/
		  seeprom_device_init(devp);
#else
		  /* Bus acquisition.*/
		  bus_acquire(devp->config->busp, devp->config->buscfg);

		  /* Device initialization.*/
		  seeprom_device_init(devp);

		  /* Bus release.*/
		  bus_release(devp->config->busp);
#endif
		  /* Driver in ready state.*/
		  devp->state = EEPROM_READY;
	  }
}

void seepromStop(SEEPROMDriver *devp) {
	  osalDbgCheck(devp != NULL);
	  osalDbgAssert(devp->state != EEPROM_UNINIT, "invalid state");

	  if (devp->state != EEPROM_STOP) {
#if !SEEPROM_SHARED_BUS
#if SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_I2C
		  i2cStop(devp->config->busp);
#elif SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_SPI
		  spiStop(devp->config->busp);
#endif
#else
		  /* Bus acquisition.*/
		  bus_acquire(devp->config->busp, devp->config->buscfg);

#if SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_I2C
		  i2cStop(devp->config->busp);
#elif SEEPROM_BUS_DRIVER == SEEPROM_BUS_DRIVER_SPI
		  spiStop(devp->config->busp);
#endif

		  /* Bus release.*/
		  bus_release(devp->config->busp);
#endif
		  /* Driver stopped.*/
		  devp->state = EEPROM_STOP;

		  /* Deleting current configuration.*/
		  devp->config = NULL;
	  }
}


/** @} */
