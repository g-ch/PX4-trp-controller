/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ADIS16497.hpp"

#define DIR_READ				0x00
#define DIR_WRITE				0x80

// ADIS16497 registers
static constexpr uint8_t PAGE_ID  	= 0x0; // Page identifier

// Page 0x00
static constexpr uint8_t SYS_E_FLAG 	= 0x08; // Output, system error flags
static constexpr uint8_t DIAG_STS   	= 0x0A; // Output, self test error flags
static constexpr uint8_t BURST_CMD  	= 0x7C; // Burst-read command
static constexpr uint8_t PROD_ID    	= 0x7E; // Output, product identification

// Page 0x03
static constexpr uint8_t GLOB_CMD   	= 0x02; // Control, global commands
static constexpr uint8_t FNCTIO_CTRL 	= 0x06; // Control, I/O pins, functional definitions
static constexpr uint8_t GPIO_CTRL  	= 0x08; // Control, I/O pins, general-purpose
static constexpr uint8_t CONFIG  	= 0x0A; // Control, clock and miscellaneous corrections
static constexpr uint8_t DEC_RATE  	= 0x0C; // Control, output sample rate decimation
static constexpr uint8_t NULL_CNFG  	= 0x0E; // Control, automatic bias correction configuration
static constexpr uint8_t SYNC_SCALE  	= 0x10; // Control, automatic bias correction configuration
static constexpr uint8_t RANG_MDL   	= 0x12; // Measurement range (model-specific) Identifier TODO use this
static constexpr uint8_t FILTR_BNK_0   	= 0x16; // Filter selection
static constexpr uint8_t FILTR_BNK_1   	= 0x18; // Filter selection

static constexpr uint16_t PROD_ID_ADIS16497 = 0x4071;	// ADIS16497 Identification, device number

// Stall time between SPI transfers
static constexpr uint8_t T_STALL = 2;

static constexpr uint32_t ADIS16497_DEFAULT_RATE = 1000;

using namespace time_literals;

ADIS16497::ADIS16497(int bus, uint32_t device, enum Rotation rotation) :
	SPI("ADIS16497", nullptr, bus, device, SPIDEV_MODE3, 5000000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), ORB_PRIO_MAX, rotation),
	_px4_gyro(get_device_id(), ORB_PRIO_MAX, rotation),
	_sample_interval_perf(perf_alloc(PC_INTERVAL, "adis16497: read interval")),
	_sample_perf(perf_alloc(PC_ELAPSED, "adis16497: read")),
	_bad_transfers(perf_alloc(PC_COUNT, "adis16497: bad transfers"))
{
#ifdef GPIO_SPI1_RESET_ADIS16497
	// Configure hardware reset line
	px4_arch_configgpio(GPIO_SPI1_RESET_ADIS16497);
#endif // GPIO_SPI1_RESET_ADIS16497

	_px4_accel.set_device_type(DRV_ACC_DEVTYPE_ADIS16497);
	_px4_accel.set_sample_rate(ADIS16497_DEFAULT_RATE);
	_px4_accel.set_scale(1.25f * CONSTANTS_ONE_G / 1000.0f); // accel 1.25 mg/LSB

	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_ADIS16497);
	_px4_gyro.set_sample_rate(ADIS16497_DEFAULT_RATE);
	_px4_gyro.set_scale(math::radians(0.025f)); // gyro 0.025 ??/sec/LSB
}

ADIS16497::~ADIS16497()
{
	// make sure we are truly inactive
	stop();

	// delete the perf counters
	perf_free(_sample_interval_perf);
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
}

int
ADIS16497::init()
{
	// do SPI init (and probe) first
	if (SPI::init() != OK) {
		// if probe/setup failed, bail now
		PX4_DEBUG("SPI setup failed");
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

int ADIS16497::reset()
{
#ifdef GPIO_SPI1_RESET_ADIS16497
	// Hardware reset
	px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16497, 0);

	// The RST line must be in a low state for at least 10 ??s to ensure a proper reset initiation and recovery.
	usleep(10_us);

	px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16497, 1);
#else
	// Software reset (global command bit 7)
	uint8_t value[2] {};
	value[0] = (1 << 7);
	write_reg16(PAGE_ID, 0x03);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
#endif // GPIO_SPI1_RESET_ADIS16497

	// Reset recovery time
	usleep(210_ms);

	// Switch to configuration page
	write_reg16(PAGE_ID, 0x03);

	// Functional IO control
	static constexpr uint16_t FNCTIO_CTRL_DEFAULT = 0x000D;

	write_reg16(FNCTIO_CTRL, FNCTIO_CTRL_DEFAULT);

	usleep(340_us);

	const uint16_t fnctio_ctrl = read_reg16(FNCTIO_CTRL);

	if (fnctio_ctrl != FNCTIO_CTRL_DEFAULT) {
		PX4_ERR("Invalid setup, FNCTIO_CTRL=%#X", fnctio_ctrl);
		return PX4_ERROR;
	}

	// Miscellaneous configuration
	static constexpr uint16_t CONFIG_DEFAULT = 0x00C0;

	write_reg16(CONFIG, CONFIG_DEFAULT);

	usleep(45_us);

	const uint16_t config = read_reg16(CONFIG);

	if (config != CONFIG_DEFAULT) {
		PX4_ERR("Invalid setup, CONFIG=%#X", config);
		return PX4_ERROR;
	}

	// Decimation Filter
	static constexpr uint16_t DEC_RATE_DEFAULT = 0x0003; //  4250/4 = 1062 samples per second

	write_reg16(DEC_RATE, DEC_RATE_DEFAULT);

	usleep(340_us);

	const uint16_t dec_rate = read_reg16(DEC_RATE);

	if (dec_rate != DEC_RATE_DEFAULT) {
		PX4_ERR("Invalid setup, DEC_RATE=%#X", dec_rate);
		return PX4_ERROR;
	}

	// Continious bias estimation
	static constexpr uint16_t NULL_CNFG_DEFAULT = 0x0000; // Disable continious bias estimation

	write_reg16(NULL_CNFG, NULL_CNFG_DEFAULT);

	usleep(71_us);

	const uint16_t null_cnfg = read_reg16(NULL_CNFG);

	if (null_cnfg != NULL_CNFG_DEFAULT) {
		PX4_ERR("Invalid setup, NULL_CNFG=%#X", null_cnfg);
		return PX4_ERROR;
	}

	// Bartlett Window FIR Filter
	static constexpr uint16_t FILTR_BNK_0_SETUP = 0x0000; // Disable FIR filter
	static constexpr uint16_t FILTR_BNK_1_SETUP = 0x0000; // Disable FIR filter

	write_reg16(FILTR_BNK_0, FILTR_BNK_0_SETUP);
	write_reg16(FILTR_BNK_1, FILTR_BNK_1_SETUP);

	usleep(65_us);

	const uint16_t filtr_bnk_0 = read_reg16(FILTR_BNK_0);

	if (filtr_bnk_0 != FILTR_BNK_0_SETUP) {
		PX4_ERR("Invalid setup, FILTR_BNK_0=%#X", filtr_bnk_0);
		return PX4_ERROR;
	}

	const uint16_t filtr_bnk_1 = read_reg16(FILTR_BNK_1);

	if (filtr_bnk_1 != FILTR_BNK_1_SETUP) {
		PX4_ERR("Invalid setup, FILTR_BNK_1=%#X", filtr_bnk_1);
		return PX4_ERROR;
	}

	/*
	// Save to flash memory (NOTE : Limited cycles!)
	uint8_t value[2] = {};
	value[0] = (1 << 3);
	write_reg16(PAGE_ID, 0x03);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);

	// save Recovery Time
	usleep(1125_ms);
	*/

	return OK;
}

int
ADIS16497::probe()
{
	reset();

	// read product id (5 attempts)
	for (int i = 0; i < 5; i++) {

		// Switch to output page
		write_reg16(PAGE_ID, 0x00);

		uint16_t product_id = read_reg16(PROD_ID);

		if (product_id == PROD_ID_ADIS16497) {
			PX4_DEBUG("PRODUCT: %X", product_id);

			if (self_test()) {
				return PX4_OK;

			} else {
				PX4_ERR("probe attempt %d: self test failed, resetting", i);
				reset();
			}

		} else {
			PX4_ERR("probe attempt %d: read product id failed, resetting", i);
			reset();
		}
	}

	return -EIO;
}

bool
ADIS16497::self_test()
{
	// Switch to configuration page
	write_reg16(PAGE_ID, 0x03);

	// Self test (globa l command bit 1)
	uint8_t value[2] {};
	value[0] = (1 << 1);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);

	usleep(20_ms); // Self test time

	// Switch to output page
	write_reg16(PAGE_ID, 0x0);

	// Read SYS_E_FLAG to check overall result
	uint16_t sys_e_flag = read_reg16(SYS_E_FLAG);

	if (sys_e_flag != 0) {
		PX4_ERR("SYS_E_FLAG: %#X", sys_e_flag);

		// Read DIAG_STS to check per-sensor results
		uint16_t diag_sts_flag = read_reg16(DIAG_STS);

		if (diag_sts_flag != 0) {
			PX4_ERR("DIAG_STS: %#X", diag_sts_flag);
		}

		return false;
	}

	return true;
}

uint16_t
ADIS16497::read_reg16(uint8_t reg)
{
	uint16_t cmd[1] {};

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;
	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(nullptr, cmd, 1);
	up_udelay(T_STALL);

	return cmd[0];
}

void
ADIS16497::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t cmd[2] {};
	cmd[0] = reg | 0x8;
	cmd[1] = val;
	transfer(cmd, cmd, sizeof(cmd));
}

void
ADIS16497::write_reg16(uint8_t reg, uint16_t value)
{
	uint16_t cmd[2] {};

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	up_udelay(T_STALL);
}

void
ADIS16497::start()
{
#ifdef GPIO_SPI1_DRDY1_ADIS16497
	// Setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY1_ADIS16497, true, false, true, &ADIS16497::data_ready_interrupt, this);
#else
	// Make sure we are stopped first
	stop();

	// start polling at the specified rate
	ScheduleOnInterval((1_s / ADIS16497_DEFAULT_RATE), 10000);
#endif
}

void
ADIS16497::stop()
{
#ifdef GPIO_SPI1_DRDY1_ADIS16497
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY1_ADIS16497, false, false, false, nullptr, nullptr);
#else
	ScheduleClear();
#endif
}

int
ADIS16497::data_ready_interrupt(int irq, void *context, void *arg)
{
	ADIS16497 *dev = reinterpret_cast<ADIS16497 *>(arg);

	// make another measurement
	dev->ScheduleNow();

	return PX4_OK;
}

void
ADIS16497::Run()
{
	// make another measurement
	measure();
}

int
ADIS16497::measure()
{
	perf_count(_sample_interval_perf);
	perf_begin(_sample_perf);

	// Fetch the full set of measurements from the ADIS16497 in one pass (burst read).
	ADISReport adis_report{};
	adis_report.cmd = ((BURST_CMD | DIR_READ) << 8) & 0xff00;

	// ADIS16497 burst report should be 320 bits
	static_assert(sizeof(adis_report) == (320 / 8), "ADIS16497 report not 320 bits");

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != transferhword((uint16_t *)&adis_report, ((uint16_t *)&adis_report), sizeof(adis_report) / sizeof(uint16_t))) {
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	// Check burst ID to make sure the correct SPI speed is being used.
	// The sensor uses a different burst report format at slower speeds
	static constexpr uint16_t BURST_ID_DEFAULT = 0xA5A5;

	if (adis_report.BURST_ID != BURST_ID_DEFAULT) {
		PX4_DEBUG("BURST_ID: %#X", adis_report.BURST_ID);

		perf_count(_bad_transfers);
		perf_end(_sample_perf);

		return -EIO;
	}

	// Check all Status/Error Flag Indicators
	if (adis_report.SYS_E_FLAG != 0) {
		PX4_DEBUG("SYS_E_FLAG: %#X", adis_report.SYS_E_FLAG);

		perf_count(_bad_transfers);
		perf_end(_sample_perf);

		return -EIO;
	}

	uint32_t checksum = uint32_t(adis_report.CRC_UPR) << 16 | adis_report.CRC_LWR;
	uint32_t checksum_calc = crc32((uint16_t *)&adis_report.SYS_E_FLAG, 15);

	if (checksum != checksum_calc) {
		PX4_DEBUG("CHECKSUM: %#X vs. calculated: %#X", checksum, checksum_calc);

		perf_count(_bad_transfers);
		perf_end(_sample_perf);

		return -EIO;
	}

	const uint64_t error_count = perf_event_count(_bad_transfers);
	_px4_accel.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);

	const float temperature = (int16_t(adis_report.TEMP_OUT) * 0.0125f) + 25.0f; // 1 LSB = 0.0125??C, 0x0000 at 25??C
	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	// TODO check data counter here to see if we're missing samples/getting repeated samples
	{
		float xraw_f = (int32_t(adis_report.X_ACCEL_OUT) << 16 | adis_report.X_ACCEL_LOW) / 65536.0f;
		float yraw_f = (int32_t(adis_report.Y_ACCEL_OUT) << 16 | adis_report.Y_ACCEL_LOW) / 65536.0f;
		float zraw_f = (int32_t(adis_report.Z_ACCEL_OUT) << 16 | adis_report.Z_ACCEL_LOW) / 65536.0f;
		_px4_accel.update(timestamp_sample, xraw_f, yraw_f, zraw_f);
	}

	{
		float xraw_f = (int32_t(adis_report.X_GYRO_OUT) << 16 | adis_report.X_GYRO_LOW) / 65536.0f;
		float yraw_f = (int32_t(adis_report.Y_GYRO_OUT) << 16 | adis_report.Y_GYRO_LOW) / 65536.0f;
		float zraw_f = (int32_t(adis_report.Z_GYRO_OUT) << 16 | adis_report.Z_GYRO_LOW) / 65536.0f;
		_px4_gyro.update(timestamp_sample, xraw_f, yraw_f, zraw_f);
	}

	perf_end(_sample_perf);

	return OK;
}

void
ADIS16497::print_info()
{
	perf_print_counter(_sample_interval_perf);
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}
