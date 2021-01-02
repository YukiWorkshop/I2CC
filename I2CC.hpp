/*
    This file is part of I2CC.

    Copyright (C) 2021 ReimuNotMoe

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include <string>
#include <vector>
#include <system_error>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>

//#include <linux/i2c-dev.h>

#include "i2c-dev.h"

namespace YukiWorkshop {
	class I2CC {
	protected:
		int fd_ = -1;
		uint16_t addr_ = 0;
		uint8_t address_bits_ = 0;
	public:
		I2CC() = default;

		I2CC(const std::string &path) {
			fd_ = open(path.c_str(), O_RDWR);

			if (fd_ < 0) {
				throw std::system_error(errno, std::system_category(), "open");
			}
		}

		I2CC(const std::string &path, uint8_t address_bits, uint16_t addr, bool force = false) : I2CC(path) {
			set_address_bits(address_bits);
			set_slave_address(addr, force);
		}

		void set_address_bits(uint8_t address_bits) {
			int iocval;
			if (address_bits == 7) {
				iocval = 0;
			} else if (address_bits == 10) {
				iocval = 1;
			} else {
				throw std::logic_error("i2c address bits should be either 7 or 10 bits");
			}

			if (ioctl(fd_, I2C_TENBIT, iocval)) {
				throw std::system_error(errno, std::system_category(), "set_address_bits");
			}

			address_bits_ = address_bits;
		}

		void set_slave_address(uint16_t addr, bool force = false) {
			if (ioctl(fd_, force ? I2C_SLAVE_FORCE : I2C_SLAVE, addr)) {
				throw std::system_error(errno, std::system_category(), "set_slave_address");
			}

			addr_ = addr;
		}

		int fd() const noexcept {
			return fd_;
		}

		void read_data(uint8_t reg_addr, void *buf, uint8_t len) {
			for (uint8_t i=0; i<len; i++) {
				((uint8_t *)buf)[i] = read_u8(reg_addr + i);
			}
		}

		void write_data(uint8_t reg_addr, const void *buf, uint8_t len) {
			for (uint8_t i=0; i<len; i++) {
				write_u8(reg_addr + i, ((uint8_t *) buf)[i]);
			}
		}

		std::vector<uint8_t> read_data(uint8_t reg_addr, uint8_t len) {
			std::vector<uint8_t> ret(len);
			read_data(reg_addr, ret.data(), len);
			return ret;
		}

		template<typename T>
		void write_data(uint8_t reg_addr, const std::vector<T>& buf) {
			write_data(reg_addr, buf.data(), buf.size() * sizeof(T));
		}

		template<typename T>
		void write_data(uint8_t reg_addr, const T& buf) {
			write_data(reg_addr, buf.data(), buf.size());
		}

		uint8_t read_u8(uint8_t reg_addr) {
			auto rc = i2c_smbus_read_byte_data(fd_, reg_addr);
			if (rc < 0) {
				throw std::system_error(errno, std::system_category(), "i2c_smbus_write_byte_data");
			}
			return rc;
		}

		void write_u8(uint8_t reg_addr, uint8_t byte) {
			if (i2c_smbus_write_byte_data(fd_, reg_addr, byte) < 0) {
				throw std::system_error(errno, std::system_category(), "i2c_smbus_write_byte_data");
			}
		}

		int8_t read_s8(uint8_t reg_addr) {
			uint8_t ret = read_u8(reg_addr);
			return *((int8_t *)&ret);
		}

		void write_s8(uint8_t reg_addr, int8_t byte) {
			write_u8(reg_addr, *((uint8_t *)&byte));
		}

		uint16_t read_u16_le(uint8_t reg_addr) {
			auto rc = le16toh(i2c_smbus_read_word_data(fd_, reg_addr));
			if (rc < 0) {
				throw std::system_error(errno, std::system_category(), "i2c_smbus_read_word_data");
			}
			return rc;
		}

		void write_u16_le(uint8_t reg_addr, uint16_t word) {
			if (i2c_smbus_write_word_data(fd_, reg_addr, htole16(word)) < 0) {
				throw std::system_error(errno, std::system_category(), "i2c_smbus_write_word_data");
			}
		}

		int16_t read_s16_le(uint8_t reg_addr) {
			uint16_t ret = read_u16_le(reg_addr);
			return *((int16_t *)&ret);
		}

		void write_s16_le(uint8_t reg_addr, int16_t byte) {
			write_u16_le(reg_addr, *((uint16_t *)&byte));
		}

		uint16_t read_u16_be(uint8_t reg_addr) {
			auto rc = be16toh(i2c_smbus_read_word_data(fd_, reg_addr));
			if (rc < 0) {
				throw std::system_error(errno, std::system_category(), "i2c_smbus_read_word_data");
			}
			return rc;
		}

		void write_u16_be(uint8_t reg_addr, uint16_t word) {
			if (i2c_smbus_write_word_data(fd_, reg_addr, htobe16(word)) < 0) {
				throw std::system_error(errno, std::system_category(), "i2c_smbus_write_word_data");
			}
		}

		int16_t read_s16_be(uint8_t reg_addr) {
			uint16_t ret = read_u16_be(reg_addr);
			return *((int16_t *)&ret);
		}

		void write_s16_be(uint8_t reg_addr, int16_t word) {
			write_u16_be(reg_addr, *((uint16_t *)&word));
		}

		uint32_t read_u32_le(uint8_t reg_addr) {
			uint32_t ret;
			read_data(reg_addr, &ret, sizeof(uint32_t));
			return le32toh(ret);
		}

		void write_u32_le(uint8_t reg_addr, uint32_t dword) {
			uint32_t buf = htole32(dword);
			write_data(reg_addr, &buf, sizeof(uint32_t));
		}

		int32_t read_s32_le(uint8_t reg_addr) {
			uint32_t ret = read_u32_le(reg_addr);
			return *((int32_t *)&ret);
		}

		void write_s32_le(uint8_t reg_addr, int32_t dword) {
			write_u32_le(reg_addr, *((uint32_t *)&dword));
		}

		uint32_t read_u32_be(uint8_t reg_addr) {
			uint32_t ret;
			read_data(reg_addr, &ret, sizeof(uint32_t));
			return be32toh(ret);
		}

		void write_u32_be(uint8_t reg_addr, uint32_t dword) {
			uint32_t buf = htobe32(dword);
			write_data(reg_addr, &buf, sizeof(uint32_t));
		}

		int32_t read_s32_be(uint8_t reg_addr) {
			uint32_t ret = read_u32_be(reg_addr);
			return *((int32_t *)&ret);
		}

		void write_s32_be(uint8_t reg_addr, int32_t dword) {
			write_u32_be(reg_addr, *((uint32_t *)&dword));
		}
	};
}