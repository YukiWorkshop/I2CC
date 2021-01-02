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

#include <I2CC.hpp>

using namespace YukiWorkshop;

int main(int argc, char **argv) {
	I2CC i2c_dev("/dev/i2c-1", 7, 0x68);

	uint8_t buf[10];
	i2c_dev.read_data(0x00, buf, 10);

	puts("Register dump 0x00 - 0x09:");

	for (int i=0; i<10; i++) {
		printf("%02x ", buf[i]);
	}

	puts("");

	auto u16be = i2c_dev.read_u16_be(0x00);
	auto u16le = i2c_dev.read_u16_le(0x00);
	auto s16be = i2c_dev.read_s16_be(0x00);
	auto s16le = i2c_dev.read_s16_le(0x00);

	auto u32be = i2c_dev.read_u32_be(0x00);
	auto u32le = i2c_dev.read_u32_le(0x00);
	auto s32be = i2c_dev.read_s32_be(0x00);
	auto s32le = i2c_dev.read_s32_le(0x00);

	printf("0x00 as u16be: 0x%04x %u\n", u16be, u16be);
	printf("0x00 as u16le: 0x%04x %u\n", u16le, u16le);
	printf("0x00 as s16be: 0x%04x %d\n", s16be, s16be);
	printf("0x00 as s16le: 0x%04x %d\n", s16le, s16le);

	printf("0x00 as u32be: 0x%08x %u\n", u32be, u32be);
	printf("0x00 as u32le: 0x%08x %u\n", u32le, u32le);
	printf("0x00 as s32be: 0x%08x %d\n", s32be, s32be);
	printf("0x00 as s32le: 0x%08x %d\n", s32le, s32le);

	return 0;
}