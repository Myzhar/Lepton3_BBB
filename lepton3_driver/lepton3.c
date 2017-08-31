/*
 * lepton3.c
 *
 * Copyright (C) 2017 Jason Kridner, Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * https://www.kernel.org/doc/html/v4.9/driver-api/spi.html
 */

#include <linux/module.h>
#include "lepton3.h"

MODULE_AUTHOR("Jason Kridner");
MODULE_LICENSE("GPL");

/*
static struct spi_driver lepton3_spi_driver = {
	.driver = {
		.name   = "lepton3",
		.owner  = THIS_MODULE,
	},
	.probe  = lepton3_probe_spi,
	.remove = lepton3_remove_spi,
};
*/

static int lepton3_init(void)
{
	return(0);
}

static void lepton3_exit(void)
{
}

module_init(lepton3_init);
module_exit(lepton3_exit);