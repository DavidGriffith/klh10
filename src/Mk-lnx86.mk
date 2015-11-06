# KLH10 Makefile for Linux on i386
# $Id: Mk-lnx86.mk,v 2.5 2002/04/24 18:03:04 klh Exp $
#
#  Copyright � 2001 Kenneth L. Harrenstien
#  All Rights Reserved
#
#  This file is part of the KLH10 Distribution.  Use, modification, and
#  re-distribution is permitted subject to the terms in the file
#  named "LICENSE", which contains the full text of the legal notices
#  and should always accompany this Distribution.
#
#  This software is provided "AS IS" with NO WARRANTY OF ANY KIND.
#
#  This notice (including the copyright and warranty disclaimer)
#  must be included in all copies or derivations of this software.
#
#####################################################################

# Local config setup, for GNU "make"!
#	Recursively invokes make with right params for local platform.

# Build definitions
SRC = ../../src
CFLAGS = -c -g3 -O3 -I. -I$(SRC)
CFLAGS_LINT = -ansi -pedantic -Wall -Wshadow \
		-Wstrict-prototypes -Wmissing-prototypes \
		-Wmissing-declarations -Wredundant-decls

# Source definitions
CENVFLAGS = -DCENV_CPU_I386=1 -DCENV_SYS_LINUX=1 -DKLH10_DEV_LITES=1 \
		-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE

# Any target with no customized rule here is simply passed on to the
# standard Makefile.  If no target is specified, "usage" is passed on
# to generate a helpful printout.

usage .DEFAULT:
	@make -f $(SRC)/Makefile.mk $@ \
	    "SRC=$(SRC)" \
	    "CFLAGS=$(CFLAGS)" \
	    "CFLAGS_LINT=$(CFLAGS_LINT)" \
	    "CENVFLAGS=$(CENVFLAGS)"

install:
	make -f $(SRC)/Makefile.mk install-unix
