AUTOMAKE_OPTIONS = subdir-objects

TB_INC = ../src/include/test
SOF_INC = $(prefix)/include/sof
DEFINE = -DSOF_INC=\"$(SOF_INC)\"

INCLUDE = -I$(TB_INC) -I$(SOF_INC)

AM_CPPFLAGS = $(INCLUDE) $(DEFINE)
AM_CFLAGS = -g -Wall
AM_LDFLAGS = -L$(prefix)/lib

bin_PROGRAMS = testbench

testbench_SOURCES = \
	testbench.c

testbench_LDADD = \
	-ldl -lm -lsof_ipc \
	libtb_common.a \
	-lsof

noinst_LIBRARIES = libtb_common.a

libtb_common_a_SOURCES = \
	common_test.c \
	topology.c \
	file.c \
	trace.c \
	ipc.c \
	schedule.c \
	alloc.c
