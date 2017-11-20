AUTOMAKE_OPTIONS = subdir-objects

INCDIR1 = ../c/include
INCDIR2 = $(prefix)/include/sof
INCLUDE = -I$(INCDIR1) -I$(INCDIR2)
DEFINE = -DCONFIG_BAYTRAIL -D__XTENSA__ -DMODULE_TEST

AM_CPPFLAGS = $(INCLUDE) $(DEFINE)
AM_CFLAGS = -g -Wall
LDADD = -lm -lsof -lsof_ipc -lsof_volume

bin_PROGRAMS = volume_test

volume_test_SOURCES = volume_test.c \
	../c/src/fileread.c \
	../c/src/filewrite.c \
	../c/src/common_test.c
