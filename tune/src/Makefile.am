AUTOMAKE_OPTIONS = subdir-objects

INCDIR1 = ../c/include
INCDIR2 = $(prefix)/include/sof
INCLUDE = -I$(INCDIR1) -I$(INCDIR2)
DEFINE = -DCONFIG_BAYTRAIL -D__XTENSA__ -DMODULE_TEST

AM_CPPFLAGS = $(INCLUDE) $(DEFINE)
LDADD = -lm -lsof -lsof_ipc -lsof_src -lsof_tone -lsof_math

bin_PROGRAMS = src_test tone_test tonesrc_test

src_test_SOURCES = src_test.c \
	../c/src/fileread.c \
	../c/src/filewrite.c \
	../c/src/common_test.h \
	../c/src/common_test.c

tone_test_SOURCES = tone_test.c \
	../c/src/filewrite.c \
	../c/src/common_test.h \
	../c/src/common_test.c

tonesrc_test_SOURCES = \
	tonesrc_test.c \
	../c/src/filewrite.c \
	../c/src/common_test.h \
	../c/src/common_test.c
