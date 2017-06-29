AUTOMAKE_OPTIONS = subdir-objects

INCDIR1 = ../c/include
INCDIR2 = $(prefix)/include/sof/
INCLUDE = -I$(INCDIR1) -I$(INCDIR2) 
DEFINE = -DCONFIG_BAYTRAIL -D__XTENSA__ -DMODULE_TEST 

AM_CPPFLAGS = $(INCLUDE) $(DEFINE)
LDADD = -lm -lsof -lsof_ipc -lsof_eq_fir -lsof_eq_iir

bin_PROGRAMS = eq_fir_test eq_iir_test

eq_fir_test_SOURCES = \
	eq_fir_test.c \
	../c/src/fileread.c \
	../c/src/filewrite.c \
	../c/src/common_test.h \
	../c/src/common_test.c

eq_iir_test_SOURCES = \
	eq_iir_test.c \
	../c/src/fileread.c \
	../c/src/filewrite.c \
	../c/src/common_test.h \
	../c/src/common_test.c
