TARGET=burnd

ifeq (,$(TOOLCHAIN_PREFIX))
$(error TOOLCHAIN_PREFIX is not set)
endif

ifeq (,$(CFLAGS))
$(error CFLAGS is not set)
endif

ifeq (,$(LDFLAGS))
$(error LDFLAGS is not set)
endif

CXX = $(TOOLCHAIN_PREFIX)gcc

CFLAGS += -I$(SYSROOT)/usr/include

LDFLAGS += -L$(SYSROOT)/lib
LDFLAGS += -L$(SYSROOT)/usr/lib

SOURCE = $(wildcard *.cpp)
OBJS = $(patsubst %.cpp,%.o,$(SOURCE))

$(TARGET): $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CXX) $(CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@rm *.o -rf
	@rm $(OBJS) -rf
	@rm $(TARGET)
