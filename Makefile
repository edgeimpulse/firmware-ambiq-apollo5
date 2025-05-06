include make/helpers.mk
include make/local_overrides.mk
include make/neuralspot_config.mk
include make/neuralspot_toolchain.mk
include make/jlink.mk
include autogen_$(BOARD)_$(EVB)_$(TOOLCHAIN).mk

ifeq ($(TOOLCHAIN),arm)
COMPDIR := armclang
else ifeq ($(TOOLCHAIN),arm-none-eabi)
COMPDIR := gcc
endif

# local_app_name := main <-- moved to autogen
TARGET = $(local_app_name)
sources := $(wildcard src/*.c)
sources += $(wildcard src/*.cc)
sources += $(wildcard src/*.cpp)
sources += $(wildcard src/*.s)
sources += $(wildcard src/ns-core/*.c)
sources += $(wildcard src/ns-core/*.cc)
sources += $(wildcard src/ns-core/*.cpp)
sources += $(wildcard src/ns-core/*.s)
sources += $(wildcard src/ns-core/$(BOARD)/*.c)
sources += $(wildcard src/ns-core/$(BOARD)/*.cc)
sources += $(wildcard src/ns-core/$(BOARD)/*.cpp)
sources += $(wildcard src/ns-core/$(BOARD)/*.s)
sources += $(wildcard src/ns-core/$(BOARD)/$(COMPDIR)/*.c)
sources += $(wildcard src/ns-core/$(BOARD)/$(COMPDIR)/*.cc)
sources += $(wildcard src/ns-core/$(BOARD)/$(COMPDIR)/*.cpp)
sources += $(wildcard src/ns-core/$(BOARD)/$(COMPDIR)/*.s)

# EdgeImpulse Stuff

# Common Code
# CMSIS stuff
sources +=   $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Source/TransformFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Source/CommonTables/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Source/BasicMathFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Source/ComplexMathFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Source/FastMathFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Source/SupportFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Source/MatrixFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Source/StatisticsFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/ActivationFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/BasicMathFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/ConcatenationFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/ConvolutionFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/FullyConnectedFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/NNSupportFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/PoolingFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/ReshapeFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/SoftmaxFunctions/*.c) \
			 $(wildcard src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Source/SVDFunctions/*.c) \

# Edge Impulse sources
sources +=		$(wildcard src/edge-impulse/edge-impulse-sdk/classifier/*.cpp) \
 				$(wildcard src/edge-impulse/edge-impulse-sdk/dsp/dct/*.cpp) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/dsp/kissfft//*.cpp) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/dsp/image/*.cpp ) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/dsp/memory.cpp) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/porting/ambiq/*.cpp) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/tensorflow/lite/kernels/*.cc) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/*.cc) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/tensorflow/lite/micro/kernels/*.cc) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/tensorflow/lite/micro/*.cc) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/tensorflow/lite/micro/memory_planner/*.cc) \
				$(wildcard src/edge-impulse/edge-impulse-sdk/tensorflow/lite/core/api/*.cc) \
				$(wildcard src/edge-impulse/tflite-model/*.cpp) \
				$(wildcard src/edge-impulse/ingestion-sdk-platform/apollo5/*.cpp) \
				$(wildcard src/edge-impulse/ingestion-sdk-platform/sensor/*.cpp) \
				$(wildcard src/edge-impulse/ingestion-sdk-c/*.cpp) \
				$(wildcard src/edge-impulse/inference/*.cpp) \
				$(wildcard src/edge-impulse/firmware-sdk/*.cpp) \
				$(wildcard src/edge-impulse/firmware-sdk/at-server/*.cpp) \
				$(wildcard src/edge-impulse/firmware-sdk/QCBOR/src/*.c) \
				$(wildcard src/edge-impulse/firmware-sdk/sensor-aq/*.cpp) \
				$(wildcard src/edge-impulse/firmware-sdk/jpeg/*.cpp) \				

# peripheral
sources +=	$(wildcard src/peripheral/*.c) \
			$(wildcard src/peripheral/usb/*.c)

VPATH+=$(dir $(sources))

targets  := $(BINDIR)/$(local_app_name).axf
targets  += $(BINDIR)/$(local_app_name).bin

objects      = $(call source-to-object,$(sources))
dependencies = $(subst .o,.d,$(objects))

DEFINES += EI_CLASSIFIER_ALLOCATION_STATIC=1
DEFINES += EI_PORTING_AMBIQ=1				  # Enable CMSIS-DSP optimized features
#DEFINES += HEAP_SIZE=4096
DEFINES += EI_SENSOR_AQ_STREAM=FILE
DEFINES += EI_TENSOR_ARENA_LOCATION=".shared"

LOCAL_INCLUDES += src/
LOCAL_INCLUDES += src/ns-core/
LOCAL_INCLUDES += src/edge-impulse/
LOCAL_INCLUDES += src/edge-impulse/edge-impulse-sdk/CMSIS/Core/Include
LOCAL_INCLUDES += src/edge-impulse/edge-impulse-sdk/CMSIS/NN/Include/
LOCAL_INCLUDES += src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/Include/
LOCAL_INCLUDES += src/edge-impulse/edge-impulse-sdk/CMSIS/DSP/PrivateInclude/
LOCAL_INCLUDES += src/edge-impulse/edge-impulse-sdk/classifier/
LOCAL_INCLUDES += src/edge-impulse/ingestion-sdk-c/

CFLAGS += -flax-vector-conversions
CFLAGS     += $(addprefix -D,$(DEFINES))
CFLAGS     += $(addprefix -I includes/,$(INCLUDES))
CFLAGS     += $(addprefix -I ,$(LOCAL_INCLUDES))
ifeq ($(TOOLCHAIN),arm)
LINKER_FILE := src/ns-core/$(BOARD)/$(COMPDIR)/linker_script.sct
else ifeq ($(TOOLCHAIN),arm-none-eabi)
LINKER_FILE := src/ns-core/$(BOARD)/$(COMPDIR)/linker_script_$(BOOTLOADER).ld
endif

all: $(BINDIR) $(objects) $(targets)

.PHONY: clean
clean:
ifeq ($(OS),Windows_NT)
	@echo "Windows_NT"
	@echo $(Q) $(RM) -rf $(BINDIR)/*
	$(Q) $(RM) -rf $(BINDIR)/*
else
	$(Q) $(RM) -rf $(BINDIR) $(JLINK_CF)
endif

ifneq "$(MAKECMDGOALS)" "clean"
  include $(dependencies)
endif

$(BINDIR):
	$(Q) $(MKD) -p $@

$(BINDIR)/%.o: %.cc
	@echo " Compiling $(COMPILERNAME) $< to make $@"
	$(Q) $(MKD) -p $(@D)
	$(Q) $(CC) -c $(CFLAGS) $(CCFLAGS) $< -o $@

$(BINDIR)/%.o: %.cpp
	@echo " Compiling $(COMPILERNAME) $< to make $@"
	$(Q) $(MKD) -p $(@D)
	$(Q) $(CC) -c $(CFLAGS) $(CCFLAGS) $< -o $@

$(BINDIR)/%.o: %.c
	@echo " Compiling $(COMPILERNAME) $< to make $@"
	$(Q) $(MKD) -p $(@D)
	$(Q) $(CC) -c $(CFLAGS) $(CONLY_FLAGS) $< -o $@

$(BINDIR)/%.o: %.s
	@echo " Assembling $(COMPILERNAME) $<"
	$(Q) $(MKD) -p $(@D)
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(BINDIR)/$(local_app_name).axf: $(objects)
	@echo " Linking $(COMPILERNAME) $@"
	$(Q) $(MKD) -p $(@D)
	$(Q) $(CC) -Wl,-T,$(LINKER_FILE) -o $@ $(objects) $(LFLAGS)

$(BINDIR)/$(local_app_name).bin: $(BINDIR)/$(local_app_name).axf
	@echo " Copying $(COMPILERNAME) $@..."
	$(Q) $(MKD) -p $(@D)
	$(Q) $(CP) $(CPFLAGS) $< $@
	$(Q) $(OD) $(ODFLAGS) $< > $(BINDIR)/$(local_app_name).lst
	$(Q) $(SIZE) $(objects) $(lib_prebuilt) $< > $(BINDIR)/$(local_app_name).size

$(JLINK_CF):
	@echo " Creating JLink command sequence input file..."
	$(Q) echo "ExitOnError 1" > $@
	$(Q) echo "Reset" >> $@
	$(Q) echo "LoadFile $(BINDIR)/$(TARGET).bin, $(JLINK_PF_ADDR)" >> $@
	$(Q) echo "Exit" >> $@

.PHONY: deploy
deploy: $(JLINK_CF)
	@echo " Deploying $< to device (ensure JLink USB connected and powered on)..."
	$(Q) $(JLINK) $(JLINK_CMD)
	$(Q) $(RM) $(JLINK_CF)

.PHONY: view
view:
	@echo " Printing SWO output (ensure JLink USB connected and powered on)..."
	$(Q) $(JLINK_SWO) $(JLINK_SWO_CMD)
	$(Q) $(RM) $(JLINK_CF)

%.d: ;
