# ----------------------------
# Program Options
# ----------------------------

NAME ?= BOIDS
ICON ?= icon.png
DESCRIPTION  ?= "2D Boids Simulation, based on a javascript version by CubeDHuang."
COMPRESSED ?= YES
ARCHIVED ?= YES

CFLAGS ?= -Wall -Wextra -Oz
CXXFLAGS ?= -Wall -Wextra -Oz

# ----------------------------

ifndef CEDEV
$(error CEDEV environment path variable is not set)
endif

include $(CEDEV)/meta/makefile.mk
