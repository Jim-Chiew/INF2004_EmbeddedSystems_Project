################################################################################
# mtk_car -- build from the project root
#
# This folder is self-contained: copy mtk_car anywhere, and
#
#     make                 build the car's firmware (.uf2 in build_make/)
#     make TEST=1          ...with the on-target tests
#     make clean           remove everything the build produced
#     make NET=none MQTT=0 ...without WiFi or the broker
#
# builds it, with no paths to set up and nothing outside the folder needed
# except the two tools no project can carry: the Arm compiler and the Pico C
# SDK. Both are found automatically where the VS Code Pico extension puts them
# (~/.pico-sdk); see build_make/pico_rp2040.mk if yours live elsewhere.
#
# The real makefile is build_make/makefile -- this one only hands the work
# over, so that the folder root is a sensible place to type "make" and so the
# VS Code build task has one obvious target. Every option above is passed
# straight through.
################################################################################

# Not .PHONY-only: the default goal must come first so a bare "make" works.
.DEFAULT_GOAL := all

all clean:
	@$(MAKE) -C build_make $@

# Anything else (TEST=1 is a variable, not a target, so it needs no rule here)
# is handed over as well.
%:
	@$(MAKE) -C build_make $@

.PHONY: all clean
