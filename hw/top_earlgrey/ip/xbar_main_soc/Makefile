
XBAR_PERI       = xbar_periph
XBAR_MAIN       = xbar_main

all: peri


peri:
	rm -rf tmp
	mkdir tmp
	../../../../util/tlgen.py -t data/autogen/$(XBAR_PERI).hjson -o tmp
	cp tmp/rtl/autogen/* rtl
	rm -rf tmp

main:
	rm -rf tmp
	mkdir tmp
	../../../../util/tlgen.py -t data/autogen/$(XBAR_MAIN).hjson -o tmp
	cp tmp/rtl/autogen/* rtl
	rm -rf tmp
