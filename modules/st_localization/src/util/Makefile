SHELL := /bin/bash
DEB_VER := 0.$$(date '+%y%m%d.%H%M')+$$(git rev-parse --short HEAD)
CLEAN_VER := $$([[ -z $$(git status -s) ]] || echo '.dirty')
MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKEFILE_FOLDER_PATH := $(subst Makefile,,$(MAKEFILE_PATH))

COLOR_RED := \033[1;31m
COLOR_GREEN := \033[1;32m
COLOR_YELLOW := \033[1;33m
COLOR_RESET := \033[0m

all:
	@mkdir -p build
	@echo ""
	@echo "###################################################"
	@echo "# CMake"
	@echo "###################################################"
	@echo ""
	@cd build && cmake .. -DWITH_TDA=ON
	@echo ""
	@echo "###################################################"
	@echo "# Build"
	@echo "###################################################"
	@echo ""
	@cd build && make --no-print-directory -j4

clean:
	@rm -r build

install_deps:
	@echo "###################################################"
	@echo "# Install Dependencies"
	@echo "###################################################"
	@echo ""
	@if [ -f $(MAKEFILE_FOLDER_PATH)/deb_dependencies.txt ]; then \
	cat $(MAKEFILE_FOLDER_PATH)/deb_dependencies.txt | while read line; \
	do \
		/bin/echo -e "installing dependency : $(COLOR_GREEN)$$line$(COLOR_RESET)"; \
		sudo apt install -y $$line; \
	done \
	else \
	/bin/echo -e "File deb_dependencies.txt not found. \n$(COLOR_YELLOW)Skip installing dependencies.$(COLOR_RESET)"; \
	fi

hozon_deb_adcu: hozon_deb

hozon_deb_pdcu: hozon_deb

hozon_deb:
	@echo "###################################################"
	@echo "# Make QNX Deb"
	@echo "###################################################"
	@echo ""

	@sudo rm -rf /usr/local/adframework/*
	@source /usr/local/qnx710/qnxsdp-env.sh 
	@mkdir -p build
	@cd build; \
	cmake .. \
	    -DWITH_TDA=ON  -DWITH_QNX=ON  \
		-DCMAKE_BUILD_TYPE="Release" \
		-DCMAKE_INSTALL_PREFIX="/usr/local/adframework/qnx_aarch64" \
		-DMODULE_VERSION="$(DEB_VER)$(CLEAN_VER)"; \
	make package --no-print-directory -j4

hozon_linux_deb:
	@echo "###################################################"
	@echo "# Make Linux Deb"
	@echo "###################################################"
	@echo ""

	@sudo rm -rf /usr/local/adframework/*
	@sudo dpkg -r senseauto-hozon-framework-sdk
	@mkdir -p x86_build
	@cd x86_build;			\
	cmake .. -DWITH_TDA=ON -DWITH_X86=ON \
		-DCMAKE_INSTALL_PREFIX=/usr/local/adframework/linux_x86_64; \
	make package --no-print-directory -j4; \
	sudo dpkg -i *.deb

	@mkdir -p build
	@cd build; \
	cmake .. \
	    -DWITH_TDA=ON \
		-DCMAKE_BUILD_TYPE="Release" \
		-DCMAKE_INSTALL_PREFIX="/usr/local/adframework/linux_aarch64" \
		-DMODULE_VERSION="$(DEB_VER)$(CLEAN_VER)"; \
	make package --no-print-directory -j4

hozon_deb_x86:
	@echo "###################################################"
	@echo "# Make X86 Deb"
	@echo "###################################################"
	@echo ""

	@sudo rm -rf /usr/local/adframework/*
	@mkdir -p build
	@cd build;			\
	cmake .. -DWITH_TDA=ON -DWITH_X86=ON  -DWITH_ADCU=ON \
		-DCMAKE_BUILD_TYPE="Release" \
		-DCMAKE_INSTALL_PREFIX=/usr/local/adframework/linux_x86_64 \
		-DMODULE_VERSION="$(DEB_VER)$(CLEAN_VER)"; \
	make package --no-print-directory -j4

test:
	@echo "###################################################"
	@echo "# Test"
	@echo "###################################################"
	@echo ""
	@mkdir -p build
	@cd build; \
	make test
