# Makefile — SAS HBA TLM 模型（SystemC）  [2026-10-08 建]
# 用法：make（构建并跑全部 TB）；make libs（出 libsas_hba_tlm.a）；make regress（全回归）；make clean
# ⚠ SYSTEMC_HOME / AMBA_PV_HOME 可用环境变量覆盖（交付件不绑死本机路径 ✓）
SYSTEMC_HOME  ?= /opt/tools/systemc2.34
# 指向你的 AMBA-PV 安装（VDK 自带 header-only：$(VDK)/SLS/linux/IP_common/AMBA_TLM2_BL/AMBA-PV；亦可环境变量覆盖）
AMBA_PV_HOME  ?= /opt/AMBA-PV
# HDD 模型（联合台用；发布树里由 stage 脚本定点改为 -Ihdd ✓）
# 发布树：HDD 模型在同层 hdd/（⚠ 值行勿带行尾注释：make 会连注释前空格一起取值 ✗）
HDD_DIR       ?= .

CXX      := g++
CXXFLAGS := -std=c++14 -O1 -g -Wall \
            -I$(SYSTEMC_HOME)/include -I$(AMBA_PV_HOME)/include \
            -Ihba -Ilink -Itop -Itb -I../sw -I$(HDD_DIR)/hdd
LDFLAGS  := -L$(SYSTEMC_HOME)/lib-linux64 -lsystemc -Wl,-rpath,$(SYSTEMC_HOME)/lib-linux64

OBJS := hba/sas_hba_tlm.o link/sas_link_tlm.o

# ── TB 清单（新增台在此加一行 + 一条规则 ✓）──
TB_LIST := tb_hba_regs tb_pair_bringup tb_neg tb_hdd_scsi
TB_BINS := $(addprefix tb/,$(TB_LIST))

all: run

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 联合台依赖 HDD 模型源码（同台编译 ✓；发布树里路径由 stage 定点修 ✓）
$(HDD_DIR)/hdd/sas_hdd_tlm.o: $(HDD_DIR)/hdd/sas_hdd_tlm.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

tb/%: tb/%.cpp $(OBJS) $(HDD_DIR)/hdd/sas_hdd_tlm.o
	$(CXX) $(CXXFLAGS) $< $(OBJS) $(HDD_DIR)/hdd/sas_hdd_tlm.o $(LDFLAGS) -o $@

run: $(TB_BINS)
	@fail=0; for t in $(TB_BINS); do \
	  out=$$(./$$t 2>&1); echo "$$out" | grep -q "^TB_.* PASS" && r=PASS || r=FAIL; \
	  line=$$(echo "$$out" | grep -E "^\[合计\]" | tail -1); \
	  printf "%-22s %-5s %s\n" "$$t" "$$r" "$$line"; \
	  [ "$$r" = PASS ] || { fail=1; echo "$$out" | tail -25; }; \
	done; [ $$fail -eq 0 ] && echo "全部 PASS ✓"

libs: $(OBJS)
	ar rcs libsas_hba_tlm.a $(OBJS)
	@echo "libsas_hba_tlm.a ✓"

regress:
	@sh regress.sh

clean:
	rm -f $(OBJS) $(HDD_DIR)/hdd/sas_hdd_tlm.o $(TB_BINS) libsas_hba_tlm.a

.PHONY: all run libs clean regress
