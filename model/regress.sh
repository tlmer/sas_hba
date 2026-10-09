#!/bin/sh
# regress.sh — SAS HBA TLM 全回归入口  [2026-10-08 建]
# 口径（照 rdma400/tlm 先例 ✓）：**只认 TB 自带终判行**（`TB_<NAME> PASS`）；
#   本脚本只收集/汇总、不重新解释判据 ✓；先用 `make -q` 做**陈旧二进制守卫** ✓。
# 用法：sh regress.sh   （脚本无执行位是项目惯例 ✓）
set -u
cd "$(dirname "$0")" || exit 1

# ① 陈旧二进制守卫：对象/可执行比源码旧 ⇒ 先构建 ✓
if ! make -q run >/dev/null 2>&1; then
    echo "[regress] 构建中（有陈旧目标 ✓）…"
    make -s run >/dev/null 2>&1 || true
fi

TB_SRC=$(ls tb/tb_*.cpp 2>/dev/null)
[ -z "$TB_SRC" ] && { echo "[regress] 未发现 tb/tb_*.cpp ✗"; exit 1; }

pass=0; fail=0
printf "%-24s %-6s %s\n" "TB" "结果" "自带终判/读数"
printf -- "------------------------------------------------------------\n"
for src in $TB_SRC; do
    name=$(basename "$src" .cpp)
    [ -x "tb/$name" ] || { printf "%-24s %-6s %s\n" "$name" "缺" "（未构建 ✗）"; fail=$((fail+1)); continue; }
    out=$(./"tb/$name" 2>&1)
    verdict=$(echo "$out" | grep -E "^TB_.* PASS" | tail -1)
    summary=$(echo "$out" | grep -E "^\[合计\]" | tail -1)
    if [ -n "$verdict" ]; then
        pass=$((pass+1)); printf "%-24s %-6s %s\n" "$name" "PASS" "$summary"
    else
        fail=$((fail+1)); printf "%-24s %-6s %s\n" "$name" "FAIL" "$summary"
        echo "$out" | tail -25
    fi
done
printf -- "------------------------------------------------------------\n"
if [ "$fail" -eq 0 ]; then
    echo "全部 PASS ✓ ($pass/$((pass+fail)))"; exit 0
else
    echo "有失败 ✗ (PASS=$pass FAIL=$fail)"; exit 1
fi
