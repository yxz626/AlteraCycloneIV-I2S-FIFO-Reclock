import re
import sys
from pathlib import Path
import openpyxl

sys.stdout.reconfigure(encoding="utf-8")
root = Path(__file__).resolve().parent.parent
req = (root / "工程要求说明.txt").read_text(encoding="utf-8")
qsf = (root / "Cy4_I2Sq.qsf").read_text(encoding="utf-8")

expected = {}
for pin, sig in re.findall(r"PIN_([A-Z0-9]+)\s+-to\s+(\S+)", req):
    expected[sig] = "PIN_" + pin

actual = {}
for pin, sig in re.findall(r"set_location_assignment\s+PIN_([A-Z0-9]+)\s+-to\s+(\S+)", qsf):
    actual[sig] = "PIN_" + pin

# 来自核心板 IO 引脚分配表的 SDRAM 引脚
xlsx = root / "EP4CE6F17C8核心板资料和datasheet" / "EP4CE6F17C8核心板IO引脚分配表.xlsx"
ws = openpyxl.load_workbook(xlsx, data_only=True).worksheets[0]
for row in ws.iter_rows(values_only=True):
    sig, direction, pin = row[0], row[1], row[2]
    if isinstance(sig, str) and sig.startswith("sdram_") and isinstance(pin, str):
        m = re.fullmatch(r"\s*([A-Z]\d+)\s*", pin)
        if m:
            key = sig.strip()
            # 按引脚对照表修正：dqm[2] → dqm[0]
            if key == "sdram_dqm[2]":
                key = "sdram_dqm[0]"
            expected[key] = "PIN_" + m.group(1)

# 新增 DSD 格式指示引脚（用户追加需求，编码表见 f0~F3.txt，QSF 第 6b 组）
extra = {
    "dsd_on":  "PIN_F2",
    "fsel[0]": "PIN_C2",   # F0
    "fsel[1]": "PIN_A2",   # F1
    "fsel[2]": "PIN_A3",   # F2（注意：物理 PIN_F2 是 dsd_on，不是这个）
    "fsel[3]": "PIN_A4",   # F3
}
expected.update(extra)

errors = []
for sig, pin in sorted(expected.items()):
    if sig not in actual:
        errors.append("缺少引脚分配: %s -> %s" % (sig, pin))
    elif actual[sig] != pin:
        errors.append("引脚不一致: %s 期望 %s 实际 %s" % (sig, pin, actual[sig]))

print("需求/资料中定义的引脚数: %d" % len(expected))
print("QSF 中已分配的引脚数  : %d" % len(actual))
print("逐一核对            : %s" % ("全部一致" if not errors else "存在差异"))
for e in errors:
    print("  [FAIL] " + e)
print("RESULT: " + ("ALL PIN CHECKS PASSED" if not errors else "FAILED (%d)" % len(errors)))
sys.exit(1 if errors else 0)
