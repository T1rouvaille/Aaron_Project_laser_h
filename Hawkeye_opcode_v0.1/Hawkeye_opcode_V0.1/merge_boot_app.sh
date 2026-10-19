#!/bin/bash
# ============================================================================
# 合并 Bootloader + APP → bin / hex
#   烧录后 Bootloader 自动跳转到 APP (无需手动按键)
#
# 内存布局:
#   0x00000000 - 0x00003FFF : Bootloader (16KB)
#   0x00004000 - 0x0001FFFF : APP (112KB, 含 CRC + OTA flag)
#
# OTA flag:
#   bootloader 检测 0x1FFFC == 0x55555555 → CRC 校验通过 → 直接跳转 APP
#
# 用法:
#   1. 在 e2 studio 中编译 bootloader (ra2e1_boot) 和 APP
#   2. Git Bash 运行: ./merge_boot_app.sh
#   3. 输出:
#      Debug/Hawkeye_boot_app.bin  — 合并的 bin (128KB)
#      Debug/Hawkeye_boot_app.hex  — 合并的 hex (可用 Renesas Flash Programmer 烧录)
# ============================================================================

TOOLCHAIN="/c/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.2 Rel1/bin"
OBJCOPY="${TOOLCHAIN}/arm-none-eabi-objcopy"
PROJ="$(cd "$(dirname "$0")" && pwd)"

BOOT_ELF="${PROJ}/ra2e1_boot/Debug/ra2e1_boot.elf"
APP_CRC_BIN="${PROJ}/Debug/Hawkeye_V0.1_crc.bin"   # 完整 112KB APP 镜像 (含 CRC)
TMP_DIR="${PROJ}/Debug/_merge_tmp"
OUT_BIN="${PROJ}/Debug/Hawkeye_boot_app.bin"
OUT_HEX="${PROJ}/Debug/Hawkeye_boot_app.hex"

APP_FLASH_START=0x4000

mkdir -p "$TMP_DIR"

if [ ! -f "$BOOT_ELF" ]; then
    echo "[ERROR] Bootloader ELF not found: $BOOT_ELF"
    exit 1
fi
if [ ! -f "$APP_CRC_BIN" ]; then
    echo "[ERROR] APP crc.bin not found: $APP_CRC_BIN"
    echo "        请先在 e2 studio 中编译 APP (需要生成 crc.bin)"
    exit 1
fi

echo "========================================"
echo "  Merge Bootloader + APP (OTA Jump)"
echo "========================================"

# 1. 从 bootloader ELF 提取 flash 内容
echo "[1/5] Extracting bootloader..."
"$OBJCOPY" -O binary -j .text -j .data "$BOOT_ELF" "${TMP_DIR}/boot.bin"
BOOT_SIZE=$(wc -c < "${TMP_DIR}/boot.bin" | tr -d ' ')
echo "      Bootloader: ${BOOT_SIZE} bytes (max 0x${APP_FLASH_START})"

# 2. 复制 APP crc.bin 并写入 OTA flag
echo "[2/5] Patching OTA flag for auto-jump..."
cp "$APP_CRC_BIN" "${TMP_DIR}/app.bin"
APP_SIZE=$(wc -c < "${TMP_DIR}/app.bin" | tr -d ' ')

# crc.bin 大小 = 112KB (0x1C000), 最后4字节是 OTA flag (0x1BFFC 偏移)
# 写入 0x55555555 → bootloader 会校验 CRC 然后直接跳转 APP
OTA_OFFSET=$((APP_SIZE - 4))
printf '\x55\x55\x55\x55' | dd of="${TMP_DIR}/app.bin" bs=1 seek=$OTA_OFFSET count=4 conv=notrunc 2>/dev/null

# 验证
OTA_VAL=$(od -A n -t x4 -v -j $OTA_OFFSET "${TMP_DIR}/app.bin" | head -1 | tr -d ' ')
echo "      APP image: ${APP_SIZE} bytes (full partition)"
echo "      OTA flag @ +${OTA_OFFSET}: ${OTA_VAL}"

# 3. 填充到 APP 起始地址
echo "[3/5] Padding..."
PAD_SIZE=$((APP_FLASH_START - BOOT_SIZE))
echo "      Gap fill: ${PAD_SIZE} bytes (0x$(printf '%X' $BOOT_SIZE) → 0x${APP_FLASH_START})"

# 4. 合并 → bin
echo "[4/5] Merging bin..."
cat "${TMP_DIR}/boot.bin" > "$OUT_BIN"
dd if=/dev/zero bs=1 count=$PAD_SIZE 2>/dev/null | tr '\0' '\377' >> "$OUT_BIN"
cat "${TMP_DIR}/app.bin" >> "$OUT_BIN"

TOTAL=$(wc -c < "$OUT_BIN" | tr -d ' ')
echo "      Combined bin: ${TOTAL} bytes ($((TOTAL/1024)) KB)"

# 5. bin → hex (Intel HEX, base=0x00000000)
echo "[5/5] Generating hex..."
"$OBJCOPY" -O ihex -I binary --change-addresses 0x00000000 "$OUT_BIN" "$OUT_HEX"
HEX_LINES=$(wc -l < "$OUT_HEX" | tr -d ' ')
echo "      Combined hex: ${HEX_LINES} lines"

# 清理
rm -rf "$TMP_DIR"

echo "========================================"
echo "  Done!"
echo "  BIN: ${OUT_BIN}"
echo "  HEX: ${OUT_HEX}"
echo ""
echo "  烧录后 Bootloader 会自动校验 CRC 并跳转 APP"
echo "========================================"
