# NanoPi NEO Plus2 — IO / Pin Configuration Snapshot

Captured **2026-06-17** from the live device (`pi@192.168.178.133`) so the
peripheral + GPIO setup can be reproduced after flashing Armbian.

- **Board:** FriendlyElec NanoPi-NEO-Plus2
- **SoC:** Allwinner **H5** (`sun50iw2`, aarch64, 4 cores)
- **Old OS captured from:** Ubuntu Core 16.04 (FriendlyCore), vendor kernel **4.14.111**
- **Active device tree:** `sun50i-h5-nanopi-neo-plus2.dtb`
- **No FriendlyELEC overlays were applied** — the `overlays` u-boot var is unset
  in `uEnv.txt`, so everything below is baked into the **base DTB**, not an overlay.

> ⚠️ Reassurance for the Armbian migration: Armbian for the H5 uses the **same
> mainline `sunxi` pinctrl driver**, so the global GPIO numbering below
> (bank×32 + pin) is **identical** on Armbian. Only the *mechanism* for enabling
> buses changes (FriendlyELEC `uEnv.txt`/`.dtbo` → Armbian `armbianEnv.txt`
> overlays). See the "Re-enabling on Armbian" section.

---

## GPIO numbering scheme

Two pin controllers / gpiochips:

| gpiochip | sysfs base | lines | controller | banks |
|---|---|---|---|---|
| main PIO  | **0**   | 224 | `1c20800.pinctrl` | PA, PC, PD, PE, PF, PG |
| R_PIO (PMIC-side) | **352** | 32 | `1f02c00.pinctrl` | PL |

Global number = `bank_index × 32 + pin`, where
PA=0, PB=32, PC=64, PD=96, PE=128, PF=160, PG=192, and **PL=352** (separate chip).
Example: `PG13 = 192 + 13 = 205`; `PL10 = 352 + 10 = 362`.

---

## Enabled peripheral buses (muxed in the base DTB)

| Bus / device | SoC pins | Linux node | Notes |
|---|---|---|---|
| **UART0** (debug console) | PA4 (TX), PA5 (RX) | `1c28000.serial` → `ttyS0` | serial console @115200, login getty |
| **UART1** | PG6, PG7 | `1c28400.serial` → `ttyS1` | wired to on-board **Bluetooth** (BCM/AP6212) |
| **UART2** | PA0, PA1 | `1c2dc00`… → `ttyS2` | exposed on header |
| **UART3** | PA13, PA14, PA15, PA16 | `1c28c00.serial` → `ttyS3` | 4-wire (incl. RTS/CTS) |
| **I2C0** | PA11 (SCK), PA12 (SDA) | `1c2ac00.i2c` → `i2c-0` | header |
| **I2C1** | PA18 (SCK), PA19 (SDA) | `1c2b000.i2c` → `i2c-1` | header |
| **I2C2** | PE12 (SCK), PE13 (SDA) | `1c2b400.i2c` → `i2c-2` | header |
| **SPI0** | PC0 (CLK), PC1 (MOSI), PC2 (MISO), PC3 (CS0) | `1c68000.spi` → `spidev0.0` | CS0 driven as gpio_out; PA6 also held by spi as gpio_out (aux CS / TFT ctrl) |
| **MMC0** | PF0–PF5 (+ PF6 card-detect) | `1c0f000.mmc` | **microSD slot** |
| **MMC1 (SDIO)** | PG0–PG5 | `1c10000.mmc` | on-board **WiFi** (SDIO) |
| **MMC2** | PC5–PC16 (8-bit) | `1c11000.mmc` | **on-board 8 GB eMMC** (current root) |
| **EMAC (GbE)** | PD0–PD5, PD7–PD13, PD15–PD17 (RGMII) | `1c30000.ethernet` | gigabit Ethernet |
| **HDMI DDC** | — | `i2c-3` (DesignWare HDMI) | internal, not a header bus |

`i2cdetect -l` confirmed: `i2c-0/1/2` = `mv64xxx_i2c adapter`, `i2c-3` = HDMI DDC.

---

## Special-function GPIOs (claimed lines)

From `/sys/kernel/debug/gpio`:

| GPIO | Pin | Name / function | Dir | Default |
|---|---|---|---|---|
| 10  | PA10 | `status_led` (trigger = **heartbeat**) | out | lo |
| 102 | PD6  | `gmac-3v3` — Ethernet PHY 3V3 regulator enable | out | **hi** |
| 204 | PG12 | `usb0_id_det` — USB-OTG ID detect | in (IRQ) | lo |
| 205 | PG13 | `rfkill_bt` reset — **Bluetooth** enable/reset | out | hi |
| 354 | PL2  | `usb0-vbus` — USB-OTG VBUS enable | out | hi |
| 355 | PL3  | `k1` — on-board **K1 / KEY button** | in (IRQ) | hi |
| 358 | PL6  | (unlabeled) | out | lo |
| 359 | PL7  | `wifi_pwrseq` reset — **WiFi** power/reset | out | hi |
| 362 | PL10 | `nanopi:green:pwr` — green **power LED** (trigger = none) | out | hi |

**LEDs** (`/sys/class/leds`): `status_led` (PA10, heartbeat), `nanopi:green:pwr` (PL10).

---

## Loaded kernel modules of interest

- WiFi: `brcmfmac` + `brcmutil` (on-board AP6212), plus USB-WiFi drivers present:
  `8189es`, `88XXau`, `8821cu` (likely for optional USB dongles).
- Bluetooth: `bluetooth`, `hci_uart`, `btqca`, `btintel`, `bnep` (BT over UART1/PG6-7).
- Audio: `snd_soc_simple_card` (+ utils) — analog codec / simple-card.
- USB gadget: `g_mass_storage` / `usb_f_mass_storage` / `libcomposite`
  (USB-OTG configured as mass-storage gadget — note for reproduction).

---

## The LCD daemon

A service runs:
```
/usr/bin/lcd2usb_print CPU: {{CPU}} Mem: {{MEM}} IP: {{IP}} LoadAvg: {{LOADAVG}}
```
- Binary: `/usr/bin/lcd2usb_print` (FriendlyELEC-provided, dated 2020-01-02, 150 KB).
- It's FriendlyELEC's helper for their character LCD/OLED accessory.
- `lsusb` returned nothing (tool missing or no USB display currently attached) — the
  physical display connection could not be confirmed in this snapshot.
- **To reproduce on Armbian:** this binary is not in Armbian repos. Either copy
  `/usr/bin/lcd2usb_print` off the eMMC, or replace it with a small script that
  reads CPU/MEM/IP/LoadAvg and writes to the display (driver depends on whether
  the panel is I2C/SPI/USB — determine from the actual hardware).

---

## FriendlyELEC peripheral toggles (for reference)

`/boot/uEnv.txt` exposes these overlay switches (all currently default/off-comment):
`uart0..3`, `i2c0..2`, `spi0`, `pwm0`, `ir`, `tft28`, `tft13`.

`/boot/overlays/` `.dtbo` files available on the old image:
`i2c0, i2c1, i2c2, spi0, uart0, uart1, uart2, uart3, pwm0, ir, tft13, tft28,
gpio-dvfs-overlay`, plus `sun50i-h5-fixup.scr`.

Boot chain: U-Boot → `boot.scr` (from `boot.cmd`) → loads `Image` + `rootfs.cpio.gz`
+ `sun50i-h5-${board}.dtb`, applies overlays listed in `overlays` env var, then
`booti`. Root = `/dev/mmcblk0p2` (ext4), overlay data = `/dev/mmcblk0p3`.

---

## Re-enabling on Armbian (translation table)

Armbian H5 enables buses via `/boot/armbianEnv.txt` → `overlays=` line; overlay
`.dtbo`s live in `/boot/dtb/allwinner/overlay/`. Easiest path: `sudo armbian-config`
→ *System → Hardware*, tick the buses, reboot.

| Want | FriendlyELEC overlay | Armbian overlay name | armbianEnv.txt token |
|---|---|---|---|
| I2C0 (PA11/12) | `i2c0` | `sun50i-h5-i2c0` | `i2c0` |
| I2C1 (PA18/19) | `i2c1` | `sun50i-h5-i2c1` | `i2c1` |
| I2C2 (PE12/13) | `i2c2` | `sun50i-h5-i2c2` | `i2c2` |
| SPI0 + spidev (PC0-3) | `spi0` | `spi-spidev` (set `param_spidev_spi_bus=0`) | `spi-spidev` |
| UART1 (PG6/7) | `uart1` | `uart1` | `uart1` |
| UART2 (PA0/1) | `uart2` | `uart2` | `uart2` |
| UART3 (PA13-16) | `uart3` | `uart3` | `uart3` |
| PWM | `pwm0` | `pwm` | `pwm` |

Example `armbianEnv.txt` line to match this device's header buses:
```
overlays=i2c0 i2c1 i2c2 spi-spidev uart1 uart2 uart3
param_spidev_spi_bus=0
```
On-board functions (eMMC `mmc2`, SD `mmc0`, GbE `emac`, WiFi `mmc1`/`brcmfmac`,
BT on `uart1`, LEDs, K1 button, USB-OTG VBUS/ID) are part of Armbian's
`nanopineoplus2` base DT — no overlay needed; they come up automatically.

UART0 (PA4/5) remains the **serial console** on Armbian by default — keep a
USB-TTL adapter handy for first-boot debugging.
