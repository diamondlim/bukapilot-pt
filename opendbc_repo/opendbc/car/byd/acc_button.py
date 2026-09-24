#!/usr/bin/env python3
"""Stock-ACC button frames (0x3B0 PCM_BUTTONS) for the BYD port.

The frame is built here and transmitted by card, never by a helper process. msgq allows one publisher
per topic and 'sendcan' is card's: a second process publishing it makes msgq raise
MultiplePublishersError inside card and card dies (that is what happened on 24 Sep 2026, taking the car
daemon down mid-drive-log). So this module only packs bytes; the caller appends them to the CAN sends it
already publishes.

Layout. The DBC (opendbc/dbc/byd_general_pt.dbc message 944) gives the button bits, but it is wrong about
everything else: it shows no counter and no checksum, and the car's own button module transmits both. Read
off this car's live CAN (2026-09-24, idle frames from the car itself):

    04 10 00 00 00 00 00 eb      04 10 00 00 00 00 c0 2b
    04 10 00 00 00 00 f0 fb      04 10 00 00 00 00 10 db      04 10 00 00 00 00 20 cb

  * byte 6 high nibble = a 4-bit rolling counter (00, 10, 20, 30, f0, c0 ...)
  * byte 7 = checksum, and the whole 8-byte message sums to 0xFF:
    (04+10+00+00+00+00+00+eb) = 0xFF, (04+10+00+00+00+00+c0+2b) = 0x1FF

A frame with zeros there (what this module sent at first) sums to 0x14 and the car ignores it - which is
exactly what happened on the bench: ACC engaged, press sent, set speed unmoved. So both fields are
mandatory, not decoration.

  SET_BTN            byte0 bit3      RES_BTN   byte0 bit4      LKAS_ON_BTN   byte0 bit6
  DEC_DISTANCE_BTN   byte1 bit7      INC_DISTANCE_BTN byte2 bit0      ACC_ON_BTN    byte2 bit3
  SET_ME_1_1 (const) byte0 bit2      SET_ME_1_2 (const) byte1 bit4
"""

ADDR_PCM_BUTTONS = 0x3B0
BUS = 0                     # Sealion 7 takes the press on bus 0, the same bus the car's own module uses
FRAMES = 6                  # frames the button is held, ~60 ms at card's 100 Hz
COUNTER_BYTE = 6            # high nibble carries a 4-bit rolling counter
CHECKSUM_BYTE = 7
CHECKSUM_TOTAL = 0xFF       # all eight bytes must add up to this

# Deliberately no "lkas": that is the driver's lane-keep switch, not ours to press.
# Each entry is the set of bits to set. "step" is the one that matters: the car's OWN speed rocker
# transmits SET and RES *together* (0x1c with the constant bit), which is why single-bit presses from the
# box changed nothing even with a valid counter and checksum.
REQUESTS = {
    "step": ((0, 3), (0, 4)),   # SET+RES together - the car's own speed-button pattern
    "res": ((0, 4),),           # RES_BTN alone
    "set": ((0, 3),),           # SET_BTN alone (the car also sends this on its own)
    "cancel": ((2, 3),),        # ACC_ON_BTN     ACC off / cancel
    "dec_dist": ((1, 7),),      # DEC_DISTANCE_BTN
    "inc_dist": ((2, 0),),      # INC_DISTANCE_BTN
}
CONSTANTS = ((0, 2), (1, 4))          # SET_ME_1_1, SET_ME_1_2 - must be 1


def _finish(data: bytearray, counter: int) -> bytes:
    """Add the rolling counter and the checksum the car expects, and return the frame."""
    data[COUNTER_BYTE] |= (counter & 0x0F) << 4
    data[CHECKSUM_BYTE] = (CHECKSUM_TOTAL - sum(data[:CHECKSUM_BYTE])) & 0xFF
    return bytes(data)


def build(button: str, counter: int = 0) -> bytes:
    """8 data bytes for one PCM_BUTTONS frame with a single button pressed."""
    if button not in REQUESTS:
        raise ValueError("unknown or forbidden button %r (have: %s)" % (button, ", ".join(sorted(REQUESTS))))
    data = bytearray(8)
    for byte, bit in CONSTANTS:
        data[byte] |= 1 << bit
    for byte, bit in REQUESTS[button]:
        data[byte] |= 1 << bit
    return _finish(data, counter)


def release(counter: int = 0) -> bytes:
    """The frame with no button pressed - byte-for-byte the car's own idle frame."""
    data = bytearray(8)
    for byte, bit in CONSTANTS:
        data[byte] |= 1 << bit
    return _finish(data, counter)
